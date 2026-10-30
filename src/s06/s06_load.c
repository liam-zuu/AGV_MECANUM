#ifdef S06_LOAD

#include "s06_load.h"
#include "build_id_gen.h"            // FW_GIT_SHA, FW_GIT_DIRTY, FW_ENV: scripts/build_id.py sinh lúc build
#include "imu/imu.h"
#include "encoder/encoder.h"
#include "motor/motor.h"
#include "logger/logger.h"
#include "wifi/wifi.h"
#include "traj_receiver/traj_receiver.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "driver/temperature_sensor.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <fcntl.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "S06";

// ─────────────────────────────────────────────────────────────────────────────
// Dòng gửi về Pi (UDP S06_EVT_PORT, nhiều dòng một gói, mỗi dòng kết thúc '\n'):
//   J,qseq,t_us,kind,queued,sseq,epoch                      report IMU tại cổng nhận SPI (sensor_event_cb,
//                                                           trước update_data; Liam 29/10 "đếm ở cổng SPI")
//   I,qseq,t_us,kind,applied,sseq,epoch,n,lib_n,v0,v1,v2    report IMU tại callback ứng dụng (kind 1 accel,
//                                                           2 gyro, 8 game RV; giá trị float đủ chữ số)
//   R,qseq,t_us,reset_count                                 thông báo reset hoàn tất ESP32 nhận [V4-1]
//   P,qseq,t_before,t_after,wheel,speed,gpioA,cmpA,gpioB,cmpB   một lần ghi compare [L7-13]
//   E,eseq,t_before,t_after,fl,fr,rl,rr                     đọc count 4 bánh, mỗi 1 ms trong lượt [Y1, Z6]
//   S,qseq,t_us,run,ev_qdrop,enc_ovf,imu_rx,imu_cbq,imu_mutex,gyro_gap_max_us,relay_ok,relay_bad,
//     log_qdrop,log_send_err,ev_send_err,temp_c             thống kê mỗi 1 s; temp_c: cảm biến nhiệt trong
//                                                           chip ESP32-S3, °C, "nan" khi không đọc được
//                                                           (T5, Liam 30/10; không phải nhiệt độ môi trường)
//   #fw,..  #run_start,..  #run_end,..  #ack,..             dòng văn bản, cuối dòng có qseq=
//                                                           (#run_start có imu_init, imu_init_ms: T4 (1);
//                                                           ngay sau #run_end: #run_stats,log_send_err_run,
//                                                           enc_reads_run)
// qseq: tăng liên tục cho mọi dòng trừ E; eseq: riêng dòng E. Pi thấy chỗ hở = mất trên đường UDP.
// Mất trước khi gửi (hàng đợi đầy, vòng đệm encoder đầy) đếm ở ev_qdrop, enc_ovf; #run_end báo số
// tăng trong lượt (*_run); > 0 thì lượt không đủ điều kiện đo (spec S06 §4.3). Sự kiện IMU chỉ đẩy khi
// tx_task đã có socket, để hàng đợi không đầy lúc chờ WiFi khởi động.
// ─────────────────────────────────────────────────────────────────────────────

#ifndef S06_DEST_IP
#define S06_DEST_IP        LOGGER_PC_IP
#endif
#define S06_EVT_PORT       5007
#define S06_CMD_PORT       5011
#define S06_QLEN           160            // phần tử ~270 byte (dòng văn bản tới 256 ký tự)
#define S06_ENC_RING       1024           // 1 s đọc ở 1 kHz
#define S06_ENC_PERIOD_US  1000           // Q-S1, Liam chốt 29/10
#define S06_HEAVY_SWAP_US  15000000LL     // khối tải nặng: đổi vai 2 bánh 0,5 / 2 bánh 0,03 mỗi 15 s [Z8]
#define S06_T6_HOLD_US     1000000LL      // T6: giữ mỗi mức 1 s (≥ 100 frame bỏ + 200 frame chấm) [ĐỀ XUẤT]
#define S06_STAT_US        1000000LL

enum { PROF_ZERO = 0, PROF_HEAVY, PROF_T6 };
static const char *const PROF_NAME[] = { "zero", "heavy", "t6" };

// ─── Hàng đợi sự kiện ────────────────────────────────────────────────────────
enum { EV_IMU = 1, EV_PWM, EV_TEXT };
typedef struct {
    uint8_t kind;
    union {
        imu_event_t imu;
        struct { motor_set_info_t m; uint8_t wheel; int16_t speed; } pwm;
        char text[256];
    } u;
} s06_ev_t;

static QueueHandle_t     s_q;
static temperature_sensor_handle_t s_tsens;   // NULL khi cài không được: dòng S ghi "nan"
static volatile bool     s_tx_ready;
static volatile uint32_t s_ev_qdrop;
static volatile uint32_t s_ev_send_err;

// ─── Vòng đệm đọc encoder (một nơi ghi: callback esp_timer; một nơi đọc: tx_task) ─
typedef struct { int64_t t_b; int32_t dur; int32_t c[4]; } enc_rd_t;
static enc_rd_t          s_ring[S06_ENC_RING];
static volatile uint32_t s_ring_head, s_ring_tail;
static volatile uint32_t s_enc_ovf;
// Số lần đọc count trong lượt, đếm ở nơi đọc (kể cả lần vòng đệm đầy): #run_stats báo enc_reads_run để
// Pi so số dòng E nhận được với số producer đã đọc, thấy cả mất đầu/đuôi (Codex Sol vòng 2/2 S41)
static uint32_t s_enc_reads;

// ─── Trạng thái lượt ─────────────────────────────────────────────────────────
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_run_active;
static int64_t  s_t_start, s_dur_us;
static int      s_profile;
static bool     s_auto_end;
// Lệnh chờ control_task xử lý (cmd_task ghi, s06_tick đọc, trong s_mux)
static bool     s_req_start, s_req_stop;
static int      s_req_profile;
static int64_t  s_req_dur_us;
static bool     s_req_auto_end;
static char     s_req_reason[24];
static bool     s_force_apply;
static int      s_last_speed[4];
// Hở gyro lớn nhất trong lượt, trên chuỗi liên tục [L7-14] (hook IMU ghi, reset khi bắt đầu lượt)
static volatile bool    s_gap_reset;
static volatile int64_t s_gyro_last_t, s_gyro_gap_max;
// Bộ đếm lúc bắt đầu lượt, để #run_end báo số tăng trong lượt
static uint32_t s_run0_ev_qdrop, s_run0_enc_ovf, s_run0_log_qdrop;
// log_send_err tại START: #run_end báo log_send_err_run (Codex Sol vòng 1/2 S37: mốc lấy từ DATA đầu
// tiên PC nhận được có thể muộn hơn START, làm thiếu lỗi gửi đầu lượt)
static uint32_t s_run0_log_send_err;
static imu_drops_t s_run0_imu;

// ─── Đẩy sự kiện ─────────────────────────────────────────────────────────────
static void push(const s06_ev_t *e) {
    if (!s_q || xQueueSend(s_q, e, 0) != pdTRUE) s_ev_qdrop = s_ev_qdrop + 1;
}

static void push_text(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void push_text(const char *fmt, ...) {
    s06_ev_t e = { .kind = EV_TEXT };
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(e.u.text, sizeof e.u.text, fmt, ap);
    va_end(ap);
    push(&e);
}

static void imu_hook(const imu_event_t *ie) {
    if (!s_tx_ready) return;
    if (ie->kind == IMU_RPT_GYRO && ie->stage == IMU_STAGE_CB) {
        if (s_gap_reset) { s_gap_reset = false; s_gyro_last_t = 0; s_gyro_gap_max = 0; }
        if (s_gyro_last_t && ie->t_us - s_gyro_last_t > s_gyro_gap_max)
            s_gyro_gap_max = ie->t_us - s_gyro_last_t;
        s_gyro_last_t = ie->t_us;
    }
    s06_ev_t e = { .kind = EV_IMU };
    e.u.imu = *ie;
    push(&e);
}

static void enc_timer_cb(void *arg) {
    // S41: kiểm lượt và đếm cùng một vùng khóa với end_run, để mọi lần đọc đã đếm đều thuộc lượt
    taskENTER_CRITICAL(&s_mux);
    bool active = s_run_active;
    if (active) s_enc_reads++;
    taskEXIT_CRITICAL(&s_mux);
    if (!active) return;
    enc_rd_t r;
    int32_t c[ENC_COUNT];
    r.t_b = esp_timer_get_time();
    encoder_read_all(c);
    r.dur = (int32_t)(esp_timer_get_time() - r.t_b);
    for (int i = 0; i < 4; i++) r.c[i] = c[i];
    uint32_t h = s_ring_head, next = (h + 1) % S06_ENC_RING;
    if (next == s_ring_tail) { s_enc_ovf = s_enc_ovf + 1; return; }
    s_ring[h] = r;
    s_ring_head = next;
}

// ─── Gửi UDP (gom nhiều dòng một gói) ────────────────────────────────────────
static int                s_sock = -1;
static struct sockaddr_in s_dest;
static char               s_pkt[1400];
static int                s_pkt_len;
static uint32_t           s_qseq, s_eseq;

static void flush_pkt(void) {
    if (s_pkt_len == 0) return;
    if (sendto(s_sock, s_pkt, s_pkt_len, 0, (struct sockaddr *)&s_dest, sizeof s_dest) < 0)
        s_ev_send_err = s_ev_send_err + 1;
    s_pkt_len = 0;
}

static void append(const char *line, int n) {
    if (n <= 0) return;
    if (n > (int)sizeof s_pkt) n = sizeof s_pkt;
    if (s_pkt_len + n > (int)sizeof s_pkt) flush_pkt();
    memcpy(s_pkt + s_pkt_len, line, n);
    s_pkt_len += n;
}

static void format_ev(const s06_ev_t *e) {
    char l[300];
    int n = 0;
    uint32_t q = ++s_qseq;
    if (e->kind == EV_IMU) {
        const imu_event_t *i = &e->u.imu;
        if (i->kind == IMU_EVT_RESET)
            n = snprintf(l, sizeof l, "R,%lu,%lld,%lu\n", (unsigned long)q, (long long)i->t_us,
                         (unsigned long)i->n);
        else if (i->stage == IMU_STAGE_RX)
            n = snprintf(l, sizeof l, "J,%lu,%lld,%u,%u,%u,%lu\n", (unsigned long)q, (long long)i->t_us,
                         i->kind, i->applied, i->seq, (unsigned long)i->epoch);
        else
            n = snprintf(l, sizeof l, "I,%lu,%lld,%u,%u,%u,%lu,%lu,%lu,%.9g,%.9g,%.9g\n",
                         (unsigned long)q, (long long)i->t_us, i->kind, i->applied, i->seq,
                         (unsigned long)i->epoch, (unsigned long)i->n, (unsigned long)i->lib_n,
                         (double)i->v[0], (double)i->v[1], (double)i->v[2]);
    } else if (e->kind == EV_PWM) {
        const motor_set_info_t *m = &e->u.pwm.m;
        n = snprintf(l, sizeof l, "P,%lu,%lld,%lld,%u,%d,%d,%lu,%d,%lu\n", (unsigned long)q,
                     (long long)m->t_before, (long long)m->t_after, e->u.pwm.wheel, e->u.pwm.speed,
                     m->gpio[0], (unsigned long)m->cmp[0], m->gpio[1], (unsigned long)m->cmp[1]);
    } else {
        n = snprintf(l, sizeof l, "#%s,qseq=%lu\n", e->u.text, (unsigned long)q);
    }
    if (n >= (int)sizeof l) n = sizeof l - 1;
    append(l, n);
}

static void format_stat(int64_t t_now) {
    imu_drops_t d;
    imu_get_drops(&d);
    uint32_t ok = 0, bad = 0;
    int64_t last = 0;
    traj_receiver_stats(&ok, &bad, &last);
    float temp_c = NAN;
    if (!s_tsens || temperature_sensor_get_celsius(s_tsens, &temp_c) != ESP_OK) temp_c = NAN;
    char l[240];
    int n = snprintf(l, sizeof l, "S,%lu,%lld,%d,%lu,%lu,%lu,%lu,%lu,%lld,%lu,%lu,%lu,%lu,%lu,%.1f\n",
                     (unsigned long)++s_qseq, (long long)t_now, (int)s_run_active,
                     (unsigned long)s_ev_qdrop, (unsigned long)s_enc_ovf,
                     (unsigned long)d.rx_queue, (unsigned long)d.cb_queue, (unsigned long)d.cb_mutex,
                     (long long)s_gyro_gap_max, (unsigned long)ok, (unsigned long)bad,
                     (unsigned long)logger_qdrop(), (unsigned long)logger_send_err(),
                     (unsigned long)s_ev_send_err, (double)temp_c);
    append(l, n < (int)sizeof l ? n : (int)sizeof l - 1);
}

static void tx_task(void *pv) {
    while (!wifi_is_connected()) vTaskDelay(pdMS_TO_TICKS(200));
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "event socket failed");
        vTaskDelete(NULL);
        return;
    }
    fcntl(s_sock, F_SETFL, fcntl(s_sock, F_GETFL, 0) | O_NONBLOCK);
    s_dest.sin_family = AF_INET;
    s_dest.sin_port   = htons(S06_EVT_PORT);
    inet_pton(AF_INET, S06_DEST_IP, &s_dest.sin_addr);
    ESP_LOGI(TAG, "events -> %s:%d, commands on udp %d", S06_DEST_IP, S06_EVT_PORT, S06_CMD_PORT);
    s_tx_ready = true;

    int64_t t_stat = esp_timer_get_time() + S06_STAT_US;
    s06_ev_t e;
    while (1) {
        if (xQueueReceive(s_q, &e, pdMS_TO_TICKS(2)) == pdTRUE) {
            format_ev(&e);
            while (xQueueReceive(s_q, &e, 0) == pdTRUE) format_ev(&e);
        }
        while (s_ring_tail != s_ring_head) {
            const enc_rd_t *r = &s_ring[s_ring_tail];
            char l[120];
            int n = snprintf(l, sizeof l, "E,%lu,%lld,%lld,%ld,%ld,%ld,%ld\n", (unsigned long)++s_eseq,
                             (long long)r->t_b, (long long)(r->t_b + r->dur),
                             (long)r->c[0], (long)r->c[1], (long)r->c[2], (long)r->c[3]);
            s_ring_tail = (s_ring_tail + 1) % S06_ENC_RING;
            append(l, n);
        }
        int64_t now = esp_timer_get_time();
        if (now >= t_stat) {
            format_stat(now);
            t_stat += S06_STAT_US;
        }
        flush_pkt();
    }
}

// ─── Lệnh UDP từ Pi (Q-S2): "START <zero|heavy|t6> <thời lượng s> <tự kết thúc 0|1>", "STOP <lý do>" ─
static int profile_id(const char *s) {
    for (int i = 0; i < (int)(sizeof PROF_NAME / sizeof PROF_NAME[0]); i++)
        if (strcmp(s, PROF_NAME[i]) == 0) return i;
    return -1;
}

static void cmd_task(void *pv) {
    while (!wifi_is_connected()) vTaskDelay(pdMS_TO_TICKS(200));
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(S06_CMD_PORT),
                             .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (sock < 0 || bind(sock, (struct sockaddr *)&a, sizeof a) < 0) {
        ESP_LOGE(TAG, "command socket failed");
        vTaskDelete(NULL);
        return;
    }
    char buf[96];
    while (1) {
        int n = recv(sock, buf, sizeof buf - 1, 0);
        if (n <= 0) continue;
        buf[n] = '\0';
        for (int i = 0; i < n; i++)
            if (buf[i] == '\n' || buf[i] == '\r' || buf[i] == ',') buf[i] = ' ';
        char what[16] = "", prof[16] = "";
        unsigned long dur_s = 0;
        int auto_end = -1;
        const char *why = "ok";
        if (sscanf(buf, "%15s", what) != 1) {
            why = "empty";
        } else if (strcmp(what, "START") == 0) {
            int p = -1;
            if (sscanf(buf, "%*s %15s %lu %d", prof, &dur_s, &auto_end) != 3) why = "usage";
            else if ((p = profile_id(prof)) < 0) why = "profile";
            else if (dur_s == 0 || (auto_end != 0 && auto_end != 1)) why = "args";
            else {
                taskENTER_CRITICAL(&s_mux);
                if (s_run_active || s_req_start) why = "busy";
                else {
                    s_req_start    = true;
                    s_req_profile  = p;
                    s_req_dur_us   = (int64_t)dur_s * 1000000LL;
                    s_req_auto_end = auto_end;
                }
                taskEXIT_CRITICAL(&s_mux);
            }
        } else if (strcmp(what, "STOP") == 0) {
            char reason[24] = "stop_cmd";
            sscanf(buf, "%*s %23s", reason);
            taskENTER_CRITICAL(&s_mux);
            if (!s_run_active) why = "idle";
            else {
                s_req_stop = true;
                memcpy(s_req_reason, reason, sizeof s_req_reason);
            }
            taskEXIT_CRITICAL(&s_mux);
        } else {
            why = "unknown";
        }
        push_text("ack,t_us=%lld,ok=%d,why=%s,cmd=%.60s", (long long)esp_timer_get_time(),
                  strcmp(why, "ok") == 0, why, buf);
    }
}

// ─── Cấu hình, hash (Z21, L7-23) ─────────────────────────────────────────────
static uint32_t fnv1a(const char *s) {
    uint32_t h = 2166136261u;
    while (*s) { h ^= (uint8_t)*s++; h *= 16777619u; }
    return h;
}

static void start_run(int64_t t_now, int prof, int64_t dur_us, bool auto_end) {
    // Cấu hình lượt (dòng #cfg) và hash trên cấu hình + bản build (dòng #run_start)
    char cfg[128], all[224];
    snprintf(cfg, sizeof cfg, "profile=%s;duration_us=%lld;auto_end=%d;imu_period_us=%lu;"
             "enc_period_us=%d;heavy_swap_us=%lld;t6_hold_us=%lld",
             PROF_NAME[prof], (long long)dur_us, auto_end, (unsigned long)IMU_REPORT_PERIOD_US,
             S06_ENC_PERIOD_US, S06_HEAVY_SWAP_US, S06_T6_HOLD_US);
    snprintf(all, sizeof all, "%s;fw=%s;dirty=%d;env=%s", cfg, FW_GIT_SHA, FW_GIT_DIRTY, FW_ENV);
    s_run0_ev_qdrop  = s_ev_qdrop;
    s_run0_enc_ovf   = s_enc_ovf;
    s_run0_log_qdrop = logger_qdrop();
    s_run0_log_send_err = logger_send_err();
    imu_get_drops(&s_run0_imu);
    s_gap_reset  = true;
    s_profile    = prof;
    s_dur_us     = dur_us;
    s_auto_end   = auto_end;
    s_t_start    = t_now;
    s_force_apply = true;
    taskENTER_CRITICAL(&s_mux);
    s_enc_reads  = 0;
    s_run_active = true;
    taskEXIT_CRITICAL(&s_mux);
    // imu_init = mã imu_init() (0 = IMU_INIT_OK), imu_init_ms = thời gian khởi tạo: spec S06 T4 (1), Liam
    // duyệt 29/10 ở phiên THESIS ("Theo khuyến nghị"), vì dòng "BNO085 init OK" chỉ ra cổng serial
    push_text("run_start,first_data_seq=%lu,timestamp_us=%lld,profile=%s,duration_s=%lld,auto_end=%d,"
              "fw=%s,dirty=%d,env=%s,cfg_hash=%08lx,imu_init=%d,imu_init_ms=%lu",
              (unsigned long)(logger_seq() + 1), (long long)t_now, PROF_NAME[prof],
              (long long)(dur_us / 1000000LL), auto_end, FW_GIT_SHA, FW_GIT_DIRTY, FW_ENV,
              (unsigned long)fnv1a(all), imu_init_status(), (unsigned long)imu_init_ms());
    push_text("cfg,%s", cfg);
}

static void end_run(int64_t t_now, const char *reason) {
    taskENTER_CRITICAL(&s_mux);
    s_run_active = false;
    uint32_t enc_reads = s_enc_reads;
    taskEXIT_CRITICAL(&s_mux);
    imu_drops_t d;
    imu_get_drops(&d);
    push_text("run_end,last_data_seq=%lu,log_qdrop=%lu,log_send_err=%lu,timestamp_us=%lld,reason=%s,"
              "elapsed_us=%lld,log_qdrop_run=%lu,ev_qdrop_run=%lu,enc_ovf_run=%lu,imu_drop_run=%lu/%lu/%lu,"
              "gyro_gap_max_us=%lld",
              (unsigned long)logger_seq(), (unsigned long)logger_qdrop(),
              (unsigned long)logger_send_err(), (long long)t_now, reason,
              (long long)(t_now - s_t_start), (unsigned long)(logger_qdrop() - s_run0_log_qdrop),
              (unsigned long)(s_ev_qdrop - s_run0_ev_qdrop), (unsigned long)(s_enc_ovf - s_run0_enc_ovf),
              (unsigned long)(d.rx_queue - s_run0_imu.rx_queue),
              (unsigned long)(d.cb_queue - s_run0_imu.cb_queue),
              (unsigned long)(d.cb_mutex - s_run0_imu.cb_mutex), (long long)s_gyro_gap_max);
    // S37: own short line (#run_end is already close to the 256-character text buffer)
    // S41: enc_reads_run = số lần đọc count của lượt (dòng E Pi phải nhận = enc_reads_run − enc_ovf_run)
    push_text("run_stats,log_send_err_run=%lu,enc_reads_run=%lu",
              (unsigned long)(logger_send_err() - s_run0_log_send_err), (unsigned long)enc_reads);
}

// ─── Lịch PWM ────────────────────────────────────────────────────────────────
// T6 [ĐỀ XUẤT CODE 29/10]: mỗi mức giữ S06_T6_HOLD_US, 4 bánh cùng mức, chiều + rồi chiều −
// (một lần đổi chiều mỗi bánh), mức 0 ở mỗi chiều; một mức 4 bánh khác nhau (W15).
static const int16_t T6_STEPS[][4] = {
    {0, 0, 0, 0},
    {30, 30, 30, 30}, {50, 50, 50, 50}, {100, 100, 100, 100}, {300, 300, 300, 300},
    {330, 330, 330, 330}, {500, 500, 500, 500}, {550, 550, 550, 550}, {700, 700, 700, 700},
    {770, 770, 770, 770}, {900, 900, 900, 900}, {950, 950, 950, 950}, {970, 970, 970, 970},
    {1000, 1000, 1000, 1000},
    {100, 330, 550, 770},
    {0, 0, 0, 0},
    {-30, -30, -30, -30}, {-50, -50, -50, -50}, {-100, -100, -100, -100}, {-300, -300, -300, -300},
    {-330, -330, -330, -330}, {-500, -500, -500, -500}, {-550, -550, -550, -550},
    {-700, -700, -700, -700}, {-770, -770, -770, -770}, {-900, -900, -900, -900},
    {-950, -950, -950, -950}, {-970, -970, -970, -970}, {-1000, -1000, -1000, -1000},
    {-100, -330, -550, -770},
    {0, 0, 0, 0},
};
#define T6_N ((int)(sizeof T6_STEPS / sizeof T6_STEPS[0]))

static void schedule(int64_t t_now, int out[4]) {
    for (int i = 0; i < 4; i++) out[i] = 0;
    if (!s_run_active) return;
    int64_t t = t_now - s_t_start;
    if (s_profile == PROF_HEAVY) {
        bool swap = (t / S06_HEAVY_SWAP_US) % 2;
        out[0] = out[1] = swap ? 30 : 500;      // FL, FR
        out[2] = out[3] = swap ? 500 : 30;      // RL, RR
    } else if (s_profile == PROF_T6) {
        int64_t k = t / S06_T6_HOLD_US;
        if (k < T6_N) for (int i = 0; i < 4; i++) out[i] = T6_STEPS[k][i];
    }
}

void s06_tick(int64_t t_now, int speed_out[4]) {
    bool req_start, req_stop, req_auto;
    int req_prof;
    int64_t req_dur;
    char reason[24];
    taskENTER_CRITICAL(&s_mux);
    req_start = s_req_start;  s_req_start = false;
    req_stop  = s_req_stop;   s_req_stop  = false;
    req_prof  = s_req_profile;
    req_dur   = s_req_dur_us;
    req_auto  = s_req_auto_end;
    memcpy(reason, s_req_reason, sizeof reason);
    taskEXIT_CRITICAL(&s_mux);

    if (req_start && !s_run_active) start_run(t_now, req_prof, req_dur, req_auto);
    if (s_run_active) {
        if (req_stop) end_run(t_now, reason);
        else if (s_auto_end && t_now - s_t_start >= s_dur_us) end_run(t_now, "duration");
        if (!s_run_active) s_force_apply = true;   // về 0 và ghi lại lần ghi compare đó
    }
    schedule(t_now, speed_out);
}

void s06_apply(const int speed[4], bool force) {
    force = force || s_force_apply;
    s_force_apply = false;
    for (int i = 0; i < 4; i++) {
        if (!force && speed[i] == s_last_speed[i]) continue;
        s06_ev_t e = { .kind = EV_PWM };
        motor_set_info((motor_id_t)i, speed[i], &e.u.pwm.m);
        e.u.pwm.wheel = (uint8_t)i;
        e.u.pwm.speed = (int16_t)speed[i];
        s_last_speed[i] = speed[i];
        push(&e);
    }
}

void s06_init(void) {
    s_q = xQueueCreate(S06_QLEN, sizeof(s06_ev_t));
    configASSERT(s_q);
    esp_wifi_set_ps(WIFI_PS_NONE);   // tắt tiết kiệm điện WiFi: luồng sự kiện ~100 kB/s [ĐỀ XUẤT]
    imu_set_event_hook(imu_hook);

    // Nhiệt độ chip cho T5 (Liam 30/10); lỗi thì chỉ báo, lượt đo vẫn chạy
    temperature_sensor_config_t tc = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&tc, &s_tsens) != ESP_OK || temperature_sensor_enable(s_tsens) != ESP_OK) {
        ESP_LOGW(TAG, "temperature sensor unavailable, S lines carry nan");
        s_tsens = NULL;
    }

    const esp_timer_create_args_t ta = { .callback = enc_timer_cb, .name = "s06_enc",
                                         .dispatch_method = ESP_TIMER_TASK };
    esp_timer_handle_t th;
    ESP_ERROR_CHECK(esp_timer_create(&ta, &th));
    ESP_ERROR_CHECK(esp_timer_start_periodic(th, S06_ENC_PERIOD_US));

    push_text("fw,sha=%s,dirty=%d,env=%s,imu_period_us=%lu,enc_period_us=%d,dest=%s:%d,cmd_port=%d",
              FW_GIT_SHA, FW_GIT_DIRTY, FW_ENV, (unsigned long)IMU_REPORT_PERIOD_US, S06_ENC_PERIOD_US,
              S06_DEST_IP, S06_EVT_PORT, S06_CMD_PORT);
    ESP_LOGI(TAG, "#fw sha=%s dirty=%d env=%s", FW_GIT_SHA, FW_GIT_DIRTY, FW_ENV);

    xTaskCreatePinnedToCore(tx_task,  "s06_tx",  6144, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(cmd_task, "s06_cmd", 4096, NULL, 2, NULL, 0);
}

#endif // S06_LOAD
