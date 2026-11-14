#include "logger.h"
#include "../fw_id_gen.h"      // FW_GIT_SHA, FW_GIT_DIRTY, FW_ENV: scripts/build_id.py sinh lúc build
#include "wifi/wifi.h"
#include "gain_set.h"
#include "gain_rx/gain_rx.h"   // gain_rx_boot_id(): trường boot của dòng #gain
#include "esp_log.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdio.h>
#include <fcntl.h>

static const char *TAG = "LOGGER";

static int              sock = -1;
static struct sockaddr_in dest_addr;
static QueueHandle_t    log_queue;

// Mất log khép kín (spec controller–IMU v2.2 §8): log_seq tăng mỗi lần push (kể cả khi hàng đợi
// đầy, nên PC thấy chỗ hổng), log_qdrop đếm dòng không vào được hàng đợi, log_send_err đếm sendto lỗi.
static uint32_t          s_log_seq;
static uint32_t          s_log_qdrop;
static volatile uint32_t s_log_send_err;

// ─────────────────────────────────────────
// logger_init — chỉ tạo queue, KHÔNG tạo socket
// Socket tạo trong logger_task sau khi WiFi connect
// ─────────────────────────────────────────
void logger_init(void) {
    log_queue = xQueueCreate(LOGGER_QUEUE_SIZE, sizeof(log_data_t));
    if (log_queue == NULL) {
        ESP_LOGE(TAG, "Queue create failed");
        return;
    }

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(LOGGER_UDP_PORT);
    inet_pton(AF_INET, LOGGER_PC_IP, &dest_addr.sin_addr);

    ESP_LOGI(TAG, "Logger init done → %s:%d", LOGGER_PC_IP, LOGGER_UDP_PORT);
}

// ─────────────────────────────────────────
// logger_push — non-blocking, drop nếu queue đầy
// ─────────────────────────────────────────
void logger_push(const log_data_t *data) {
    if (log_queue == NULL || data == NULL) return;
    log_data_t d = *data;
    d.log_seq   = ++s_log_seq;
    d.log_qdrop = s_log_qdrop;
    if (xQueueSend(log_queue, &d, 0) != pdTRUE) s_log_qdrop++;
}

// Dòng #fw (LOG_KIND_FW): không lấy log_seq, không đếm vào log_qdrop, để số thứ tự dòng dữ liệu và
// phép kiểm mất log phía PC (tools/agv_logger.py) không đổi; hàng đợi đầy thì dòng này mất, không đếm
void logger_push_fw(void) {
    if (log_queue == NULL) return;
    log_data_t d = { .kind = LOG_KIND_FW };
    (void)xQueueSend(log_queue, &d, 0);
}

// Dòng #gain (L90): mỗi phần tử hàng đợi mang bản sao bộ gain chụp lúc gọi (gọi trong control_task,
// cùng task áp gain), nên dòng còn trong hàng đợi không bị đổi sang bộ áp sau (Codex L90-2).
// first_seq gắn dòng vào lượt (L90-4); line_drop đếm dòng #gain không vào được hàng đợi (tích lũy).
static volatile uint32_t s_gain_line_drop;

void logger_push_gain(uint32_t first_seq) {
    if (log_queue == NULL) return;
    log_data_t d = { .kind = LOG_KIND_GAIN, .gain_first_seq = first_seq };
    gain_set_current(&d.gain);
    d.gain_applied_n = gain_set_applied_n();
    if (xQueueSend(log_queue, &d, 0) != pdTRUE) s_gain_line_drop++;
}

// Sự kiện #us / #us_end (L95): bản sao trạng thái lượt chụp lúc gọi (trong control_task); không lấy
// log_seq (như #fw, #gain). Một phần tử hàng đợi mỗi sự kiện, logger_task phát 3 bản (Codex L95-3: không
// chiếm 3 chỗ của dòng dữ liệu). s_us_drop đếm sự kiện không vào được hàng đợi; chỉ control_task đọc/ghi,
// giá trị chụp vào phần tử lúc đưa vào (Codex L95-4: không đọc lúc định dạng ở lõi khác).
static uint32_t s_us_drop;

void logger_push_us(int ev, const us_run_t *r, uint32_t ev_seq) {
    if (log_queue == NULL || r == NULL) return;
    // L95-R2-1: chừa chỗ cho dòng dữ liệu / #run_end đi sau (us_state.h mục (d)). Mọi phần tử của hàng đợi
    // đều do control_task đưa vào, logger_task chỉ lấy ra, nên chỗ trống đọc ở đây không giảm trước lần
    // đưa vào kế tiếp của chính task này.
    int need = us_queue_need(ev);
    if (need <= 0 || (int)uxQueueSpacesAvailable(log_queue) < need) { s_us_drop++; return; }
    log_data_t d = { .kind = LOG_KIND_US, .us = *r, .us_ev = (uint8_t)ev, .us_seq = ev_seq, .us_drop = s_us_drop };
    if (xQueueSend(log_queue, &d, 0) != pdTRUE) s_us_drop++;
}

uint32_t logger_seq_next(void) { return s_log_seq + 1; }

uint32_t logger_seq(void)      { return s_log_seq; }
uint32_t logger_qdrop(void)    { return s_log_qdrop; }
uint32_t logger_send_err(void) { return s_log_send_err; }

// ─────────────────────────────────────────
// Tạo UDP socket non-blocking
// ─────────────────────────────────────────
static bool create_socket(void) {
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket create failed");
        return false;
    }
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    ESP_LOGI(TAG, "UDP socket created");
    return true;
}

// ─────────────────────────────────────────
// logger_task — đợi WiFi, tạo socket, drain queue
// ─────────────────────────────────────────
void logger_task(void *pv) {
    // Đợi WiFi connect
    int wait_count = 0;
    while (!wifi_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (++wait_count % 10 == 0) {
            ESP_LOGW(TAG, "Waiting for WiFi...");
        }
    }

    if (!create_socket()) {
        ESP_LOGE(TAG, "Logger task exit — no socket");
        vTaskDelete(NULL);
        return;
    }

    // Gửi CSV header 1 lần để PC biết column order
    const char *header =
        "timestamp_us,"
        "fl_count,fr_count,rl_count,rr_count,"
        "fl_rpm,fr_rpm,rl_rpm,rr_rpm,"
        "vx,vy,wz,"
        "sp_vx,sp_vy,sp_wz,"
        "pwm_fl,pwm_fr,pwm_rl,pwm_rr,"
        "ax,ay,az,"
        "gx,gy,gz,"
        "yaw,pitch,roll,"
        "tick,dpwm_fl,dpwm_fr,dpwm_rl,dpwm_rr,sat_fl,sat_fr,sat_rl,sat_rr,"
        "log_seq,log_qdrop,log_send_err,motor_on,imu_init,"
        "wz_meas,wz_cmd,gz_fused,yaw_rad,age_g_ms,age_r_ms,gyro_n,rv_n,reset_n,"
        "g_valid,r_valid,wz_src,src_switch_count,psi_ref,e_psi,head_on,psi_reset_count,"
        "slip_flag,slip_on_count,r,bias_gz,"
        "src_switch_dwz,bias_updates,slip_eval,"
        "pos_on,pos_x,pos_y,pos_th,pos_x_ref,pos_y_ref,pos_th_ref,pos_corr_vx,pos_corr_vy,"
        "pos_th_src,pos_in_pos,still,run_active,pos_reset_n,"
        "ws_res,ws_th,ws_flag,ws_on_count,ws_eval,traj_age_ms\n";
    sendto(sock, header, strlen(header), 0,
           (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    ESP_LOGI(TAG, "Logger streaming → %s:%d", LOGGER_PC_IP, LOGGER_UDP_PORT);

    log_data_t data;
    // 04/11: 768 → 1024 vì thêm cột pos_*, ws_*. Ước tính dòng dài nhất khoảng 820 byte (80 cột) khi
    // mọi số thực |x| < 1e5 (miền vật lý), không phải biên kiểu C (Codex P14); dòng quá dài bị bỏ và
    // tăng log_send_err. static để không tăng stack 4096 của task (chỉ một logger_task).
    static char buf[1024];

    while (1) {
        // Block cho đến khi có data — không busy-wait
        if (xQueueReceive(log_queue, &data, portMAX_DELAY) != pdTRUE) continue;
        if (sock < 0) continue;

        if (data.kind == LOG_KIND_FW) {
            // Nhận dạng firmware (Liam 09/11): '#' nên PC tách khỏi CSV, vào file events
            int n = snprintf(buf, sizeof(buf), "#fw,sha=%s,dirty=%d,env=%s\n", FW_GIT_SHA, FW_GIT_DIRTY, FW_ENV);
            if (sendto(sock, buf, n, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0)
                s_log_send_err++;
            continue;
        }
        if (data.kind == LOG_KIND_GAIN) {
            int n = snprintf(buf, sizeof(buf), "#gain");
            int m = gain_set_format(buf + n, sizeof(buf) - (size_t)n, &data.gain, data.gain_applied_n);
            if (m < 0) { s_log_send_err++; continue; }
            n += m;
            // boot (Liam 14/11 "Thêm boot vào #gain", R134C3-1): cùng s_boot của #us/#us_end và #gain_ack,
            // đặt một lần ở gain_rx_init() trước mọi logger_push_gain; trường cuối để trường cũ giữ chỗ
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, ",first_seq=%lu,line_drop=%lu,boot=%08lx\n",
                          (unsigned long)data.gain_first_seq, (unsigned long)s_gain_line_drop,
                          (unsigned long)gain_rx_boot_id());
            if (n >= (int)sizeof(buf)) { s_log_send_err++; continue; }
            if (sendto(sock, buf, n, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0)
                s_log_send_err++;
            continue;
        }
        if (data.kind == LOG_KIND_US) {
            int n = us_format_line(buf, sizeof(buf), data.us_ev, &data.us, data.us_seq, data.us_drop);
            if (n < 0 || n >= (int)sizeof(buf)) { s_log_send_err++; continue; }
            for (int k = 0; k < 3; k++)            // 3 bản giống hệt nhau, phòng mất gói UDP (như #run_end)
                if (sendto(sock, buf, n, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0)
                    s_log_send_err++;
            continue;
        }
        if (data.kind == LOG_KIND_RUN_END) {
            // Dòng tổng kết cuối lượt (bắt đầu bằng '#': PC tách khỏi CSV)
            int n = snprintf(buf, sizeof(buf), "#run_end,seq=%lu,last_data_seq=%lu,log_qdrop=%lu,log_send_err=%lu,udp_drop=%lu,udp_take=%lu\n",
                             (unsigned long)data.log_seq, (unsigned long)data.tick,
                             (unsigned long)data.log_qdrop, (unsigned long)s_log_send_err,
                             (unsigned long)data.udp_drop, (unsigned long)data.udp_take);
            if (sendto(sock, buf, n, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0)
                s_log_send_err++;
            continue;
        }

        int len = snprintf(buf, sizeof(buf),
            "%lld,"
            "%ld,%ld,%ld,%ld,"
            "%.2f,%.2f,%.2f,%.2f,"
            "%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,"
            "%d,%d,%d,%d,"
            "%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,"
            "%.2f,%.2f,%.2f,"
            "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"
            "%lu,%lu,%lu,%u,%d,"
            "%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%lu,%lu,%lu,"
            "%u,%u,%u,%lu,%.4f,%.4f,%u,%lu,"
            "%u,%lu,%.4f,%.5f,"
            "%.4f,%lu,%u,"
            "%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
            "%u,%u,%u,%u,%lu,"
            "%.0f,%.1f,%u,%lu,%u,%ld\n",
            data.timestamp_us,
            data.fl_count, data.fr_count, data.rl_count, data.rr_count,
            data.fl_rpm,   data.fr_rpm,   data.rl_rpm,   data.rr_rpm,
            data.vx,  data.vy,  data.wz,
            data.sp_vx, data.sp_vy, data.sp_wz,
            data.pwm_fl, data.pwm_fr, data.pwm_rl, data.pwm_rr,
            data.ax, data.ay, data.az,
            data.gx, data.gy, data.gz,
            data.yaw, data.pitch, data.roll,
            (unsigned long)data.tick,
            (unsigned long)data.dpwm_fl, (unsigned long)data.dpwm_fr,
            (unsigned long)data.dpwm_rl, (unsigned long)data.dpwm_rr,
            (unsigned long)data.sat_fl,  (unsigned long)data.sat_fr,
            (unsigned long)data.sat_rl,  (unsigned long)data.sat_rr,
            (unsigned long)data.log_seq, (unsigned long)data.log_qdrop,
            (unsigned long)s_log_send_err, (unsigned)data.motor_on, (int)data.imu_init,
            data.wz_meas, data.wz_cmd, data.gz_fused, data.yaw_rad, data.age_g_ms, data.age_r_ms,
            (unsigned long)data.gyro_n, (unsigned long)data.rv_n, (unsigned long)data.reset_n,
            (unsigned)data.g_valid, (unsigned)data.r_valid, (unsigned)data.wz_src,
            (unsigned long)data.src_switch_count, data.psi_ref, data.e_psi,
            (unsigned)data.head_on, (unsigned long)data.psi_reset_count,
            (unsigned)data.slip_flag, (unsigned long)data.slip_on_count, data.r, data.bias_gz,
            data.src_switch_dwz, (unsigned long)data.bias_updates, (unsigned)data.slip_eval,
            (unsigned)data.pos_on, data.pos_x, data.pos_y, data.pos_th,
            data.pos_x_ref, data.pos_y_ref, data.pos_th_ref, data.pos_corr_vx, data.pos_corr_vy,
            (unsigned)data.pos_th_src, (unsigned)data.pos_in_pos, (unsigned)data.still,
            (unsigned)data.run_active, (unsigned long)data.pos_reset_n,
            data.ws_res, data.ws_th, (unsigned)data.ws_flag, (unsigned long)data.ws_on_count,
            (unsigned)data.ws_eval, (long)data.traj_age_ms
        );

        if (len > 0 && len < (int)sizeof(buf)) {
            if (sendto(sock, buf, len, 0,
                       (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0)
                s_log_send_err++;
        } else {
            s_log_send_err++;           // dòng quá dài hoặc lỗi định dạng: bỏ dòng, có đếm (Codex P14)
        }
    }
}
