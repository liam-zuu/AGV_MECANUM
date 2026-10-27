#include "logger.h"
#include "wifi/wifi.h"
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
        "slip_flag,slip_on_count,r,bias_gz\n";
    sendto(sock, header, strlen(header), 0,
           (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    ESP_LOGI(TAG, "Logger streaming → %s:%d", LOGGER_PC_IP, LOGGER_UDP_PORT);

    log_data_t data;
    char buf[768];

    while (1) {
        // Block cho đến khi có data — không busy-wait
        if (xQueueReceive(log_queue, &data, portMAX_DELAY) != pdTRUE) continue;
        if (sock < 0) continue;

        if (data.kind == LOG_KIND_RUN_END) {
            // Dòng tổng kết cuối lượt (bắt đầu bằng '#': PC tách khỏi CSV)
            int n = snprintf(buf, sizeof(buf), "#run_end,seq=%lu,last_data_seq=%lu,log_qdrop=%lu,log_send_err=%lu\n",
                             (unsigned long)data.log_seq, (unsigned long)data.tick,
                             (unsigned long)data.log_qdrop, (unsigned long)s_log_send_err);
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
            "%u,%lu,%.4f,%.5f\n",
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
            (unsigned)data.slip_flag, (unsigned long)data.slip_on_count, data.r, data.bias_gz
        );

        if (len > 0 && len < (int)sizeof(buf)) {
            if (sendto(sock, buf, len, 0,
                       (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0)
                s_log_send_err++;
        }
    }
}
