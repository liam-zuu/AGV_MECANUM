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
    xQueueSend(log_queue, data, 0);
}

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
        "yaw,pitch,roll\n";
    sendto(sock, header, strlen(header), 0,
           (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    ESP_LOGI(TAG, "Logger streaming → %s:%d", LOGGER_PC_IP, LOGGER_UDP_PORT);

    log_data_t data;
    char buf[320];

    while (1) {
        // Block cho đến khi có data — không busy-wait
        if (xQueueReceive(log_queue, &data, portMAX_DELAY) != pdTRUE) continue;
        if (sock < 0) continue;

        int len = snprintf(buf, sizeof(buf),
            "%lld,"
            "%ld,%ld,%ld,%ld,"
            "%.2f,%.2f,%.2f,%.2f,"
            "%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,"
            "%d,%d,%d,%d,"
            "%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,"
            "%.2f,%.2f,%.2f\n",
            data.timestamp_us,
            data.fl_count, data.fr_count, data.rl_count, data.rr_count,
            data.fl_rpm,   data.fr_rpm,   data.rl_rpm,   data.rr_rpm,
            data.vx,  data.vy,  data.wz,
            data.sp_vx, data.sp_vy, data.sp_wz,
            data.pwm_fl, data.pwm_fr, data.pwm_rl, data.pwm_rr,
            data.ax, data.ay, data.az,
            data.gx, data.gy, data.gz,
            data.yaw, data.pitch, data.roll
        );

        if (len > 0 && len < (int)sizeof(buf)) {
            sendto(sock, buf, len, 0,
                   (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        }
    }
}
