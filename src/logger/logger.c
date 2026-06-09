#include "logger.h"
#include "wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "LOGGER";
static int sock = -1;
static struct sockaddr_in dest_addr;
static QueueHandle_t log_queue;

void logger_init(void) {
    log_queue = xQueueCreate(LOGGER_QUEUE_SIZE, sizeof(log_data_t));

    if (!wifi_is_connected()) {
        ESP_LOGW(TAG, "WiFi not connected, logger disabled");
        return;
    }

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket create failed");
        return;
    }

    // Non-blocking
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(LOGGER_UDP_PORT);
    inet_pton(AF_INET, LOGGER_PC_IP, &dest_addr.sin_addr);

    ESP_LOGI(TAG, "Logger init done → %s:%d", LOGGER_PC_IP, LOGGER_UDP_PORT);
}

void logger_push(const log_data_t *data) {
    if (log_queue == NULL) return;
    xQueueSend(log_queue, data, 0);  // Non-blocking, bỏ qua nếu queue đầy
}

void logger_task(void *pv) {
    log_data_t data;
    char buf[512];

    while (1) {
        if (xQueueReceive(log_queue, &data, portMAX_DELAY)) {
            if (sock < 0) continue;

            int len = snprintf(buf, sizeof(buf),
                "%lld,"
                "%ld,%ld,%ld,%ld,"
                "%.2f,%.2f,%.2f,%.2f,"
                "%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,"
                "%.2f,%.2f,%.2f\n",
                data.timestamp_us,
                data.fl_count, data.fr_count, data.rl_count, data.rr_count,
                data.fl_rpm,   data.fr_rpm,   data.fl_rpm,   data.rr_rpm,
                data.vx, data.vy, data.wz,
                data.sp_vx, data.sp_vy, data.sp_wz,
                data.ax, data.ay, data.az,
                data.gx, data.gy, data.gz,
                data.yaw, data.pitch, data.roll
            );

            sendto(sock, buf, len, 0,
                   (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        }
    }
}