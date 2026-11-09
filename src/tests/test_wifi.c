#include "test_wifi.h"
#include "wifi/wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "TEST_WIFI";

// SSID/password: src/wifi/wifi_secrets.h (local, not in git); IP: the Pi, like logger.h
#include "logger/logger.h"
#define TEST_UDP_PORT   LOGGER_UDP_PORT
#define TEST_PC_IP      LOGGER_PC_IP

void test_wifi(void) {
    ESP_LOGI(TAG, "WiFi test start");
    wifi_init();   // non-blocking since fix/wifi-log: wait here for the connection

    for (int i = 0; i < 150 && !wifi_is_connected(); i++) vTaskDelay(pdMS_TO_TICKS(100));
    if (!wifi_is_connected()) {
        ESP_LOGE(TAG, "WiFi not connected after 15 s — check SSID/password (2.4 GHz)");
        return;
    }
    ESP_LOGI(TAG, "WiFi connected OK");

    // Tạo UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket create failed");
        return;
    }

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(TEST_UDP_PORT),
    };
    inet_pton(AF_INET, TEST_PC_IP, &dest.sin_addr);

    int counter = 0;
    while (1) {
        char msg[64];
        int len = snprintf(msg, sizeof(msg),
            "ESP32 hello #%d | uptime=%lldms\n",
            counter++,
            esp_timer_get_time() / 1000);

        int sent = sendto(sock, msg, len, 0,
            (struct sockaddr *)&dest, sizeof(dest));

        if (sent > 0) {
            ESP_LOGI(TAG, "Sent: %s", msg);
        } else {
            ESP_LOGE(TAG, "Send failed");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}