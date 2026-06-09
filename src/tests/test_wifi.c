#include "test_wifi.h"
#include "wifi/wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "TEST_WIFI";

// TODO: điền IP laptop và SSID/password trong wifi.c
#define TEST_UDP_PORT   5005
#define TEST_PC_IP      "192.168.10.114"  // TODO: đổi theo IP laptop

void test_wifi(void) {
    ESP_LOGI(TAG, "WiFi test start");
    wifi_init();

    if (!wifi_is_connected()) {
        ESP_LOGE(TAG, "WiFi connect failed — check SSID/password");
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