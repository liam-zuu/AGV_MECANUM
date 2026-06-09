#include "test_logger.h"
#include "logger/logger.h"
#include "wifi/wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TEST_LOGGER";

void test_logger(void) {
    ESP_LOGI(TAG, "Logger test start");
    wifi_init();
    logger_init();

    xTaskCreatePinnedToCore(logger_task, "logger", 4096, NULL, 1, NULL, 0);

    int counter = 0;
    while (1) {
        log_data_t log = {
            .timestamp_us = esp_timer_get_time(),
            .fl_count = counter,
            .fr_count = counter,
            .rl_count = counter,
            .rr_count = counter,
            .fl_rpm = 10.0f, .fr_rpm = 10.0f,
            .rl_rpm = 10.0f, .rr_rpm = 10.0f,
            .vx = 0.1f, .vy = 0.0f, .wz = 0.0f,
            .sp_vx = 0.1f, .sp_vy = 0.0f, .sp_wz = 0.0f,
        };
        logger_push(&log);
        ESP_LOGI(TAG, "Pushed log #%d", counter++);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}