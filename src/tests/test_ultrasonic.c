#include "test_ultrasonic.h"
#include "ultrasonic/ultrasonic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TEST_ULTRASONIC";

void test_ultrasonic(void) {
    ultrasonic_init();
    ESP_LOGI(TAG, "Ultrasonic test start");

    us_data_t data;
    while (1) {
        ultrasonic_read_all(&data);
        ESP_LOGI(TAG, "FRONT:%.1fcm BACK:%.1fcm LEFT:%.1fcm RIGHT:%.1fcm",
            data.valid[US_FRONT] ? data.distance_cm[US_FRONT] : -1,
            data.valid[US_BACK]  ? data.distance_cm[US_BACK]  : -1,
            data.valid[US_LEFT]  ? data.distance_cm[US_LEFT]  : -1,
            data.valid[US_RIGHT] ? data.distance_cm[US_RIGHT] : -1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}