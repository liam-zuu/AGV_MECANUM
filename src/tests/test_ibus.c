#include "test_ibus.h"
#include "ibus/ibus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TEST_IBUS";

void test_ibus(void) {
    ibus_init();
    ESP_LOGI(TAG, "IBus test start — move sticks");

    ibus_data_t data;
    while (1) {
        if (ibus_read(&data)) {
            ESP_LOGI(TAG, "CH1:%d CH2:%d CH3:%d CH4:%d CH5:%d CH6:%d",
                data.channel[0], data.channel[1], data.channel[2],
                data.channel[3], data.channel[4], data.channel[5]);
        } else {
            ESP_LOGW(TAG, "No IBus data");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}