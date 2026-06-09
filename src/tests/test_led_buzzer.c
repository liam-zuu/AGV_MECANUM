#include "test_led_buzzer.h"
#include "led_buzzer/led_buzzer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TEST_LED_BUZZER";

void test_led_buzzer(void) {
    led_buzzer_init();
    ESP_LOGI(TAG, "LED Buzzer test start");

    while (1) {
        ESP_LOGI(TAG, "RED");
        led_set_color(COLOR_RED);
        buzzer_beep(1000, 200);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "GREEN");
        led_set_color(COLOR_GREEN);
        buzzer_beep(2000, 200);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "BLUE");
        led_set_color(COLOR_BLUE);
        buzzer_beep(3000, 200);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "YELLOW");
        led_set_color(COLOR_YELLOW);
        buzzer_beep(4000, 200);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "OFF");
        led_set_color(COLOR_OFF);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}