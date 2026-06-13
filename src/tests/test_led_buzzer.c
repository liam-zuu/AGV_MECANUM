#include "test_led_buzzer.h"
#include "led_buzzer/led_buzzer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TEST_LED_BUZZER";

// void test_led_buzzer(void) {
//     led_buzzer_init();
//     ESP_LOGI(TAG, "LED Buzzer test start");

//     while (1) {
//         ESP_LOGI(TAG, "RED");
//         led_set_color(COLOR_RED);
//         buzzer_beep(1000, 200);
//         vTaskDelay(pdMS_TO_TICKS(1000));

//         ESP_LOGI(TAG, "GREEN");
//         led_set_color(COLOR_GREEN);
//         buzzer_beep(2000, 200);
//         vTaskDelay(pdMS_TO_TICKS(1000));

//         ESP_LOGI(TAG, "BLUE");
//         led_set_color(COLOR_BLUE);
//         buzzer_beep(3000, 200);
//         vTaskDelay(pdMS_TO_TICKS(1000));

//         ESP_LOGI(TAG, "YELLOW");
//         led_set_color(COLOR_YELLOW);
//         buzzer_beep(4000, 200);
//         vTaskDelay(pdMS_TO_TICKS(1000));

//         ESP_LOGI(TAG, "OFF");
//         led_set_color(COLOR_OFF);
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

void test_led_buzzer(void) {
    led_buzzer_init();
    ESP_LOGI(TAG, "LED Buzzer test start");

    // Happy Birthday
    typedef struct { uint32_t freq; uint32_t dur; } note_t;
    note_t song[] = {
        {264, 200}, {264, 200}, {297, 400}, {264, 400}, {352, 400}, {330, 800},
        {264, 200}, {264, 200}, {297, 400}, {264, 400}, {396, 400}, {352, 800},
        {264, 200}, {264, 200}, {528, 400}, {440, 400}, {352, 400}, {330, 400}, {297, 800},
        {470, 200}, {470, 200}, {440, 400}, {352, 400}, {396, 400}, {352, 800},
    };
    int n = sizeof(song) / sizeof(song[0]);

    while (1) {
        led_set_color(COLOR_GREEN);
        for (int i = 0; i < n; i++) {
            buzzer_beep(song[i].freq, song[i].dur);
            vTaskDelay(pdMS_TO_TICKS(50)); // khoảng lặng giữa nốt
        }
        led_set_color(COLOR_OFF);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}