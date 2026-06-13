#include "ultrasonic.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ULTRASONIC";

static const int echo_pins[ULTRASONIC_COUNT] = {
    ULTRASONIC_ECHO_2,
    ULTRASONIC_ECHO_0,
    ULTRASONIC_ECHO_1,
    ULTRASONIC_ECHO_3,
};

void ultrasonic_init(void) {
    // TRIG pin
    gpio_config_t trig_cfg = {
        .pin_bit_mask = (1ULL << ULTRASONIC_TRIG),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig_cfg);
    gpio_set_level(ULTRASONIC_TRIG, 0);

    // ECHO pins
    uint64_t echo_mask = 0;
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        echo_mask |= (1ULL << echo_pins[i]);
    }
    gpio_config_t echo_cfg = {
        .pin_bit_mask = echo_mask,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo_cfg);

    ESP_LOGI(TAG, "Ultrasonic init done");
}

void ultrasonic_read_all(us_data_t *data) {
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        // Trigger
        gpio_set_level(ULTRASONIC_TRIG, 1);
        esp_rom_delay_us(20);
        gpio_set_level(ULTRASONIC_TRIG, 0);

        // Chờ ECHO[i] lên
        int64_t t0 = esp_timer_get_time();
        while (!gpio_get_level(echo_pins[i])) {
            if (esp_timer_get_time() - t0 > 5000) {
                data->distance_cm[i] = -1.0f;
                data->valid[i]       = false;
                goto next;
            }
        }

        // Đo pulse ECHO[i]
        {
            int64_t start = esp_timer_get_time();
            while (gpio_get_level(echo_pins[i])) {
                if (esp_timer_get_time() - start > ULTRASONIC_TIMEOUT_US) {
                    data->distance_cm[i] = -1.0f;
                    data->valid[i]       = false;
                    goto next;
                }
            }
            int64_t end = esp_timer_get_time();

            float d = (float)(end - start) * 0.0343f / 2.0f;
            if (d >= ULTRASONIC_MIN_CM && d <= ULTRASONIC_MAX_CM) {
                data->distance_cm[i] = d;
                data->valid[i]       = true;
            } else {
                data->distance_cm[i] = -1.0f;
                data->valid[i]       = false;
            }
        }

        next:
        vTaskDelay(pdMS_TO_TICKS(10)); // yield cho RTOS, không block core
    }
}

// void ultrasonic_read_all(us_data_t *data) {
//     for (int i = 0; i < ULTRASONIC_COUNT; i++) {

//         // Đợi sóng cũ tắt trước khi trigger mới
//         esp_rom_delay_us(10000); // 10ms — sóng 4m cần 23ms, 10ms đủ cho range thực tế <2m

//         // Trigger
//         gpio_set_level(ULTRASONIC_TRIG, 1);
//         esp_rom_delay_us(20);
//         gpio_set_level(ULTRASONIC_TRIG, 0);

//         // Chờ ECHO[i] lên
//         int64_t t0 = esp_timer_get_time();
//         while (!gpio_get_level(echo_pins[i])) {
//             if (esp_timer_get_time() - t0 > 5000) {
//                 data->distance_cm[i] = -1.0f;
//                 data->valid[i]       = false;
//                 goto next;
//             }
//         }

//         // Đo pulse ECHO[i]
//         {
//             int64_t start = esp_timer_get_time();
//             while (gpio_get_level(echo_pins[i])) {
//                 if (esp_timer_get_time() - start > ULTRASONIC_TIMEOUT_US) {
//                     data->distance_cm[i] = -1.0f;
//                     data->valid[i]       = false;
//                     goto next;
//                 }
//             }
//             int64_t end = esp_timer_get_time();

//             float d = (float)(end - start) * 0.0343f / 2.0f;
//             if (d >= ULTRASONIC_MIN_CM && d <= ULTRASONIC_MAX_CM) {
//                 data->distance_cm[i] = d;
//                 data->valid[i]       = true;
//             } else {
//                 data->distance_cm[i] = -1.0f;
//                 data->valid[i]       = false;
//             }
//         }

//         next:;
//     }
// }