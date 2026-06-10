#include "ultrasonic.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "ULTRASONIC";

static const int echo_pins[ULTRASONIC_COUNT] = {
    ULTRASONIC_ECHO_0,
    ULTRASONIC_ECHO_1,
    ULTRASONIC_ECHO_2,
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
    int64_t start[ULTRASONIC_COUNT]   = {0};
    int64_t end[ULTRASONIC_COUNT]     = {0};
    bool    started[ULTRASONIC_COUNT] = {false};
    bool    done[ULTRASONIC_COUNT]    = {false};

    // Trigger chung 1 lần
    gpio_set_level(ULTRASONIC_TRIG, 1);
    esp_rom_delay_us(20);
    gpio_set_level(ULTRASONIC_TRIG, 0);

    // Đọc ECHO song song
    int64_t timeout = esp_timer_get_time() + ULTRASONIC_TIMEOUT_US;

    while (esp_timer_get_time() < timeout) {
        bool all_done = true;
        for (int i = 0; i < ULTRASONIC_COUNT; i++) {
            if (done[i]) continue;
            all_done = false;

            int level = gpio_get_level(echo_pins[i]);
            if (!started[i] && level) {
                start[i]   = esp_timer_get_time();
                started[i] = true;
            } else if (started[i] && !level) {
                end[i]  = esp_timer_get_time();
                done[i] = true;
            }
        }
        if (all_done) break;
    }

    // Tính distance
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        if (!done[i]) {
            data->distance_cm[i] = -1.0f;
            data->valid[i]       = false;
            continue;
        }
        float d = (float)(end[i] - start[i]) * 0.0343f / 2.0f;
        if (d < ULTRASONIC_MIN_CM || d > ULTRASONIC_MAX_CM) {
            data->distance_cm[i] = -1.0f;
            data->valid[i]       = false;
        } else {
            data->distance_cm[i] = d;
            data->valid[i]       = true;
        }
    }
}