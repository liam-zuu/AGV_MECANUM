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

#ifdef US_DIAG
// ─────────────────────────────────────────
// Bản chẩn đoán (CODE_US, us_diag.h): CÙNG trình tự phát TRIG, chờ, đo, ngưỡng, timeout và
// vTaskDelay như bản thường ở nhánh #else (giữ nguyên 5bde4c5), chỉ thêm đọc esp_timer và gán
// vào s_diag. Mỗi lần đọc chân trong vòng chờ có thêm một phép trừ, so sánh (khoảng vài trăm ns),
// nên độ phân giải thời gian xấp xỉ bản thường. Chỉ ultrasonic_task gọi hàm này và ultrasonic_diag_get.
// ─────────────────────────────────────────
static us_diag_sample_t s_diag[ULTRASONIC_COUNT];
static int64_t          s_prev_trig;

void ultrasonic_diag_get(us_diag_sample_t out[ULTRASONIC_COUNT]) {
    for (int i = 0; i < ULTRASONIC_COUNT; i++) out[i] = s_diag[i];
}

void ultrasonic_read_all(us_data_t *data) {
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        us_diag_sample_t *dg = &s_diag[i];
        dg->pre    = (uint8_t)(gpio_get_level(echo_pins[i]) ? 1 : 0);
        dg->rise_us = 0;
        dg->w_us    = 0;
        dg->gap_us  = 0;
        int64_t t_trig = esp_timer_get_time();
        dg->t_trig  = t_trig;
        dg->dtp_us  = s_prev_trig > 0 ? (uint32_t)(t_trig - s_prev_trig) : 0u;
        s_prev_trig = t_trig;
        int64_t last = t_trig;

        // Trigger
        gpio_set_level(ULTRASONIC_TRIG, 1);
        esp_rom_delay_us(20);
        gpio_set_level(ULTRASONIC_TRIG, 0);

        // Chờ ECHO[i] lên
        int64_t t0 = esp_timer_get_time();
        while (!gpio_get_level(echo_pins[i])) {
            int64_t now = esp_timer_get_time();
            if (now - last > (int64_t)dg->gap_us) dg->gap_us = (uint32_t)(now - last);
            last = now;
            if (now - t0 > 5000) {
                data->distance_cm[i] = -1.0f;
                data->valid[i]       = false;
                dg->res = US_DIAG_RES_NORISE;
                goto next;
            }
        }

        // Đo pulse ECHO[i]
        {
            int64_t start = esp_timer_get_time();
            if (start - last > (int64_t)dg->gap_us) dg->gap_us = (uint32_t)(start - last);
            last = start;
            dg->rise_us = (uint32_t)(start - t_trig);
            while (gpio_get_level(echo_pins[i])) {
                int64_t now = esp_timer_get_time();
                if (now - last > (int64_t)dg->gap_us) dg->gap_us = (uint32_t)(now - last);
                last = now;
                if (now - start > ULTRASONIC_TIMEOUT_US) {
                    data->distance_cm[i] = -1.0f;
                    data->valid[i]       = false;
                    dg->w_us = (uint32_t)(now - start);
                    dg->res  = US_DIAG_RES_TMO;
                    goto next;
                }
            }
            int64_t end = esp_timer_get_time();
            if (end - last > (int64_t)dg->gap_us) dg->gap_us = (uint32_t)(end - last);
            dg->w_us = (uint32_t)(end - start);

            float d = (float)(end - start) * 0.0343f / 2.0f;
            if (d >= ULTRASONIC_MIN_CM && d <= ULTRASONIC_MAX_CM) {
                data->distance_cm[i] = d;
                data->valid[i]       = true;
                dg->res = US_DIAG_RES_OK;
            } else {
                data->distance_cm[i] = -1.0f;
                data->valid[i]       = false;
                dg->res = US_DIAG_RES_RNG;
            }
        }

        next:
        vTaskDelay(pdMS_TO_TICKS(10)); // yield cho RTOS, không block core
    }
}
#else
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
#endif // US_DIAG

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