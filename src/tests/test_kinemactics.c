#include "test_kinematics.h"
#include "kinematics/kinematics.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TEST_KINEMATICS";

void test_kinematics(void) {
    ESP_LOGI(TAG, "Kinematics test start");

    while (1) {
        // Test tiến thẳng
        agv_velocity_t vel = {.vx = 0.5f, .vy = 0.0f, .wz = 0.0f};
        wheel_velocity_t w = kinematics_inverse(vel);
        ESP_LOGI(TAG, "Forward vx=0.5: FL=%.2f FR=%.2f RL=%.2f RR=%.2f",
            w.fl, w.fr, w.rl, w.rr);

        // Test strafe trái
        vel = (agv_velocity_t){.vx = 0.0f, .vy = 0.5f, .wz = 0.0f};
        w = kinematics_inverse(vel);
        ESP_LOGI(TAG, "Strafe vy=0.5:  FL=%.2f FR=%.2f RL=%.2f RR=%.2f",
            w.fl, w.fr, w.rl, w.rr);

        // Test quay
        vel = (agv_velocity_t){.vx = 0.0f, .vy = 0.0f, .wz = 1.0f};
        w = kinematics_inverse(vel);
        ESP_LOGI(TAG, "Rotate wz=1.0:  FL=%.2f FR=%.2f RL=%.2f RR=%.2f",
            w.fl, w.fr, w.rl, w.rr);

        ESP_LOGI(TAG, "─────────────────");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}