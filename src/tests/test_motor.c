#include "test_motor.h"
#include "motor/motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TEST_MOTOR";

void test_motor(void) {
    motor_init();
    motor_enable(true);
    ESP_LOGI(TAG, "Motor test start");

    while (1) {
        ESP_LOGI(TAG, "All forward 50%%");
        motor_set(MOTOR_FL, 500);
        motor_set(MOTOR_FR, 500);
        motor_set(MOTOR_RL, 500);
        motor_set(MOTOR_RR, 500);
        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI(TAG, "Stop");
        motor_stop_all();
        vTaskDelay(pdMS_TO_TICKS(1000));

        // ESP_LOGI(TAG, "All reverse 50%%");
        // motor_set(MOTOR_FL, -500);
        // motor_set(MOTOR_FR, -500);
        // motor_set(MOTOR_RL, -500);
        // motor_set(MOTOR_RR, -500);
        // vTaskDelay(pdMS_TO_TICKS(5000));

        // ESP_LOGI(TAG, "Stop");
        // motor_stop_all();
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}