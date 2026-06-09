#include "test_encoder.h"
#include "encoder/encoder.h"
#include "motor/motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TEST_ENCODER";

void test_encoder(void) {
    encoder_init();
    motor_init();
    motor_enable(true);
    ESP_LOGI(TAG, "Encoder + Motor test start");

    while (1) {
        ESP_LOGI(TAG, "All forward 50%%");
        motor_set(MOTOR_FL, 800);
        motor_set(MOTOR_FR, 800);
        motor_set(MOTOR_RL, 800);
        motor_set(MOTOR_RR, 800);
        vTaskDelay(pdMS_TO_TICKS(3000));

        motor_stop_all();
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "FL: %ld cnt | %.2f RPM", encoder_get_count(ENC_FL), encoder_get_rpm(ENC_FL));
        ESP_LOGI(TAG, "FR: %ld cnt | %.2f RPM", encoder_get_count(ENC_FR), encoder_get_rpm(ENC_FR));
        ESP_LOGI(TAG, "RL: %ld cnt | %.2f RPM", encoder_get_count(ENC_RL), encoder_get_rpm(ENC_RL));
        ESP_LOGI(TAG, "RR: %ld cnt | %.2f RPM", encoder_get_count(ENC_RR), encoder_get_rpm(ENC_RR));
        ESP_LOGI(TAG, "─────────────────");

        encoder_clear(ENC_FL);
        encoder_clear(ENC_FR);
        encoder_clear(ENC_RL);
        encoder_clear(ENC_RR);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}