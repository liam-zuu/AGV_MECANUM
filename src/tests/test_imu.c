#include "test_imu.h"
#include "imu/imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TEST_IMU";

void test_imu(void) {
    imu_init();
    ESP_LOGI(TAG, "IMU test start");

    imu_data_t data;
    while (1) {
        if (imu_read(&data)) {
            ESP_LOGI(TAG, "Accel: ax=%.3f ay=%.3f az=%.3f",  data.ax, data.ay, data.az);
            ESP_LOGI(TAG, "Gyro:  gx=%.3f gy=%.3f gz=%.3f",  data.gx, data.gy, data.gz);
            ESP_LOGI(TAG, "Euler: yaw=%.1f pitch=%.1f roll=%.1f", data.yaw, data.pitch, data.roll);
            ESP_LOGI(TAG, "─────────────────");
        } else {
            ESP_LOGW(TAG, "No IMU data");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}