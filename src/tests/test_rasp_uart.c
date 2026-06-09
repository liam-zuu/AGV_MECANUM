#include "test_rasp_uart.h"
#include "rasp_uart/rasp_uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TEST_RASP_UART";

void test_rasp_uart(void) {
    rasp_uart_init();
    ESP_LOGI(TAG, "RASP UART test start");

    rasp_setpoint_t sp;
    int counter = 0;
    while (1) {
        // Đọc setpoint từ RPi5
        if (rasp_uart_read_setpoint(&sp)) {
            ESP_LOGI(TAG, "Setpoint: vx=%.3f vy=%.3f wz=%.3f", sp.vx, sp.vy, sp.wz);
        }

        // Gửi status về RPi5 mỗi 500ms
        if (counter % 5 == 0) {
            rasp_uart_send_status(0.1f, 0.0f, 0.0f);
            ESP_LOGI(TAG, "Sent status");
        }
        counter++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}