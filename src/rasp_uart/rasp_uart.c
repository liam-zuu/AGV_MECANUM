#include "rasp_uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "RASP_UART";

void rasp_uart_init(void) {
    uart_config_t uart_cfg = {
        .baud_rate  = RASP_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(RASP_UART_NUM, RASP_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(RASP_UART_NUM, &uart_cfg);
    uart_set_pin(RASP_UART_NUM, RASP_GPIO_TX, RASP_GPIO_RX, -1, -1);

    ESP_LOGI(TAG, "RASP UART init done");
}

static uint8_t calc_checksum(uint8_t *data, int len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return sum;
}

bool rasp_uart_read_setpoint(rasp_setpoint_t *setpoint) {
    uint8_t buf[RASP_BUF_SIZE];
    setpoint->valid = false;

    int len = uart_read_bytes(RASP_UART_NUM, buf, RASP_BUF_SIZE, pdMS_TO_TICKS(5));
    if (len < 6) return false;

    // Tìm START byte
    int start = -1;
    for (int i = 0; i < len - 1; i++) {
        if (buf[i] == RASP_START_BYTE && buf[i+1] == RASP_MSG_SETPOINT) {
            start = i;
            break;
        }
    }
    if (start < 0) return false;

    uint8_t msg_len = buf[start + 2];
    if (start + 3 + msg_len + 1 > len) return false;

    // Verify checksum
    uint8_t chk = calc_checksum(&buf[start], 3 + msg_len);
    if (chk != buf[start + 3 + msg_len]) return false;

    // Parse vx, vy, wz (3 × float = 12 bytes)
    if (msg_len < 12) return false;
    memcpy(&setpoint->vx, &buf[start + 3],     4);
    memcpy(&setpoint->vy, &buf[start + 3 + 4], 4);
    memcpy(&setpoint->wz, &buf[start + 3 + 8], 4);
    setpoint->valid = true;
    return true;
}

void rasp_uart_send_status(float vx, float vy, float wz) {
    uint8_t buf[16];
    buf[0] = RASP_START_BYTE;
    buf[1] = RASP_MSG_STATUS;
    buf[2] = 12; // 3 floats
    memcpy(&buf[3],     &vx, 4);
    memcpy(&buf[3 + 4], &vy, 4);
    memcpy(&buf[3 + 8], &wz, 4);
    buf[15] = calc_checksum(buf, 15);

    uart_write_bytes(RASP_UART_NUM, buf, 16);
}