#include "ibus.h"
#include "esp_log.h"

static const char *TAG = "IBUS";

void ibus_init(void) {
    uart_config_t uart_cfg = {
        .baud_rate  = IBUS_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(IBUS_UART_NUM, 256, 0, 0, NULL, 0);
    uart_param_config(IBUS_UART_NUM, &uart_cfg);
    uart_set_pin(IBUS_UART_NUM, UART_PIN_NO_CHANGE, IBUS_GPIO_RX, -1, -1);

    ESP_LOGI(TAG, "IBus init done");
}

bool ibus_read(ibus_data_t *data) {
    uint8_t buf[IBUS_PACKET_LEN];
    data->valid = false;

    // Đọc đủ 32 byte
    int len = uart_read_bytes(IBUS_UART_NUM, buf, IBUS_PACKET_LEN, pdMS_TO_TICKS(10));
    if (len < IBUS_PACKET_LEN) return false;

    // Tìm header: 0x20 0x40
    int start = -1;
    for (int i = 0; i <= len - IBUS_PACKET_LEN; i++) {
        if (buf[i] == 0x20 && buf[i+1] == 0x40) {
            start = i;
            break;
        }
    }
    if (start < 0) return false;

    // Verify checksum
    uint16_t checksum = 0xFFFF;
    for (int i = start; i < start + IBUS_PACKET_LEN - 2; i++) {
        checksum -= buf[i];
    }
    uint16_t recv_chk = buf[start + 30] | (buf[start + 31] << 8);
    if (checksum != recv_chk) return false;

    // Parse channels
    for (int i = 0; i < IBUS_NUM_CHANNELS; i++) {
        int idx = start + 2 + i * 2;
        data->channel[i] = buf[idx] | (buf[idx + 1] << 8);
    }
    data->valid = true;
    return true;
}

int ibus_channel_normalized(ibus_data_t *data, int ch) {
    if (!data->valid || ch >= IBUS_NUM_CHANNELS) return 0;

    int val = (int)data->channel[ch] - IBUS_MID;
    // Clamp về -500 đến +500 rồi scale lên -1000 đến +1000
    if (val > 500)  val = 500;
    if (val < -500) val = -500;
    return val * 2;
}

// int ibus_channel_normalized(ibus_data_t *data, int ch) {
//     if (!data->valid || ch >= IBUS_NUM_CHANNELS) return 0;

//     int val = (int)data->channel[ch] - IBUS_MID;
    
//     // Deadband
//     #define IBUS_DEADBAND 30
//     if (val > -IBUS_DEADBAND && val < IBUS_DEADBAND) return 0;

//     // Clamp về -500 đến +500 rồi scale lên -1000 đến +1000
//     if (val > 500)  val = 500;
//     if (val < -500) val = -500;
//     return val * 2;
// }