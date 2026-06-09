#include "ibus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "IBUS";

// ─── Globals ──────────────────────────────────────────────────────────────────
SemaphoreHandle_t g_ibus_mutex        = NULL;
ibus_data_t       g_ibus_data         = {0};
int               g_ibus_failsafe_count = 0;

// ─── Init ─────────────────────────────────────────────────────────────────────
void ibus_init(void) {
    g_ibus_mutex = xSemaphoreCreateMutex();
    configASSERT(g_ibus_mutex);

    uart_config_t uart_cfg = {
        .baud_rate  = IBUS_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Buffer 512 byte — đủ chứa ~15 packet IBus (32 byte/packet)
    ESP_ERROR_CHECK(uart_driver_install(IBUS_UART_NUM, 512, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(IBUS_UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(IBUS_UART_NUM,
                                 UART_PIN_NO_CHANGE, IBUS_GPIO_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "IBus init done — RX GPIO%d, UART%d", IBUS_GPIO_RX, IBUS_UART_NUM);
}

// ─── Internal: parse 1 packet từ buf[0..31] ───────────────────────────────────
// Giả định buf đã align đúng (buf[0]=0x20, buf[1]=0x40)
// Return true nếu checksum pass
static bool _parse_packet(const uint8_t *buf, ibus_data_t *out) {
    // Checksum = 0xFFFF - sum(byte[0..29])
    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < IBUS_PACKET_LEN - 2; i++) {
        checksum -= buf[i];
    }
    uint16_t recv_chk = (uint16_t)buf[30] | ((uint16_t)buf[31] << 8);
    if (checksum != recv_chk) {
        ESP_LOGW(TAG, "Checksum fail: calc=0x%04X recv=0x%04X", checksum, recv_chk);
        return false;
    }

    // Parse 10 channel, mỗi channel 2 byte little-endian, bắt đầu từ byte[2]
    for (int i = 0; i < IBUS_NUM_CHANNELS; i++) {
        int idx = 2 + i * 2;
        out->channel[i] = (uint16_t)buf[idx] | ((uint16_t)buf[idx + 1] << 8);
    }
    out->valid = true;
    return true;
}

// ─── Task: state machine đọc byte-by-byte ─────────────────────────────────────
//
// Tại sao state machine thay vì uart_read_bytes(32)?
//   IBus gửi liên tục 7ms/packet. Khi ESP32 khởi động hoặc bị reset,
//   UART buffer có thể chứa phần giữa của một packet → đọc 32 byte một lần
//   sẽ lấy dữ liệu lệch phase, header không bao giờ ở đúng byte[0].
//   State machine tìm header trước, sau đó mới lấy đủ body → không bao giờ
//   bị mất sync dù cắm giữa chừng hay bị nhiễu 1 byte.
//
// State:
//   WAIT_H0  — chờ byte 0x20
//   WAIT_H1  — chờ byte 0x40
//   READ_BODY — đọc 30 byte còn lại vào buf[2..31]
//
void ibus_task(void *pvParam) {
    uint8_t  buf[IBUS_PACKET_LEN];
    uint8_t  byte;
    int      body_idx;

    typedef enum { WAIT_H0, WAIT_H1, READ_BODY } state_t;
    state_t state = WAIT_H0;

    while (1) {
        // Đọc 1 byte, block tối đa 20ms
        // 20ms > 7ms (chu kỳ IBus) → luôn có dữ liệu nếu receiver đang hoạt động
        int n = uart_read_bytes(IBUS_UART_NUM, &byte, 1, pdMS_TO_TICKS(20));

        if (n <= 0) {
            // Không có byte nào trong 20ms → receiver mất tín hiệu
            if (xSemaphoreTake(g_ibus_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                g_ibus_data.valid = false;
                g_ibus_failsafe_count++;
                xSemaphoreGive(g_ibus_mutex);
            }
            if (g_ibus_failsafe_count >= IBUS_FAILSAFE_FRAMES) {
                ESP_LOGW(TAG, "FAILSAFE: %d frames invalid — no signal", g_ibus_failsafe_count);
                // TODO: gọi emergency stop ở đây nếu cần
                // motor_set_all(0);
            }
            state = WAIT_H0;  // reset state khi mất tín hiệu
            continue;
        }

        switch (state) {
            case WAIT_H0:
                if (byte == IBUS_HEADER_0) {
                    buf[0] = byte;
                    state  = WAIT_H1;
                }
                break;

            case WAIT_H1:
                if (byte == IBUS_HEADER_1) {
                    buf[1] = byte;
                    body_idx = 2;
                    state    = READ_BODY;
                } else if (byte == IBUS_HEADER_0) {
                    // 0x20 0x20 → byte đầu là rác, byte này mới là header thật
                    buf[0] = byte;
                    state  = WAIT_H1;
                } else {
                    state = WAIT_H0;
                }
                break;

            case READ_BODY:
                buf[body_idx++] = byte;
                if (body_idx == IBUS_PACKET_LEN) {
                    // Đủ 32 byte → parse
                    ibus_data_t tmp = {0};
                    bool ok = _parse_packet(buf, &tmp);

                    if (xSemaphoreTake(g_ibus_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        if (ok) {
                            g_ibus_data           = tmp;
                            g_ibus_failsafe_count = 0;
                        } else {
                            g_ibus_data.valid = false;
                            g_ibus_failsafe_count++;
                        }
                        xSemaphoreGive(g_ibus_mutex);
                    }
                    state = WAIT_H0;
                }
                break;
        }
    }
}

// ─── Thread-safe getter ───────────────────────────────────────────────────────
bool ibus_get(ibus_data_t *out) {
    if (xSemaphoreTake(g_ibus_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;
    *out = g_ibus_data;
    xSemaphoreGive(g_ibus_mutex);
    return out->valid;
}

// ─── Normalize channel ────────────────────────────────────────────────────────
int ibus_channel_normalized(const ibus_data_t *data, int ch) {
    if (!data->valid || ch < 0 || ch >= IBUS_NUM_CHANNELS) return 0;

    int val = (int)data->channel[ch] - IBUS_MID;

    // Deadband: tránh drift khi joystick ở giữa bị rơ
    // Set IBUS_DEADBAND = 0 trong .h để disable khi debug
    if (val > -IBUS_DEADBAND && val < IBUS_DEADBAND) return 0;

    // Clamp [-500, +500] rồi scale lên [-1000, +1000]
    if (val >  500) val =  500;
    if (val < -500) val = -500;
    return val * 2;
}