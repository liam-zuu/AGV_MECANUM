#ifndef IBUS_H
#define IBUS_H

#include <stdbool.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ─── Hardware config ──────────────────────────────────────────────────────────
#define IBUS_UART_NUM           UART_NUM_1
#define IBUS_GPIO_RX            18
#define IBUS_BAUD               115200

// ─── Protocol ─────────────────────────────────────────────────────────────────
#define IBUS_PACKET_LEN         32
#define IBUS_NUM_CHANNELS       10
#define IBUS_HEADER_0           0x20
#define IBUS_HEADER_1           0x40

// ─── Channel range ────────────────────────────────────────────────────────────
#define IBUS_MIN                1000
#define IBUS_MID                1500
#define IBUS_MAX                2000

// ─── Tuning ───────────────────────────────────────────────────────────────────
// Deadband: joystick bị rơ sẽ nhiễu ±10–30 ở vị trí giữa
// Set = 0 để disable khi debug raw value
#define IBUS_DEADBAND           25

// Failsafe: số frame invalid liên tiếp trước khi gọi emergency stop
#define IBUS_FAILSAFE_FRAMES    10

// ─── Shared data (access qua ibus_get / mutex) ────────────────────────────────
typedef struct {
    uint16_t channel[IBUS_NUM_CHANNELS];
    bool     valid;
} ibus_data_t;

extern SemaphoreHandle_t g_ibus_mutex;
extern ibus_data_t       g_ibus_data;
extern int               g_ibus_failsafe_count;  // tăng mỗi frame invalid, reset về 0 khi valid

// ─── API ──────────────────────────────────────────────────────────────────────
void ibus_init(void);

// Task chính — gọi xTaskCreate với stack ≥ 2048
void ibus_task(void *pvParam);

// Thread-safe copy g_ibus_data ra ngoài
// Trả về false nếu mutex timeout hoặc data invalid
bool ibus_get(ibus_data_t *out);

// Map channel ch về [-1000, +1000], áp deadband
// Trả về 0 nếu data invalid hoặc ch out-of-range
int  ibus_channel_normalized(const ibus_data_t *data, int ch);

#endif // IBUS_H