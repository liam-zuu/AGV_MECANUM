#include "test_imu_log.h"
#include "imu/imu.h"
#include "motor/motor.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
// IMU raw log (build with -DTEST_MODE -DTEST_IMU_LOG; envs imu_log_20, imu_log_100).
//
// UART0 at 921600 baud, ESP log output off. After boot the ESP32 prints
//   IMU READY v2 period_us=<IMU_REPORT_PERIOD_US>
// then ONE CSV line PER REPORT received from the BNO085 (no change filter, no polling):
//   kind,t_us,n,v0,v1,v2,qdrop
//   kind  A = accelerometer (m/s^2), G = calibrated gyro (rad/s), R = game RV euler (deg)
//   t_us  host receive time (esp_timer); the library gives no usable chip timestamp
//   n     running count of that report since boot (host side)
//   qdrop lines lost because the print queue was full (cumulative)
// imu.cpp hands each report to imu_report_hook in the library's callback task; the hook
// only copies it into a queue, this task formats and prints. Motors are disabled (EN = 0).
// v1 (21/10) printed only lines where a value changed, polled every 10 ms: that biased the
// noise statistics (repeated quantised values were dropped) and could miss 100 Hz reports.
// ─────────────────────────────────────────────────────────────────────────────

#define IL_UART  UART_NUM_0
#define IL_BAUD  921600
#define IL_QLEN  256

typedef struct {
    uint8_t  kind;
    int64_t  t_us;
    uint32_t n;
    float    v[3];
} il_rec_t;

static QueueHandle_t     s_q;
static volatile uint32_t s_qdrop;

static void out(const char *s) { uart_write_bytes(IL_UART, s, strlen(s)); }

static void hook(uint8_t rpt_id, int64_t t_us, uint32_t n, const float v[4]) {
    il_rec_t r;
    r.kind = (rpt_id == IMU_RPT_ACCEL) ? 'A' : (rpt_id == IMU_RPT_GYRO) ? 'G' : 'R';
    r.t_us = t_us;
    r.n    = n;
    r.v[0] = v[0]; r.v[1] = v[1]; r.v[2] = v[2];
    if (xQueueSend(s_q, &r, 0) != pdTRUE) s_qdrop++;
}

void test_imu_log(void) {
    motor_init();                         /* EN = 0: drivers off during the test */
    esp_log_level_set("*", ESP_LOG_NONE);
    uart_driver_install(IL_UART, 256, 8192, 0, NULL, 0);
    uart_set_baudrate(IL_UART, IL_BAUD);

    s_q = xQueueCreate(IL_QLEN, sizeof(il_rec_t));
    configASSERT(s_q);
    imu_set_report_hook(hook);
    imu_init();

    char line[160];
    snprintf(line, sizeof line, "IMU READY v2 period_us=%lu\n", (unsigned long)IMU_REPORT_PERIOD_US);
    out(line);

    il_rec_t r;
    while (1) {
        if (xQueueReceive(s_q, &r, portMAX_DELAY) == pdTRUE) {
            const char *fmt = (r.kind == 'R') ? "%c,%lld,%lu,%.3f,%.3f,%.3f,%lu\n"
                                              : "%c,%lld,%lu,%.6f,%.6f,%.6f,%lu\n";
            int n = snprintf(line, sizeof line, fmt, r.kind, (long long)r.t_us,
                             (unsigned long)r.n, r.v[0], r.v[1], r.v[2], (unsigned long)s_qdrop);
            uart_write_bytes(IL_UART, line, n);
        }
    }
}
