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
//   IMU READY v3 period_us=<IMU_REPORT_PERIOD_US> t_us=<esp_timer at READY>
// then ONE CSV line PER REPORT received from the BNO085 (no change filter, no polling):
//   kind,t_us,n,v0,v1,v2,qdrop
//   kind  A = accelerometer (m/s^2), G = calibrated gyro (rad/s), R = game RV euler (deg)
//   t_us  host receive time (esp_timer); the library gives no usable chip timestamp
//   n     running count of that report since boot (imu.cpp accel_n / gyro_n / rv_n)
//   qdrop lines lost because the print queue was full (cumulative)
// imu.cpp hands each report to imu_report_hook in the library's callback task; the hook
// only copies it into a queue, this task formats and prints. Motors are disabled (EN = 0).
//
// v3 (29/10, spec S06 v3.4 Z18, L7-20, V2-10, for tools/g2_bno_check.py): READY carries t_us so
// the PC can cut a fixed window [t_READY; t_READY + D] on the ESP32 clock. When the PC sends
// "STOP\n" the hook stops taking reports and an end line goes through the SAME queue, after
// every report line:
//   END,t_us,accel_n,gyro_n,rv_n,qdrop,via
//   accel_n, gyro_n, rv_n  source counters of imu.cpp when the hook stopped
//   via   H = written by the hook in the library callback task, right after its own report, so
//           the counters equal the last n handed to the hook (no report between count and hook);
//         T = no report within IL_STOP_WAIT_MS after STOP (no data flowing), written by this
//           task; a report counted at the source in that instant could miss the queue.
// The PC passes a boot only if each kind arrived as 1, 2, ..., <its END counter>.
// v1 (21/10) printed only lines where a value changed, polled every 10 ms: that biased the
// noise statistics (repeated quantised values were dropped) and could miss 100 Hz reports.
// ─────────────────────────────────────────────────────────────────────────────

#define IL_UART  UART_NUM_0
#define IL_BAUD  921600
#define IL_QLEN  256
#define IL_STOP_WAIT_MS 500

typedef struct {
    uint8_t  kind;                        // 'A', 'G', 'R', or 'E' = end line
    uint8_t  via;                         // end line only: 'H' or 'T'
    int64_t  t_us;
    uint32_t n;
    float    v[3];
    uint32_t c[3];                        // end line only: accel_n, gyro_n, rv_n
    uint32_t qdrop;                       // end line only
} il_rec_t;

static QueueHandle_t     s_q;
static volatile uint32_t s_qdrop;
static volatile int      s_stop_req;      // set by this task on "STOP"
static int               s_stopped;       // 0 -> 1 once, by whoever writes the end line

static void out(const char *s) { uart_write_bytes(IL_UART, s, strlen(s)); }

// Exactly one caller (hook or this task) wins and writes the end line.
static bool claim_stop(void) {
    int expected = 0;
    return __atomic_compare_exchange_n(&s_stopped, &expected, 1, false,
                                       __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
}

static void send_end(uint8_t via) {
    imu_data_t d = {0};                   // mutex timeout -> counters 0 -> the PC fails the boot
    imu_get(&d);                          // copies the counters even while valid is false
    il_rec_t e = {0};
    e.kind = 'E';
    e.via  = via;
    e.t_us = esp_timer_get_time();
    e.c[0] = d.accel_n; e.c[1] = d.gyro_n; e.c[2] = d.rv_n;
    e.qdrop = s_qdrop;
    xQueueSend(s_q, &e, portMAX_DELAY);   // must not be lost: wait for room
}

static void hook(uint8_t rpt_id, int64_t t_us, uint32_t n, const float v[4]) {
    if (__atomic_load_n(&s_stopped, __ATOMIC_SEQ_CST)) return;
    il_rec_t r = {0};
    r.kind = (rpt_id == IMU_RPT_ACCEL) ? 'A' : (rpt_id == IMU_RPT_GYRO) ? 'G' : 'R';
    r.t_us = t_us;
    r.n    = n;
    r.v[0] = v[0]; r.v[1] = v[1]; r.v[2] = v[2];
    if (xQueueSend(s_q, &r, 0) != pdTRUE) s_qdrop++;
    if (s_stop_req && claim_stop()) send_end('H');
}

// Reads "STOP\n" from the PC without blocking.
static void poll_cmd(void) {
    static char buf[8];
    static int  len;
    uint8_t c;
    while (uart_read_bytes(IL_UART, &c, 1, 0) == 1) {
        if (c == '\n' || c == '\r') {
            buf[len] = '\0';
            if (strcmp(buf, "STOP") == 0) s_stop_req = 1;
            len = 0;
        } else if (len < (int)sizeof buf - 1) {
            buf[len++] = (char)c;
        } else {
            len = 0;                      // too long: not a command
        }
    }
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
    snprintf(line, sizeof line, "IMU READY v3 period_us=%lu t_us=%lld\n",
             (unsigned long)IMU_REPORT_PERIOD_US, (long long)esp_timer_get_time());
    out(line);

    il_rec_t r;
    int64_t t_stop_req = 0;
    while (1) {
        poll_cmd();
        if (s_stop_req && !t_stop_req) t_stop_req = esp_timer_get_time();
        if (t_stop_req && esp_timer_get_time() - t_stop_req > IL_STOP_WAIT_MS * 1000LL
            && claim_stop())
            send_end('T');
        if (xQueueReceive(s_q, &r, pdMS_TO_TICKS(10)) != pdTRUE) continue;
        int n;
        if (r.kind == 'E') {
            n = snprintf(line, sizeof line, "END,%lld,%lu,%lu,%lu,%lu,%c\n", (long long)r.t_us,
                         (unsigned long)r.c[0], (unsigned long)r.c[1], (unsigned long)r.c[2],
                         (unsigned long)r.qdrop, r.via);
        } else {
            const char *fmt = (r.kind == 'R') ? "%c,%lld,%lu,%.3f,%.3f,%.3f,%lu\n"
                                              : "%c,%lld,%lu,%.6f,%.6f,%.6f,%lu\n";
            n = snprintf(line, sizeof line, fmt, r.kind, (long long)r.t_us,
                         (unsigned long)r.n, r.v[0], r.v[1], r.v[2], (unsigned long)s_qdrop);
        }
        uart_write_bytes(IL_UART, line, n);
    }
}
