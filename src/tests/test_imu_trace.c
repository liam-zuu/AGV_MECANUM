// src/ is compiled by glob in every env: this file only has content in the trace envs.
#ifdef TEST_IMU_TRACE

#include "test_imu_trace.h"
#include "imu/imu.h"
#include "motor/motor.h"
#include "BNO08xTrace.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
// BNO085 SPI trace (QĐ 26, G0 v1 §7; build -DTEST_MODE -DTEST_IMU_TRACE -DBNO_TRACE,
// envs imu_trace_20 / imu_trace_100). BNO085 nối ESP32 bằng SPI + INT + RST; env này ghi
// những gì thư viện làm trên SPI vào RAM, hết mỗi kịch bản mới in ra UART0 921600 (cổng
// serial USB về PC), nên lúc đo không có in ấn. Cùng env chạy trên xe (chip thật) và trên
// bàn HIL (H7 giả lập), rồi so bằng tools/bno_trace_capture.py --compare.
//
// Kịch bản tự chạy nối tiếp sau khi khởi động:
//   S1 (env 20 Hz) / S2 (env 100 Hz): khởi động + 3 report, TRACE_BOOT_S giây
//   S3  đổi chu kỳ khi đang chạy: 5 s chu kỳ env → 5 s TRACE_ALT_PERIOD_US → 5 s chu kỳ env
//   S4  xung RST giữa chừng (hard_reset), thư viện tự bật lại report, chạy thêm 8 s
//   S5  host trễ đọc: 50 lần đọc kế tiếp chờ 20 ms sau INT rồi mới hạ CS
// Dòng in: "TRACE READY v1 ...", mỗi kịch bản một khối "TRACE BEGIN" … "TRACE END" kèm
// "TRACE COUNTS" (số report thư viện giao lên), cuối cùng "TRACE DONE".
// Bộ nhớ: không có PSRAM; khi đầy, record mới bị bỏ và đếm ở lost= (in trong TRACE BEGIN).
// ─────────────────────────────────────────────────────────────────────────────

#ifndef TRACE_BOOT_S
#define TRACE_BOOT_S 60
#endif
#ifndef TRACE_ALT_PERIOD_US
#define TRACE_ALT_PERIOD_US ((IMU_REPORT_PERIOD_US) == 50000UL ? 10000UL : 50000UL)
#endif

#define TR_UART  UART_NUM_0
#define TR_BAUD  921600

enum { MK_PHASE = 1, MK_INIT_DONE, MK_PERIOD, MK_RESET, MK_HOST_DELAY, MK_END };

static void out(const char *s, size_t n) { uart_write_bytes(TR_UART, s, n); }

static void outs(const char *s) { out(s, strlen(s)); }

static void wait_s(int s) { vTaskDelay(pdMS_TO_TICKS(1000 * s)); }

static void counts_line(const char *phase) {
    imu_data_t d;
    bool ok = imu_get(&d);
    char line[128];
    snprintf(line, sizeof line, "TRACE COUNTS phase=%s valid=%d A=%lu G=%lu R=%lu\n", phase, ok,
             (unsigned long)d.accel_n, (unsigned long)d.gyro_n, (unsigned long)d.rv_n);
    outs(line);
}

static void end_phase(const char *phase) {
    bno_trace_mark(MK_END, 0);
    counts_line(phase);
    bno_trace_dump(out, phase);
    uart_wait_tx_done(TR_UART, pdMS_TO_TICKS(5000));
}

void test_imu_trace(void) {
    motor_init();                         /* EN = 0: drivers off during the test */
    esp_log_level_set("*", ESP_LOG_NONE);
    uart_driver_install(TR_UART, 256, 16384, 0, NULL, 0);
    uart_set_baudrate(TR_UART, TR_BAUD);

    size_t cap = bno_trace_begin();
    char line[160];
    snprintf(line, sizeof line, "TRACE READY v1 period_us=%lu alt_us=%lu boot_s=%d cap=%u\n",
             (unsigned long)IMU_REPORT_PERIOD_US, (unsigned long)TRACE_ALT_PERIOD_US, TRACE_BOOT_S,
             (unsigned)cap);
    outs(line);
    uart_wait_tx_done(TR_UART, pdMS_TO_TICKS(1000));

    const char *boot = (IMU_REPORT_PERIOD_US == 50000UL) ? "S1" : "S2";

    /* S1 / S2: boot sequence and steady reports */
    bno_trace_enable(true);
    bno_trace_mark(MK_PHASE, 1);
    imu_init();                           /* traced: reset, advertisement, product id, enables */
    bno_trace_mark(MK_INIT_DONE, 0);
    wait_s(TRACE_BOOT_S);
    end_phase(boot);

    /* S3: period change while running */
    bno_trace_mark(MK_PHASE, 3);
    wait_s(5);
    bno_trace_mark(MK_PERIOD, (uint16_t)(TRACE_ALT_PERIOD_US / 100));
    imu_trace_set_period(TRACE_ALT_PERIOD_US);
    wait_s(5);
    bno_trace_mark(MK_PERIOD, (uint16_t)(IMU_REPORT_PERIOD_US / 100));
    imu_trace_set_period(IMU_REPORT_PERIOD_US);
    wait_s(5);
    end_phase("S3");

    /* S4: RST pulse in the middle of a run */
    bno_trace_mark(MK_PHASE, 4);
    wait_s(2);
    bool rc = imu_trace_hard_reset();
    bno_trace_mark(MK_RESET, rc);
    wait_s(8);
    end_phase("S4");

    /* S5: host late to read (task held 20 ms after INT) */
    bno_trace_mark(MK_PHASE, 5);
    wait_s(2);
    bno_trace_mark(MK_HOST_DELAY, 20);
    bno_trace_set_host_delay(20, 50);
    wait_s(5);
    end_phase("S5");

    outs("TRACE DONE\n");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}

#endif // TEST_IMU_TRACE
