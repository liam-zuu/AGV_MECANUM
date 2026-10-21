#include "test_imu_log.h"
#include "imu/imu.h"
#include "motor/motor.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
// IMU raw log (build with -DTEST_MODE -DTEST_IMU_LOG, env imu_log).
//
// UART0 at 921600 baud, ESP log output off. After boot the ESP32 prints
//   IMU READY v1
// then, polling imu_get() every FreeRTOS tick (10 ms), one CSV line whenever any field changed:
//   t_us,ax,ay,az,gx,gy,gz,roll,pitch,yaw
// (m/s^2, rad/s, degrees). imu.cpp enables accelerometer, calibrated gyro and
// game rotation vector at 50 ms (20 Hz) each; the reports arrive separately, so
// each line holds the latest value of every field (sample-and-hold) and the
// analysis keeps only the lines where the channel of interest changed.
// imu.cpp and the main firmware are not modified. Motors are disabled (EN = 0).
// ─────────────────────────────────────────────────────────────────────────────

#define IL_UART  UART_NUM_0
#define IL_BAUD  921600

static void out(const char *s) { uart_write_bytes(IL_UART, s, strlen(s)); }

void test_imu_log(void) {
    motor_init();                         /* EN = 0: drivers off during the test */
    esp_log_level_set("*", ESP_LOG_NONE);
    uart_driver_install(IL_UART, 256, 8192, 0, NULL, 0);
    uart_set_baudrate(IL_UART, IL_BAUD);

    imu_init();
    out("IMU READY v1\n");

    imu_data_t d, last;
    memset(&last, 0, sizeof last);
    char line[160];
    while (1) {
        if (imu_get(&d) && d.valid) {
            if (memcmp(&d, &last, sizeof d) != 0) {
                int n = snprintf(line, sizeof line,
                                 "%lld,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f\n",
                                 (long long)esp_timer_get_time(),
                                 d.accel_x, d.accel_y, d.accel_z,
                                 d.gyro_x, d.gyro_y, d.gyro_z,
                                 d.roll, d.pitch, d.yaw);
                uart_write_bytes(IL_UART, line, n);
                last = d;
            }
        }
        vTaskDelay(1);                     /* FreeRTOS tick (10 ms at 100 Hz) */
    }
}
