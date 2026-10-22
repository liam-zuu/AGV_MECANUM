#include "test_enc_count.h"
#include "encoder/encoder.h"
#include "imu/imu.h"
#include "motor/motor.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// Raw encoder edge count (build with -DTEST_MODE -DTEST_ENC_COUNT, env enc_count).
// Purpose: independent check of the H7 EncoderGen frequency (bug H7-ENC-PI, 22/10).
// RPi5 sends constant wheel speeds (run_hil.py --fixed-omega ...); the ESP32 PCNT
// hardware counts every edge of A and B. Every 10 s window the ESP32 prints, per wheel:
//   counts in the window, edges/s, channel-A frequency (edges/s / 4) and the implied
//   wheel speed with the firmware PPR (1320 counts/rev).
// Compare channel-A frequency with |omega| / (2*pi) * 330 for the commanded omega:
// it does not depend on any H7 formula. Motors are disabled (EN = 0).
// IMU is initialised too (every firmware has the IMU, Outline V7.4 QD 23); its gyro
// report count is printed so a missing IMU is visible.
// ─────────────────────────────────────────────────────────────────────────────

#define EC_WINDOW_US 10000000LL

void test_enc_count(void) {
    motor_init();                         /* EN = 0: drivers off during the test */
    encoder_init();
    imu_init();
    printf("ENC COUNT READY v1 window_s=%.1f ppr=%d\n", EC_WINDOW_US / 1e6, PPR);

    static const char *name[ENC_COUNT] = {"FL", "FR", "RL", "RR"};
    int32_t c0[ENC_COUNT];
    for (int i = 0; i < ENC_COUNT; i++) c0[i] = encoder_get_count((enc_id_t)i);
    int64_t t0 = esp_timer_get_time();
    uint32_t win = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));
        int64_t t = esp_timer_get_time();
        if (t - t0 < EC_WINDOW_US) continue;

        int32_t c1[ENC_COUNT];
        for (int i = 0; i < ENC_COUNT; i++) c1[i] = encoder_get_count((enc_id_t)i);
        double dt = (t - t0) * 1e-6;
        imu_data_t d = {0};
        imu_get(&d);
        win++;
        for (int i = 0; i < ENC_COUNT; i++) {
            int32_t n = c1[i] - c0[i];
            double eps = n / dt;
            printf("win=%lu wheel=%s counts=%ld dt=%.4f edges_per_s=%.2f fA_hz=%.2f omega_fw=%.3f\n",
                   (unsigned long)win, name[i], (long)n, dt, eps, fabs(eps) / 4.0,
                   eps * 2.0 * M_PI / PPR);
        }
        printf("win=%lu imu_gyro_reports=%lu\n", (unsigned long)win, (unsigned long)d.gyro_n);
        for (int i = 0; i < ENC_COUNT; i++) c0[i] = c1[i];
        t0 = t;
    }
}
