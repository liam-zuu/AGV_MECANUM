#include "test_pwm_hold.h"
#include "motor/motor.h"
#include "imu/imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

// ─────────────────────────────────────────────────────────────────────────────
// Fixed PWM commands for checking the H7 PWMCapture (build with -DTEST_MODE
// -DTEST_PWM_HOLD, env pwm_hold). Bugs under test: H7-TIM17-PIN, H7-TIM15-ARM,
// H7-PWM-LEVEL (22/10). Runs on the bench ESP32 (only wired to the H7).
// The ESP32 cycles through STEPS, holding each for STEP_MS, through the real
// motor_set() (same pins, same motor_dir, same idle level as the main firmware).
// It prints "STEP k FL=.. FR=.. RL=.. RR=.." at the start of each step; the H7
// prints "PWM: fl fr rl rr" every 0.5 s on its ST-Link VCP. Compare with the
// expected table in 00_OVERVIEW/Setup_Guides/2026-10-22_Huong_dan_Kiem_PWMCapture.md.
// Motor enable stays off (EN = 0); on the bench there is no driver anyway.
// IMU init runs in its own task (every firmware has the IMU, QD 23; the bench
// ESP32 has no BNO085 and the library can block, bug ESP-BNOLIB-D).
// Env pwm_hold_drv46 (-DPWM_HOLD_DRV46, 27/10): raises the pad drive strength of
// GPIO46 to GPIO_DRIVE_CAP_3 after motor_init(). The H7 misreads only the signal
// from GPIO46 (PWM_CAPTURE run01-run05); this checks whether a weak edge is the
// cause. Test env only: the car firmware keeps the default drive strength.
// ─────────────────────────────────────────────────────────────────────────────

#define STEP_MS 5000

static const int STEPS[][MOTOR_COUNT] = {
    {    0,    0,    0,    0 },
    {  300, -300,  700, -700 },
    { -300,  300, -700,  700 },
    {  700,  700,  300,  300 },
};
#define N_STEPS (sizeof(STEPS) / sizeof(STEPS[0]))

static void print_drive_caps(const char *tag) {
    gpio_drive_cap_t c46 = 0, c47 = 0;
    gpio_get_drive_capability(RL_RPWM, &c46);   /* GPIO46 */
    gpio_get_drive_capability(RR_RPWM, &c47);   /* GPIO47, reads correctly on the H7 */
    printf("DRIVE %s: GPIO%d=%d GPIO%d=%d\n", tag, RL_RPWM, (int)c46, RR_RPWM, (int)c47);
}

static void imu_init_task(void *pv) {
    imu_init();
    vTaskDelete(NULL);
}

void test_pwm_hold(void) {
    motor_init();                         /* EN = 0 */
    print_drive_caps("default");
#ifdef PWM_HOLD_DRV46
    gpio_set_drive_capability(RL_RPWM, GPIO_DRIVE_CAP_3);
    print_drive_caps("after_set");
#endif
    xTaskCreate(imu_init_task, "imu_init", 4096, NULL, 3, NULL);
#ifdef PWM_HOLD_DRV46
    printf("PWM HOLD READY v2 drv46 step_ms=%d\n", STEP_MS);
#else
    printf("PWM HOLD READY v2 step_ms=%d\n", STEP_MS);
#endif

    unsigned round = 0;
    while (1) {
        round++;
        for (unsigned k = 0; k < N_STEPS; k++) {
            for (int m = 0; m < MOTOR_COUNT; m++) motor_set((motor_id_t)m, STEPS[k][m]);
            printf("round=%u STEP %u FL=%d FR=%d RL=%d RR=%d\n", round, k,
                   STEPS[k][0], STEPS[k][1], STEPS[k][2], STEPS[k][3]);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }
    }
}
