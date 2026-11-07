#include "test_mu_floor.h"
#include "encoder/encoder.h"
#include "motor/motor.h"
#include "kinematics/kinematics.h"
#include "imu/imu.h"
#include "led_buzzer/led_buzzer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
// Effective wheel-floor friction (mu_hd) test on the car, spec controller-IMU v2 section 1b.4
// (Liam 27/10: the car pushes itself into a force gauge tied to a fixed point; PWM is raised very
// slowly until the wheels spin; mu_hd = F_max / (m g)). Build with -DTEST_MODE -DTEST_MU_FLOOR
// (env mu_floor). Liam 07/11, CODE session: "lam het viec A di".
//
// Link: UART0 (USB-UART of `pio device monitor`), 921600 baud, text lines ending in '\n';
// ESP log output off. PC side: hil-framework tools/mu_floor_runner.py.
//
// PC -> ESP32
//   PING                                      -> PONG
//   RAMP <dir> <pwm0> <rate> <pwm_max> <w_slip>
//        dir     XP (forward), XN (backward), YP (left), YN (right): the sign of each wheel is
//                the sign of kinematics_inverse() for a unit vx or vy, so the four wheels push
//                the car the way the controller would (motor_set() applies motor_dir)
//        pwm0    first PWM magnitude (0..1000), rate PWM/s (1..200), pwm_max (pwm0..1000)
//        w_slip  rad/s: stop when the mean |wheel speed| over the last 200 ms is above it
//                (0 = no automatic stop)                       -> OK RAMP
//   H        heartbeat, at least every 500 ms during a run;  STOP  abort now
// ESP32 -> PC
//   MU READY v1 imu=<ok|fail>                at boot (fail: no IMU report within 2 s)
//   EV START t_ms=0                          ramp starts (two short beeps just before)
//   D,<t_ms>,<pwm>,<c_fl>,<c_fr>,<c_rl>,<c_rr>,<ax>,<ay>,<az>,<gz>,<imu_n>   every 10 ms
//        pwm: magnitude applied (the per-wheel sign is fixed by dir); counts cumulative from
//        the start; ax..gz BNO085 accel (m/s^2) and gyro z (rad/s), imu_n accel report count
//   EV SLIP t_ms=<t> pwm=<p> w=<rad/s>       automatic stop on w_slip
//   DONE reason=<max|slip|stop|heartbeat|imu|time> t_ms=<t> pwm=<p>
//        max: 2 s held at pwm_max without slip; time: 120 s
//
// Safety: motors are off (EN = 0, PWM 0) at boot, at the end of every ramp and on abort; the
// independent stop is the motor power switch (spec 1b.4). A run aborts with no line for 500 ms,
// on STOP, at 120 s, and when IMU reports stop for 1 s. RAMP is refused when the IMU never
// started (every firmware has the IMU, Outline QD 23: an IMU failure stops with an error).
// ─────────────────────────────────────────────────────────────────────────────

#define MU_UART          UART_NUM_0
#define MU_BAUD          921600
#define MU_PERIOD_MS     10
#define MU_HB_TIMEOUT_US 500000
#define MU_MAX_RUN_MS    120000
#define MU_IMU_WAIT_MS   2000
#define MU_IMU_STALE_US  1000000
#define MU_W_WIN         20                 /* 20 x 10 ms = 200 ms window for w_slip */
#define COUNTS_PER_REV   1320.0f

static bool s_imu_ok;

static void out(const char *s) { uart_write_bytes(MU_UART, s, strlen(s)); }

static void motors_safe(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) motor_set((motor_id_t)i, 0);
    gpio_set_level(GPIO_EN, 0);
}

static bool dir_signs(const char *dir, int sign[4]) {
    agv_velocity_t v = {0.0f, 0.0f, 0.0f};
    if      (strcmp(dir, "XP") == 0) v.vx =  1.0f;
    else if (strcmp(dir, "XN") == 0) v.vx = -1.0f;
    else if (strcmp(dir, "YP") == 0) v.vy =  1.0f;
    else if (strcmp(dir, "YN") == 0) v.vy = -1.0f;
    else return false;
    wheel_velocity_t w = kinematics_inverse(v);
    float f[4] = { w.fl, w.fr, w.rl, w.rr };
    for (int i = 0; i < 4; i++) sign[i] = f[i] > 0.0f ? 1 : -1;
    return true;
}

/* Non-blocking: returns 1 on a heartbeat line, 2 on STOP, 0 when nothing complete arrived. */
static int poll_line(char *ln, int *k) {
    uint8_t c;
    int got = 0;
    while (uart_read_bytes(MU_UART, &c, 1, 0) == 1) {
        if (c == '\n' || c == '\r') {
            ln[*k] = 0;
            if (*k > 0) got = (strncmp(ln, "STOP", 4) == 0) ? 2 : (got == 2 ? 2 : 1);
            *k = 0;
        } else if (*k < 31) {
            ln[(*k)++] = (char)c;
        }
    }
    return got;
}

static void ramp(const char *dir, int pwm0, int rate, int pwm_max, float w_slip) {
    int sign[4];
    if (!s_imu_ok)                                       { out("ERR imu\n"); return; }
    if (!dir_signs(dir, sign))                           { out("ERR dir\n"); return; }
    if (pwm0 < 0 || pwm_max > PWM_MAX || pwm0 > pwm_max) { out("ERR pwm\n"); return; }
    if (rate < 1 || rate > 200 || w_slip < 0.0f)         { out("ERR rate\n"); return; }
    out("OK RAMP\n");

    buzzer_beep(2700, 80); vTaskDelay(pdMS_TO_TICKS(150)); buzzer_beep(2700, 80);
    led_set_color(COLOR_RED);
    for (int i = 0; i < ENC_COUNT; i++) encoder_clear((enc_id_t)i);
    int32_t hist[MU_W_WIN][4] = {{0}};
    int nh = 0;
    char ln[32], line[200];
    int k = 0;
    const char *reason = "max";
    int64_t t0 = esp_timer_get_time(), last_hb = t0;
    uint32_t t_ms = 0;
    int pwm = pwm0;
    out("EV START t_ms=0\n");
    gpio_set_level(GPIO_EN, 1);
    TickType_t wake = xTaskGetTickCount();
    while (1) {
        int64_t now = esp_timer_get_time();
        t_ms = (uint32_t)((now - t0) / 1000);
        int hb = poll_line(ln, &k);
        if (hb) last_hb = now;
        if (hb == 2)                                     { reason = "stop"; break; }
        if (now - last_hb > MU_HB_TIMEOUT_US)            { reason = "heartbeat"; break; }
        if (t_ms > MU_MAX_RUN_MS)                        { reason = "time"; break; }

        long p = pwm0 + (long)rate * (long)t_ms / 1000L;
        pwm = p > pwm_max ? pwm_max : (int)p;
        for (int i = 0; i < MOTOR_COUNT; i++) motor_set((motor_id_t)i, sign[i] * pwm);

        int32_t c[4];
        for (int i = 0; i < ENC_COUNT; i++) c[i] = encoder_get_count((enc_id_t)i);
        imu_data_t im = {0};
        imu_get(&im);
        if (now - im.accel_t_us > MU_IMU_STALE_US)       { reason = "imu"; break; }
        int n = snprintf(line, sizeof line, "D,%lu,%d,%ld,%ld,%ld,%ld,%.4f,%.4f,%.4f,%.5f,%lu\n",
                         (unsigned long)t_ms, pwm, (long)c[0], (long)c[1], (long)c[2], (long)c[3],
                         im.accel_x, im.accel_y, im.accel_z, im.gyro_z, (unsigned long)im.accel_n);
        uart_write_bytes(MU_UART, line, n);

        /* mean |wheel speed| over the last 200 ms, from the counts of the oldest sample kept */
        int slot = nh % MU_W_WIN;
        if (nh >= MU_W_WIN && w_slip > 0.0f) {
            float w = 0.0f;
            for (int i = 0; i < 4; i++)
                w += fabsf((float)(c[i] - hist[slot][i])) / COUNTS_PER_REV * 2.0f * (float)M_PI
                     / (MU_W_WIN * MU_PERIOD_MS * 1e-3f);
            w /= 4.0f;
            if (w > w_slip) {
                motors_safe();
                snprintf(line, sizeof line, "EV SLIP t_ms=%lu pwm=%d w=%.2f\n", (unsigned long)t_ms, pwm, w);
                out(line);
                reason = "slip";
                break;
            }
        }
        for (int i = 0; i < 4; i++) hist[slot][i] = c[i];
        nh++;
        if (pwm >= pwm_max && p > (long)pwm_max + (long)rate * 2L) { reason = "max"; break; }  /* 2 s at pwm_max */
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(MU_PERIOD_MS));
    }
    motors_safe();
    led_set_color(COLOR_GREEN);
    buzzer_beep(2000, 300);
    snprintf(line, sizeof line, "DONE reason=%s t_ms=%lu pwm=%d\n", reason, (unsigned long)t_ms, pwm);
    out(line);
}

static void handle(char *ln) {
    if (strcmp(ln, "PING") == 0) { out("PONG\n"); return; }
    if (strncmp(ln, "RAMP ", 5) == 0) {
        char dir[4] = {0};
        int pwm0, rate, pwm_max;
        float w_slip;
        if (sscanf(ln + 5, "%2s %d %d %d %f", dir, &pwm0, &rate, &pwm_max, &w_slip) != 5) { out("ERR ramp\n"); return; }
        ramp(dir, pwm0, rate, pwm_max, w_slip);
        return;
    }
    if (strcmp(ln, "STOP") == 0 || strcmp(ln, "H") == 0) { motors_safe(); out("OK\n"); return; }
    out("ERR cmd\n");
}

void test_mu_floor(void) {
    led_buzzer_init();            /* before imu_init (GPIO wiring note in main.c) */
    led_set_color(COLOR_YELLOW);
    motor_init();                 /* leaves EN = 0 */
    encoder_init();
    motors_safe();
    esp_log_level_set("*", ESP_LOG_NONE);   /* keep UART0 for data only */
    uart_driver_install(MU_UART, 1024, 8192, 0, NULL, 0);
    uart_set_baudrate(MU_UART, MU_BAUD);

    imu_init();
    imu_data_t im = {0};
    for (int i = 0; i < MU_IMU_WAIT_MS / 50 && !(imu_get(&im) && im.valid && im.accel_n > 0); i++)
        vTaskDelay(pdMS_TO_TICKS(50));
    s_imu_ok = im.valid && im.accel_n > 0;
    led_set_color(s_imu_ok ? COLOR_GREEN : COLOR_BLUE);
    if (!s_imu_ok) { buzzer_beep(2000, 800); }
    out(s_imu_ok ? "MU READY v1 imu=ok\n" : "MU READY v1 imu=fail\n");

    char ln[64];
    int k = 0;
    uint8_t c;
    while (1) {
        if (uart_read_bytes(MU_UART, &c, 1, pdMS_TO_TICKS(100)) != 1) continue;
        if (c == '\n' || c == '\r') {
            ln[k] = 0;
            if (k > 0) handle(ln);
            k = 0;
        } else if (k < (int)sizeof ln - 1) {
            ln[k++] = (char)c;
        }
    }
}
