#include "test_mu_floor.h"
#include "encoder/encoder.h"
#include "motor/motor.h"
#include "kinematics/kinematics.h"
#include "imu/imu.h"
#include "led_buzzer/led_buzzer.h"
#include "wifi/wifi.h"
#include "fw_id_gen.h"            /* FW_GIT_SHA, FW_GIT_DIRTY, FW_ENV: scripts/build_id.py at build time */
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
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
// Protocol v2 (Liam 09/11, CODE session: no USB cable while the car pushes, commands over WiFi from
// the laptop; "Duyet, code di" for the design with UDP port 5013 and the 2 % missing-sample rule).
// Two links carry the same text lines ending in '\n', both always on:
//   UART0 (USB-UART), 921600 baud, for the bench; ESP log output off.
//   UDP port MU_UDP_PORT on the WiFi of src/wifi (wifi_secrets.h). Every datagram may hold one or
//   more lines. Every output line goes to UART0 and, once a datagram has been received, also to
//   the source address of the LAST datagram (no fixed laptop IP). PING may come as a broadcast.
// PC side: hil-framework tools/mu_floor_runner.py (--udp auto|IP, or --port COMx).
//
// PC -> ESP32
//   PING            -> PONG ip=<a.b.c.d|none> imu=<ok|fail> fw=<sha> dirty=<0|1> env=<env>
//                           last_id=<id> last_reason=<reason|none> last_n=<samples>
//   RAMP <id> <dir> <pwm0> <rate> <pwm_max> <w_slip>
//        id      run number, > every id accepted since boot; a RAMP whose id is not greater (a
//                repeated or late datagram) does not start a run: -> DUP RAMP id=<id> last=<last_id>
//        dir     XP (forward), XN (backward), YP (left), YN (right): the sign of each wheel is
//                the sign of kinematics_inverse() for a unit vx or vy, so the four wheels push
//                the car the way the controller would (motor_set() applies motor_dir)
//        pwm0    first PWM magnitude (0..1000), rate PWM/s (1..200), pwm_max (pwm0..1000)
//        w_slip  rad/s: stop when the mean |wheel speed| over the last 200 ms is above it
//                (0 = no automatic stop)                       -> OK RAMP id=<id>
//   H        heartbeat; during a run ANY line on either link counts as one (a repeated RAMP too)
//   STOP     abort now (during a run); idle: motors off -> OK
// ESP32 -> PC
//   MU READY v2 imu=<ok|fail>                at boot (UART only: no UDP peer yet)
//   EV WIFI ip=<a.b.c.d>                     when the WiFi gets an address
//   EV START id=<id> t_ms=0                  ramp starts (two short beeps just before)
//   D,<t_ms>,<pwm>,<c_fl>,<c_fr>,<c_rl>,<c_rr>,<ax>,<ay>,<az>,<gz>,<imu_n>,<seq>   every 10 ms
//        pwm: magnitude applied (the per-wheel sign is fixed by dir); counts cumulative from
//        the start; ax..gz BNO085 accel (m/s^2) and gyro z (rad/s), imu_n accel report count;
//        seq 0, 1, 2 ... within the run (the PC counts lost datagrams from it)
//   EV SLIP t_ms=<t> pwm=<p> w=<rad/s>       automatic stop on w_slip
//   DONE reason=<max|slip|stop|heartbeat|imu|time> t_ms=<t> pwm=<p> id=<id> n=<D lines> udp_err=<k>
//        max: 2 s held at pwm_max without slip; time: 120 s; udp_err: sendto failures in the run
//
// Safety (unchanged from v1, independent of the link): motors are off (EN = 0, PWM 0) at boot, at
// the end of every ramp and on abort; the independent stop is the motor power switch (spec 1b.4).
// A run aborts with no line for 500 ms on both links (WiFi lost = heartbeat stop), on STOP, at
// 120 s, and when IMU reports stop for 1 s. PWM never exceeds pwm_max <= PWM_MAX. RAMP is refused
// when the IMU never started (every firmware has the IMU, Outline QD 23).
// WiFi power save is off in this env, so a heartbeat is not held back until the next beacon.
// ─────────────────────────────────────────────────────────────────────────────

#define MU_UART          UART_NUM_0
#define MU_BAUD          921600
#define MU_UDP_PORT      5013              /* free: logger 5005, s06 5007/5011, traj 5010 */
#define MU_PERIOD_MS     10
#define MU_HB_TIMEOUT_US 500000
#define MU_MAX_RUN_MS    120000
#define MU_IMU_WAIT_MS   2000
#define MU_IMU_STALE_US  1000000
#define MU_W_WIN         20                 /* 20 x 10 ms = 200 ms window for w_slip */
#define COUNTS_PER_REV   1320.0f
#define MU_LINE          64

static bool s_imu_ok;

/* ── links ─────────────────────────────────────────────────────────────────── */
static int s_sock = -1;
static struct sockaddr_in s_peer;
static bool s_have_peer;
static uint32_t s_udp_err;                  /* sendto failures, reset at each RAMP */
static char s_uart_ln[MU_LINE];
static int s_uart_k;
static char s_udp_buf[256];
static int s_udp_len, s_udp_pos;

/* last run, for PONG (a DONE datagram can be lost) */
static unsigned long s_last_id;
static const char *s_last_reason = "none";
static unsigned long s_last_n;

static void out(const char *s) {
    size_t n = strlen(s);
    uart_write_bytes(MU_UART, s, n);
    if (s_sock >= 0 && s_have_peer) {
        if (sendto(s_sock, s, n, MSG_DONTWAIT, (struct sockaddr *)&s_peer, sizeof s_peer) < 0) s_udp_err++;
    }
}

static void ip_str(char *buf, size_t n) {
    esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    if (nif && wifi_is_connected() && esp_netif_get_ip_info(nif, &ip) == ESP_OK && ip.ip.addr != 0)
        snprintf(buf, n, IPSTR, IP2STR(&ip.ip));
    else
        snprintf(buf, n, "none");
}

/* Opens the UDP socket once the WiFi is up, and says so once per connection. */
static void udp_service(void) {
    static bool announced;
    bool up = wifi_is_connected();
    if (!up) { announced = false; return; }
    if (s_sock < 0) {
        int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s < 0) return;
        int one = 1;
        setsockopt(s, SOL_SOCKET, SO_BROADCAST, &one, sizeof one);   /* lwIP drops broadcast without it */
        struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(MU_UDP_PORT),
                                 .sin_addr.s_addr = htonl(INADDR_ANY) };
        if (bind(s, (struct sockaddr *)&a, sizeof a) < 0) { close(s); return; }
        s_sock = s;
    }
    if (!announced) {
        char ip[16], line[48];
        ip_str(ip, sizeof ip);
        if (strcmp(ip, "none") == 0) return;
        snprintf(line, sizeof line, "EV WIFI ip=%s\n", ip);
        out(line);
        announced = true;
    }
}

/* Non-blocking: copies one complete, non-empty line into ln (UART first, then UDP) and returns
 * true; false when no complete line is waiting. A UDP datagram sets the peer for out(). */
static bool next_line(char *ln) {
    uint8_t c;
    while (uart_read_bytes(MU_UART, &c, 1, 0) == 1) {
        if (c == '\n' || c == '\r') {
            if (s_uart_k > 0) {
                s_uart_ln[s_uart_k] = 0;
                memcpy(ln, s_uart_ln, (size_t)s_uart_k + 1);
                s_uart_k = 0;
                return true;
            }
        } else if (s_uart_k < MU_LINE - 1) {
            s_uart_ln[s_uart_k++] = (char)c;
        }
    }
    for (;;) {
        while (s_udp_pos < s_udp_len) {
            int k = 0;
            while (s_udp_pos < s_udp_len && s_udp_buf[s_udp_pos] != '\n' && s_udp_buf[s_udp_pos] != '\r') {
                if (k < MU_LINE - 1) ln[k++] = s_udp_buf[s_udp_pos];
                s_udp_pos++;
            }
            s_udp_pos++;                                  /* the end of line (or past the end) */
            if (k > 0) { ln[k] = 0; return true; }
        }
        if (s_sock < 0) return false;
        struct sockaddr_in src;
        socklen_t sl = sizeof src;
        int r = recvfrom(s_sock, s_udp_buf, sizeof s_udp_buf, MSG_DONTWAIT, (struct sockaddr *)&src, &sl);
        if (r <= 0) return false;
        s_peer = src;
        s_have_peer = true;
        s_udp_len = r;
        s_udp_pos = 0;
    }
}

/* ── test ──────────────────────────────────────────────────────────────────── */
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

static void ramp(unsigned long id, const char *dir, int pwm0, int rate, int pwm_max, float w_slip) {
    int sign[4];
    static char line[200];                            /* static: the main task stack is 3584 B */
    if (!s_imu_ok)                                       { out("ERR imu\n"); return; }
    if (!dir_signs(dir, sign))                           { out("ERR dir\n"); return; }
    if (pwm0 < 0 || pwm_max > PWM_MAX || pwm0 > pwm_max) { out("ERR pwm\n"); return; }
    if (rate < 1 || rate > 200 || w_slip < 0.0f)         { out("ERR rate\n"); return; }
    if (id <= s_last_id) {
        snprintf(line, sizeof line, "DUP RAMP id=%lu last=%lu\n", id, s_last_id);
        out(line);
        return;
    }
    s_last_id = id;
    s_last_reason = "running";
    s_last_n = 0;
    s_udp_err = 0;
    snprintf(line, sizeof line, "OK RAMP id=%lu\n", id);
    out(line);

    buzzer_beep(2700, 80); vTaskDelay(pdMS_TO_TICKS(150)); buzzer_beep(2700, 80);
    led_set_color(COLOR_RED);
    for (int i = 0; i < ENC_COUNT; i++) encoder_clear((enc_id_t)i);
    static int32_t hist[MU_W_WIN][4];
    memset(hist, 0, sizeof hist);
    int nh = 0;
    unsigned long seq = 0;
    char ln[MU_LINE];
    const char *reason = "max";
    int64_t t0 = esp_timer_get_time(), last_hb = t0;
    uint32_t t_ms = 0;
    int pwm = pwm0;
    snprintf(line, sizeof line, "EV START id=%lu t_ms=0\n", id);
    out(line);
    gpio_set_level(GPIO_EN, 1);
    TickType_t wake = xTaskGetTickCount();
    while (1) {
        int64_t now = esp_timer_get_time();
        t_ms = (uint32_t)((now - t0) / 1000);
        bool stop = false;
        while (next_line(ln)) {                           /* any line on either link = heartbeat */
            last_hb = now;
            if (strcmp(ln, "STOP") == 0) stop = true;
        }
        if (stop)                                        { reason = "stop"; break; }
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
        snprintf(line, sizeof line, "D,%lu,%d,%ld,%ld,%ld,%ld,%.4f,%.4f,%.4f,%.5f,%lu,%lu\n",
                 (unsigned long)t_ms, pwm, (long)c[0], (long)c[1], (long)c[2], (long)c[3],
                 im.accel_x, im.accel_y, im.accel_z, im.gyro_z, (unsigned long)im.accel_n, seq);
        out(line);
        seq++;

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
    s_last_reason = reason;
    s_last_n = seq;
    snprintf(line, sizeof line, "DONE reason=%s t_ms=%lu pwm=%d id=%lu n=%lu udp_err=%lu\n",
             reason, (unsigned long)t_ms, pwm, id, seq, (unsigned long)s_udp_err);
    out(line);
}

static void handle(char *ln) {
    static char line[200];
    if (strcmp(ln, "PING") == 0) {
        char ip[16];
        ip_str(ip, sizeof ip);
        snprintf(line, sizeof line, "PONG ip=%s imu=%s fw=%s dirty=%d env=%s last_id=%lu last_reason=%s last_n=%lu\n",
                 ip, s_imu_ok ? "ok" : "fail", FW_GIT_SHA, FW_GIT_DIRTY, FW_ENV,
                 s_last_id, s_last_reason, s_last_n);
        out(line);
        return;
    }
    if (strncmp(ln, "RAMP ", 5) == 0) {
        char dir[4] = {0};
        unsigned long id;
        int pwm0, rate, pwm_max;
        float w_slip;
        if (sscanf(ln + 5, "%lu %2s %d %d %d %f", &id, dir, &pwm0, &rate, &pwm_max, &w_slip) != 6 || id == 0) {
            out("ERR ramp\n");
            return;
        }
        ramp(id, dir, pwm0, rate, pwm_max, w_slip);
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

    wifi_init();                  /* non-blocking; the socket opens in udp_service() once connected */
    esp_wifi_set_ps(WIFI_PS_NONE);

    imu_init();
    imu_data_t im = {0};
    for (int i = 0; i < MU_IMU_WAIT_MS / 50 && !(imu_get(&im) && im.valid && im.accel_n > 0); i++)
        vTaskDelay(pdMS_TO_TICKS(50));
    s_imu_ok = im.valid && im.accel_n > 0;
    led_set_color(s_imu_ok ? COLOR_GREEN : COLOR_BLUE);
    if (!s_imu_ok) { buzzer_beep(2000, 800); }
    out(s_imu_ok ? "MU READY v2 imu=ok\n" : "MU READY v2 imu=fail\n");

    char ln[MU_LINE];
    while (1) {
        udp_service();
        if (next_line(ln)) handle(ln);
        else vTaskDelay(pdMS_TO_TICKS(5));
    }
}
