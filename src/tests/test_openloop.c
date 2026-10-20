#include "test_openloop.h"
#include "encoder/encoder.h"
#include "motor/motor.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
// Spec S01 open-loop test (wheels lifted). Build with -DTEST_MODE -DTEST_OPENLOOP.
//
// Link: UART0 (the USB-UART port used by `pio device monitor`), 921600 baud,
// text lines ending in '\n'. ESP log output is turned off so it does not mix
// with the data. EN (GPIO 45) is shared by all four drivers.
//
// PC -> ESP32
//   PING                    -> PONG
//   CLR                     clear the schedule                 -> OK CLR
//   SEG <pwm> <en> <dur_ms> append a segment (pwm -1000..1000, en 0/1)
//                                                              -> OK SEG <i>
//   RUN <wheel>             run the schedule on wheel 0..3 (FL, FR, RL, RR)
//                                                              -> OK RUN
//   H                       heartbeat; required at least every 500 ms during a run
//   STOP                    abort now
// ESP32 -> PC after a run
//   DONE n=<samples> aborted=<0|1> reason=<ok|stop|heartbeat|overflow>
//   <t_us>,<count>,<pwm>,<en>      one line per sample, t_us from run start
//   END
//
// Safety: motors are off (EN=0, PWM 0) at boot, after every run and on abort.
// A run aborts if no line arrives for 500 ms, on STOP, or if the buffer fills.
// Brake vs coast: PWM 0 with EN=1 drives both half-bridges low (short-circuit
// brake on BTS7960-type drivers); EN=0 leaves the motor open (free coast).
// ─────────────────────────────────────────────────────────────────────────────

#define OL_UART            UART_NUM_0
#define OL_BAUD            921600
#define OL_PERIOD_US       1000
#define OL_MAX_SAMPLES     12000            /* 12 s at 1 kHz */
#define OL_MAX_SEGS        8
#define OL_MAX_RUN_MS      12000
#define OL_HB_TIMEOUT_US   500000

typedef struct { int16_t pwm; uint8_t en; uint32_t dur_ms; } ol_seg_t;
typedef struct __attribute__((packed)) { uint32_t t_us; int32_t count; int16_t pwm; uint8_t en; } ol_sample_t;

static ol_seg_t     s_segs[OL_MAX_SEGS];
static uint32_t     s_seg_end_ms[OL_MAX_SEGS];
static int          s_nseg = 0;
static ol_sample_t *s_buf  = NULL;

static volatile int      s_wheel;
static volatile int      s_seg;
static volatile int      s_n;
static volatile bool     s_running, s_abort_req;
static volatile int      s_reason;          /* 0 ok, 1 stop, 2 heartbeat, 3 overflow */
static volatile int64_t  s_t0, s_last_hb;
static esp_timer_handle_t s_timer;

static const char *REASON[] = {"ok", "stop", "heartbeat", "overflow"};

static void out(const char *s) { uart_write_bytes(OL_UART, s, strlen(s)); }

static void motors_safe(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) motor_set((motor_id_t)i, 0);
    gpio_set_level(GPIO_EN, 0);
}

static void finish(int reason) {
    motors_safe();
    s_reason  = reason;
    s_running = false;
}

// 1 kHz, esp_timer task context: apply the current segment, then sample.
static void ol_tick(void *arg) {
    if (!s_running) return;
    int64_t now = esp_timer_get_time();
    uint32_t t_ms = (uint32_t)((now - s_t0) / 1000);

    if (s_abort_req)                              { finish(1); return; }
    if (now - s_last_hb > OL_HB_TIMEOUT_US)      { finish(2); return; }
    while (s_seg < s_nseg && t_ms >= s_seg_end_ms[s_seg]) s_seg++;
    if (s_seg >= s_nseg)                          { finish(0); return; }
    if (s_n >= OL_MAX_SAMPLES)                    { finish(3); return; }

    const ol_seg_t *g = &s_segs[s_seg];
    gpio_set_level(GPIO_EN, g->en ? 1 : 0);
    motor_set((motor_id_t)s_wheel, g->en ? g->pwm : 0);

    ol_sample_t *p = &s_buf[s_n];
    p->t_us  = (uint32_t)(now - s_t0);
    p->count = encoder_get_count((enc_id_t)s_wheel);
    p->pwm   = g->en ? g->pwm : 0;
    p->en    = g->en;
    s_n++;
}

static void send_samples(void) {
    char line[64];
    snprintf(line, sizeof line, "DONE n=%d aborted=%d reason=%s\n",
             s_n, s_reason != 0, REASON[s_reason]);
    out(line);
    for (int i = 0; i < s_n; i++) {
        const ol_sample_t *p = &s_buf[i];
        int len = snprintf(line, sizeof line, "%lu,%ld,%d,%u\n",
                           (unsigned long)p->t_us, (long)p->count, p->pwm, p->en);
        uart_write_bytes(OL_UART, line, len);
    }
    out("END\n");
}

static void start_run(int wheel) {
    uint32_t total = 0;
    for (int i = 0; i < s_nseg; i++) { total += s_segs[i].dur_ms; s_seg_end_ms[i] = total; }
    if (wheel < 0 || wheel >= MOTOR_COUNT)  { out("ERR wheel\n"); return; }
    if (s_nseg == 0)                        { out("ERR no segments\n"); return; }
    if (total > OL_MAX_RUN_MS)              { out("ERR too long\n"); return; }

    encoder_clear((enc_id_t)wheel);
    s_wheel = wheel; s_seg = 0; s_n = 0; s_reason = 0; s_abort_req = false;
    s_t0 = s_last_hb = esp_timer_get_time();
    s_running = true;
    out("OK RUN\n");

    // Read heartbeats / STOP while the timer runs the schedule.
    char ln[32]; int k = 0; uint8_t c;
    while (s_running) {
        if (uart_read_bytes(OL_UART, &c, 1, pdMS_TO_TICKS(10)) == 1) {
            if (c == '\n' || c == '\r') {
                ln[k] = 0;
                if (k > 0) {
                    s_last_hb = esp_timer_get_time();
                    if (strncmp(ln, "STOP", 4) == 0) s_abort_req = true;
                }
                k = 0;
            } else if (k < (int)sizeof ln - 1) {
                ln[k++] = (char)c;
            }
        }
    }
    motors_safe();
    send_samples();
}

static void handle(char *ln) {
    char msg[48];
    if (strcmp(ln, "PING") == 0) { out("PONG\n"); }
    else if (strcmp(ln, "CLR") == 0) { s_nseg = 0; out("OK CLR\n"); }
    else if (strncmp(ln, "SEG ", 4) == 0) {
        int pwm, en, dur;
        if (sscanf(ln + 4, "%d %d %d", &pwm, &en, &dur) != 3 || s_nseg >= OL_MAX_SEGS ||
            pwm < -PWM_MAX || pwm > PWM_MAX || (en != 0 && en != 1) || dur <= 0) {
            out("ERR seg\n");
            return;
        }
        s_segs[s_nseg] = (ol_seg_t){ .pwm = (int16_t)pwm, .en = (uint8_t)en, .dur_ms = (uint32_t)dur };
        snprintf(msg, sizeof msg, "OK SEG %d\n", s_nseg++);
        out(msg);
    }
    else if (strncmp(ln, "RUN ", 4) == 0) { start_run(atoi(ln + 4)); }
    else if (strcmp(ln, "STOP") == 0 || strcmp(ln, "H") == 0) { motors_safe(); out("OK\n"); }
    else { out("ERR cmd\n"); }
}

void test_openloop(void) {
    motor_init();                 /* leaves EN = 0 */
    encoder_init();
    motors_safe();

    s_buf = heap_caps_malloc(OL_MAX_SAMPLES * sizeof(ol_sample_t), MALLOC_CAP_8BIT);

    esp_log_level_set("*", ESP_LOG_NONE);   /* keep UART0 for data only */
    uart_driver_install(OL_UART, 1024, 4096, 0, NULL, 0);
    uart_set_baudrate(OL_UART, OL_BAUD);

    const esp_timer_create_args_t targs = { .callback = ol_tick, .name = "ol_tick" };
    esp_timer_create(&targs, &s_timer);
    esp_timer_start_periodic(s_timer, OL_PERIOD_US);

    if (s_buf == NULL) {
        while (1) { out("ERR no memory\n"); vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    out("OL READY v1\n");

    char ln[64]; int k = 0; uint8_t c;
    while (1) {
        if (uart_read_bytes(OL_UART, &c, 1, pdMS_TO_TICKS(100)) != 1) continue;
        if (c == '\n' || c == '\r') {
            ln[k] = 0;
            if (k > 0) handle(ln);
            k = 0;
        } else if (k < (int)sizeof ln - 1) {
            ln[k++] = (char)c;
        }
    }
}
