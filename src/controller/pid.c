#include "pid.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "PID";

// ─────────────────────────────────────────
// Default gains — PLACEHOLDER, tune trên HIL
// Dựa trên: w_max=34.56 rad/s, r=0.0485m → v_max≈1.67 m/s
// ─────────────────────────────────────────
#define PID_VX_KP           1.5f
#define PID_VX_KI           0.05f
#define PID_VX_KD           0.0f
#define PID_VX_INT_LIMIT    0.5f
#define PID_VX_OUT_LIMIT    1.5f

#define PID_VY_KP           1.5f
#define PID_VY_KI           0.05f
#define PID_VY_KD           0.0f
#define PID_VY_INT_LIMIT    0.5f
#define PID_VY_OUT_LIMIT    1.5f

#define PID_WZ_KP           1.0f
#define PID_WZ_KI           0.05f
#define PID_WZ_KD           0.0f
#define PID_WZ_INT_LIMIT    1.0f
#define PID_WZ_OUT_LIMIT    2.0f
// ─────────────────────────────────────────
// Internal: update single axis
// ─────────────────────────────────────────
static float pid_axis_update(pid_t *p, float sp, float actual, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = sp - actual;

    // Proportional
    float p_term = p->kp * error;

    // Integral với anti-windup clamp
    p->integral += error * dt;
    if      (p->integral >  p->integral_limit) p->integral =  p->integral_limit;
    else if (p->integral < -p->integral_limit) p->integral = -p->integral_limit;
    float i_term = p->ki * p->integral;

    // Derivative — bỏ qua lần đầu để tránh spike
    float d_term = 0.0f;
    if (p->initialized) {
        d_term = p->kd * (error - p->prev_error) / dt;
    }
    p->prev_error  = error;
    p->initialized = true;

    // Output clamp
    float output = p_term + i_term + d_term;
    if      (output >  p->output_limit) output =  p->output_limit;
    else if (output < -p->output_limit) output = -p->output_limit;

    return output;
}

// ─────────────────────────────────────────
// Internal: init single axis
// ─────────────────────────────────────────
static void pid_axis_init(pid_t *p,
                          float kp, float ki, float kd,
                          float int_limit, float out_limit) {
    memset(p, 0, sizeof(pid_t));
    p->kp             = kp;
    p->ki             = ki;
    p->kd             = kd;
    p->integral_limit = int_limit;
    p->output_limit   = out_limit;
    p->initialized    = false;
}

// ─────────────────────────────────────────
// API — public
// ─────────────────────────────────────────
void pid_init(pid_controller_t *c) {
    pid_axis_init(&c->vx,
        PID_VX_KP, PID_VX_KI, PID_VX_KD,
        PID_VX_INT_LIMIT, PID_VX_OUT_LIMIT);
    pid_axis_init(&c->vy,
        PID_VY_KP, PID_VY_KI, PID_VY_KD,
        PID_VY_INT_LIMIT, PID_VY_OUT_LIMIT);
    pid_axis_init(&c->wz,
        PID_WZ_KP, PID_WZ_KI, PID_WZ_KD,
        PID_WZ_INT_LIMIT, PID_WZ_OUT_LIMIT);

    ESP_LOGI(TAG, "PID init done — gains are PLACEHOLDER, tune on HIL");
}

void pid_reset(pid_controller_t *c) {
    c->vx.integral    = 0.0f;
    c->vy.integral    = 0.0f;
    c->wz.integral    = 0.0f;
    c->vx.prev_error  = 0.0f;
    c->vy.prev_error  = 0.0f;
    c->wz.prev_error  = 0.0f;
    c->vx.initialized = false;
    c->vy.initialized = false;
    c->wz.initialized = false;
}

void pid_reset_vx(pid_controller_t *c) {
    c->vx.integral   = 0.0f;
    c->vx.prev_error = 0.0f;
}

void pid_reset_vy(pid_controller_t *c) {
    c->vy.integral   = 0.0f;
    c->vy.prev_error = 0.0f;
}

void pid_reset_wz(pid_controller_t *c) {
    c->wz.integral   = 0.0f;
    c->wz.prev_error = 0.0f;
}

float pid_update_vx(pid_controller_t *c, float sp, float actual, float dt) {
    return pid_axis_update(&c->vx, sp, actual, dt);
}

float pid_update_vy(pid_controller_t *c, float sp, float actual, float dt) {
    return pid_axis_update(&c->vy, sp, actual, dt);
}

float pid_update_wz(pid_controller_t *c, float sp, float actual, float dt) {
    return pid_axis_update(&c->wz, sp, actual, dt);
}
