#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────
// PID single-axis — dùng cho 1 kênh (vx, vy, hoặc wz)
// ─────────────────────────────────────────
typedef struct {
    // Gains — điền sau khi HIL tune
    float kp;
    float ki;
    float kd;

    // Anti-windup — clamp integral
    float integral_limit;   // |integral| <= limit

    // Output clamp
    float output_limit;     // |output| <= limit

    // Internal state
    float integral;
    float prev_error;
    bool  initialized;
} pid_t;

// ─────────────────────────────────────────
// PID controller — 3 kênh độc lập
// ─────────────────────────────────────────
typedef struct {
    pid_t vx;   // tiến/lùi
    pid_t vy;   // strafe
    pid_t wz;   // xoay
} pid_controller_t;

// ─────────────────────────────────────────
// API
// ─────────────────────────────────────────
void  pid_init(pid_controller_t *c);
void  pid_reset(pid_controller_t *c);

// Trả về correction velocity (m/s hoặc rad/s) cho từng kênh
float pid_update_vx(pid_controller_t *c, float sp, float actual, float dt);
float pid_update_vy(pid_controller_t *c, float sp, float actual, float dt);
float pid_update_wz(pid_controller_t *c, float sp, float actual, float dt);

#endif // PID_H
