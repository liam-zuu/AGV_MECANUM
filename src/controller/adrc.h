#ifndef ADRC_H
#define ADRC_H

#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────
// ADRC — Active Disturbance Rejection Control
// Kiến trúc: TD + ESO + NLSEF (simplified linear ESO)
//
// Cho mỗi kênh (vx, vy, wz):
//   Plant order: 1st order (velocity control)
//   State: [x1=velocity, x2=disturbance]
//   ESO estimate cả x1 và x2 (disturbance tổng hợp)
//   NLSEF = kp*(r - z1) - z2/b0  (compensation + setpoint tracking)
// ─────────────────────────────────────────

// ESO (Extended State Observer) — linear version
typedef struct {
    float z1;          // estimate của state (velocity)
    float z2;          // estimate của disturbance tổng hợp
    float beta1;       // observer gain 1 — tune: beta1 = 2*omega_o
    float beta2;       // observer gain 2 — tune: beta2 = omega_o^2
    float b0;          // control gain ≈ 1/M hoặc 1/J
    bool  initialized;
} eso_t;

// ADRC single axis
typedef struct {
    eso_t  eso;
    float  kp;          // setpoint tracking gain
    float  output_limit;
} adrc_axis_t;

// ADRC controller — 3 kênh độc lập
typedef struct {
    adrc_axis_t vx;
    adrc_axis_t vy;
    adrc_axis_t wz;
} adrc_controller_t;

// ─────────────────────────────────────────
// API
// ─────────────────────────────────────────
void  adrc_init(adrc_controller_t *c);
void  adrc_reset(adrc_controller_t *c);

float adrc_update_vx(adrc_controller_t *c, float sp, float actual, float dt);
float adrc_update_vy(adrc_controller_t *c, float sp, float actual, float dt);
float adrc_update_wz(adrc_controller_t *c, float sp, float actual, float dt);

#endif // ADRC_H
