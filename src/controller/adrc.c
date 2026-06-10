#include "adrc.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "ADRC";

// ─────────────────────────────────────────
// Default params — PLACEHOLDER, tune trên HIL
//
// Tuning guide:
//   omega_o = bandwidth ESO (rad/s) — thường 3~10x omega_c
//   beta1   = 2 * omega_o
//   beta2   = omega_o^2
//   b0      = 1/M cho vx/vy, 1/J cho wz
//   kp      = omega_c (controller bandwidth)
//
// Hardware: M=4kg, J=0.00247 kg·m², dt=0.01s (100Hz)
// ─────────────────────────────────────────

// vx / vy — translational
#define ADRC_VXY_OMEGA_O    20.0f   // ESO bandwidth (rad/s) — PLACEHOLDER
#define ADRC_VXY_B0         (1.0f / 4.0f)   // 1/M
#define ADRC_VXY_KP         5.0f            // controller bandwidth — PLACEHOLDER
#define ADRC_VXY_OUT_LIMIT  1.5f            // m/s

// wz — rotational
#define ADRC_WZ_OMEGA_O     20.0f
#define ADRC_WZ_B0          (1.0f / 0.00247f)   // 1/J
#define ADRC_WZ_KP          5.0f
#define ADRC_WZ_OUT_LIMIT   2.0f            // rad/s

// ─────────────────────────────────────────
// Internal: ESO update — linear first-order ESO
// Plant model: dx/dt = b0*u + d  (d = disturbance)
// ESO: dz1/dt = z2 + beta1*(x - z1) + b0*u
//      dz2/dt = beta2*(x - z1)
// ─────────────────────────────────────────
static void eso_update(eso_t *e, float actual, float u, float dt) {
    float err = actual - e->z1;

    // Euler integration
    float dz1 = e->z2 + e->beta1 * err + e->b0 * u;
    float dz2 = e->beta2 * err;

    e->z1 += dz1 * dt;
    e->z2 += dz2 * dt;
}

// ─────────────────────────────────────────
// Internal: ADRC update single axis
// u0    = kp * (sp - z1)          setpoint tracking
// u     = (u0 - z2) / b0          disturbance rejection
// ─────────────────────────────────────────
static float adrc_axis_update(adrc_axis_t *a, float sp, float actual, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // Lần đầu: khởi tạo ESO state từ actual
    if (!a->eso.initialized) {
        a->eso.z1 = actual;
        a->eso.z2 = 0.0f;
        a->eso.initialized = true;
    }

    // NLSEF (simplified linear)
    float u0 = a->kp * (sp - a->eso.z1);

    // Disturbance rejection — compensate z2
    float u = (u0 - a->eso.z2) / a->eso.b0;

    // Output clamp
    if      (u >  a->output_limit) u =  a->output_limit;
    else if (u < -a->output_limit) u = -a->output_limit;

    // Update ESO với u đã clamp
    eso_update(&a->eso, actual, u, dt);

    return u;
}

// ─────────────────────────────────────────
// Internal: init single axis
// ─────────────────────────────────────────
static void adrc_axis_init(adrc_axis_t *a,
                           float omega_o, float b0,
                           float kp, float out_limit) {
    memset(a, 0, sizeof(adrc_axis_t));
    a->eso.beta1      = 2.0f * omega_o;
    a->eso.beta2      = omega_o * omega_o;
    a->eso.b0         = b0;
    a->eso.initialized = false;
    a->kp             = kp;
    a->output_limit   = out_limit;
}

// ─────────────────────────────────────────
// API — public
// ─────────────────────────────────────────
void adrc_init(adrc_controller_t *c) {
    adrc_axis_init(&c->vx,
        ADRC_VXY_OMEGA_O, ADRC_VXY_B0,
        ADRC_VXY_KP, ADRC_VXY_OUT_LIMIT);
    adrc_axis_init(&c->vy,
        ADRC_VXY_OMEGA_O, ADRC_VXY_B0,
        ADRC_VXY_KP, ADRC_VXY_OUT_LIMIT);
    adrc_axis_init(&c->wz,
        ADRC_WZ_OMEGA_O, ADRC_WZ_B0,
        ADRC_WZ_KP, ADRC_WZ_OUT_LIMIT);

    ESP_LOGI(TAG, "ADRC init done — gains are PLACEHOLDER, tune on HIL");
}

void adrc_reset(adrc_controller_t *c) {
    c->vx.eso.z1 = 0.0f; c->vx.eso.z2 = 0.0f; c->vx.eso.initialized = false;
    c->vy.eso.z1 = 0.0f; c->vy.eso.z2 = 0.0f; c->vy.eso.initialized = false;
    c->wz.eso.z1 = 0.0f; c->wz.eso.z2 = 0.0f; c->wz.eso.initialized = false;
}

float adrc_update_vx(adrc_controller_t *c, float sp, float actual, float dt) {
    return adrc_axis_update(&c->vx, sp, actual, dt);
}

float adrc_update_vy(adrc_controller_t *c, float sp, float actual, float dt) {
    return adrc_axis_update(&c->vy, sp, actual, dt);
}

float adrc_update_wz(adrc_controller_t *c, float sp, float actual, float dt) {
    return adrc_axis_update(&c->wz, sp, actual, dt);
}
