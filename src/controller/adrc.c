#include "adrc.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "ADRC";

// ─────────────────────────────────────────
// Default params — LADRC bậc 1 ở mức vận tốc thân (21/10/2026)
//
// Mô hình mỗi kênh: dv/dt ≈ b0*u + f, u = lệnh vận tốc thân đưa vào IK → PWM
// (kinematics_radps_to_pwm: pwm = w/W_MAX*PWM_MAX, nên hệ số tĩnh u → v ≈ 1).
// b0 = 1/tau. tau lấy từ plant motor DC dùng tham số đo ở S01 run01
// (05_DATA/results/S01/2026-10-20_run01), xe trên sàn, đáp ứng bậc lệnh hở:
//   vx, vy: tau ≈ 0.17 s → b0 ≈ 5.9 1/s
//   wz:     tau ≈ 0.10 s → b0 ≈ 10 1/s
// (S01 đo kê bánh, chỉ có quán tính bánh: tau ≈ 0.075 s → b0 ≈ 13.)
// Bản cũ đặt b0 = 1/M, 1/J (đơn vị lực/mô-men), sai với u là vận tốc.
//
//   omega_c = kp: băng thông vòng điều khiển [rad/s]
//   omega_o: băng thông ESO, beta1 = 2*omega_o, beta2 = omega_o^2;
//            dt = 0.01 s nên giữ beta1*dt < 1 (omega_o < 50)
//   out_limit: lệnh tối đa = W_MAX*r = 1.676 m/s; wz: 1.676/(LX+LY) = 6.98 rad/s
// omega_c, omega_o là giá trị khởi đầu, tune bằng fwsim (hil-framework) rồi HIL.
// ─────────────────────────────────────────

// vx / vy — translational
#define ADRC_VXY_OMEGA_O    30.0f
#define ADRC_VXY_B0         5.9f
#define ADRC_VXY_KP         8.0f            // omega_c
#define ADRC_VXY_OUT_LIMIT  1.6f            // m/s

// wz — rotational
#define ADRC_WZ_OMEGA_O     30.0f
#define ADRC_WZ_B0          10.0f
#define ADRC_WZ_KP          8.0f
#define ADRC_WZ_OUT_LIMIT   6.5f            // rad/s

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
// 1. ESO nhận số đo mới cùng lệnh u của chu kỳ trước
// 2. u0 = kp * (sp - z1)          setpoint tracking
//    u  = (u0 - z2) / b0          disturbance rejection
// ESO dùng u đã clamp nên không bị windup khi bão hòa.
// ─────────────────────────────────────────
static float adrc_axis_update(adrc_axis_t *a, float sp, float actual, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // Lần đầu: khởi tạo ESO state từ actual
    if (!a->eso.initialized) {
        a->eso.z1 = actual;
        a->eso.z2 = 0.0f;
        a->u_prev = 0.0f;
        a->eso.initialized = true;
    } else {
        eso_update(&a->eso, actual, a->u_prev, dt);
    }

    float u0 = a->kp * (sp - a->eso.z1);
    float u  = (u0 - a->eso.z2) / a->eso.b0;

    // Output clamp
    if      (u >  a->output_limit) u =  a->output_limit;
    else if (u < -a->output_limit) u = -a->output_limit;

    a->u_prev = u;
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
    a->u_prev         = 0.0f;
}

void adrc_set_params(adrc_axis_t *a, float omega_c, float omega_o, float b0, float out_limit) {
    a->eso.beta1    = 2.0f * omega_o;
    a->eso.beta2    = omega_o * omega_o;
    a->eso.b0       = b0;
    a->kp           = omega_c;
    a->output_limit = out_limit;
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

    ESP_LOGI(TAG, "ADRC init done — omega_c, omega_o are starting values, tune on fwsim/HIL");
}

void adrc_reset(adrc_controller_t *c) {
    c->vx.eso.z1 = 0.0f; c->vx.eso.z2 = 0.0f; c->vx.eso.initialized = false; c->vx.u_prev = 0.0f;
    c->vy.eso.z1 = 0.0f; c->vy.eso.z2 = 0.0f; c->vy.eso.initialized = false; c->vy.u_prev = 0.0f;
    c->wz.eso.z1 = 0.0f; c->wz.eso.z2 = 0.0f; c->wz.eso.initialized = false; c->wz.u_prev = 0.0f;
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
