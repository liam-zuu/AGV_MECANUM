#include "heading.h"
#include <string.h>

#define PI_F 3.14159265f

static heading_params_t P = HEADING_PARAMS_DEFAULT;

static struct {
    bool     running;     // đã thấy run_active ở tick trước
    bool     on;
    bool     need_psi0;   // chờ yaw hợp lệ để đặt ψ0 mới (sau reset chip / đổi mode, hoặc lúc bắt đầu chạy)
    float    psi0;
    float    psi_ref;
    float    psi_imu;
    uint32_t psi_reset_count;
    uint32_t toggle_count;
} H;

static float wrap_pi(float a) {
    while (a >  PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

void heading_init(const heading_params_t *p) {
    if (p) P = *p;
    else   P = (heading_params_t)HEADING_PARAMS_DEFAULT;
    memset(&H, 0, sizeof(H));
}

heading_params_t *heading_params(void) { return &P; }

static void set_psi0(float yaw) {
    H.psi0      = yaw;
    H.psi_imu   = 0.0f;
    H.psi_ref   = 0.0f;
    H.need_psi0 = false;
    H.on        = true;
}

void heading_step(const heading_in_t *in, heading_out_t *out) {
    bool was_on = H.on;
    if (in->yaw_valid) H.psi_imu = wrap_pi(in->yaw - H.psi0);

    if (in->reset_event) {
        // 1. Chip reset / đổi mode: tắt, đặt ψ0 mới khi yaw hợp lệ lại
        H.on = false;
        if (H.running) {
            H.need_psi0 = true;
            H.psi_reset_count++;
        }
    } else if (!in->run_active) {
        // 2. STOP / motor disable / quỹ đạo hết VALID: tắt, ψ_ref bám ψ_imu
        H.on = false;
        H.running = false;
        H.need_psi0 = false;
        if (in->yaw_valid) H.psi_ref = H.psi_imu;
    } else if (!H.running) {
        // Bắt đầu chạy: ψ0 = yaw hiện tại, ψ_ref = 0
        H.running = true;
        if (in->yaw_valid) set_psi0(in->yaw);
        else { H.need_psi0 = true; H.on = false; H.psi_ref = 0.0f; }
    } else if (H.need_psi0) {
        if (in->yaw_valid) set_psi0(in->yaw);
        else H.on = false;
    } else {
        H.psi_ref = wrap_pi(H.psi_ref + in->wz_ref * in->dt);
        if (!in->yaw_valid) {
            H.on = false;                                    // 3. yaw mất: ψ_ref vẫn tích phân
        } else if (!H.on) {
            float e = wrap_pi(H.psi_ref - H.psi_imu);
            if (e > P.e_psi_reset || e < -P.e_psi_reset) {   // 4. lệch lớn: đặt lại ψ_ref
                H.psi_ref = H.psi_imu;
                H.psi_reset_count++;
            }
            H.on = true;                                     // 4–5. bật lại
        }
    }

    float e = 0.0f, corr = 0.0f;
    if (H.running && !H.need_psi0 && in->yaw_valid) e = wrap_pi(H.psi_ref - H.psi_imu);
    if (H.on) {
        corr = P.k_psi * e;
        if (corr >  P.wz_corr_max) corr =  P.wz_corr_max;
        if (corr < -P.wz_corr_max) corr = -P.wz_corr_max;
    }
    if (H.on != was_on) H.toggle_count++;

    out->wz_cmd          = in->wz_ref + corr;
    out->on              = H.on;
    out->psi_ref         = H.psi_ref;
    out->psi_imu         = H.psi_imu;
    out->e_psi           = e;
    out->psi_reset_count = H.psi_reset_count;
    out->toggle_count    = H.toggle_count;
}
