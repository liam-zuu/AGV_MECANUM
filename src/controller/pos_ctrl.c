#include "pos_ctrl.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265f

static pos_params_t P = POS_PARAMS_DEFAULT;

static struct {
    bool  running;
    bool  imu_prev;      // tick trước θ_est lấy từ ψ_imu
    float th_off;        // θ_est = ψ_imu + th_off
    float x_ref, y_ref, th_ref;
    float x, y, th;
    float ix, iy;        // tích phân sai số hệ thân [m·s]
    bool  in_pos;        // (a) trong cửa sổ "đã tới vị trí"
} S;

static float wrap_pi(float a) {
    while (a >  PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

void pos_init(const pos_params_t *p) {
    if (p) P = *p;
    else   P = (pos_params_t)POS_PARAMS_DEFAULT;
    memset(&S, 0, sizeof(S));
}

pos_params_t *pos_params(void) { return &P; }

static float sat(float v, float m) { return v > m ? m : (v < -m ? -m : v); }
static int   sgn(float v) { return (v > 0.0f) - (v < 0.0f); }

// Một trục PI có anti-windup như M6: tích phân mới chỉ nhận khi đầu ra thử chưa bão hòa
// hoặc sai số khác dấu với tích phân cũ (sign(e) ~= sign(int) của M6).
static float axis(float e, float *integ, float dt) {
    float i_new = *integ + e * dt;
    float u = P.kp * e + P.ki * i_new;
    if (fabsf(u) < P.v_corr_max || sgn(e) != sgn(*integ)) *integ = i_new;
    return sat(P.kp * e + P.ki * *integ, P.v_corr_max);
}

void pos_step(const pos_in_t *in, pos_out_t *out) {
    memset(out, 0, sizeof(*out));
    if (!in->run_active) {
        S.running = false;
    } else {
        if (!S.running) {                                  // bắt đầu chạy: mọi thứ về 0
            memset(&S, 0, sizeof(S));
            S.running = true;
        }
        float dt = in->dt;

        // Tư thế lệnh (A1)
        float c = cosf(S.th_ref), s = sinf(S.th_ref);
        S.x_ref += (in->vx_ref * c - in->vy_ref * s) * dt;
        S.y_ref += (in->vx_ref * s + in->vy_ref * c) * dt;
        S.th_ref = wrap_pi(S.th_ref + in->wz_ref * dt);

        // Hướng ước lượng
        if (in->head_on) {
            if (!S.imu_prev) S.th_off = wrap_pi(S.th - in->psi_imu);  // liên tục khi chuyển nguồn
            S.th = wrap_pi(in->psi_imu + S.th_off);
        } else {
            S.th = wrap_pi(S.th + in->wz_enc * dt);
        }
        S.imu_prev = in->head_on;

        // Vị trí ước lượng
        c = cosf(S.th); s = sinf(S.th);
        S.x += (in->vx_enc * c - in->vy_enc * s) * dt;
        S.y += (in->vx_enc * s + in->vy_enc * c) * dt;

        // Sai số hệ thân, PI
        float ex = S.x_ref - S.x, ey = S.y_ref - S.y;
        out->ex_b =  c * ex + s * ey;
        out->ey_b = -s * ex + c * ey;

        // (a) Cửa sổ "đã tới vị trí" (Liam 04/11 "đồng ý làm a+d"): chỉ khi lệnh vận tốc = 0.
        // Vào khi |e| < e_win, ra khi |e| > e_exit hoặc có lệnh khác 0; trong cửa sổ không hiệu
        // chỉnh, tích phân giữ nguyên. e_win ≤ 0: tắt cửa sổ.
        bool cmd_zero = in->vx_ref == 0.0f && in->vy_ref == 0.0f && in->wz_ref == 0.0f;
        float e = sqrtf(ex * ex + ey * ey);
        if (!cmd_zero || P.e_win <= 0.0f) S.in_pos = false;
        else if (e < P.e_win)             S.in_pos = true;
        else if (e > P.e_exit)            S.in_pos = false;
        out->in_pos = S.in_pos;

        if (P.on && !S.in_pos && !in->hold) {
            out->on = true;
            out->corr_vx = axis(out->ex_b, &S.ix, dt);
            out->corr_vy = axis(out->ey_b, &S.iy, dt);
        }
    }
    out->x_ref = S.x_ref; out->y_ref = S.y_ref; out->th_ref = S.th_ref;
    out->x = S.x; out->y = S.y; out->th = S.th;
    out->th_src = S.imu_prev ? 1u : 0u;
}
