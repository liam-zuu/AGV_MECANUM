#include "pos_ctrl.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265f

static pos_params_t P = POS_PARAMS_DEFAULT;

static struct {
    bool     running;
    bool     imu_prev;     // tick trước θ_est lấy từ ψ_imu
    bool     off_set;      // θ_off đã đặt cho hệ ψ_imu hiện tại
    uint32_t psi0_n;       // psi0_n của heading lúc đặt θ_off
    float    th_off;       // θ_est = ψ_imu + th_off
    float    x_ref, y_ref, th_ref;
    float    x, y, th;
    float    ix, iy;       // tích phân sai số hệ thân [m·s]
    bool     in_pos;       // (a) trong cửa sổ "đã tới vị trí"
} S;
static uint32_t s_reset_n;  // số lần đặt lại tư thế; không nằm trong S để không bị xóa lúc bắt đầu chạy

static float wrap_pi(float a) {
    if (!isfinite(a)) return 0.0f;   // ±Inf không thoát được vòng while (Codex P18, 05/11)
    // Số hữu hạn lớn: float32 quanh 1e10 có a − 2π == a, vòng while không thoát (Codex vòng 2 P18).
    // Chỉ dùng remainderf khi |a| lớn để giá trị thường giữ nguyên từng bit như trước.
    if (fabsf(a) > 4.0f * PI_F) a = remainderf(a, 2.0f * PI_F);
    while (a >  PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

void pos_init(const pos_params_t *p) {
    if (p) P = *p;
    else   P = (pos_params_t)POS_PARAMS_DEFAULT;
    memset(&S, 0, sizeof(S));
    s_reset_n = 0;
}

pos_params_t *pos_params(void) { return &P; }

static float sat(float v, float m) { return v > m ? m : (v < -m ? -m : v); }
static int   sgn(float v) { return (v > 0.0f) - (v < 0.0f); }

// Một trục PI có anti-windup như M6: tích phân mới chỉ nhận khi đầu ra thử chưa bão hòa
// hoặc sai số khác dấu với tích phân cũ (sign(e) ~= sign(int) của M6).
// Hướng dương/âm của trục bị chặn (vật cản, Codex P1): đầu ra về phía đó = 0 và tích phân
// không lớn thêm (chỉ được xả về 0).
static float axis(float e, float *integ, float dt, bool block_pos, bool block_neg) {
    float i_old = *integ;
    float i_new = i_old + e * dt;
    float u = P.kp * e + P.ki * i_new;
    if (fabsf(u) < P.v_corr_max || sgn(e) != sgn(i_old)) *integ = i_new;
    float out = sat(P.kp * e + P.ki * *integ, P.v_corr_max);
    if ((out > 0.0f && block_pos) || (out < 0.0f && block_neg)) {
        // Không cho tích phân lớn thêm về phía bị chặn, nhưng vẫn cho xả (|i_new| < |i_old|), để
        // lúc hết chặn không còn lệnh Ki·∫e cũ (Codex vòng 2 P1)
        if (fabsf(*integ) > fabsf(i_old)) *integ = i_old;
        out = 0.0f;
    }
    return out;
}

void pos_step(const pos_in_t *in, pos_out_t *out) {
    memset(out, 0, sizeof(*out));
    if (!in->run_active) {
        S.running = false;
    } else {
        if (!S.running) {                                  // bắt đầu chạy: mọi thứ về 0
            memset(&S, 0, sizeof(S));
            S.running = true;
            s_reset_n++;
        }
        float dt = in->dt;

        // Tư thế lệnh (A1). Xoay theo góc giữa tick, cùng quy ước với tư thế ước lượng (Codex P7)
        float th_mid = S.th_ref + 0.5f * in->wz_ref * dt;
        float c = cosf(th_mid), s = sinf(th_mid);
        S.x_ref += (in->vx_ref * c - in->vy_ref * s) * dt;
        S.y_ref += (in->vx_ref * s + in->vy_ref * c) * dt;
        S.th_ref = wrap_pi(S.th_ref + in->wz_ref * dt);

        // Hướng ước lượng. θ_off chỉ đặt lại khi hệ ψ_imu đổi (heading đặt ψ0 mới: bắt đầu chạy,
        // reset chip, đổi mode) để θ liên tục; yaw mất rồi có lại trong cùng hệ thì giữ θ_off cũ,
        // θ quay về theo IMU, không giữ sai số encoder của quãng mất yaw (Codex P2, 05/11).
        float th_old = S.th;
        if (in->head_on) {
            if (!S.off_set || in->psi0_n != S.psi0_n) {
                S.th_off  = wrap_pi(S.th - in->psi_imu);
                S.psi0_n  = in->psi0_n;
                S.off_set = true;
            }
            S.th = wrap_pi(in->psi_imu + S.th_off);
        } else {
            S.th = wrap_pi(S.th + in->wz_enc * dt);
        }
        S.imu_prev = in->head_on;

        // Vị trí ước lượng: góc giữa tick (Codex P7)
        float th_m = wrap_pi(th_old + 0.5f * wrap_pi(S.th - th_old));
        c = cosf(th_m); s = sinf(th_m);
        S.x += (in->vx_enc * c - in->vy_enc * s) * dt;
        S.y += (in->vx_enc * s + in->vy_enc * c) * dt;

        // Sai số hệ thân (theo hướng hiện tại), PI
        c = cosf(S.th); s = sinf(S.th);
        float ex = S.x_ref - S.x, ey = S.y_ref - S.y;
        out->ex_b =  c * ex + s * ey;
        out->ey_b = -s * ex + c * ey;

        // (a) Cửa sổ "đã tới vị trí" (Liam 04/11 "đồng ý làm a+d"): chỉ khi lệnh vận tốc = 0.
        // Vào khi |e| < e_win, ra khi |e| > e_exit hoặc có lệnh khác 0. e_win ≤ 0: tắt cửa sổ.
        bool cmd_zero = in->vx_ref == 0.0f && in->vy_ref == 0.0f && in->wz_ref == 0.0f;
        float e = sqrtf(ex * ex + ey * ey);
        if (!cmd_zero || P.e_win <= 0.0f) S.in_pos = false;
        else if (e < P.e_win)             S.in_pos = true;
        else if (e > P.e_exit)            S.in_pos = false;
        out->in_pos = S.in_pos;

        // Trong cửa sổ (a) hoặc khi (d) giữ: không hiệu chỉnh, xóa tích phân, để lúc ra không
        // nhảy lệnh Ki·∫e (Codex P5, 05/11)
        if (S.in_pos || in->hold) {
            S.ix = 0.0f;
            S.iy = 0.0f;
        } else if (P.on) {
            out->on = true;
            out->corr_vx = axis(out->ex_b, &S.ix, dt, in->block_fwd,  in->block_bwd);
            out->corr_vy = axis(out->ey_b, &S.iy, dt, in->block_left, in->block_right);
        }
    }
    out->x_ref = S.x_ref; out->y_ref = S.y_ref; out->th_ref = S.th_ref;
    out->x = S.x; out->y = S.y; out->th = S.th;
    out->th_src = S.imu_prev ? 1u : 0u;
    out->reset_n = s_reset_n;
}
