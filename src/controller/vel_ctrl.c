#include "vel_ctrl.h"
#include "motor.h"      // PWM_MAX
#include <math.h>

static still_params_t    g_still_p = STILL_PARAMS_DEFAULT;
static bool              g_still;           // (d) logic dừng đang giữ
static int64_t           g_still_since = -1;

static pid_controller_t  g_pid;
static adrc_controller_t g_adrc;
static int               g_mode = CTRL_MODE;
static bool              g_mode_event;      // đổi mode chờ báo cho vòng hướng (§5 ưu tiên 1)
static bool              g_pwm_zero = true; // PWM thực áp = 0 cả 4 bánh ở tick trước (điều kiện đứng yên)
static bool              g_motor_prev;      // motor_on ở tick trước (cạnh tắt → bật: reset bộ điều khiển)

// Bộ đếm metric (xem vel_ctrl.h)
static uint32_t g_tick;
static uint32_t g_dpwm_sum[4];
static uint32_t g_sat_ticks[4];
static int      g_pwm_prev[4];

static void counters_reset(void) {
    g_tick = 0;
    for (int i = 0; i < 4; i++) { g_dpwm_sum[i] = 0; g_sat_ticks[i] = 0; g_pwm_prev[i] = 0; }
}

void vel_ctrl_init(void) {
    pid_init(&g_pid);
    adrc_init(&g_adrc);
    imu_fusion_init(NULL);
    heading_init(NULL);
    pos_init(NULL);
    g_still_p = (still_params_t)STILL_PARAMS_DEFAULT;
    g_still = false;
    g_still_since = -1;
    wheel_slip_init(NULL);
    g_mode = CTRL_MODE;
    g_mode_event = false;
    g_pwm_zero = true;
    g_motor_prev = false;
    counters_reset();
}

void vel_ctrl_set_mode(int mode) {
    g_mode = (mode == CTRL_ADRC) ? CTRL_ADRC : CTRL_PID;
    pid_reset(&g_pid);
    adrc_reset(&g_adrc);
    g_mode_event = true;
    counters_reset();
}

still_params_t    *vel_ctrl_still_params(void) { return &g_still_p; }
int                vel_ctrl_mode(void) { return g_mode; }
pid_controller_t  *vel_ctrl_pid(void)  { return &g_pid; }
adrc_controller_t *vel_ctrl_adrc(void) { return &g_adrc; }

vel_ctrl_out_t vel_ctrl_step(const vel_ctrl_in_t *in) {
    vel_ctrl_out_t o = {0};
    agv_velocity_t vel = in->sp;
    float dt = in->dt;

    // ── 8. Forward kinematics → actual velocity ───────────────────────────
    wheel_velocity_t enc_wheels = {
        .fl = in->enc_rpm[0] * 2.0f * 3.14159f / 60.0f,
        .fr = in->enc_rpm[1] * 2.0f * 3.14159f / 60.0f,
        .rl = in->enc_rpm[2] * 2.0f * 3.14159f / 60.0f,
        .rr = in->enc_rpm[3] * 2.0f * 3.14159f / 60.0f,
    };
    o.actual = kinematics_forward(enc_wheels);

    // ── 8b. Hợp nhất IMU: nguồn wz, hợp lệ, cờ trượt, bias (imu_fusion.c) ──
    imu_fusion_in_t fi = {
        .now_us        = in->now_us,
        .imu           = in->imu,
        .wz_enc        = o.actual.wz,
        .omega         = { enc_wheels.fl, enc_wheels.fr, enc_wheels.rl, enc_wheels.rr },
        .sp_zero       = vel.vx == 0.0f && vel.vy == 0.0f && vel.wz == 0.0f,
        .drive_off     = !in->motor_on || g_pwm_zero,
        .wheels_lifted = in->wheels_lifted,
    };
    for (int i = 0; i < 4; i++) fi.enc_count[i] = in->enc_count[i];
    imu_fusion_step(&fi, &o.imu);

    // ── 8b'. Cờ trượt không đều giữa các bánh (wheel_slip.c, chỉ ghi log) ──
    wheel_slip_in_t wi = { .now_us = in->now_us };
    for (int i = 0; i < 4; i++) wi.count[i] = in->enc_count[i];
    wheel_slip_step(&wi, &o.wslip);

    // ── 8c. Vòng giữ hướng (heading.c) ───────────────────────────────────
    heading_in_t hi = {
        .wz_ref      = vel.wz,
        .dt          = dt,
        .run_active  = in->run_active,
        .yaw_valid   = o.imu.r_valid,
        .yaw         = o.imu.yaw,
        .reset_event = o.imu.reset_event || g_mode_event,
    };
    g_mode_event = false;
    heading_step(&hi, &o.head);
    o.wz_cmd = o.head.wz_cmd;

    // ── 8c'. (d) Logic dừng (Liam 04/11 "đồng ý làm a+d"; mặc định BẬT) ───
    // Lệnh = 0 và |vx|, |vy| < v_still, |wz| < v_still/(LX+LY) liên tục t_still → giữ: PWM 0,
    // reset PID/LADRC mỗi tick, vòng vị trí không hiệu chỉnh. Thoát khi lệnh khác 0 hoặc motor tắt;
    // đang giữ thì xe có trôi nhanh hơn ngưỡng cũng vẫn giữ (lệnh vẫn = 0).
    {
        bool sp0 = vel.vx == 0.0f && vel.vy == 0.0f && vel.wz == 0.0f;
        float vs = g_still_p.v_still, ws = vs / (LX + LY);
        bool slow = fabsf(o.actual.vx) < vs && fabsf(o.actual.vy) < vs && fabsf(o.actual.wz) < ws;
        if (!g_still_p.on || !in->motor_on || !sp0) {
            g_still = false;
            g_still_since = -1;
        } else if (!g_still) {
            if (!slow) g_still_since = -1;
            else if (g_still_since < 0) g_still_since = in->now_us;
            if (slow && in->now_us - g_still_since >= g_still_p.t_still_us) g_still = true;
        }
        o.still = g_still;
    }

    // ── 8d. Vòng vị trí x, y (pos_ctrl.c, O1 PA3; mặc định BẬT) ───────────
    pos_in_t pi = {
        .vx_ref = vel.vx, .vy_ref = vel.vy, .wz_ref = vel.wz,
        .vx_enc = o.actual.vx, .vy_enc = o.actual.vy, .wz_enc = o.actual.wz,
        .psi_imu = o.head.psi_imu, .head_on = o.head.on,
        .dt = dt, .run_active = in->run_active, .hold = g_still,
        .psi0_n = o.head.psi0_n,
        .block_fwd = in->block_fwd, .block_bwd = in->block_bwd,
        .block_left = in->block_left, .block_right = in->block_right,
    };
    pos_step(&pi, &o.pos);
    vel.vx += o.pos.corr_vx;
    vel.vy += o.pos.corr_vy;
    // Cắt vật cản lần nữa sau hiệu chỉnh (Codex P1, 05/11): interlock ở main.c chỉ thấy lệnh gốc
    if (in->block_fwd   && vel.vx > 0.0f) vel.vx = 0.0f;
    if (in->block_bwd   && vel.vx < 0.0f) vel.vx = 0.0f;
    if (in->block_left  && vel.vy > 0.0f) vel.vy = 0.0f;
    if (in->block_right && vel.vy < 0.0f) vel.vy = 0.0f;
    o.v_cmd = (agv_velocity_t){ .vx = vel.vx, .vy = vel.vy, .wz = o.wz_cmd };
    o.run_active = in->run_active;

    // Bộ đếm metric hiện tại; khi motor bật, vel_ctrl_applied() cập nhật sau khi biết PWM thực áp
    o.tick = g_tick;
    for (int i = 0; i < 4; i++) { o.dpwm_sum[i] = g_dpwm_sum[i]; o.sat_ticks[i] = g_sat_ticks[i]; }

    if (!in->motor_on) {
        g_pwm_zero = true;
        g_motor_prev = false;
        return o;
    }

    // Motor tắt → bật (SWA, hết lỗi): bỏ tích phân PID, trạng thái ESO và u_prev cũ (Codex
    // ctrl-imu #1, Liam duyệt 28/10). Lúc motor tắt PWM thực áp là 0, nên bước PWM đầu tính từ 0.
    if (!g_motor_prev) {
        pid_reset(&g_pid);
        adrc_reset(&g_adrc);
        for (int i = 0; i < 4; i++) g_pwm_prev[i] = 0;
    }
    g_motor_prev = true;

    // ── 9. Controller ─────────────────────────────────────────────────────
    float wz_meas = o.imu.wz_meas;
    if (g_mode == CTRL_ADRC) {
        o.ctrl_out.vx = adrc_update_vx(&g_adrc, vel.vx, o.actual.vx, dt);
        o.ctrl_out.vy = adrc_update_vy(&g_adrc, vel.vy, o.actual.vy, dt);
        o.ctrl_out.wz = adrc_update_wz(&g_adrc, o.wz_cmd, wz_meas, dt);
    } else {
        o.ctrl_out.vx = vel.vx + pid_update_vx(&g_pid, vel.vx, o.actual.vx, dt);
        o.ctrl_out.vy = vel.vy + pid_update_vy(&g_pid, vel.vy, o.actual.vy, dt);
        o.ctrl_out.wz = o.wz_cmd + pid_update_wz(&g_pid, o.wz_cmd, wz_meas, dt);
    }
    // Đầu ra vòng trong không đi vào hướng bị chặn (Codex vòng 2 P1): tích phân PID hay ESO còn
    // lại có thể đẩy xe về phía vật cản khi lệnh đã về 0; hard cutoff ở main.c chỉ xét intent
    // và v_cmd. Kẹp từng trục (không cắt cả 4 PWM) để vẫn đi ngang được khi trước bị chặn.
    // ESO nhận u đã kẹp, như khi bão hòa.
    if ((in->block_fwd && o.ctrl_out.vx > 0.0f) || (in->block_bwd && o.ctrl_out.vx < 0.0f)) {
        o.ctrl_out.vx = 0.0f;
        if (g_mode == CTRL_ADRC) g_adrc.vx.u_prev = 0.0f;
    }
    if ((in->block_left && o.ctrl_out.vy > 0.0f) || (in->block_right && o.ctrl_out.vy < 0.0f)) {
        o.ctrl_out.vy = 0.0f;
        if (g_mode == CTRL_ADRC) g_adrc.vy.u_prev = 0.0f;
    }

    // ── 10. Inverse kinematics → PWM ─────────────────────────────────────
    wheel_velocity_t wheels = kinematics_inverse(o.ctrl_out);
    o.pwm[0] = kinematics_radps_to_pwm(wheels.fl);
    o.pwm[1] = kinematics_radps_to_pwm(wheels.fr);
    o.pwm[2] = kinematics_radps_to_pwm(wheels.rl);
    o.pwm[3] = kinematics_radps_to_pwm(wheels.rr);

    // (d) Logic dừng đang giữ: PWM 0, bỏ trạng thái bộ điều khiển (tích phân PID, ESO) để không
    // tích lũy trong lúc đứng; lần chạy sau bắt đầu sạch.
    if (g_still) {
        pid_reset(&g_pid);
        adrc_reset(&g_adrc);
        o.ctrl_out = (agv_velocity_t){0};
        for (int i = 0; i < 4; i++) o.pwm[i] = 0;
    }

    return o;
}

// PWM thực áp sau mọi lớp cắt của main.c (vật cản), gọi mỗi tick motor bật, ngay sau
// motor_set(). Điều kiện đứng yên và bộ đếm metric tính theo PWM này, không theo lệnh
// (Codex ctrl-imu #12, Liam duyệt 28/10).
void vel_ctrl_applied(const int pwm[4], vel_ctrl_out_t *o) {
    g_pwm_zero = pwm[0] == 0 && pwm[1] == 0 && pwm[2] == 0 && pwm[3] == 0;
    g_tick++;
    for (int i = 0; i < 4; i++) {
        int d = pwm[i] - g_pwm_prev[i];
        g_dpwm_sum[i] += (uint32_t)(d < 0 ? -d : d);
        if (pwm[i] >= PWM_MAX || pwm[i] <= -PWM_MAX) g_sat_ticks[i]++;
        g_pwm_prev[i] = pwm[i];
        o->dpwm_sum[i]  = g_dpwm_sum[i];
        o->sat_ticks[i] = g_sat_ticks[i];
    }
    o->tick = g_tick;
}
