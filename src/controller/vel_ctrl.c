#include "vel_ctrl.h"
#include "motor.h"      // PWM_MAX

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

    // ── 10. Inverse kinematics → PWM ─────────────────────────────────────
    wheel_velocity_t wheels = kinematics_inverse(o.ctrl_out);
    o.pwm[0] = kinematics_radps_to_pwm(wheels.fl);
    o.pwm[1] = kinematics_radps_to_pwm(wheels.fr);
    o.pwm[2] = kinematics_radps_to_pwm(wheels.rl);
    o.pwm[3] = kinematics_radps_to_pwm(wheels.rr);

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
