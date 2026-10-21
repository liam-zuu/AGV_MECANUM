#include "vel_ctrl.h"
#include "motor.h"      // PWM_MAX

static pid_controller_t  g_pid;
static adrc_controller_t g_adrc;
static int               g_mode = CTRL_MODE;

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
    g_mode = CTRL_MODE;
    counters_reset();
}

void vel_ctrl_set_mode(int mode) {
    g_mode = (mode == CTRL_ADRC) ? CTRL_ADRC : CTRL_PID;
    pid_reset(&g_pid);
    adrc_reset(&g_adrc);
    counters_reset();
}

int                vel_ctrl_mode(void) { return g_mode; }
pid_controller_t  *vel_ctrl_pid(void)  { return &g_pid; }
adrc_controller_t *vel_ctrl_adrc(void) { return &g_adrc; }

vel_ctrl_out_t vel_ctrl_step(agv_velocity_t vel, const float enc_rpm[4], float dt) {
    vel_ctrl_out_t o = {0};

    // ── 8. Forward kinematics → actual velocity ───────────────────────────
    wheel_velocity_t enc_wheels = {
        .fl = enc_rpm[0] * 2.0f * 3.14159f / 60.0f,
        .fr = enc_rpm[1] * 2.0f * 3.14159f / 60.0f,
        .rl = enc_rpm[2] * 2.0f * 3.14159f / 60.0f,
        .rr = enc_rpm[3] * 2.0f * 3.14159f / 60.0f,
    };
    o.actual = kinematics_forward(enc_wheels);

    // ── 9. Controller ─────────────────────────────────────────────────────
    if (g_mode == CTRL_ADRC) {
        o.ctrl_out.vx = adrc_update_vx(&g_adrc, vel.vx, o.actual.vx, dt);
        o.ctrl_out.vy = adrc_update_vy(&g_adrc, vel.vy, o.actual.vy, dt);
        o.ctrl_out.wz = adrc_update_wz(&g_adrc, vel.wz, o.actual.wz, dt);
    } else {
        o.ctrl_out.vx = vel.vx + pid_update_vx(&g_pid, vel.vx, o.actual.vx, dt);
        o.ctrl_out.vy = vel.vy + pid_update_vy(&g_pid, vel.vy, o.actual.vy, dt);
        o.ctrl_out.wz = vel.wz + pid_update_wz(&g_pid, vel.wz, o.actual.wz, dt);
    }

    // ── 10. Inverse kinematics → PWM ─────────────────────────────────────
    wheel_velocity_t wheels = kinematics_inverse(o.ctrl_out);
    o.pwm[0] = kinematics_radps_to_pwm(wheels.fl);
    o.pwm[1] = kinematics_radps_to_pwm(wheels.fr);
    o.pwm[2] = kinematics_radps_to_pwm(wheels.rl);
    o.pwm[3] = kinematics_radps_to_pwm(wheels.rr);

    // ── Bộ đếm metric ─────────────────────────────────────────────────────
    g_tick++;
    for (int i = 0; i < 4; i++) {
        int d = o.pwm[i] - g_pwm_prev[i];
        g_dpwm_sum[i] += (uint32_t)(d < 0 ? -d : d);
        if (o.pwm[i] >= PWM_MAX || o.pwm[i] <= -PWM_MAX) g_sat_ticks[i]++;
        g_pwm_prev[i] = o.pwm[i];
        o.dpwm_sum[i]  = g_dpwm_sum[i];
        o.sat_ticks[i] = g_sat_ticks[i];
    }
    o.tick = g_tick;

    return o;
}
