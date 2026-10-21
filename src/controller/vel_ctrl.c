#include "vel_ctrl.h"

static pid_controller_t g_pid;

void vel_ctrl_init(void) {
#if CTRL_MODE == CTRL_PID
    pid_init(&g_pid);
#endif
}

pid_controller_t *vel_ctrl_pid(void) { return &g_pid; }

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
    o.ctrl_out.vx = vel.vx + pid_update_vx(&g_pid, vel.vx, o.actual.vx, dt);
    o.ctrl_out.vy = vel.vy + pid_update_vy(&g_pid, vel.vy, o.actual.vy, dt);
    o.ctrl_out.wz = vel.wz + pid_update_wz(&g_pid, vel.wz, o.actual.wz, dt);

    // ── 10. Inverse kinematics → PWM ─────────────────────────────────────
    wheel_velocity_t wheels = kinematics_inverse(o.ctrl_out);
    o.pwm[0] = kinematics_radps_to_pwm(wheels.fl);
    o.pwm[1] = kinematics_radps_to_pwm(wheels.fr);
    o.pwm[2] = kinematics_radps_to_pwm(wheels.rl);
    o.pwm[3] = kinematics_radps_to_pwm(wheels.rr);

    return o;
}
