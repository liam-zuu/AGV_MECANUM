#include "kinematics.h"
#include "motor.h"
#include <math.h>

// w_max từ thesis: 34.56 rad/s
#define W_MAX   34.56f

wheel_velocity_t kinematics_inverse(agv_velocity_t vel) {
    wheel_velocity_t w;
    float k = 1.0f / WHEEL_RADIUS;
    float lxy = LX + LY;

    w.fl = k * (vel.vx - vel.vy - lxy * vel.wz);
    w.fr = k * (vel.vx + vel.vy + lxy * vel.wz);
    w.rl = k * (vel.vx + vel.vy - lxy * vel.wz);
    w.rr = k * (vel.vx - vel.vy + lxy * vel.wz);

    return w;
}

agv_velocity_t kinematics_forward(wheel_velocity_t w) {
    agv_velocity_t vel;
    float r4 = WHEEL_RADIUS / 4.0f;
    float lxy = LX + LY;

    vel.vx = r4 * (w.fl + w.fr + w.rl + w.rr);
    vel.vy = r4 * (-w.fl + w.fr + w.rl - w.rr);
    vel.wz = r4 / lxy * (-w.fl + w.fr - w.rl + w.rr);

    return vel;
}

int kinematics_radps_to_pwm(float radps) {
    int pwm = (int)(radps / W_MAX * PWM_MAX);
    if (pwm >  PWM_MAX) pwm =  PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;
    return pwm;
}