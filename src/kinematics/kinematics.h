#ifndef KINEMATICS_H
#define KINEMATICS_H

// AGV params (đã confirm)
#define WHEEL_RADIUS    0.0485f   // m
#define LX              0.12f     // m, half wheelbase
#define LY              0.12f     // m, half track

typedef struct {
    float vx;    // m/s, tiến/lùi
    float vy;    // m/s, trái/phải (strafing)
    float wz;    // rad/s, quay
} agv_velocity_t;

typedef struct {
    float fl;    // rad/s
    float fr;
    float rl;
    float rr;
} wheel_velocity_t;

// Inverse kinematics: velocity → wheel speed
wheel_velocity_t kinematics_inverse(agv_velocity_t vel);

// Forward kinematics: wheel speed → velocity
agv_velocity_t kinematics_forward(wheel_velocity_t wheels);

// Convert rad/s → PWM (-1023 đến +1023)
int kinematics_radps_to_pwm(float radps);

#endif