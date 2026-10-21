#ifndef VEL_CTRL_H
#define VEL_CTRL_H

#include "kinematics.h"
#include "pid.h"

// ─────────────────────────────────────────
// Vòng vận tốc thân — bước 8–10 của control_task (main.c):
//   encoder rpm → FK → bộ điều khiển (vx, vy, wz) → IK → PWM
// Tách khỏi main.c (21/10/2026) để mô phỏng trên PC (hil-framework fwsim/)
// biên dịch và chạy đúng mã này. File không dùng API ESP-IDF/FreeRTOS.
// ─────────────────────────────────────────

#define CTRL_PID   0
#define CTRL_ADRC  1
#define CTRL_MODE  CTRL_PID   // ← đổi ở đây

typedef struct {
    agv_velocity_t actual;     // vận tốc thân tính từ encoder (FK)
    agv_velocity_t ctrl_out;   // setpoint + correction, trước IK
    int            pwm[4];     // FL, FR, RL, RR (-PWM_MAX..PWM_MAX)
} vel_ctrl_out_t;

void              vel_ctrl_init(void);
pid_controller_t *vel_ctrl_pid(void);

// sp: setpoint thân; enc_rpm: FL, FR, RL, RR [rpm]; dt: chu kỳ thực [s]
vel_ctrl_out_t    vel_ctrl_step(agv_velocity_t sp, const float enc_rpm[4], float dt);

#endif // VEL_CTRL_H
