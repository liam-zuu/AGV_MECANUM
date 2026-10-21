#ifndef VEL_CTRL_H
#define VEL_CTRL_H

#include "kinematics.h"
#include "pid.h"
#include "adrc.h"
#include <stdint.h>

// ─────────────────────────────────────────
// Vòng vận tốc thân — bước 8–10 của control_task (main.c):
//   encoder rpm → FK → bộ điều khiển (vx, vy, wz) → IK → PWM
// Tách khỏi main.c (21/10/2026) để mô phỏng trên PC (hil-framework fwsim/)
// biên dịch và chạy đúng mã này. File không dùng API ESP-IDF/FreeRTOS.
//
//   CTRL_PID : lệnh = setpoint + correction PID (như trước 21/10)
//   CTRL_ADRC: lệnh = đầu ra LADRC (lệnh vận tốc toàn phần, adrc.c)
// ─────────────────────────────────────────

#define CTRL_PID   0
#define CTRL_ADRC  1
#define CTRL_MODE  CTRL_PID   // ← đổi ở đây (chế độ lúc khởi động)

typedef struct {
    agv_velocity_t actual;     // vận tốc thân tính từ encoder (FK)
    agv_velocity_t ctrl_out;   // lệnh vận tốc thân, trước IK
    int            pwm[4];     // FL, FR, RL, RR (-PWM_MAX..PWM_MAX)

    // Bộ đếm cộng dồn cho metric (spec S02): log mất mẫu vẫn tính đúng,
    // khoảng trống trong tick cho biết số mẫu mất. Reset ở init / set_mode.
    uint32_t       tick;          // số lần gọi vel_ctrl_step
    uint32_t       dpwm_sum[4];   // Σ|pwm_k − pwm_{k−1}| (đơn vị pwm, chia PWM_MAX ra u)
    uint32_t       sat_ticks[4];  // số tick có |pwm| ≥ PWM_MAX
} vel_ctrl_out_t;

void               vel_ctrl_init(void);              // khởi tạo cả PID và ADRC, chế độ = CTRL_MODE
void               vel_ctrl_set_mode(int mode);      // CTRL_PID / CTRL_ADRC; reset trạng thái bộ điều khiển
int                vel_ctrl_mode(void);
pid_controller_t  *vel_ctrl_pid(void);
adrc_controller_t *vel_ctrl_adrc(void);

// sp: setpoint thân; enc_rpm: FL, FR, RL, RR [rpm]; dt: chu kỳ thực [s]
vel_ctrl_out_t     vel_ctrl_step(agv_velocity_t sp, const float enc_rpm[4], float dt);

#endif // VEL_CTRL_H
