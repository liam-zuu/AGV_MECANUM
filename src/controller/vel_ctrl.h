#ifndef VEL_CTRL_H
#define VEL_CTRL_H

#include "kinematics.h"
#include "pid.h"
#include "adrc.h"
#include "imu_fusion.h"
#include "heading.h"
#include <stdint.h>

// ─────────────────────────────────────────
// Vòng vận tốc thân — bước 8–10 của control_task (main.c):
//   encoder rpm → FK → hợp nhất IMU (wz) + vòng giữ hướng → bộ điều khiển (vx, vy, wz) → IK → PWM
// Tách khỏi main.c (21/10/2026) để mô phỏng trên PC (hil-framework fwsim/)
// biên dịch và chạy đúng mã này. File không dùng API ESP-IDF/FreeRTOS.
//
//   CTRL_PID : lệnh = setpoint + correction PID (như trước 21/10)
//   CTRL_ADRC: lệnh = đầu ra LADRC (lệnh vận tốc toàn phần, adrc.c)
//
// IMU (spec controller–IMU v2.2, 28/10): wz đo lấy từ gyro khi hợp lệ, không thì từ encoder
// (imu_fusion.c); setpoint wz đi qua vòng giữ hướng (heading.c); vx, vy từ encoder.
// Gọi MỖI tick, kể cả khi motor tắt (để theo dõi hợp lệ IMU, bias khi đứng yên, ψ_ref):
// khi motor_on = false bộ điều khiển không chạy, pwm = 0, bộ đếm metric không đổi.
// Cạnh motor tắt → bật: reset PID/LADRC (tích phân, ESO, u_prev). Khi motor bật, người gọi
// áp PWM (sau lớp cắt vật cản) rồi gọi vel_ctrl_applied() với PWM thực áp: điều kiện đứng
// yên tick sau và bộ đếm metric tính theo PWM đó.
// ─────────────────────────────────────────

#define CTRL_PID   0
#define CTRL_ADRC  1
#define CTRL_MODE  CTRL_PID   // ← đổi ở đây (chế độ lúc khởi động)

typedef struct {
    agv_velocity_t    sp;             // setpoint thân (sau interlock)
    float             enc_rpm[4];     // FL, FR, RL, RR [rpm]
    int32_t           enc_count[4];   // encoder_get_count() (điều kiện đứng yên)
    const imu_data_t *imu;            // bản sao imu_get() mới nhất; NULL = chưa có
    int64_t           now_us;
    float             dt;             // chu kỳ thực [s]
    bool              motor_on;       // driver enable (SWA, hoặc hil_bench)
    bool              run_active;     // motor_on và đang chạy theo quỹ đạo VALID (MODE_AUTO)
    bool              wheels_lifted;  // bài chạy kê bánh: tắt ước lượng bias
} vel_ctrl_in_t;

typedef struct {
    agv_velocity_t actual;     // vận tốc thân tính từ encoder (FK)
    agv_velocity_t ctrl_out;   // lệnh vận tốc thân, trước IK
    int            pwm[4];     // FL, FR, RL, RR (-PWM_MAX..PWM_MAX)
    float          wz_cmd;     // setpoint wz sau vòng giữ hướng
    imu_fusion_out_t imu;      // hợp lệ, nguồn wz, cờ trượt, bias (cột log §8)
    heading_out_t    head;     // vòng giữ hướng (cột log §8)

    // Bộ đếm cộng dồn cho metric (spec S02): log mất mẫu vẫn tính đúng,
    // khoảng trống trong tick cho biết số mẫu mất. Reset ở init / set_mode.
    uint32_t       tick;          // số tick bộ điều khiển chạy (motor_on)
    uint32_t       dpwm_sum[4];   // Σ|pwm_k − pwm_{k−1}| (đơn vị pwm, chia PWM_MAX ra u)
    uint32_t       sat_ticks[4];  // số tick có |pwm| ≥ PWM_MAX
} vel_ctrl_out_t;

void               vel_ctrl_init(void);              // khởi tạo PID, ADRC, imu_fusion, heading; chế độ = CTRL_MODE
void               vel_ctrl_set_mode(int mode);      // CTRL_PID / CTRL_ADRC; reset trạng thái bộ điều khiển, đặt lại vòng hướng
int                vel_ctrl_mode(void);
pid_controller_t  *vel_ctrl_pid(void);
adrc_controller_t *vel_ctrl_adrc(void);

vel_ctrl_out_t     vel_ctrl_step(const vel_ctrl_in_t *in);
void               vel_ctrl_applied(const int pwm[4], vel_ctrl_out_t *o);   // PWM thực áp; cập nhật tick, dpwm_sum, sat_ticks của o

#endif // VEL_CTRL_H
