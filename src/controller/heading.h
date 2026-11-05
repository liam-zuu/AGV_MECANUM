#ifndef HEADING_H
#define HEADING_H

#include <stdbool.h>
#include <stdint.h>

// ─────────────────────────────────────────
// Vòng giữ hướng bằng yaw IMU (spec controller–IMU v2.2 §5). C thuần (fwsim).
//
//   ψ_imu = wrap(yaw − ψ0), ψ0 = yaw lúc bắt đầu chạy (ψ_ref = 0 khi đó)
//   ψ_ref tích phân wz_ref mỗi tick (quỹ đạo gửi vx, vy, wz trong hệ thân)
//   wz_cmd = wz_ref + sat(k_ψ · wrap(ψ_ref − ψ_imu), ±wz_corr_max)
//
// Chạy ngoài PID/LADRC. Nhiều sự kiện cùng tick: áp sự kiện đầu tiên khớp theo bảng §5
// (1 reset chip / đổi mode, 2 không chạy, 3 yaw mất, 4–5 yaw có lại).
// Bật/tắt vòng hướng không làm mềm (Liam 22/10): bước wz_cmd bị chặn bởi wz_corr_max.
// ─────────────────────────────────────────

typedef struct {
    float k_psi;         // [1/s]  KHỞI ĐẦU, chưa tune; vào miền GA (QĐ 12)
    float wz_corr_max;   // [rad/s] [ĐỀ XUẤT 0,5]
    float e_psi_reset;   // [rad]   [ĐỀ XUẤT 0,35 ≈ 20°]
} heading_params_t;

#define HEADING_PARAMS_DEFAULT { .k_psi = 1.0f, .wz_corr_max = 0.5f, .e_psi_reset = 0.35f }

typedef struct {
    float wz_ref;        // wz setpoint (sau interlock) [rad/s]
    float dt;            // [s]
    bool  run_active;    // motor enable + quỹ đạo VALID
    bool  yaw_valid;     // imu_fusion r_valid
    float yaw;           // [rad], [−π, π]
    bool  reset_event;   // chip reset (reset_n tăng) hoặc đổi PID ↔ LADRC
} heading_in_t;

typedef struct {
    float    wz_cmd;
    bool     on;               // vòng hướng đang chạy (head_on)
    float    psi_ref;          // [rad]
    float    psi_imu;          // [rad]
    float    e_psi;            // wrap(ψ_ref − ψ_imu) khi đang chạy và yaw hợp lệ, không thì 0
    uint32_t psi_reset_count;  // số lần đặt lại ψ0 hoặc ψ_ref (§5 ưu tiên 1, 4)
    uint32_t toggle_count;     // số lần bật/tắt vòng hướng
} heading_out_t;

void              heading_init(const heading_params_t *p);   // NULL = mặc định
heading_params_t *heading_params(void);
void              heading_step(const heading_in_t *in, heading_out_t *out);

#endif // HEADING_H
