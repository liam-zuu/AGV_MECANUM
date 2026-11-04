#ifndef POS_CTRL_H
#define POS_CTRL_H

#include <stdbool.h>
#include <stdint.h>

// ─────────────────────────────────────────
// Vòng vị trí x, y (O1 PA3, Liam duyệt A1 + B1 ở phiên CODE 04/11/2026). C thuần (fwsim).
//
// A1: tư thế lệnh tích phân trên ESP32 từ vận tốc lệnh (hệ thân), không đổi giao thức:
//   θ_ref += wz_ref·dt;  x_ref += (vx_ref·cos θ_ref − vy_ref·sin θ_ref)·dt;  y_ref tương tự
// Tư thế ước lượng: vx, vy encoder (FK) xoay theo θ_est;
//   θ_est = ψ_imu + lệch khi vòng hướng (heading.c) đang chạy, không thì tích phân wz encoder;
//   lệch đặt lại mỗi lần chuyển sang ψ_imu để θ_est liên tục (reset chip, mất yaw).
// B1: hướng do heading.c giữ; ở đây chỉ x, y. Sai số xoay về hệ thân, PI có feedforward
// theo M6 (position_controller.m): v_cmd = v_ref + sat(Kp·e + Ki·∫e), anti-windup như M6
// (giữ tích phân khi đầu ra bão hòa và sai số làm tích phân lớn thêm).
// Bắt đầu chạy (run_active lên): tư thế lệnh và ước lượng về 0, tích phân về 0.
// Mặc định BẬT, cả (a) (Liam 04/11, phiên CODE: "làm cách 1 ban khuyến nghị đi"): firmware
// chiến dịch RQ3 (env chính PID và env ladrc) cùng cấu trúc, không env riêng. Chưa tune,
// chưa kiểm trên board. Bản không có vòng vị trí: fwsim đặt pos_on = 0 lúc chạy.
// ─────────────────────────────────────────

typedef struct {
    bool  on;            // BẬT mặc định
    float kp;            // [1/s]    6  (M6 params_mecanum.m, chỉnh cho plant mô-men cũ; chưa tune)
    float ki;            // [1/s²]   3  (như trên)
    float v_corr_max;    // [m/s]    [ĐỀ XUẤT 0,5] chặn phần hiệu chỉnh mỗi trục
    // (a) cửa sổ "đã tới vị trí" khi lệnh = 0 (04/11): lớn hơn dải sai số lúc săn trên fwsim
    // (2–4 mm, O1_FEAS run05_diag), rất nhỏ so với D = 0,10 m của O1. e_win ≤ 0: tắt.
    float e_win;         // [m]      [ĐỀ XUẤT 0,005, mặc định]
    float e_exit;        // [m]      [ĐỀ XUẤT 0,010]
} pos_params_t;

#define POS_PARAMS_DEFAULT { .on = true, .kp = 6.0f, .ki = 3.0f, .v_corr_max = 0.5f, \
                             .e_win = 0.005f, .e_exit = 0.010f }

typedef struct {
    float vx_ref, vy_ref, wz_ref;   // vận tốc lệnh hệ thân (sau interlock)
    float vx_enc, vy_enc, wz_enc;   // FK encoder
    float psi_imu;                  // heading.c psi_imu [rad]
    bool  head_on;                  // heading.c on: ψ_imu dùng được
    float dt;                       // [s]
    bool  run_active;
    bool  hold;                     // (d) logic dừng của vel_ctrl.c đang giữ: không hiệu chỉnh, tích phân giữ
} pos_in_t;

typedef struct {
    bool     on;                    // vòng đang hiệu chỉnh
    float    corr_vx, corr_vy;      // [m/s] cộng vào vx, vy lệnh
    float    x_ref, y_ref, th_ref;  // tư thế lệnh (hệ lúc bắt đầu chạy)
    float    x, y, th;              // tư thế ước lượng
    float    ex_b, ey_b;            // sai số hệ thân [m]
    uint8_t  th_src;                // 0 encoder, 1 IMU
    bool     in_pos;                // (a) trong cửa sổ "đã tới vị trí"
} pos_out_t;

void          pos_init(const pos_params_t *p);   // NULL = mặc định
pos_params_t *pos_params(void);
void          pos_step(const pos_in_t *in, pos_out_t *out);

#endif // POS_CTRL_H
