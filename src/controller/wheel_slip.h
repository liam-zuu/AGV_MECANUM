#ifndef WHEEL_SLIP_H
#define WHEEL_SLIP_H

#include <stdbool.h>
#include <stdint.h>

// ─────────────────────────────────────────
// Cờ "nghi trượt không đều giữa các bánh" từ dư thừa động học 4 bánh (Liam duyệt ở phiên CODE
// 04/11/2026: "giờ sẽ làm luôn cách 1 nhé"). C thuần (fwsim). CHỈ GHI LOG, không đổi điều khiển
// (như cờ trượt của imu_fusion.c).
//
// Xe Mecanum 4 bánh, 3 bậc tự do: từ kinematics_inverse(), mọi chuyển động không trượt cho
//   ω_fl + ω_fr − ω_rl − ω_rr = 0   (trung bình trục trước = trục sau, không phụ thuộc LX, LY)
// nên phần dư khác 0 khi các bánh trượt KHÔNG ĐỀU. Trượt đều cả 4 bánh không thấy được.
// Cờ trượt của imu_fusion.c (wz encoder − gyro) chỉ thấy trượt làm xe quay khác encoder.
//
// Tính trên số đếm cộng dồn encoder_get_count() trong cửa sổ win_ticks tick:
//   d_i = count_i(k) − count_i(k − win);  res = d_fl + d_fr − d_rl − d_rr  [count]
// Lượng tử: mỗi d_i sai tối đa ±1 count so với góc thật, nên |res| ≤ 4 count khi không trượt,
// không lớn dần theo cửa sổ. Ngưỡng th = max(n_min, k_rel · Σ|d_i|): k_rel phủ sai lệch bán kính
// bánh (tỉ lệ với quãng quay). Xét khi max|d_i| ≥ act_min (bánh đang quay); bật khi |res| > th
// liên tục t_hold, tắt khi |res| < th/2 liên tục t_hold, hoặc không xét được liên tục t_hold.
// Mọi ngưỡng là [ĐỀ XUẤT], chưa kiểm trên xe.
// ─────────────────────────────────────────

#define WHEEL_SLIP_WIN_MAX 50

typedef struct {
    uint16_t win_ticks;   // [tick]  [ĐỀ XUẤT 20 = 200 ms ở 100 Hz]
    float    n_min;       // [count] [ĐỀ XUẤT 8]: > 4 count lượng tử tối đa
    float    k_rel;       // [-]     [ĐỀ XUẤT 0,03]
    float    act_min;     // [count] [ĐỀ XUẤT 84 ≈ 2 rad/s trong 200 ms, như omega_min của imu_fusion]
    int64_t  t_hold_us;   // [µs]    [ĐỀ XUẤT 200 000, như t_slip_us]
} wheel_slip_params_t;

#define WHEEL_SLIP_PARAMS_DEFAULT \
    { .win_ticks = 20, .n_min = 8.0f, .k_rel = 0.03f, .act_min = 84.0f, .t_hold_us = 200000 }

typedef struct {
    int32_t  count[4];    // encoder_get_count() FL, FR, RL, RR
    int64_t  now_us;
} wheel_slip_in_t;

typedef struct {
    float    res;         // [count] phần dư trong cửa sổ
    float    th;          // [count] ngưỡng tick này
    bool     eval;        // tick này xét được (đủ cửa sổ, bánh đang quay)
    bool     flag;        // cờ nghi trượt không đều
    uint32_t on_count;    // số lần cờ bật
} wheel_slip_out_t;

void                 wheel_slip_init(const wheel_slip_params_t *p);   // NULL = mặc định
wheel_slip_params_t *wheel_slip_params(void);
void                 wheel_slip_step(const wheel_slip_in_t *in, wheel_slip_out_t *out);

#endif // WHEEL_SLIP_H
