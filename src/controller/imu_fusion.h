#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "imu.h"        // imu_data_t, IMU_REPORT_PERIOD_US (header không gọi ESP-IDF)
#include <stdbool.h>
#include <stdint.h>

// ─────────────────────────────────────────
// Hợp nhất IMU–encoder cho vòng vận tốc (spec controller–IMU v2.2 §3, §4, §6;
// docs/2026-10-27_Spec_Controller_IMU_v2.md). C thuần, không gọi ESP-IDF: fwsim
// (hil-framework) biên dịch đúng file này.
//
//   - kiểm hợp lệ gyro (wz) và game RV (yaw) theo mẫu mới, tuổi mẫu, sự kiện reset;
//   - chọn nguồn wz: IMU khi gyro hợp lệ (quay về IMU sau T_back mẫu tốt), không thì encoder;
//   - bias gyro dư khi đứng yên (§4.4);
//   - cờ nghi trượt / nghi bất nhất r = wz_enc − gz (§6): chỉ báo, không đổi điều khiển.
//
// Mọi ngưỡng là [ĐỀ XUẤT] của spec, dùng tạm tới khi đo nhiễu IMU 20/100 Hz (K1).
// sigma_gz, sigma_a chưa đo nên mặc định 0: r_th chỉ còn phần lượng tử encoder, và
// ước lượng bias không bao giờ chạy (|gz| < 3·0 không thỏa) cho tới khi có số đo.
// ─────────────────────────────────────────

typedef struct {
    int64_t t_age_us;       // tuổi mẫu tối đa: 2,5 chu kỳ report
    int64_t t_settle_us;    // chờ sau sự kiện reset chip
    float   gyro_sat;       // |gz| bão hòa [rad/s]: 0,9 × 2000 °/s
    int     n_back;         // số mẫu gyro tốt liên tiếp để quay về IMU (T_back)

    float   sigma_gz;       // σ nhiễu gyro [rad/s]   (CHƯA ĐO, K1)
    float   sigma_a;        // σ nhiễu gia tốc [m/s²] (CHƯA ĐO, K1)

    // Cờ trượt (§6)
    float   omega_min;      // bánh coi là đang quay khi max|ω_i| ≥ omega_min [rad/s]
    float   k_r;            // r_th = k_r · √(σ_gz² + q_wz²)
    float   q_wz;           // RMS lượng tử wz encoder [rad/s] (03_CODE/CLAUDE.md)
    int64_t t_slip_us;

    // Bias khi đứng yên (§4.4)
    int64_t t_still_us;
    float   b_max;          // |bias| tối đa [rad/s]
    float   g_ref;          // trọng trường dùng cho điều kiện | |a| − g | [m/s²]
} imu_fusion_params_t;

#define IMU_FUSION_PARAMS_DEFAULT {                     \
    .t_age_us    = (int64_t)(IMU_REPORT_PERIOD_US * 5 / 2), \
    .t_settle_us = 500000,                              \
    .gyro_sat    = 31.4f,                               \
    .n_back      = 3,                                   \
    .sigma_gz    = 0.0f,                                \
    .sigma_a     = 0.0f,                                \
    .omega_min   = 2.0f,                                \
    .k_r         = 4.0f,                                \
    .q_wz        = 0.0196f,                             \
    .t_slip_us   = 200000,                              \
    .t_still_us  = 2000000,                             \
    .b_max       = 0.02f,                               \
    .g_ref       = 9.80665f,                            \
}

#define WZ_SRC_ENC 0
#define WZ_SRC_IMU 1

// Đầu vào mỗi tick 10 ms
typedef struct {
    int64_t           now_us;
    const imu_data_t *imu;           // NULL: chưa có bản sao nào từ imu_get()
    float             wz_enc;        // wz từ encoder (FK) [rad/s]
    float             omega[4];      // tốc độ bánh từ encoder [rad/s]
    int32_t           enc_count[4];  // encoder_get_count(): gia số mỗi tick cho điều kiện đứng yên
    bool              sp_zero;       // setpoint thân = 0 cả 3 trục
    bool              drive_off;     // motor disable, hoặc PWM = 0 cả 4 bánh ở tick trước
    bool              wheels_lifted; // chế độ kê bánh (cấu hình bài chạy): tắt ước lượng bias
} imu_fusion_in_t;

// Kết quả mỗi tick (cũng là cột log §8)
typedef struct {
    bool     g_valid;        // gyro hợp lệ (§4.2)
    bool     r_valid;        // yaw hợp lệ (§4.2)
    bool     gyro_new;       // tick này có mẫu gyro mới
    int      wz_src;         // WZ_SRC_ENC / WZ_SRC_IMU
    float    wz_meas;        // wz đưa vào bộ điều khiển [rad/s]
    float    gz;             // gz đã trừ bias [rad/s] (0 khi chưa có mẫu)
    float    yaw;            // yaw [rad], quấn về [−π, π]
    float    age_g_ms, age_r_ms;   // −1 khi chưa có mẫu
    bool     reset_event;    // tick này thấy reset_n tăng
    float    bias_gz;        // bias dư đang trừ [rad/s]
    uint32_t bias_updates;

    float    r;              // wz_enc − gz ở mẫu gyro gần nhất được xét [rad/s]
    float    r_th;
    bool     slip_flag;
    bool     slip_eval;      // tick này xét được r (gyro hợp lệ, max|ω_i| ≥ omega_min)
    uint32_t slip_on_count;  // số lần cờ bật

    uint32_t src_switch_count;  // số lần đổi nguồn wz
    float    src_switch_dwz;    // Δ = wz_mới − wz_cũ ở lần đổi gần nhất
} imu_fusion_out_t;

void                 imu_fusion_init(const imu_fusion_params_t *p);   // NULL = mặc định
imu_fusion_params_t *imu_fusion_params(void);                         // sửa ngưỡng lúc chạy (fwsim, bài đo)
void                 imu_fusion_step(const imu_fusion_in_t *in, imu_fusion_out_t *out);

#endif // IMU_FUSION_H
