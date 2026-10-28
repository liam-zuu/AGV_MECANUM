#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

// Chu kỳ report của BNO085 (µs). Mặc định 50 ms = 20 Hz; env đo nhiễu đặt qua build flag
// (-DIMU_REPORT_PERIOD_US=10000 cho 100 Hz). Tần số cuối cùng chốt bằng thí nghiệm fwsim.
#ifndef IMU_REPORT_PERIOD_US
#define IMU_REPORT_PERIOD_US 50000UL
#endif

// Report id (khớp sh2.h)
#define IMU_RPT_ACCEL   0x01
#define IMU_RPT_GYRO    0x02
#define IMU_RPT_GAME_RV 0x08

// ─── Data struct (C-compatible) ───────────────────────────────────────────────
typedef struct {
    float accel_x;   // m/s²
    float accel_y;
    float accel_z;
    float gyro_x;    // rad/s
    float gyro_y;
    float gyro_z;    // wz — dùng cho heading control
    float roll;      // độ
    float pitch;
    float yaw;
    bool  valid;     // đã nhận ít nhất một report (chưa phải kiểm hợp lệ cho controller)

    // Mỗi report: thời điểm nhận (esp_timer, µs) và số report đã nhận từ lúc khởi động.
    // Thư viện không cho timestamp/sequence của chip dùng được (shtp.c đặt t_us = 0),
    // nên đây là mốc phía host.
    int64_t  accel_t_us, gyro_t_us, rv_t_us;
    uint32_t accel_n,    gyro_n,    rv_n;
} imu_data_t;

// Hook gọi mỗi report (trong task callback của thư viện, không phải ngắt).
// v: accel x,y,z | gyro x,y,z | roll,pitch,yaw (độ); v[3] = 0.
typedef void (*imu_report_hook_t)(uint8_t rpt_id, int64_t t_us, uint32_t n, const float v[4]);

// ─── API (C-compatible, implement bên trong .cpp) ─────────────────────────────
#ifdef __cplusplus
extern "C" {
#endif

void imu_init(void);
bool imu_get(imu_data_t *out);   // thread-safe copy data ra ngoài
void imu_set_report_hook(imu_report_hook_t hook);

#ifdef TEST_IMU_TRACE
// Chỉ cho env imu_trace_* (QĐ 26, G0 v1 §7): kịch bản S3 đổi chu kỳ, S4 xung RST giữa chừng.
bool imu_trace_set_period(uint32_t period_us);
bool imu_trace_hard_reset(void);
#endif

#ifdef __cplusplus
}
#endif

#endif // IMU_H
