#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

// Chu kỳ report của BNO085 (µs). Mặc định 50 ms = 20 Hz; env đo nhiễu đặt qua build flag
// (-DIMU_REPORT_PERIOD_US=10000 cho 100 Hz). Tần số cuối cùng chốt bằng thí nghiệm fwsim.
#ifndef IMU_REPORT_PERIOD_US
#define IMU_REPORT_PERIOD_US 50000UL
#endif

// |gz| coi là bão hòa: 0,9 × 2000 °/s (dải gyro hiện tại). Dùng chung cho imu_fusion (hợp lệ)
// và tiêu chí khởi tạo bên dưới.
#define IMU_GYRO_SAT_RADPS 31.4f

// Khởi tạo có hạn (spec controller–IMU v2.2 §7): đạt khi trong IMU_INIT_TIMEOUT_MS (T_init)
// nhận được ít nhất IMU_INIT_MIN_REPORTS report gyro VÀ game RV. [ĐỀ XUẤT 2 s, 3 mẫu]
// Chỉ đếm report dùng được (gyro hữu hạn và |gz| < IMU_GYRO_SAT_RADPS; game RV hữu hạn), sau
// lần reset chip gần nhất (Codex ctrl-imu #13, Liam duyệt 28/10).
#ifndef IMU_INIT_TIMEOUT_MS
#define IMU_INIT_TIMEOUT_MS 2000
#endif
#ifndef IMU_INIT_MIN_REPORTS
#define IMU_INIT_MIN_REPORTS 3
#endif

// Mã trả về của imu_init()
#define IMU_INIT_OK          0
#define IMU_INIT_ERR_LIB     1   // BNO08x::initialize() trả false (SPI, GPIO, sh2_open…)
#define IMU_INIT_ERR_HANG    2   // thư viện chưa khởi tạo xong khi hết T_init (treo, chip không trả lời)
#define IMU_INIT_ERR_REPORTS 3   // thư viện báo xong nhưng thiếu report gyro / game RV trong T_init

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

    // Số lần chip báo SH2_RESET từ lúc khởi động và thời điểm lần gần nhất (spec
    // controller–IMU v2 §3, §4.1). Chip cũng báo reset một lần lúc khởi động.
    uint32_t reset_n;
    int64_t  reset_t_us;
} imu_data_t;

// Hook gọi mỗi report (trong task callback của thư viện, không phải ngắt).
// v: accel x,y,z | gyro x,y,z | roll,pitch,yaw (độ); v[3] = 0.
typedef void (*imu_report_hook_t)(uint8_t rpt_id, int64_t t_us, uint32_t n, const float v[4]);

// ─── API (C-compatible, implement bên trong .cpp) ─────────────────────────────
#ifdef __cplusplus
extern "C" {
#endif

// Chặn tới khi đạt hoặc hết T_init; thư viện chạy trong task riêng nên treo cũng không chặn quá
// T_init. Trả IMU_INIT_*; lỗi thì người gọi dừng motor và báo (§7). Gọi một lần.
int         imu_init(void);
int         imu_init_status(void);         // kết quả imu_init() (−1: chưa gọi / đang chạy)
uint32_t    imu_init_ms(void);             // thời gian imu_init() đã dùng [ms]
const char *imu_init_err_str(int code);
bool imu_get(imu_data_t *out);   // thread-safe copy data ra ngoài
void imu_set_report_hook(imu_report_hook_t hook);

// Gọi từ thư viện (BNO08xSH2HAL::hal_cb) khi chip báo SH2_RESET: tăng reset_n (§3, §4.1).
void imu_note_chip_reset(void);

#ifdef __cplusplus
}
#endif

#endif // IMU_H
