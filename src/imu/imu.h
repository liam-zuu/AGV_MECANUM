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

// Hook mở rộng cho env đo S06 (29/10/2026, spec S06 v3.4 V3-8, V4-1, V4-2, Y13). Gọi cùng chỗ với
// hook trên (task callback của thư viện) cho mỗi report, và thêm một lần cho mỗi thông báo reset
// hoàn tất (kind = IMU_EVT_RESET, gọi trong task dịch vụ SH2 HAL lúc nhận). Hook chỉ chép, không chặn.
// Report ghi ở hai chỗ (Liam chốt 29/10 ở phiên THESIS: "đếm ở cổng SPI"): stage RX = cổng nhận
// SPI (sensor_event_cb, trước update_data có thể ghi đè), stage CB = callback ứng dụng.
#define IMU_EVT_RESET 0xF0
#define IMU_STAGE_CB  0
#define IMU_STAGE_RX  1
typedef struct {
    uint8_t  kind;       // IMU_RPT_ACCEL / IMU_RPT_GYRO / IMU_RPT_GAME_RV, hoặc IMU_EVT_RESET
    uint8_t  stage;      // IMU_STAGE_CB / IMU_STAGE_RX (report); 0 với IMU_EVT_RESET
    uint8_t  applied;    // CB: 1 đã ghi vào imu_data_t, 0 callback không lấy được mutex (bỏ);
                         // RX: 1 đã vào hàng đợi HAL → data_proc_task, 0 hàng đợi đầy (bỏ tầng 1)
    uint8_t  seq;        // số thứ tự report SH-2 (8 bit, theo từng loại) của dữ liệu trong v
    int64_t  t_us;       // esp_timer lúc callback chạy (report) / lúc nhận thông báo (reset)
    uint32_t n;          // report: accel_n / gyro_n / rv_n sau report này (0 nếu không applied);
                         // reset: số thông báo reset hoàn tất đã nhận từ lúc khởi động
    uint32_t epoch;      // report: số thông báo reset đã nhận lúc HAL nhận report (sensor_event_cb)
    uint32_t lib_n;      // CB: số report loại này thư viện đã lưu (thấy được ghi đè trước callback)
    float    v[4];       // CB: như hook trên; game RV: roll, pitch, yaw (độ, get_euler()), v[3] = 0.
                         // RX: không điền (giá trị chấm ở callback, T4)
} imu_event_t;
typedef void (*imu_event_hook_t)(const imu_event_t *e);

// Bộ đếm report bị bỏ theo tầng, cộng dồn từ lúc khởi động (Y13). Không gồm report chip không gửi.
typedef struct {
    uint32_t rx_queue;   // tầng 1: hàng đợi HAL → data_proc_task đầy (sensor_event_cb)
    uint32_t cb_queue;   // tầng 2: hàng đợi data_proc_task → task callback đầy
    uint32_t cb_mutex;   // tầng 3: callback không lấy được mutex của imu_data_t trong 5 ms
} imu_drops_t;

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
void imu_set_event_hook(imu_event_hook_t hook);
void imu_get_drops(imu_drops_t *out);

// Gọi từ thư viện (BNO08xSH2HAL::hal_cb) khi chip báo SH2_RESET: tăng reset_n (§3, §4.1).
void imu_note_chip_reset(void);
// Gọi từ thư viện: số thông báo reset đã nhận (đóng dấu report lúc nhận) và report bị bỏ ở tầng 1, 2.
uint32_t imu_hal_reset_count(void);
void imu_note_lib_drop(int layer);
// Gọi từ thư viện (sensor_event_cb) cho mỗi report nhận qua SPI: loại, số thứ tự SH-2, epoch, vào hàng đợi hay không.
void imu_note_rx_report(uint8_t rpt_id, uint8_t seq, uint32_t epoch, int queued);

#ifdef __cplusplus
}
#endif

#endif // IMU_H
