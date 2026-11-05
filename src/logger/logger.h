#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────
// Config — TODO: đổi IP laptop
// ─────────────────────────────────────────
#ifndef LOGGER_PC_IP                // env đo đặt qua build flag (s06_load: IP của Pi, Q2 29/10)
#define LOGGER_PC_IP        "192.168.10.114"
#endif
#define LOGGER_UDP_PORT     5005
#define LOGGER_QUEUE_SIZE   32      // buffer 32 frame ở 100Hz = 320ms headroom

// ─────────────────────────────────────────
// Log data struct
// ─────────────────────────────────────────
typedef struct {
    int64_t timestamp_us;

    // Encoder
    int32_t fl_count, fr_count, rl_count, rr_count;
    float   fl_rpm,   fr_rpm,   rl_rpm,   rr_rpm;

    // Velocity actual (forward kinematics)
    float vx, vy, wz;

    // Setpoint (từ IBus hoặc RPi5)
    float sp_vx, sp_vy, sp_wz;

    // PWM command (-1000 ~ +1000)
    int pwm_fl, pwm_fr, pwm_rl, pwm_rr;

    // IMU — accelerometer (m/s²)
    float ax, ay, az;

    // IMU — gyroscope (rad/s)
    float gx, gy, gz;

    // IMU — euler (degrees)
    float yaw, pitch, roll;

    // Bộ đếm cộng dồn của vel_ctrl (spec S02): tick, Σ|Δpwm|, số tick bão hòa
    uint32_t tick;
    uint32_t dpwm_fl, dpwm_fr, dpwm_rl, dpwm_rr;
    uint32_t sat_fl,  sat_fr,  sat_rl,  sat_rr;

    // ── Spec controller–IMU v2.2 §8 (28/10). Cột cũ giữ tên: wz = wz encoder (FK),
    //    gz = gyro thô của chip, yaw = độ. ─────────────────────────────────────
    uint8_t  kind;             // LOG_KIND_DATA / LOG_KIND_RUN_END (dòng tổng kết, gửi 3 lần)
    // Mất log khép kín: điền trong logger_push / logger_task, người gọi không cần đặt
    uint32_t log_seq;          // tăng mỗi lần logger_push từ lúc khởi động, không reset
    uint32_t log_qdrop;        // số dòng mất vì hàng đợi đầy (tích lũy)
    uint32_t log_send_err;     // số lần sendto lỗi (tích lũy)
    uint8_t  motor_on;         // 0: dòng nhịp thưa 10 Hz khi motor tắt
    int8_t   imu_init;         // mã imu_init() (0 = đạt)
    // Hợp nhất IMU / vòng hướng (imu_fusion.c, heading.c)
    float    wz_meas, wz_cmd, gz_fused, yaw_rad;
    float    age_g_ms, age_r_ms;
    uint32_t gyro_n, rv_n, reset_n;
    uint8_t  g_valid, r_valid, wz_src, head_on, slip_flag;
    uint32_t src_switch_count, psi_reset_count, slip_on_count;
    float    psi_ref, e_psi, r, bias_gz;
    // Thêm 28/10 (Codex ctrl-imu #11, #8 (c)): cuối dòng để cột cũ giữ chỗ
    float    src_switch_dwz;
    uint32_t bias_updates;
    uint8_t  slip_eval;        // 1: tick này xét được r; 0: không (cờ 0 không có nghĩa "không trượt")
    // Chỉ dòng tổng kết (QĐ 6 bậc 1): gói UDP bị bỏ / được nhận trong lượt
    uint32_t udp_drop, udp_take;
    // Thêm 04/11 (vòng vị trí pos_ctrl.c, O1 v6 mục 5.8; Liam "Làm cột log"): cuối dòng để cột cũ
    // giữ chỗ. Thứ tự cột chốt: 66 cột ctrl-imu, 14 cột pos_*/still/run_active/pos_reset_n này, rồi
    // 5 cột ws_* (feat/wheel-slip; khi gộp đặt sau các cột này), như tools/agv_logger.py COLUMNS.
    uint8_t  pos_on;                       // vòng vị trí đang hiệu chỉnh
    float    pos_x, pos_y, pos_th;         // tư thế ước lượng [m, m, rad] (hệ lúc bắt đầu chạy)
    float    pos_x_ref, pos_y_ref, pos_th_ref;  // tư thế lệnh tích phân trên ESP32 (A1)
    float    pos_corr_vx, pos_corr_vy;     // [m/s] hiệu chỉnh cộng vào vx, vy lệnh
    uint8_t  pos_th_src;                   // 0 encoder, 1 IMU
    uint8_t  pos_in_pos;                   // (a) trong cửa sổ "đã tới vị trí"
    uint8_t  still;                        // (d) logic dừng đang giữ (PWM 0)
    // Thêm 05/11 (Codex P6): pose chỉ tích phân khi run_active; mỗi lần bắt đầu chạy pos_reset_n tăng
    uint8_t  run_active;                   // motor bật + AUTO + quỹ đạo còn mới
    uint32_t pos_reset_n;                  // số lần đặt lại tư thế từ lúc khởi động
    // Thêm 04/11 (wheel_slip.c, trượt không đều giữa các bánh): cuối dòng, SAU các cột pos_*
    // (gộp int/post-g2 05/11; 66 + 14 + 5 = 85 cột, như tools/agv_logger.py COLUMNS)
    float    ws_res, ws_th;    // [count] phần dư, ngưỡng trong cửa sổ
    uint8_t  ws_flag, ws_eval;
    uint32_t ws_on_count;
} log_data_t;

#define LOG_KIND_DATA    0
#define LOG_KIND_RUN_END 1

// ─────────────────────────────────────────
// API
// ─────────────────────────────────────────
void logger_init(void);
void logger_push(const log_data_t *data);   // non-blocking, gọi từ control_task; điền log_seq, log_qdrop
uint32_t logger_seq(void);                  // log_seq của dòng push gần nhất
uint32_t logger_qdrop(void);
uint32_t logger_send_err(void);
void logger_task(void *pv);                 // tạo task trong app_main, core 0

#endif // LOGGER_H
