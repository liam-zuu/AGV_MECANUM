#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "gain_set.h"
#include "us_state.h"

// ─────────────────────────────────────────
// Config: log goes to the RPi5 of the HIL bench (Liam 08/11: "Về Pi cho mọi env")
// ─────────────────────────────────────────
#ifndef LOGGER_PC_IP                // env can still override by build flag
#define LOGGER_PC_IP        "192.168.1.76"   // Pi thesis-hil (Setup RPi5), was laptop 192.168.10.114
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
    // Thêm 12/11 (RB-4 R128, Liam 12/11 "Thêm luôn"; cùng bản L90): tuổi gói lệnh quỹ đạo cuối
    // [ms] = (thời điểm tick − thời điểm NHẬN gói cuối) / 1000, lấy ở mỗi tick 10 ms; −1 khi chưa có
    // gói nào từ lúc khởi động. Cột cuối để cột cũ giữ chỗ (tools/agv_logger.py COLUMNS, 86 cột).
    int32_t  traj_age_ms;
    // Chỉ dòng LOG_KIND_GAIN (L90; Codex vòng 1 L90-2, L90-4): bản sao bộ gain của chính dòng này
    // (không dùng bản chụp chung), số lần áp, mốc lượt. Không in vào dòng CSV dữ liệu.
    gain_set_t gain;
    uint32_t   gain_applied_n;
    uint32_t   gain_first_seq;   // log_seq của dòng dữ liệu đầu lượt; 0 = dòng in lúc áp gain
    // Chỉ dòng LOG_KIND_US (L95): bản sao trạng thái siêu âm của lượt lúc gọi, loại dòng, mốc seq
    us_run_t   us;
    uint8_t    us_ev;            // US_EV_START / US_EV_CHANGE / US_EV_END (us_state.h)
    uint32_t   us_seq;           // first_seq (start, change) hoặc last_data_seq (end)
    uint32_t   us_drop;          // số sự kiện #us/#us_end mất trước sự kiện này, chụp lúc đưa vào hàng đợi
} log_data_t;

#define LOG_KIND_DATA    0
#define LOG_KIND_RUN_END 1
#define LOG_KIND_FW      2   // dòng "#fw,sha=..,dirty=..,env=.." (fw_id_gen.h), gửi ở đầu mỗi lượt
#define LOG_KIND_GAIN    3   // dòng "#gain,id=..,applied_n=..,ctrl=..,<7 gain>,first_seq=..,line_drop=..,boot=.."
                             // (L90; boot từ 14/11, R134C3-1): khi áp, đầu mỗi lượt
#define LOG_KIND_US      4   // dòng "#us,.." / "#us_end,.." (L95, us_state.h): đầu lượt, khi SWB đổi, cuối lượt

// ─────────────────────────────────────────
// API
// ─────────────────────────────────────────
void logger_init(void);
void logger_push(const log_data_t *data);   // non-blocking, gọi từ control_task; điền log_seq, log_qdrop
uint32_t logger_seq(void);                  // log_seq của dòng push gần nhất
void logger_push_fw(void);                  // dòng #fw qua UDP; không lấy log_seq (logger.c)
void logger_push_gain(uint32_t first_seq);  // dòng #gain (bộ gain đang chạy, chụp lúc gọi, trong control_task); như #fw
void logger_push_us(int ev, const us_run_t *r, uint32_t ev_seq);  // #us / #us_end (L95): 1 phần tử, phát 3 bản
uint32_t logger_seq_next(void);             // log_seq dòng dữ liệu kế tiếp sẽ lấy
uint32_t logger_qdrop(void);
uint32_t logger_send_err(void);
void logger_task(void *pv);                 // tạo task trong app_main, core 0

#endif // LOGGER_H
