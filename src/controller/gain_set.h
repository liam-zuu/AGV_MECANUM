#ifndef GAIN_SET_H
#define GAIN_SET_H

#include <stdint.h>
#include <stddef.h>

// ─────────────────────────────────────────
// Bộ gain đổi được lúc chạy (L90, Liam 12/11: đổi gain qua WiFi, "coi như từ đầu mình thiết kế").
// Thiết kế: 03_CODE/docs/2026-11-12_Thiet_ke_doi_gain_qua_WiFi.md (repo luận văn).
//
// 7 biến mỗi controller, đúng tên và cách gán của fwsim (hil-framework fwsim/tune_ga.py
// VARS_BY_MODE, gains_for; S03 X1..X7):
//   PID : kp_xy, ki_xy (vx và vy cùng gain), kp_wz, ki_wz, pos_kp, pos_ki, k_psi
//   ADRC: omega_c_xy, omega_o_xy, omega_c_wz, omega_o_wz, pos_kp, pos_ki, k_psi
// Cố định như fwsim: PID kd, giới hạn tích phân, giới hạn đầu ra (pid.c); LADRC b0, out_limit
// (adrc.c). File C thuần (không gọi ESP-IDF) để fwsim và test PC dùng chung.
//
// Gói (một dòng ASCII):  #gain_set,v=1,id=<1..2147483647>,ctrl=PID|ADRC,<7 cặp tên=giá trị>
// Giá trị gửi bằng %.9g của float32 nên strtof cho đúng bit PC đã gửi.
// ─────────────────────────────────────────

#define GAIN_N 7

typedef struct {
    uint32_t id;           // 0 = gain build sẵn (sau khởi động / reset); gói hợp lệ: 1..2^31-1
    int      ctrl;         // CTRL_PID / CTRL_ADRC (vel_ctrl.h)
    float    v[GAIN_N];    // theo thứ tự gain_set_names(ctrl)
} gain_set_t;

enum {
    GAIN_OK = 0,
    GAIN_ERR_FORMAT,       // khuôn sai, thiếu/thừa/lặp tên, v khác 1, id ngoài 1..2^31-1
    GAIN_ERR_CTRL,         // ctrl khác controller đang chạy
    GAIN_ERR_RANGE,        // không hữu hạn hoặc ngoài giới hạn an toàn
    GAIN_ERR_MOTOR_ON,     // motor đang bật: không đổi giữa lượt
    GAIN_ERR_BUSY,         // yêu cầu trước chưa xử lý xong
    GAIN_ERR_TIMEOUT,      // control_task không trả lời kịp
};

const char        *gain_err_str(int err);
const char *const *gain_set_names(int ctrl);          // GAIN_N tên; NULL nếu ctrl lạ
const char        *gain_ctrl_str(int ctrl);           // "PID" / "ADRC" / "?"

// Đọc gói vào out (chưa kiểm ctrl đang chạy, chưa kiểm miền). Trả GAIN_OK / GAIN_ERR_FORMAT.
int  gain_set_parse(const char *line, size_t len, gain_set_t *out);
// Giới hạn an toàn (chặn số vô lý, không phải miền tune). Trả GAIN_OK / GAIN_ERR_RANGE.
int  gain_set_check_range(const gain_set_t *g);
// Kiểm ctrl khớp controller đang chạy, kiểm miền, rồi áp vào PID/LADRC, vòng hướng, vòng vị trí và
// đặt lại trạng thái PID/LADRC. Chỉ gọi khi motor tắt, từ cùng task với vel_ctrl_step().
int  gain_set_apply(const gain_set_t *g);
// Bộ gain đang chạy (đọc lại từ struct của controller) cùng id và số lần áp từ lúc khởi động.
void     gain_set_current(gain_set_t *out);
uint32_t gain_set_applied_n(void);
// ",id=..,applied_n=..,ctrl=..,<7 cặp>" (giá trị %.9g). Trả số ký tự đã ghi (như snprintf, kẹp ở n-1).
int  gain_set_format(char *buf, size_t n, const gain_set_t *g, uint32_t applied_n);

#endif
