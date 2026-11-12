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
// Gói (một dòng ASCII, sửa theo Codex L90-7, L90-10, L90-8, L90-12 ngày 12/11):
//   #gain_set,v=1,tx=<1..2^32-1>,id=<1..2^31-1>,ctrl=PID|ADRC,<7 cặp tên=giá trị>
//   #gain_get,v=1,tx=<1..2^32-1>                      (chỉ đọc bộ đang chạy)
// tx: số do PC sinh cho mỗi lần gửi, ESP32 trả lại trong #gain_ack (ghép đúng trả lời với yêu cầu).
// Thân gói chỉ gồm byte ASCII in được 0x21..0x7E (không dấu cách, không ký tự điều khiển); cho
// phép đúng một kết thúc dòng "\n" hoặc "\r\n" ở cuối. Giá trị: -?d+(.d+)?([eE][+-]?d+)? (PC gửi
// %.9g của float32, strtof cho đúng bit). Gói dài hơn GAIN_MSG_MAX bị từ chối.
// ─────────────────────────────────────────

#define GAIN_N       7
#define GAIN_MSG_MAX 320          // byte, không kể kết thúc dòng

typedef struct {
    uint32_t id;           // 0 = gain build sẵn (sau khởi động / reset); gói hợp lệ: 1..2^31-1
    int      ctrl;         // CTRL_PID / CTRL_ADRC (vel_ctrl.h)
    float    v[GAIN_N];    // theo thứ tự gain_set_names(ctrl)
} gain_set_t;

enum {
    GAIN_OK = 0,
    GAIN_ERR_FORMAT,       // khuôn sai, ký tự lạ, thiếu/thừa/lặp trường, v khác 1, id/tx ngoài miền, quá dài
    GAIN_ERR_CTRL,         // ctrl khác controller đang chạy
    GAIN_ERR_RANGE,        // không hữu hạn hoặc ngoài giới hạn an toàn
    GAIN_ERR_MOTOR_ON,     // motor đang bật, hoặc gói nhận trước khi motor tắt: không đổi giữa lượt
    GAIN_ERR_BUSY,         // yêu cầu trước chưa xử lý xong
    GAIN_ERR_TIMEOUT,      // control_task không trả lời kịp
};

enum { GAIN_MSG_SET = 1, GAIN_MSG_GET = 2 };

const char        *gain_err_str(int err);
const char *const *gain_set_names(int ctrl);          // GAIN_N tên; NULL nếu ctrl lạ
const char        *gain_ctrl_str(int ctrl);           // "PID" / "ADRC" / "?"

// Đọc một gói. *kind = GAIN_MSG_SET (điền *set) hoặc GAIN_MSG_GET; *tx = nonce của PC (0 nếu không
// đọc được). Chưa kiểm ctrl đang chạy, chưa kiểm miền. Trả GAIN_OK / GAIN_ERR_FORMAT.
int  gain_msg_parse(const char *line, size_t len, int *kind, gain_set_t *set, uint32_t *tx);
// Giới hạn an toàn (chặn số vô lý, không phải miền tune). Trả GAIN_OK / GAIN_ERR_RANGE.
int  gain_set_check_range(const gain_set_t *g);
// Kiểm ctrl khớp controller đang chạy, kiểm miền, rồi áp vào PID/LADRC, vòng hướng, vòng vị trí và
// đặt lại trạng thái PID/LADRC. Chỉ gọi khi motor tắt, từ cùng task với vel_ctrl_step().
int  gain_set_apply(const gain_set_t *g);
// Bộ gain đang chạy (đọc lại từ struct của controller) cùng id và số lần áp từ lúc khởi động.
// Gọi từ cùng task áp gain (control_task) để không đọc lúc đang áp.
void     gain_set_current(gain_set_t *out);
uint32_t gain_set_applied_n(void);
// ",id=..,applied_n=..,ctrl=..,<7 cặp>" (giá trị %.9g). Trả số ký tự đã ghi; -1 nếu bộ đệm thiếu
// (khi đó buf rỗng, người gọi không gửi dòng cụt).
int  gain_set_format(char *buf, size_t n, const gain_set_t *g, uint32_t applied_n);

#endif
