#ifndef US_STATE_H
#define US_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ─────────────────────────────────────────
// Trạng thái siêu âm trong lượt (L95, Liam 12/11 khuya "SWB + ghi log"; 13/11 hộp chọn "Đầu lượt +
// khi đổi + cuối (Recommended)"). File C thuần (không gọi ESP-IDF) để test PC dùng chung
// (hil-framework fwsim/tests/test_us_state.py).
//
// (a) Công tắc SWB (kênh 9) bật/tắt phát hiện vật cản ở MỌI chế độ (trước 13/11: chỉ MANUAL, AUTO
//     luôn bật). SWB < US_SWB_ON_BELOW là BẬT (như MANUAL cũ). Không có iBUS hợp lệ thì BẬT (an toàn).
// (b) Log của lượt (motor bật → tắt), dòng văn bản bắt đầu bằng '#' (PC tách khỏi CSV):
//       #us,ev=start,on=..,first_seq=..,line_drop=..    đầu lượt (3 bản, như #gain)
//       #us,ev=change,on=..,first_seq=..,line_drop=..   SWB đổi giữa lượt (3 bản)
//     first_seq = log_seq của dòng dữ liệu đầu tiên mang trạng thái on mới (cùng tick).
//       #us_end,on=..,changes=..,blk_ticks=..,cut_ticks=..,ticks=..,last_data_seq=..,line_drop=..
//     cuối lượt (3 bản, ngay trước #run_end). ticks: số tick motor bật của lượt; blk_ticks: số tick
//     có ít nhất một hướng bị siêu âm chặn (cờ chặn vào vel_ctrl cắt cả hiệu chỉnh vị trí);
//     cut_ticks: số tick bước 7c đổi lệnh vận tốc hoặc bước 10b ép PWM về 0. line_drop: số dòng
//     #us/#us_end không vào được hàng đợi (tích lũy từ lúc khởi động).
// Không đổi phép tính của controller: chỉ đọc cờ và đếm.
// ─────────────────────────────────────────

#define US_SWB_ON_BELOW 1500

bool us_swb_enabled(bool has_rc, int ch_swb);

typedef struct {
    uint8_t  on;          // trạng thái hiện tại (1 = phát hiện vật cản bật)
    uint32_t changes;     // số lần on đổi trong lượt
    uint32_t blk_ticks;   // số tick có ít nhất một cờ chặn
    uint32_t cut_ticks;   // số tick lệnh vận tốc bị đổi (7c) hoặc PWM bị ép 0 (10b)
    uint32_t ticks;       // số tick motor bật trong lượt
} us_run_t;

void us_run_begin(us_run_t *r, bool on);
// Gọi mỗi tick motor bật (kể cả tick đầu lượt). Trả true nếu on khác tick trước (đã tăng changes).
bool us_run_tick(us_run_t *r, bool on, bool any_blocked, bool cut);

#define US_EV_START  0
#define US_EV_CHANGE 1
#define US_EV_END    2

// Định dạng một dòng (kết thúc "\n") vào buf. ev_seq: first_seq (start, change) hoặc last_data_seq
// (end). Trả số byte (không kể '\0'); < 0 hay >= n là lỗi (dòng không gửi).
int us_format_line(char *buf, size_t n, int ev, const us_run_t *r, uint32_t ev_seq, uint32_t line_drop);

#endif // US_STATE_H
