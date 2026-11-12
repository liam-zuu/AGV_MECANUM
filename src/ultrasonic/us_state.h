#ifndef US_STATE_H
#define US_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ─────────────────────────────────────────
// Trạng thái siêu âm trong lượt (L95, Liam 12/11 khuya "SWB + ghi log"; 13/11 hộp chọn "Đầu lượt +
// khi đổi + cuối (Recommended)"; sửa theo Codex Sol vòng 1, Liam 13/11 "Sửa cả 8 điểm + vòng 2").
// File C thuần (không gọi ESP-IDF) để test PC dùng chung (hil-framework fwsim/tests/test_us_state.py).
//
// (a) Công tắc SWB (kênh 9) bật/tắt phát hiện vật cản ở MỌI chế độ (trước 13/11: chỉ MANUAL, AUTO
//     luôn bật). SWB < US_SWB_ON_BELOW là BẬT (như MANUAL cũ). Không có iBUS hợp lệ thì BẬT (an toàn).
//     iBUS hợp lệ = khung đúng checksum gần nhất không quá IBUS_FRAME_MAX_AGE_US (ibus_age.h, L95-1).
// (b) Log của lượt (motor bật → tắt), dòng văn bản bắt đầu bằng '#' (PC tách khỏi CSV). Mỗi sự kiện là
//     MỘT phần tử hàng đợi logger; logger_task phát 3 bản giống hệt nhau (L95-3):
//       #us,ev=start,boot=..,on=..,swb=..,first_seq=..,line_drop=..    đầu lượt
//       #us,ev=change,boot=..,on=..,swb=..,first_seq=..,line_drop=..   trạng thái đổi giữa lượt
//       #us_end,boot=..,on=..,swb=..,changes=..,blk_ticks=..,cut_ticks=..,ticks=..,last_data_seq=..,line_drop=..
//     boot: mã khởi động ngẫu nhiên (cùng số với #gain_ack, gain_rx.c), đổi mỗi lần ESP32 khởi động (L95-8).
//     swb: giá trị thô kênh 9 lúc ghi (−1 = không có iBUS hợp lệ) (L95-6).
//     first_seq: log_seq của dòng dữ liệu đầu tiên mang trạng thái on mới (cùng tick).
//     ticks: số tick motor bật của lượt; blk_ticks: số tick có ít nhất một cờ chặn s_*_blocked (cờ này vào
//     7c, vel_ctrl và pos_ctrl); cut_ticks: số tick bước 7c đổi lệnh hoặc bước 10b ép PWM về 0 (CHỈ BÁO:
//     không thấy các điểm kẹp bên trong vel_ctrl/pos_ctrl, và env s06_load ghi đè PWM sau đó, L95-5).
//     line_drop: số SỰ KIỆN #us/#us_end không vào được hàng đợi tính tới trước sự kiện này (chụp lúc
//     đưa vào hàng đợi, tích lũy từ lúc khởi động).
// (c) Luật cho bộ chấm (chưa viết; L95-4, L95-5, L95-8, L95-R2-2): lượt được coi là "không bị interlock
//     siêu âm" chỉ khi đủ cả:
//     - boot của #us/#us_end gắn DUY NHẤT với chính dữ liệu của lượt (CSV, #run_end không mang boot): cùng
//       một lần khởi động theo log serial ESP32 của buổi (không có reset giữa) hay #fw + #gain_ack cùng boot;
//       chưa gắn được là mơ hồ;
//     - có #us,ev=start và #us_end cùng boot; bản sao giống hệt gộp thành một sự kiện, bản khác nội dung
//       của cùng sự kiện là xung đột;
//     - first_seq, last_data_seq khớp hai cạnh motor của lượt (seq dự định, xét cả dòng dữ liệu mất),
//       ticks > 0 và ticks = last_data_seq − first_seq + 1;
//     - on=0 ở start, changes=0, on=0 ở end, không có #us,ev=change trong khoảng lượt;
//     - blk_ticks=0 (cut_ticks không dùng để quyết).
//     Thiếu, mâu thuẫn hay mơ hồ thì là thiếu bằng chứng. line_drop lịch sử khác 0 tự nó không làm hỏng lượt.
// (d) Chỗ trong hàng đợi logger (L95-R2-1): sự kiện chỉ được đưa vào khi hàng đợi còn ít nhất
//     us_queue_need(ev) chỗ: start, change cần 2 (chính nó + dòng dữ liệu của tick đó), end cần 4 (chính nó +
//     3 #run_end). Không đủ thì bỏ sự kiện và tăng line_drop, không lấy chỗ của dòng dữ liệu hay #run_end.
// Không đổi phép tính của controller: chỉ đọc cờ và đếm.
// ─────────────────────────────────────────

#define US_SWB_ON_BELOW 1500

bool us_swb_enabled(bool has_rc, int ch_swb);

typedef struct {
    uint8_t  on;          // trạng thái hiện tại (1 = phát hiện vật cản bật)
    int32_t  swb;         // giá trị thô kênh 9 ở tick gần nhất (−1 = không có iBUS hợp lệ)
    uint32_t boot;        // mã khởi động (không đổi trong một lần chạy firmware)
    uint32_t changes;     // số lần on đổi trong lượt
    uint32_t blk_ticks;   // số tick có ít nhất một cờ chặn
    uint32_t cut_ticks;   // số tick lệnh vận tốc bị đổi (7c) hoặc PWM bị ép 0 (10b); chỉ báo
    uint32_t ticks;       // số tick motor bật trong lượt
} us_run_t;

void us_run_begin(us_run_t *r, bool on, int32_t swb, uint32_t boot);
// Gọi mỗi tick motor bật (kể cả tick đầu lượt). Trả true nếu on khác tick trước (đã tăng changes).
bool us_run_tick(us_run_t *r, bool on, int32_t swb, bool any_blocked, bool cut);

#define US_EV_START  0
#define US_EV_CHANGE 1
#define US_EV_END    2

// Định dạng một dòng (kết thúc "\n") vào buf. ev_seq: first_seq (start, change) hoặc last_data_seq
// (end). Trả số byte (không kể '\0'); < 0 hay >= n là lỗi (dòng không gửi).
int us_format_line(char *buf, size_t n, int ev, const us_run_t *r, uint32_t ev_seq, uint32_t line_drop);

// Số chỗ trống hàng đợi logger cần có trước khi đưa sự kiện ev vào (mục (d)); ev lạ: 0 (không đưa vào).
int us_queue_need(int ev);

#endif // US_STATE_H
