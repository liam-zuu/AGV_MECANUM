#ifndef IBUS_AGE_H
#define IBUS_AGE_H

#include <stdint.h>
#include <stdbool.h>

// Tuổi khung iBUS hợp lệ (L95-1, Codex Sol vòng 1, Liam 13/11 "Sửa cả 8 điểm + vòng 2"): khung đúng
// checksum gần nhất quá IBUS_FRAME_MAX_AGE_US thì ibus_get() trả không hợp lệ (mất tay cầm: motor tắt,
// siêu âm bật), kể cả khi UART vẫn có byte (byte rác không thành khung). Khung iBUS khoảng 7 ms (FS-i6X,
// H7 ibus_gen), nên 50 ms là khoảng 7 khung. Hàm thuần để test PC (hil fwsim/tests/test_us_state.py).
// Giới hạn (Codex L95 vòng 2): tuổi tính từ lúc ibus_task PARSE khung, không từ lúc khung tới chân RX; khung
// nằm trong bộ đệm UART lâu (ibus_task không được chạy) vẫn được coi là mới khi vừa parse.
#define IBUS_FRAME_MAX_AGE_US 50000LL

// last_ok_us = esp_timer lúc khung đúng checksum gần nhất; 0 = chưa có khung nào
static inline bool ibus_frame_fresh(int64_t now_us, int64_t last_ok_us) {
    if (last_ok_us <= 0) return false;
    int64_t age = now_us - last_ok_us;
    return age >= 0 && age <= IBUS_FRAME_MAX_AGE_US;
}

#endif // IBUS_AGE_H
