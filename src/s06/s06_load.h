#ifndef S06_LOAD_H
#define S06_LOAD_H

// Env đo nghiệm thu S06 (build với -DS06_LOAD; env s06_load). Thiết kế:
// 03_CODE/docs/2026-10-29_Thiet_ke_env_s06_load.md (repo luận văn). Liam chốt 29/10: đọc encoder
// 1 ms bằng esp_timer (Q-S1), lệnh START/STOP qua UDP WiFi từ Pi (Q-S2).
//
// Thay controller: PWM theo lịch của hồ sơ; ghi sự kiện (IMU từng report, đọc encoder 1 ms, ghi
// compare, đầu/cuối lượt, thống kê 1 Hz) qua UDP tới Pi, cổng S06_EVT_PORT, mỗi luồng có seq riêng.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Gọi một lần trong app_main sau logger_init(), trước khi tạo task.
void s06_init(void);

// Gọi mỗi tick control_task (10 ms) khi motor bật: kiểm hết lượt, trả lệnh từng bánh
// (−1000..+1000) theo lịch; ngoài lượt trả 0.
void s06_tick(int64_t t_now, int speed_out[4]);

// Áp lệnh 4 bánh: chỉ ghi compare bánh nào đổi lệnh (hoặc khi force), ghi sự kiện P cho mỗi lần ghi.
void s06_apply(const int speed[4], bool force);

#ifdef __cplusplus
}
#endif

#endif // S06_LOAD_H
