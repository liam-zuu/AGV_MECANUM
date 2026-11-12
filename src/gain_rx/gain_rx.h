#ifndef GAIN_RX_H
#define GAIN_RX_H

#include <stdbool.h>
#include <stdint.h>
#include "gain_set.h"

// ─────────────────────────────────────────
// Nhận bộ gain qua WiFi (L90, Liam 12/11). UDP cổng GAIN_UDP_PORT, gói "#gain_set,…" hoặc
// "#gain_get,…" (gain_set.h). Task gain_rx (lõi 0, ưu tiên 1) đọc khuôn, chuyển yêu cầu cho
// control_task; control_task áp khi motor tắt và trả kết quả kèm bản chụp bộ gain đang chạy
// (gain_rx_poll / gain_rx_done); gain_rx trả "#gain_ack,…" về đúng IP:cổng đã gửi.
// Sửa theo Codex vòng 1 (12/11, Liam "Duyệt sửa cả 12"): mã giao dịch nội bộ (L90-3), thời điểm
// nhận (L90-1), MAC + mã khởi động trong trả lời (L90-8), task mở sau vel_ctrl_init (L90-12).
// Thiết kế: 03_CODE/docs/2026-11-12_Thiet_ke_doi_gain_qua_WiFi.md.
// ─────────────────────────────────────────

#define GAIN_UDP_PORT      5014    // trống: log 5005, S06 5007/5011, quỹ đạo 5010, μ sàn 5013
#define GAIN_REPLY_WAIT_MS 300     // chờ control_task (tick 10 ms) trả lời

typedef struct {
    uint32_t   txn;        // mã giao dịch nội bộ, tăng mỗi yêu cầu (không phải id bộ gain)
    int        kind;       // GAIN_MSG_SET / GAIN_MSG_GET
    int64_t    t_rx_us;    // esp_timer lúc recvfrom trả gói
    gain_set_t set;        // chỉ GAIN_MSG_SET
} gain_req_t;

// Gọi sau vel_ctrl_init() (gói tới trước đó không đọc struct controller chưa khởi tạo).
void gain_rx_init(void);

// control_task, mỗi tick: lấy yêu cầu đang chờ (không chặn). true nếu có.
bool gain_rx_poll(gain_req_t *req);
// control_task: kết quả của yêu cầu vừa lấy; gọi trong control_task nên bản chụp gain đi kèm
// (gain_set_current) không đọc lúc đang áp.
void gain_rx_done(const gain_req_t *req, int err);
// Mã khởi động ngẫu nhiên (đặt trong gain_rx_init, đổi mỗi lần boot), cũng ở #gain_ack; L95-8 ghi vào #us
uint32_t gain_rx_boot_id(void);

#endif
