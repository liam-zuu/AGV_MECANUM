#ifndef GAIN_RX_H
#define GAIN_RX_H

#include <stdbool.h>
#include <stdint.h>
#include "gain_set.h"

// ─────────────────────────────────────────
// Nhận bộ gain qua WiFi (L90, Liam 12/11). UDP cổng GAIN_UDP_PORT, một gói "#gain_set,…"
// (gain_set.h). Task gain_rx (lõi 0, ưu tiên 1) đọc khuôn, chuyển yêu cầu cho control_task; control_task
// áp khi motor tắt (gain_rx_poll / gain_rx_done); gain_rx trả "#gain_ack,…" về đúng IP:cổng đã gửi,
// kèm gain đang chạy sau khi áp. Thiết kế: 03_CODE/docs/2026-11-12_Thiet_ke_doi_gain_qua_WiFi.md.
// ─────────────────────────────────────────

#define GAIN_UDP_PORT      5014    // trống: log 5005, S06 5007/5011, quỹ đạo 5010, μ sàn 5013
#define GAIN_REPLY_WAIT_MS 300     // chờ control_task (tick 10 ms) trả lời

void gain_rx_init(void);

// control_task, mỗi tick: lấy yêu cầu đang chờ (không chặn). true nếu có.
bool gain_rx_poll(gain_set_t *req);
// control_task: kết quả của yêu cầu vừa lấy (GAIN_OK hay mã lỗi gain_set.h).
void gain_rx_done(uint32_t id, int err);

#endif
