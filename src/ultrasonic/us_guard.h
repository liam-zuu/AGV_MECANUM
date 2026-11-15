#ifndef US_GUARD_H
#define US_GUARD_H

#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────
// Lọc, khóa, mở khóa, giảm tốc theo khoảng cách của siêu âm (CODE_US, Liam 14/11 phiên CODE_US:
// "Gộp, sửa luôn", "Vật xa + nhả ga", "Có, giảm tốc theo khoảng cách", "Duyệt, Codex trước buổi";
// thiết kế hil 03_CODE/docs/2026-11-14_Thiet_ke_CODE_US_v1.md mục 11). File C thuần (không gọi
// ESP-IDF) để test PC dùng chung. Chỉ control_task gọi.
//
// Hướng: 0 trước (vx > 0), 1 sau (vx < 0), 2 trái (vy > 0), 3 phải (vy < 0) — như us_id_t.
//
// Mỗi mẫu của một cảm biến là một trong ba loại:
//   US_K_DIST    có khoảng cách (cm)
//   US_K_FAR     không có vọng trong tầm (tính là xa, KHÔNG bỏ mẫu: một mẫu ngắn lẻ không đi thẳng ra)
//   US_K_UNKNOWN cảm biến còn bận lúc phát, xung quá ngắn, không lên: giữ trạng thái khóa và chuỗi gần,
//                nhưng cắt chuỗi xa (US-3, Codex vòng 1)
// Khóa một hướng khi lock_n mẫu liền nhau DIST < lock_cm (mẫu UNKNOWN không cắt chuỗi gần). Mở khóa khi
// đủ cả: unlock_n mẫu liền nhau xa (FAR, hay DIST > unlock_cm) VÀ trong chuỗi xa đó có ít nhất một tick
// lệnh về hướng đó bằng 0 (MANUAL: ga về giữa; AUTO: PC gửi 0). unknown_n mẫu UNKNOWN liền nhau thì
// coi như một mẫu gần và bật cờ lỗi (an toàn: cảm biến hỏng thì khóa; SWB tắt được).
// Giảm tốc: d_slow = lớn hơn trong hai khoảng cách đã biết gần nhất (một mẫu lẻ không làm chậm xe);
// d_slow < slow_cm thì tốc độ lớn nhất về hướng đó = v_near + (v_ref − v_near)·(d_slow − lock_cm) /
// (slow_cm − lock_cm), kẹp trong [v_near, v_ref]; hướng bị khóa: 0.
// ─────────────────────────────────────────

#define US_G_N 4

#define US_K_UNKNOWN 0
#define US_K_DIST    1
#define US_K_FAR     2

#define US_G_FAR_CM  1000.0f   // khoảng cách gán cho mẫu FAR (ngoài mọi vùng)

typedef struct {
    float   lock_cm;     // 30
    float   unlock_cm;   // 40
    uint8_t lock_n;      // 2
    uint8_t unlock_n;    // 6 (0,3 s ở 20 Hz)
    uint8_t unknown_n;   // 10 (0,5 s)
    float   slow_cm;     // 80
    float   v_near;      // 0,15 m/s
    float   v_ref;       // 1,0 m/s (tốc độ đầy ở slow_cm)
} us_guard_cfg_t;

void us_guard_cfg_default(us_guard_cfg_t *c);

typedef struct {
    uint8_t  locked[US_G_N];
    uint8_t  near_run[US_G_N];
    uint8_t  far_run[US_G_N];
    uint8_t  unk_run[US_G_N];
    uint8_t  released[US_G_N];   // trong chuỗi xa hiện tại đã có tick lệnh hướng đó = 0
    uint8_t  fault[US_G_N];      // unknown_n mẫu UNKNOWN liền nhau (xóa khi có mẫu biết)
    uint8_t  have[US_G_N];       // số khoảng cách đã biết (0..2) cho d_slow
    float    d_last[US_G_N];     // khoảng cách đã biết gần nhất (cm; FAR = US_G_FAR_CM)
    float    d_prev[US_G_N];     // khoảng cách đã biết trước đó
    uint32_t locks;              // số lần khóa (đếm cho log)
} us_guard_t;

void us_guard_reset(us_guard_t *g);
// Một mẫu mới của cảm biến i (kind, cm). Trả true nếu trạng thái khóa của i đổi.
bool us_guard_sample(us_guard_t *g, const us_guard_cfg_t *c, int i, uint8_t kind, float cm);
// Mỗi tick: toward[i] = lệnh (trước khi kẹp) có đi về hướng i không. Trả true nếu có hướng mở khóa.
bool us_guard_intent(us_guard_t *g, const us_guard_cfg_t *c, const bool toward[US_G_N]);
// Tốc độ lớn nhất cho phép về hướng i (m/s, ≥ 0); không giới hạn thì trả số âm.
float us_guard_vmax(const us_guard_t *g, const us_guard_cfg_t *c, int i);
// Kẹp thành phần vận tốc đi về phía hướng bị khóa / trong vùng chậm. Trả true nếu đổi vx hay vy.
bool us_guard_clamp(const us_guard_t *g, const us_guard_cfg_t *c, float *vx, float *vy);
bool us_guard_any_locked(const us_guard_t *g);

// ─────────────────────────────────────────
// Nhận mẫu từ hàng đợi (US2-1, Codex vòng 2; Liam 15/11 phiên CODE_US "Sửa hẹp + Codex vòng 3"). Mỗi mẫu
// mang giờ phát t_trig và số phát ping (từ 1). Mẫu cũ hay mất không được mở khóa:
//   - bộ đếm tràn hàng đợi đổi: nạp UNKNOWN cho cả 4 TRƯỚC khi lấy mẫu tồn (us_feed_overflow);
//   - hở số phát của một cảm biến (mẫu giữa bị mất): nạp UNKNOWN trước mẫu đó;
//   - mẫu lấy ra muộn hơn US_ITEM_MAX_AGE_US kể từ lúc phát: dùng như UNKNOWN;
//   - một cảm biến quá US_STALE_US (theo giờ phát) không có mẫu: nạp UNKNOWN (us_feed_stale).
// UNKNOWN cắt chuỗi xa (US-3), nên chuỗi 6 mẫu xa để mở khóa phải toàn mẫu mới, liền số phát.
// Sửa theo Codex vòng 3 (Liam 15/11 phiên CODE_US "Sửa, đo, vòng 4 sau"):
//   - US3-1: control lấy hết mẫu ra mảng tạm RỒI mới gọi us_feed_overflow, và truyền giờ đọc ngay lúc xử
//     lý (không phải giờ đầu tick); tuổi âm cũng là UNKNOWN; hai mẫu liền số phát mà giờ phát cách nhau
//     quá US_STALE_US (cả hai lõi đã đứng) thì nạp UNKNOWN trước mẫu sau;
//   - US3-2: cờ have_ping riêng (số phát 0 hợp lệ sau quay vòng);
//   - cảm biến im: nạp UNKNOWN mỗi US_STALE_REPEAT_US (không mỗi US_STALE_US), khóa lỗi sau khoảng
//     0,2 s + 10 × 0,05 s ≈ 0,7 s (ước lượng theo mã, chưa đo; trước là khoảng 2,3 s);
//   - mẫu có giờ phát trước lần đặt lại (SWB) bị bỏ.
// ─────────────────────────────────────────
#define US_STALE_US        200000LL   // 4 chu kỳ phát (US-4)
#define US_STALE_REPEAT_US  50000LL   // một chu kỳ phát: nhịp nạp UNKNOWN khi cảm biến vẫn im
#define US_ITEM_MAX_AGE_US 100000LL   // 2 chu kỳ phát; mẫu lấy ra muộn hơn (hay tuổi âm) là UNKNOWN

typedef struct {
    int64_t  last_t[US_G_N];     // giờ phát của mẫu gần nhất (hay mốc nạp UNKNOWN bù)
    uint32_t last_ping[US_G_N];  // số phát của mẫu gần nhất (có nghĩa khi have_ping)
    uint8_t  have_ping[US_G_N];  // đã có mẫu từ lần đặt lại
    int64_t  t_reset;            // giờ đặt lại: mẫu có giờ phát trước đó bị bỏ
    uint32_t ovf_seen;           // bộ đếm tràn đã xử lý
    uint32_t n_gap, n_old, n_stale, n_ovf, n_silent, n_pre_reset;   // đếm theo lý do (log, test)
} us_feed_t;

void us_feed_reset(us_feed_t *f, int64_t t_now, uint32_t ovf_cnt);
// Gọi mỗi tick SAU khi đã lấy hết mẫu ra mảng tạm, TRƯỚC khi xử lý chúng (US3-1). Trả true nếu bộ đếm tràn
// đổi (đã nạp UNKNOWN cho cả 4).
bool us_feed_overflow(us_feed_t *f, us_guard_t *g, const us_guard_cfg_t *c, uint32_t ovf_cnt, int64_t t_now);
// Một mẫu lấy ra từ hàng đợi; t_now = giờ đọc lúc xử lý. Trả kết quả us_guard_sample của mẫu (true nếu
// trạng thái khóa đổi); mẫu bị bỏ (trước đặt lại, chỉ số sai) trả false.
bool us_feed_item(us_feed_t *f, us_guard_t *g, const us_guard_cfg_t *c, int i, uint8_t kind, float cm,
                  int64_t t_trig, uint32_t ping, int64_t t_now);
// Gọi mỗi tick SAU khi xử lý hết mẫu; no_queue = không có hàng đợi (mọi cảm biến UNKNOWN).
void us_feed_stale(us_feed_t *f, us_guard_t *g, const us_guard_cfg_t *c, int64_t t_now, bool no_queue);

#endif // US_GUARD_H
