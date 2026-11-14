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
//   US_K_UNKNOWN cảm biến còn bận lúc phát, xung quá ngắn, không lên: giữ nguyên trạng thái
// Khóa một hướng khi lock_n mẫu liền nhau DIST < lock_cm (mẫu UNKNOWN không cắt chuỗi). Mở khóa khi
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

#endif // US_GUARD_H
