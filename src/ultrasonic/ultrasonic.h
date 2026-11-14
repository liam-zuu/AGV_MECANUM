#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

// GPIO
#define ULTRASONIC_TRIG     39
#define ULTRASONIC_ECHO_0   35
#define ULTRASONIC_ECHO_1   36
#define ULTRASONIC_ECHO_2   37
#define ULTRASONIC_ECHO_3   38

#define ULTRASONIC_COUNT    4
#define ULTRASONIC_MIN_CM       2
#define ULTRASONIC_MAX_CM       400

typedef enum {
    US_FRONT = 0,
    US_BACK,
    US_LEFT,
    US_RIGHT,
} us_id_t;

// ─────────────────────────────────────────
// Đo bằng ngắt cạnh (CODE_US, Liam 14/11 phiên CODE_US "Duyệt, Codex trước buổi (Recommended)";
// thiết kế hil 03_CODE/docs/2026-11-14_Thiet_ke_CODE_US_v1.md mục 11.3 ý 1). TRIG chung (Liam 14/11:
// "Đúng, 1 dây chung") nên mỗi lần phát cả 4 cảm biến cùng phát; nghe cả 4 ECHO của CÙNG lần phát.
// Ngắt GPIO hai cạnh ghi giờ esp_timer lúc ECHO lên, xuống: task khác chen không làm sai số đo
// (thay vòng chờ bận của 5bde4c5, nghi vấn G4). Chỉ ultrasonic_task gọi ping/collect.
//
// Kết quả mỗi cảm biến mỗi lần phát (kind theo us_guard.h):
//   US_K_DIST    xung xuống trong tầm, 2–400 cm
//   US_K_FAR     không có vọng trong tầm: xung dài hơn 400 cm, hay còn cao ở lần thu cuối; hoặc cảm biến
//                còn cao từ lần phát trước mà lần trước đã là FAR (không nghe vọng nào từ đó, mẫu cũ ≤ 1 chu kỳ)
//   US_K_UNKNOWN còn cao lúc phát mà lần trước không phải FAR; không lên tới lần thu cuối; xung < 2 cm
// ─────────────────────────────────────────

#define US_PING_MS        50      // chu kỳ phát (20 Hz) [ĐỀ XUẤT, chốt theo buổi đo B1]
#define US_COLLECT1_TICKS 1       // thu lần 1 sau 1 tick (khoảng 10 ms): vật gần (< khoảng 150 cm) đã xong
#define US_COLLECT2_TICKS 3       // thu cuối sau 3 tick (khoảng 30 ms): đủ cho 400 cm (23,3 ms + lúc lên)
#define US_W_MAX_US       23324u  // 400 cm × 2 / 0,0343

typedef struct {
    uint8_t  kind;      // US_K_*
    float    cm;        // chỉ có nghĩa khi DIST
    uint8_t  pre;       // ECHO cao lúc phát
    uint8_t  stage;     // 1 = thu lần 1, 2 = thu cuối, 0 = quyết ngay lúc phát (pre)
    uint16_t edges;     // số cạnh ngắt thấy từ lúc phát tới lúc quyết (> 2: có xung nhiễu)
    uint32_t rise_us;   // từ lúc phát tới lúc ECHO lên (0 nếu không lên)
    uint32_t w_us;      // độ rộng xung (DIST), hay thời gian cao tới lúc quyết (FAR)
} us_meas_t;

void ultrasonic_init(void);
// Phát một lần cho cả 4; trả giờ phát (esp_timer). Cảm biến còn cao lúc phát được quyết ngay (stage 0).
int64_t ultrasonic_ping(void);
// Thu kết quả của lần phát gần nhất. final = false: chỉ quyết cảm biến đã xong; true: quyết hết.
// Trả mặt nạ bit các cảm biến được quyết TRONG lần gọi này (bit i ↔ out[i] mới).
uint8_t ultrasonic_collect(us_meas_t out[ULTRASONIC_COUNT], bool final);

// Bản chặn cho TEST_ULTRASONIC (src/tests/test_ultrasonic.c): một lần phát, chờ, thu cuối.
typedef struct {
    float distance_cm[ULTRASONIC_COUNT];
    bool  valid[ULTRASONIC_COUNT];
} us_data_t;
void ultrasonic_read_all(us_data_t *data);

#endif
