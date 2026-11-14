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
// Kết quả mỗi cảm biến mỗi lần phát (kind theo us_guard.h; why cho biết vì sao, ghi vào log chẩn đoán):
//   US_K_DIST    một xung 2–400 cm, không có cạnh thừa sau nó (xung < 2 cm trước đó bị bỏ, nghe tiếp)
//   US_K_FAR     không có vọng trong tầm: xung dài hơn 400 cm, hay còn cao ở lần thu cuối; hoặc cảm biến
//                còn cao lúc phát mà lần đo MỚI trước là FAR: dùng lại FAR đúng một lần (why = REUSED)
//   US_K_UNKNOWN còn cao lúc phát lần thứ hai liền hay sau mẫu khác FAR (BUSY); không lên (NORISE); lên
//                muộn hơn 5 ms (LATE); có cạnh sau khi xung xong (EXTRA); quá US_EDGE_MAX cạnh (EDGES)
// Sửa theo Codex Sol vòng 1 (Liam 15/11 "Sửa cả 14 + vòng 2"): US-1, US-2, US-5, US-7, US-8, US-9, US-16.
// ─────────────────────────────────────────

#define US_PING_MS        50      // chu kỳ phát tối thiểu (20 Hz) [ĐỀ XUẤT, chốt theo buổi đo B1]
#define US_COLLECT1_US    10000u  // thu lần 1 khi đã ≥ 10 ms sau khi phát: vật gần (< khoảng 150 cm) đã xong
#define US_W_MAX_US       23324u  // 400 cm × 2 / 0,0343
#define US_COLLECT2_US    29000u  // thu cuối khi đã ≥ 29 ms sau khi phát: lên muộn nhất 5 ms + 23,3 ms (US-9)
#define US_EDGE_MAX       16      // quá số cạnh này trong một lần đo thì tắt ngắt chân đó, mẫu UNKNOWN (US-8)

#define US_WHY_NONE   '-'
#define US_WHY_REUSED 'r'
#define US_WHY_BUSY   'b'
#define US_WHY_NORISE 'n'
#define US_WHY_LATE   't'
#define US_WHY_EXTRA  'x'
#define US_WHY_EDGES  'e'

typedef struct {
    uint8_t  kind;      // US_K_*
    char     why;       // US_WHY_*
    float    cm;        // chỉ có nghĩa khi DIST
    uint8_t  pre;       // ECHO cao lúc phát
    uint8_t  stage;     // 1 = thu lần 1, 2 = thu cuối, 0 = quyết lúc thu lần 1 vì cao lúc phát
    uint16_t edges;     // số lần ngắt từ lúc phát tới lúc quyết
    uint8_t  glitch;    // số xung < 2 cm đã bỏ
    uint32_t rise_us;   // từ lúc phát tới lúc ECHO lên (0 nếu không lên)
    uint32_t w_us;      // độ rộng xung (DIST), hay thời gian cao tới lúc quyết (FAR)
    uint32_t dec_us;    // từ lúc phát tới lúc quyết mẫu (US-16)
} us_meas_t;

void ultrasonic_init(void);
// Phát một lần cho cả 4; trả giờ phát (esp_timer). Cảm biến còn cao lúc phát được quyết ngay (stage 0).
int64_t ultrasonic_ping(void);
// Thu kết quả của lần phát gần nhất. final = false: chỉ quyết cảm biến đã xong; true: quyết hết.
// Trả mặt nạ bit các cảm biến được quyết TRONG lần gọi này (bit i ↔ out[i] mới).
uint8_t ultrasonic_collect(us_meas_t out[ULTRASONIC_COUNT], bool final);
// Chờ tới khi esp_timer ≥ t (bước 1 tick; tick 100 Hz). ultrasonic_task và ultrasonic_read_all dùng (US2-9).
void ultrasonic_wait_until(int64_t t);

// Bản chặn cho TEST_ULTRASONIC (src/tests/test_ultrasonic.c): một lần phát, chờ, thu cuối.
typedef struct {
    float distance_cm[ULTRASONIC_COUNT];
    bool  valid[ULTRASONIC_COUNT];
} us_data_t;
void ultrasonic_read_all(us_data_t *data);

#endif
