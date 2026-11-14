#ifndef US_DIAG_H
#define US_DIAG_H

#include <stdint.h>
#include <stddef.h>

// ─────────────────────────────────────────
// Chẩn đoán siêu âm (CODE_US; thiết kế hil 03_CODE/docs/2026-11-14_Thiet_ke_CODE_US_v1.md mục 5, 11).
// Chỉ có trong env us_diag, ladrc_us_diag (-DUS_DIAG). File C thuần (không gọi ESP-IDF) để test PC.
//
// Từ bản sửa (đo bằng ngắt cạnh, ultrasonic.h) có hai dòng, bắt đầu bằng '#' (agv_logger ghi vào
// <tên>.csv.events.txt). Dòng #usr của bản đo cũ (vòng chờ bận) chỉ có ở feat/us-diag f841c43.
//   #usp,t=..,seq=..,drop=..,f=..,b=..,l=..,r=..
//       một dòng mỗi lần phát, gửi sau lần thu cuối; t = esp_timer lúc phát (cùng đồng hồ với
//       timestamp_us của CSV); seq = số lần phát. Mỗi cảm biến (f trước, b sau, l trái, r phải):
//       k/cm/pre/stage/edges/rise/w
//         k     d (có khoảng cách) | f (xa, không vọng trong tầm) | u (không biết)
//         cm    khoảng cách (−1 nếu không phải d)
//         pre   1 nếu ECHO đã cao lúc phát
//         stage 0 quyết lúc phát (pre), 1 lần thu sau khoảng 10 ms, 2 lần thu cuối sau khoảng 30 ms
//         edges số cạnh ngắt từ lúc phát tới lúc quyết (> 2: có xung nhiễu trên dây ECHO)
//         rise  µs từ lúc phát tới lúc ECHO lên; w µs độ rộng xung (f: thời gian cao tới lúc quyết)
//   #usb,t=..,seq=..,drop=..,blk=FBLR,d=f/b/l/r
//       mỗi khi trạng thái khóa một hướng đổi (control_task); t = t_now của tick; seq = log_seq dòng dữ
//       liệu kế tiếp; blk = 4 chữ số 0/1 theo thứ tự trước, sau, trái, phải; d = khoảng cách đã biết gần
//       nhất của us_guard (cm; 1000 = xa).
//   drop: số bản ghi chẩn đoán không vào được hàng đợi chẩn đoán tính tới trước bản ghi này.
// ─────────────────────────────────────────

#define US_DIAG_N 4

typedef struct {
    uint8_t  kind;     // US_K_* (us_guard.h)
    uint8_t  pre;
    uint8_t  stage;
    uint16_t edges;
    float    cm;
    uint32_t rise_us;
    uint32_t w_us;
} us_diag_meas_t;

typedef struct {
    int64_t        t_us;
    uint32_t       seq;
    uint32_t       drop;
    us_diag_meas_t m[US_DIAG_N];
} us_diag_ping_t;

typedef struct {
    int64_t  t_us;
    uint32_t seq;
    uint32_t drop;
    uint8_t  blk[US_DIAG_N];
    float    d[US_DIAG_N];
} us_diag_blk_t;

char us_diag_kind_char(uint8_t kind);   // 'u', 'd', 'f'; '?' khi lạ

// Định dạng một dòng (kết thúc "\n") vào buf. Trả số byte (không kể '\0'); < 0 hay >= n là lỗi.
int us_diag_format_ping(char *buf, size_t n, const us_diag_ping_t *p);
int us_diag_format_blk(char *buf, size_t n, const us_diag_blk_t *b);

#endif // US_DIAG_H
