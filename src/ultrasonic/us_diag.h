#ifndef US_DIAG_H
#define US_DIAG_H

#include <stdint.h>
#include <stddef.h>

// ─────────────────────────────────────────
// Chẩn đoán siêu âm (CODE_US, Liam 14/11 phiên CODE_US "Duyệt, code luôn (Recommended)";
// thiết kế hil 03_CODE/docs/2026-11-14_Thiet_ke_CODE_US_v1.md mục 5). Chỉ có trong env
// us_diag, ladrc_us_diag (-DUS_DIAG); thuật toán đo, lọc, ngưỡng, cắt lệnh giữ nguyên như 5bde4c5.
// File C thuần (không gọi ESP-IDF) để test PC dùng chung.
//
// Hai dòng, bắt đầu bằng '#' (PC tách khỏi CSV, agv_logger ghi vào <tên>.csv.events.txt):
//   #usr,t=..,cyc=..,cyc_us=..,drop=..,f=..,b=..,l=..,r=..
//       một dòng mỗi vòng ultrasonic_read_all; t = esp_timer lúc phát TRIG đầu vòng (cùng đồng hồ
//       với timestamp_us của CSV); cyc_us = t − t của vòng trước (0 ở vòng đầu).
//       Mỗi cảm biến (f trước, b sau, l trái, r phải) một trường pre/res/rise/w/gap/dtp/d_f/nv:
//         pre  1 nếu ECHO đã cao ngay trước khi phát TRIG (cảm biến còn bận, G2)
//         res  ok | norise (ECHO không lên trong 5 ms) | tmo (xung quá 12 ms) | rng (ngoài 2–400 cm)
//         rise µs từ lúc phát TRIG tới lúc thấy ECHO cao
//         w    µs độ rộng xung đo được (tmo: tới lúc bỏ; norise: 0)
//         gap  µs lớn nhất giữa hai lần đọc chân liên tiếp trong mẫu (bị task khác chen, G4)
//         dtp  µs từ lần phát TRIG trước (TRIG chung: lần trước cả 4 cảm biến cùng phát; G3); 0 = chưa có
//         d_f  cm giá trị SAU lọc trung vị của cảm biến này ở vòng này (−1 = không hợp lệ)
//         nv   số mẫu hợp lệ trong 3 mẫu lọc (G1: nv = 1 là mẫu lẻ đi thẳng ra)
//   #usb,t=..,seq=..,drop=..,blk=FBLR,d=f/b/l/r
//       mỗi khi một cờ chặn s_*_blocked đổi (control_task, bước 4); t = t_now của tick; seq =
//       log_seq dòng dữ liệu kế tiếp (khi motor tắt dòng dữ liệu chỉ 10 Hz nên seq chỉ để tham khảo);
//       blk = 4 chữ số 0/1 theo thứ tự trước, sau, trái, phải; d = giá trị lọc control đã dùng (−1 =
//       không hợp lệ).
//   drop: số bản ghi chẩn đoán không vào được hàng đợi chẩn đoán tính tới trước bản ghi này.
// ─────────────────────────────────────────

#define US_DIAG_N 4

#define US_DIAG_RES_OK     0
#define US_DIAG_RES_NORISE 1
#define US_DIAG_RES_TMO    2
#define US_DIAG_RES_RNG    3

typedef struct {
    int64_t  t_trig;   // esp_timer lúc phát TRIG
    uint32_t rise_us;
    uint32_t w_us;
    uint32_t gap_us;
    uint32_t dtp_us;
    uint8_t  pre;
    uint8_t  res;
} us_diag_sample_t;

typedef struct {
    int64_t          t_us;
    uint32_t         cyc;
    uint32_t         cyc_us;
    uint32_t         drop;
    us_diag_sample_t s[US_DIAG_N];
    float            d_f[US_DIAG_N];
    uint8_t          nv[US_DIAG_N];
} us_diag_cycle_t;

typedef struct {
    int64_t  t_us;
    uint32_t seq;
    uint32_t drop;
    uint8_t  blk[US_DIAG_N];
    float    d[US_DIAG_N];
} us_diag_blk_t;

const char *us_diag_res_name(uint8_t res);

// Định dạng một dòng (kết thúc "\n") vào buf. Trả số byte (không kể '\0'); < 0 hay >= n là lỗi.
int us_diag_format_cycle(char *buf, size_t n, const us_diag_cycle_t *c);
int us_diag_format_blk(char *buf, size_t n, const us_diag_blk_t *b);

#endif // US_DIAG_H
