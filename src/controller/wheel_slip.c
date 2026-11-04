#include "wheel_slip.h"
#include <math.h>
#include <string.h>

static wheel_slip_params_t P = WHEEL_SLIP_PARAMS_DEFAULT;

static struct {
    int32_t  hist[WHEEL_SLIP_WIN_MAX + 1][4];   // vòng: số đếm của win+1 tick gần nhất
    uint16_t head;        // chỗ ghi tick kế tiếp
    uint16_t n;           // số tick đã ghi (≤ win+1)
    bool     flag;
    int64_t  above_since; // −1 = chưa
    int64_t  below_since; // −1 = chưa (dưới th/2 hoặc không xét được)
    uint32_t on_count;
} S;

void wheel_slip_init(const wheel_slip_params_t *p) {
    if (p) P = *p;
    else   P = (wheel_slip_params_t)WHEEL_SLIP_PARAMS_DEFAULT;
    if (P.win_ticks < 1) P.win_ticks = 1;
    if (P.win_ticks > WHEEL_SLIP_WIN_MAX) P.win_ticks = WHEEL_SLIP_WIN_MAX;
    memset(&S, 0, sizeof(S));
    S.above_since = -1;
    S.below_since = -1;
}

wheel_slip_params_t *wheel_slip_params(void) { return &P; }

void wheel_slip_step(const wheel_slip_in_t *in, wheel_slip_out_t *out) {
    uint16_t win = P.win_ticks;
    if (win < 1) win = 1;
    if (win > WHEEL_SLIP_WIN_MAX) win = WHEEL_SLIP_WIN_MAX;
    const uint16_t len = (uint16_t)(win + 1);

    for (int i = 0; i < 4; i++) S.hist[S.head][i] = in->count[i];
    uint16_t cur = S.head;
    S.head = (uint16_t)((S.head + 1) % len);
    if (S.n < len) S.n++;

    float res = 0.0f, th = P.n_min;
    bool eval = false;
    if (S.n == len) {
        uint16_t old = S.head;                   // tick cũ nhất trong vòng = win tick trước
        float d[4], sum_abs = 0.0f, dmax = 0.0f;
        for (int i = 0; i < 4; i++) {
            d[i] = (float)(S.hist[cur][i] - S.hist[old][i]);
            float a = fabsf(d[i]);
            sum_abs += a;
            if (a > dmax) dmax = a;
        }
        res = d[0] + d[1] - d[2] - d[3];
        float rel = P.k_rel * sum_abs;
        if (rel > th) th = rel;
        eval = dmax >= P.act_min;
    }

    int64_t now = in->now_us;
    if (!eval) {
        S.above_since = -1;
        if (S.flag) {                            // không xét được: tắt sau t_hold
            if (S.below_since < 0) S.below_since = now;
            if (now - S.below_since >= P.t_hold_us) { S.flag = false; S.below_since = -1; }
        } else {
            S.below_since = -1;
        }
    } else {
        float ar = fabsf(res);
        if (ar > th) {
            S.below_since = -1;
            if (S.above_since < 0) S.above_since = now;
            if (!S.flag && now - S.above_since >= P.t_hold_us) { S.flag = true; S.on_count++; }
        } else if (ar < 0.5f * th) {
            S.above_since = -1;
            if (S.below_since < 0) S.below_since = now;
            if (S.flag && now - S.below_since >= P.t_hold_us) S.flag = false;
        } else {                                 // giữa th/2 và th: giữ nguyên trạng thái
            S.above_since = -1;
            S.below_since = -1;
        }
    }

    out->res      = res;
    out->th       = th;
    out->eval     = eval;
    out->flag     = S.flag;
    out->on_count = S.on_count;
}
