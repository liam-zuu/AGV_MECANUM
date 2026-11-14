#include "us_diag.h"
#include <stdio.h>

// CODE_US: xem us_diag.h

const char *us_diag_res_name(uint8_t res) {
    switch (res) {
    case US_DIAG_RES_OK:     return "ok";
    case US_DIAG_RES_NORISE: return "norise";
    case US_DIAG_RES_TMO:    return "tmo";
    case US_DIAG_RES_RNG:    return "rng";
    default:                 return "bad";
    }
}

static const char k_name[US_DIAG_N] = { 'f', 'b', 'l', 'r' };

int us_diag_format_cycle(char *buf, size_t n, const us_diag_cycle_t *c) {
    if (buf == NULL || n == 0 || c == NULL) return -1;
    int len = snprintf(buf, n, "#usr,t=%lld,cyc=%lu,cyc_us=%lu,drop=%lu",
                       (long long)c->t_us, (unsigned long)c->cyc, (unsigned long)c->cyc_us,
                       (unsigned long)c->drop);
    if (len < 0 || (size_t)len >= n) return -1;
    for (int i = 0; i < US_DIAG_N; i++) {
        const us_diag_sample_t *s = &c->s[i];
        int m = snprintf(buf + len, n - (size_t)len, ",%c=%u/%s/%lu/%lu/%lu/%lu/%.1f/%u",
                         k_name[i], (unsigned)s->pre, us_diag_res_name(s->res),
                         (unsigned long)s->rise_us, (unsigned long)s->w_us, (unsigned long)s->gap_us,
                         (unsigned long)s->dtp_us, (double)c->d_f[i], (unsigned)c->nv[i]);
        if (m < 0 || (size_t)m >= n - (size_t)len) return -1;
        len += m;
    }
    if ((size_t)len + 1 >= n) return -1;
    buf[len++] = '\n';
    buf[len] = '\0';
    return len;
}

int us_diag_format_blk(char *buf, size_t n, const us_diag_blk_t *b) {
    if (buf == NULL || n == 0 || b == NULL) return -1;
    return snprintf(buf, n, "#usb,t=%lld,seq=%lu,drop=%lu,blk=%u%u%u%u,d=%.1f/%.1f/%.1f/%.1f\n",
                    (long long)b->t_us, (unsigned long)b->seq, (unsigned long)b->drop,
                    (unsigned)b->blk[0], (unsigned)b->blk[1], (unsigned)b->blk[2], (unsigned)b->blk[3],
                    (double)b->d[0], (double)b->d[1], (double)b->d[2], (double)b->d[3]);
}
