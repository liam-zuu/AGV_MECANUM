#include "us_diag.h"
#include "us_guard.h"
#include <stdio.h>

// CODE_US: xem us_diag.h

char us_diag_kind_char(uint8_t kind) {
    switch (kind) {
    case US_K_UNKNOWN: return 'u';
    case US_K_DIST:    return 'd';
    case US_K_FAR:     return 'f';
    default:           return '?';
    }
}

static const char k_name[US_DIAG_N] = { 'f', 'b', 'l', 'r' };

int us_diag_format_ping(char *buf, size_t n, const us_diag_ping_t *p) {
    if (buf == NULL || n == 0 || p == NULL) return -1;
    int len = snprintf(buf, n, "#usp,t=%lld,seq=%lu,drop=%lu",
                       (long long)p->t_us, (unsigned long)p->seq, (unsigned long)p->drop);
    if (len < 0 || (size_t)len >= n) return -1;
    for (int i = 0; i < US_DIAG_N; i++) {
        const us_diag_meas_t *m = &p->m[i];
        int k = snprintf(buf + len, n - (size_t)len, ",%c=%c/%.2f/%u/%u/%u/%lu/%lu/%lu/%c/%u",
                         k_name[i], us_diag_kind_char(m->kind), (double)m->cm, (unsigned)m->pre,
                         (unsigned)m->stage, (unsigned)m->edges, (unsigned long)m->rise_us,
                         (unsigned long)m->w_us, (unsigned long)m->dec_us, m->why ? m->why : '-',
                         (unsigned)m->glitch);
        if (k < 0 || (size_t)k >= n - (size_t)len) return -1;
        len += k;
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
