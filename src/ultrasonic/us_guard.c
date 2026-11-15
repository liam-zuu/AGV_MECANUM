#include "us_guard.h"
#include <string.h>

// CODE_US: xem us_guard.h

void us_guard_cfg_default(us_guard_cfg_t *c) {
    c->lock_cm   = 30.0f;
    c->unlock_cm = 40.0f;
    c->lock_n    = 2;
    c->unlock_n  = 6;
    c->unknown_n = 10;
    c->slow_cm   = 80.0f;
    c->v_near    = 0.15f;
    c->v_ref     = 1.0f;
}

void us_guard_reset(us_guard_t *g) {
    memset(g, 0, sizeof(*g));
    for (int i = 0; i < US_G_N; i++) { g->d_last[i] = US_G_FAR_CM; g->d_prev[i] = US_G_FAR_CM; }
}

static void push_dist(us_guard_t *g, int i, float cm) {
    g->d_prev[i] = g->d_last[i];
    g->d_last[i] = cm;
    if (g->have[i] < 2) g->have[i]++;
}

bool us_guard_sample(us_guard_t *g, const us_guard_cfg_t *c, int i, uint8_t kind, float cm) {
    if (i < 0 || i >= US_G_N) return false;
    uint8_t was = g->locked[i];
    bool near = false, far = false;

    if (kind == US_K_UNKNOWN) {
        if (g->unk_run[i] < 255) g->unk_run[i]++;
        if (g->unk_run[i] < c->unknown_n) {
            // US-3 (Codex vòng 1, Liam 15/11): giữ chuỗi gần (mẫu bận giữa hai mẫu gần vẫn khóa) nhưng
            // cắt chuỗi xa và lần nhả ga của nó: không mở khóa khi không có unlock_n mẫu xa liền nhau
            g->far_run[i]  = 0;
            g->released[i] = 0;
            return false;
        }
        g->fault[i] = 1;                                     // cảm biến không trả lời: coi như gần
        near = true;
    } else {
        g->unk_run[i] = 0;
        g->fault[i]   = 0;
        if (kind == US_K_FAR) {
            push_dist(g, i, US_G_FAR_CM);
            far = true;
        } else {
            push_dist(g, i, cm);
            near = cm < c->lock_cm;
            far  = cm > c->unlock_cm;
        }
    }

    if (near) {
        if (g->near_run[i] < 255) g->near_run[i]++;
    } else {
        g->near_run[i] = 0;
    }
    if (far) {
        if (g->far_run[i] < 255) g->far_run[i]++;
    } else {
        g->far_run[i]  = 0;
        g->released[i] = 0;          // chuỗi xa mới phải tự có lần nhả ga của nó
    }

    if (!g->locked[i] && g->near_run[i] >= c->lock_n) {
        g->locked[i] = 1;
        g->locks++;
    }
    if (g->locked[i] && g->far_run[i] >= c->unlock_n && g->released[i]) {
        g->locked[i] = 0;
    }
    return was != g->locked[i];
}

bool us_guard_intent(us_guard_t *g, const us_guard_cfg_t *c, const bool toward[US_G_N]) {
    bool opened = false;
    for (int i = 0; i < US_G_N; i++) {
        if (!toward[i] && g->far_run[i] > 0) g->released[i] = 1;
        if (g->locked[i] && g->far_run[i] >= c->unlock_n && g->released[i]) {
            g->locked[i] = 0;
            opened = true;
        }
    }
    return opened;
}

float us_guard_vmax(const us_guard_t *g, const us_guard_cfg_t *c, int i) {
    if (i < 0 || i >= US_G_N) return -1.0f;
    if (g->locked[i]) return 0.0f;
    if (g->have[i] < 2) return -1.0f;
    float d = g->d_last[i] > g->d_prev[i] ? g->d_last[i] : g->d_prev[i];
    if (d >= c->slow_cm) return -1.0f;
    float span = c->slow_cm - c->lock_cm;
    float k = span > 0.0f ? (d - c->lock_cm) / span : 0.0f;
    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;
    return c->v_near + (c->v_ref - c->v_near) * k;
}

static bool limit(float *v, float sign, float vmax) {
    if (vmax < 0.0f) return false;              // không giới hạn
    if (sign > 0.0f && *v > vmax)  { *v = vmax;  return true; }
    if (sign < 0.0f && *v < -vmax) { *v = -vmax; return true; }
    return false;
}

bool us_guard_clamp(const us_guard_t *g, const us_guard_cfg_t *c, float *vx, float *vy) {
    bool ch = false;
    ch |= limit(vx, +1.0f, us_guard_vmax(g, c, 0));
    ch |= limit(vx, -1.0f, us_guard_vmax(g, c, 1));
    ch |= limit(vy, +1.0f, us_guard_vmax(g, c, 2));
    ch |= limit(vy, -1.0f, us_guard_vmax(g, c, 3));
    return ch;
}

bool us_guard_any_locked(const us_guard_t *g) {
    for (int i = 0; i < US_G_N; i++) if (g->locked[i]) return true;
    return false;
}

// US2-1: xem us_guard.h
void us_feed_reset(us_feed_t *f, int64_t t_now, uint32_t ovf_cnt) {
    memset(f, 0, sizeof(*f));
    for (int i = 0; i < US_G_N; i++) f->last_t[i] = t_now;
    f->t_reset  = t_now;
    f->ovf_seen = ovf_cnt;
}

bool us_feed_overflow(us_feed_t *f, us_guard_t *g, const us_guard_cfg_t *c, uint32_t ovf_cnt, int64_t t_now) {
    if (ovf_cnt == f->ovf_seen) return false;
    f->ovf_seen = ovf_cnt;
    f->n_ovf++;
    for (int i = 0; i < US_G_N; i++) {
        us_guard_sample(g, c, i, US_K_UNKNOWN, 0.0f);
        f->last_t[i] = t_now;
    }
    return true;
}

bool us_feed_item(us_feed_t *f, us_guard_t *g, const us_guard_cfg_t *c, int i, uint8_t kind, float cm,
                  int64_t t_trig, uint32_t ping, int64_t t_now) {
    if (i < 0 || i >= US_G_N) return false;
    if (t_trig < f->t_reset) {                                   // phát trước lần đặt lại (SWB)
        f->n_pre_reset++;
        return false;
    }
    if (f->have_ping[i] && ping != f->last_ping[i] + 1u) {       // US3-2: so theo modulo uint32_t
        f->n_gap++;
        us_guard_sample(g, c, i, US_K_UNKNOWN, 0.0f);            // mẫu giữa bị mất
    } else if (t_trig - f->last_t[i] > US_STALE_US) {            // US3-1: im quá lâu trước mẫu này
        f->n_silent++;
        us_guard_sample(g, c, i, US_K_UNKNOWN, 0.0f);
    }
    f->have_ping[i] = 1;
    f->last_ping[i] = ping;
    const int64_t age = t_now - t_trig;
    if (age < 0 || age > US_ITEM_MAX_AGE_US) {
        f->n_old++;
        kind = US_K_UNKNOWN;                                     // mẫu cũ (hay giờ sai): không biết vật còn đó không
    }
    if (t_trig > f->last_t[i]) f->last_t[i] = t_trig;
    return us_guard_sample(g, c, i, kind, cm);
}

void us_feed_stale(us_feed_t *f, us_guard_t *g, const us_guard_cfg_t *c, int64_t t_now, bool no_queue) {
    for (int i = 0; i < US_G_N; i++) {
        if (no_queue || t_now - f->last_t[i] >= US_STALE_US) {
            f->n_stale++;
            us_guard_sample(g, c, i, US_K_UNKNOWN, 0.0f);
            // vẫn im thì nạp lại sau US_STALE_REPEAT_US (một chu kỳ phát), không đợi thêm US_STALE_US
            f->last_t[i] = t_now - US_STALE_US + US_STALE_REPEAT_US;
        }
    }
}
