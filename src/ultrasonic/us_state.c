#include "us_state.h"
#include <stdio.h>
#include <string.h>

// L95: xem us_state.h

bool us_swb_enabled(bool has_rc, int ch_swb) {
    if (!has_rc) return true;
    return ch_swb < US_SWB_ON_BELOW;
}

void us_run_begin(us_run_t *r, bool on, int32_t swb, uint32_t boot) {
    memset(r, 0, sizeof(*r));
    r->on   = on ? 1u : 0u;
    r->swb  = swb;
    r->boot = boot;
}

bool us_run_tick(us_run_t *r, bool on, int32_t swb, bool any_blocked, bool cut) {
    uint8_t o = on ? 1u : 0u;
    bool changed = (o != r->on);
    if (changed) { r->on = o; r->changes++; }
    r->swb = swb;
    r->ticks++;
    if (any_blocked) r->blk_ticks++;
    if (cut)         r->cut_ticks++;
    return changed;
}

int us_format_line(char *buf, size_t n, int ev, const us_run_t *r, uint32_t ev_seq, uint32_t line_drop) {
    if (buf == NULL || n == 0 || r == NULL) return -1;
    switch (ev) {
    case US_EV_START:
    case US_EV_CHANGE:
        return snprintf(buf, n, "#us,ev=%s,boot=%08lx,on=%u,swb=%ld,first_seq=%lu,line_drop=%lu\n",
                        ev == US_EV_START ? "start" : "change", (unsigned long)r->boot, (unsigned)r->on,
                        (long)r->swb, (unsigned long)ev_seq, (unsigned long)line_drop);
    case US_EV_END:
        return snprintf(buf, n, "#us_end,boot=%08lx,on=%u,swb=%ld,changes=%lu,blk_ticks=%lu,cut_ticks=%lu,"
                                "ticks=%lu,last_data_seq=%lu,line_drop=%lu\n",
                        (unsigned long)r->boot, (unsigned)r->on, (long)r->swb, (unsigned long)r->changes,
                        (unsigned long)r->blk_ticks, (unsigned long)r->cut_ticks, (unsigned long)r->ticks,
                        (unsigned long)ev_seq, (unsigned long)line_drop);
    default:
        return -1;
    }
}
