/**
 * @file BNO08xTrace.cpp
 * @brief RAM trace of the BNO085 SPI link (see BNO08xTrace.h). Only built with -DBNO_TRACE.
 */

#include "BNO08xTrace.h"

#ifdef BNO_TRACE

#include <stdio.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

// Room left for what imu_init() allocates after us (library tasks, queues, SPI bus).
#define BT_HEAP_RESERVE (64 * 1024)
#define BT_POOL_BYTES (16 * 1024)   ///< full bytes of the first packets (init sequence)
#define BT_MAX_RECORDS (10000)

static bno_trace_rec_t* s_rec = nullptr;
static uint8_t* s_pool = nullptr;
static size_t s_cap = 0;
static volatile size_t s_n = 0;
static volatile size_t s_pool_used = 0;
static volatile uint32_t s_lost = 0;
static volatile bool s_on = false;
static volatile uint16_t s_delay_ms = 0;
static volatile uint16_t s_delay_reads = 0;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

extern "C" size_t bno_trace_begin(void)
{
    if (s_rec)
        return s_cap;
    s_pool = (uint8_t*) heap_caps_malloc(BT_POOL_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t big = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t n = big > BT_HEAP_RESERVE ? (big - BT_HEAP_RESERVE) / sizeof(bno_trace_rec_t) : 0;
    if (n > BT_MAX_RECORDS)
        n = BT_MAX_RECORDS;
    if (n)
        s_rec = (bno_trace_rec_t*) heap_caps_malloc(n * sizeof(bno_trace_rec_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    s_cap = (s_rec && s_pool) ? n : 0;
    return s_cap;
}

extern "C" void bno_trace_enable(bool on)
{
    s_on = on;
}

extern "C" void IRAM_ATTR bno_trace_rec(
        uint8_t type, uint16_t a, uint16_t b, uint8_t flags, const uint8_t* bytes, uint16_t n, uint32_t t_us)
{
    if (!s_on || !s_cap)
        return;
    portENTER_CRITICAL_SAFE(&s_mux);
    if (s_n >= s_cap)
    {
        s_lost = s_lost + 1;
        portEXIT_CRITICAL_SAFE(&s_mux);
        return;
    }
    bno_trace_rec_t* r = &s_rec[s_n];
    s_n = s_n + 1;
    r->t_us = t_us;
    r->a = a;
    r->b = b;
    r->type = type;
    r->poff = 0xFFFF;
    memset(r->d, 0, sizeof(r->d));
    if (bytes && n)
    {
        memcpy(r->d, bytes, n < sizeof(r->d) ? n : sizeof(r->d));
        // keep the whole transfer while the pool has room: [u16 length][bytes]
        if (s_pool_used + 2 + n <= BT_POOL_BYTES && s_pool_used < 0xFFFF)
        {
            r->poff = (uint16_t) s_pool_used;
            s_pool[s_pool_used] = (uint8_t) (n & 0xFF);
            s_pool[s_pool_used + 1] = (uint8_t) (n >> 8);
            memcpy(&s_pool[s_pool_used + 2], bytes, n);
            s_pool_used = s_pool_used + 2 + n;
            flags |= BT_F_BYTES;
        }
    }
    r->flags = flags;
    portEXIT_CRITICAL_SAFE(&s_mux);
}

extern "C" void bno_trace_mark(uint16_t code, uint16_t arg)
{
    bno_trace_rec(BT_MARK, code, arg, 0, nullptr, 0, (uint32_t) esp_timer_get_time());
}

extern "C" void bno_trace_set_host_delay(uint16_t ms, uint16_t reads)
{
    portENTER_CRITICAL_SAFE(&s_mux);
    s_delay_ms = ms;
    s_delay_reads = reads;
    portEXIT_CRITICAL_SAFE(&s_mux);
}

extern "C" uint16_t bno_trace_take_host_delay(void)
{
    uint16_t ms = 0;
    portENTER_CRITICAL_SAFE(&s_mux);
    if (s_delay_reads)
    {
        s_delay_reads = s_delay_reads - 1;
        ms = s_delay_ms;
    }
    portEXIT_CRITICAL_SAFE(&s_mux);
    return ms;
}

static void hex(char* dst, const uint8_t* p, size_t n)
{
    static const char H[] = "0123456789ABCDEF";
    for (size_t i = 0; i < n; i++)
    {
        dst[2 * i] = H[p[i] >> 4];
        dst[2 * i + 1] = H[p[i] & 0xF];
    }
    dst[2 * n] = 0;
}

extern "C" void bno_trace_dump(void (*out)(const char* s, size_t n), const char* phase)
{
    bool was_on = s_on;
    s_on = false;
    static char line[2 * 1024 + 32]; // static: app_main's stack is only a few KB
    size_t n = s_n;
    int k = snprintf(line, sizeof(line), "TRACE BEGIN phase=%s cap=%u n=%u lost=%lu pool_used=%u rec_bytes=%u\n",
            phase, (unsigned) s_cap, (unsigned) n, (unsigned long) s_lost, (unsigned) s_pool_used,
            (unsigned) sizeof(bno_trace_rec_t));
    out(line, k);
    for (size_t i = 0; i < n; i++)
    {
        const bno_trace_rec_t* r = &s_rec[i];
        size_t nd = (r->type == BT_RD || r->type == BT_WR) ? sizeof(r->d) : 0;
        char dh[2 * sizeof(r->d) + 1];
        hex(dh, r->d, nd);
        k = snprintf(line, sizeof(line), "R,%u,%u,%lu,%u,%u,%u,%s\n", (unsigned) i, r->type, (unsigned long) r->t_us,
                r->a, r->b, r->flags, dh);
        out(line, k);
        if (r->flags & BT_F_BYTES)
        {
            uint16_t len = s_pool[r->poff] | (s_pool[r->poff + 1] << 8);
            size_t shown = len < 1000 ? len : 1000;
            int h = snprintf(line, sizeof(line), "B,%u,%u,", (unsigned) i, len);
            hex(line + h, &s_pool[r->poff + 2], shown);
            size_t m = strlen(line);
            line[m] = '\n';
            out(line, m + 1);
        }
        if ((i & 63) == 63)
            vTaskDelay(1); // let the UART drain and the watchdog run
    }
    k = snprintf(line, sizeof(line), "TRACE END phase=%s\n", phase);
    out(line, k);
    portENTER_CRITICAL_SAFE(&s_mux);
    s_n = 0;
    s_pool_used = 0;
    s_lost = 0;
    portEXIT_CRITICAL_SAFE(&s_mux);
    s_on = was_on;
}

#endif // BNO_TRACE
