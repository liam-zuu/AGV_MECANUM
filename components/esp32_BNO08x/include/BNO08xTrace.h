/**
 * @file BNO08xTrace.h
 * @brief SPI trace of the host <-> BNO085 link, seen from the ESP32 (AGV_MECANUM).
 *
 * Added 28/10/2026 for the thesis check QD 26 / G0 v1 section 7: record what the
 * firmware's own library does on SPI (bytes, chip select time, INT edges, resets)
 * with the real chip on the car and with the H7 emulator on the HIL bench, then
 * compare. Compiled only with -DBNO_TRACE (envs imu_trace_20 / imu_trace_100);
 * the main firmware contains none of this.
 *
 * Records go to RAM and are printed by bno_trace_dump() after each scenario, so
 * printing does not change the timing being measured.
 */
#pragma once

#ifdef BNO_TRACE

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    BT_INT = 1, ///< INT falling edge (ISR)
    BT_RD,      ///< host read: header + body, one chip select
    BT_WR,      ///< host write, one chip select
    BT_RST,     ///< host pulls RST (toggle_reset)
    BT_TMO,     ///< host waited for INT and gave up (library then resets the chip)
    BT_MARK     ///< scenario marker from the test (a = code, b = argument)
};

enum
{
    BT_F_INT_LOW = 1, ///< INT still low just before chip select went low
    BT_F_FAIL = 2,    ///< transfer failed (bad length or SPI driver error)
    BT_F_BYTES = 4,   ///< full bytes kept in the pool (poff)
    BT_F_DELAYED = 8  ///< host delayed this read on purpose (scenario S5)
};

/// One record, 24 bytes. RD/WR: t_us = chip select low, a = chip select low time [us],
/// b = SHTP length (RD) or bytes written (WR), d = first bytes of the transfer.
typedef struct
{
    uint32_t t_us;
    uint16_t a;
    uint16_t b;
    uint8_t type;
    uint8_t flags;
    uint16_t poff;
    uint8_t d[12];
} bno_trace_rec_t;

/// Allocate the buffers (call before imu_init). Returns the record capacity.
size_t bno_trace_begin(void);
void bno_trace_enable(bool on);
/// Called by the library hooks; safe from the INT ISR and from any task.
void bno_trace_rec(uint8_t type, uint16_t a, uint16_t b, uint8_t flags, const uint8_t* bytes, uint16_t n,
        uint32_t t_us);
void bno_trace_mark(uint16_t code, uint16_t arg);
/// Scenario S5: delay the next `reads` host reads by `ms` after INT, before chip select.
void bno_trace_set_host_delay(uint16_t ms, uint16_t reads);
/// Used by the HAL read hook: ms to wait before this read (0 = none).
uint16_t bno_trace_take_host_delay(void);
/// Print every record of the phase (lines "TRACE BEGIN", "R,...", "B,...", "TRACE END"),
/// then clear the buffers. Recording is paused while printing.
void bno_trace_dump(void (*out)(const char* s, size_t n), const char* phase);

#ifdef __cplusplus
}
#endif

#endif // BNO_TRACE
