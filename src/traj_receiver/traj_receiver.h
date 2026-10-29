#ifndef TRAJ_RECEIVER_H
#define TRAJ_RECEIVER_H

#include "driver/uart.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * traj_receiver — Trajectory setpoint receiver
 *
 * Hai nguồn input, cùng 1 parser, cùng 1 output struct:
 *   (1) HIL mode:        UART từ H7  (H7 relay từ SPI frame RPi5)
 *   (2) AGV mobile mode: WiFi UDP từ PC / mission computer
 *
 * Packet format 14 bytes (cả 2 nguồn dùng chung):
 *   [0]     0xBB        start byte
 *   [1-4]   vx          float LE
 *   [5-8]   vy          float LE
 *   [9-12]  wz          float LE
 *   [13]    checksum    XOR of bytes [0..12]
 *
 * Fail-safe: nếu không có packet hợp lệ trong TRAJ_TIMEOUT_MS → setpoint = 0.
 */

/* ── UART config (H7 → ESP32) ─────────────────────────────────────────── */
#define TRAJ_UART_NUM       UART_NUM_2
#define TRAJ_UART_TX        16          /* ESP32 TX → H7 RX (status) */
#define TRAJ_UART_RX        15          /* ESP32 RX ← H7 TX (setpoint) */
#define TRAJ_UART_BAUD      115200
#define TRAJ_UART_BUF       256

/* ── WiFi UDP config ──────────────────────────────────────────────────── */
#define TRAJ_UDP_PORT       5010

/* ── Protocol ─────────────────────────────────────────────────────────── */
#define TRAJ_START_BYTE     0xBB
#define TRAJ_PACKET_LEN     14          /* 1 + 4 + 4 + 4 + 1 */
#define TRAJ_TIMEOUT_MS     200

/* ── Setpoint struct ──────────────────────────────────────────────────── */
typedef struct {
    float vx;
    float vy;
    float wz;
} traj_setpoint_t;

/* ── Source tracking (for debug/logging) ─────────────────────────────── */
typedef enum {
    TRAJ_SRC_NONE = 0,
    TRAJ_SRC_UART,
    TRAJ_SRC_WIFI,
} traj_source_t;

/* ── Snapshot of the last packet (DUT group 1, Liam 29/10) ────────────── */
/* One consistent copy, taken under the receiver lock. t_rx_us is the time the
 * packet was received, so a consumer that stamps freshness with it keeps ONE
 * timeout (TRAJ_TIMEOUT_MS) from the last packet. Before, traj_task stamped the
 * time it saw a still-fresh setpoint, so control_task could keep an old
 * setpoint up to about 2 × 200 ms after the last packet. */
typedef struct {
    traj_setpoint_t sp;
    traj_source_t   src;          /* source of that packet (NONE before the first) */
    int64_t         t_rx_us;      /* esp_timer time it was received, 0 = none yet */
    uint32_t        n_rx;         /* packets committed since boot */
    uint32_t        udp_ignored;  /* good UDP packets dropped (env hil_bench, TRAJ_UART_ONLY) */
} traj_snapshot_t;

/* ── Public API ───────────────────────────────────────────────────────── */

/* Init UART + WiFi UDP socket, spawn receiver tasks.
 * Call after wifi_init() if WiFi source is needed.
 * Build flag TRAJ_UART_ONLY (env hil_bench): only the UART source is used; good
 * UDP packets are dropped and counted in udp_ignored. */
void traj_receiver_init(void);

/* Last packet, not timed out by this call (check with traj_receiver_is_fresh). */
void traj_receiver_snapshot(traj_snapshot_t *out);

/* true if the snapshot's packet is at most TRAJ_TIMEOUT_MS old at now_us */
bool traj_receiver_is_fresh(const traj_snapshot_t *s, int64_t now_us);

/* Get current setpoint. Returns zeros if timed out (fail-safe).
 * Thread-safe — can be called from any task. */
traj_setpoint_t traj_receiver_get(void);

/* Last active source (for LED/logging). */
traj_source_t traj_receiver_source(void);

/* Send status back to H7 via UART (actual vx, vy, wz).
 * Same UART port as receive — half-duplex OK at 100Hz status rate. */
void traj_receiver_send_status(float vx, float vy, float wz);

/* UART packet counters since boot (relay check, env relay_check) and receive time of the last good one */
void traj_receiver_stats(uint32_t *ok, uint32_t *bad, int64_t *last_us);

#endif /* TRAJ_RECEIVER_H */
