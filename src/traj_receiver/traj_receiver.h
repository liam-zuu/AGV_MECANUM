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

/* ── Public API ───────────────────────────────────────────────────────── */

/* Init UART + WiFi UDP socket, spawn receiver tasks.
 * Call after wifi_init() if WiFi source is needed. */
void traj_receiver_init(void);

/* Get current setpoint. Returns zeros if timed out (fail-safe).
 * Thread-safe — can be called from any task. */
traj_setpoint_t traj_receiver_get(void);

/* Last active source (for LED/logging). */
traj_source_t traj_receiver_source(void);

/* Send status back to H7 via UART (actual vx, vy, wz).
 * Same UART port as receive — half-duplex OK at 100Hz status rate. */
void traj_receiver_send_status(float vx, float vy, float wz);

#endif /* TRAJ_RECEIVER_H */
