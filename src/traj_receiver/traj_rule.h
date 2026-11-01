#ifndef TRAJ_RULE_H
#define TRAJ_RULE_H

#include <stdint.h>
#include <stdbool.h>

/* One binary for the HIL bench and the car (Outline QĐ 6, level 1): no build flag
 * chooses the setpoint source. A good UDP packet is taken only when no UART packet
 * is recent, so while the H7 relay runs (one packet every 20 ms) UDP never reaches
 * the controller. No ESP-IDF here: test/host/test_traj_rule.c runs this on the PC.
 *   uart_last_us: receive time of the last good UART packet, 0 = none since boot.
 *   Exactly timeout_us old still counts as recent, like traj_receiver_is_fresh(). */
static inline bool traj_udp_allowed(int64_t now_us, int64_t uart_last_us, int64_t timeout_us)
{
    return uart_last_us <= 0 || (now_us - uart_last_us) > timeout_us;
}

#endif /* TRAJ_RULE_H */
