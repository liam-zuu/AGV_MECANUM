/* PC test of the UDP rule (no ESP-IDF, no board):
 *   gcc -Wall -Wextra -Werror -I src/traj_receiver test/host/test_traj_rule.c -o <exe>
 * Exit code 0 = all passed. */
#include <stdio.h>
#include "traj_rule.h"

#define TIMEOUT_US 200000LL

static int s_fail = 0;

static void expect(const char *name, bool got, bool want)
{
    if (got != want) {
        printf("FAIL %s: got %d, want %d\n", name, got, want);
        s_fail++;
    }
}

int main(void)
{
    /* No UART packet since boot: UDP is the only source (car on the floor) */
    expect("no uart yet", traj_udp_allowed(5000000, 0, TIMEOUT_US), true);
    expect("no uart yet, now 0", traj_udp_allowed(0, 0, TIMEOUT_US), true);

    /* Relay running: one packet every 20 ms */
    expect("uart 20 ms ago", traj_udp_allowed(1020000, 1000000, TIMEOUT_US), false);
    expect("uart same time", traj_udp_allowed(1000000, 1000000, TIMEOUT_US), false);

    /* The boundary: exactly 200 ms is still recent, 1 us more is not */
    expect("uart 199999 us", traj_udp_allowed(1199999, 1000000, TIMEOUT_US), false);
    expect("uart 200000 us", traj_udp_allowed(1200000, 1000000, TIMEOUT_US), false);
    expect("uart 200001 us", traj_udp_allowed(1200001, 1000000, TIMEOUT_US), true);

    /* The UART task committed between the UDP time stamp and the lock */
    expect("uart after now", traj_udp_allowed(999990, 1000000, TIMEOUT_US), false);

    /* Beyond 32 bits of microseconds (71.6 min after boot) */
    expect("late uart recent", traj_udp_allowed(5000000000LL, 4999980000LL, TIMEOUT_US), false);
    expect("late uart old", traj_udp_allowed(5000000000LL, 4000000000LL, TIMEOUT_US), true);
    /* Age of 2^32 + 1000 us: cut to 32 bits it would look like 1 ms */
    expect("age above 32 bits", traj_udp_allowed(5294968296LL, 1000000000LL, TIMEOUT_US), true);

    if (s_fail) {
        printf("%d failed\n", s_fail);
        return 1;
    }
    printf("all passed\n");
    return 0;
}
