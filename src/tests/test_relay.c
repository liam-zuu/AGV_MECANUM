#include "test_relay.h"
#include "traj_receiver/traj_receiver.h"
#include "motor/motor.h"
#include "imu/imu.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

// ─────────────────────────────────────────────────────────────────────────────
// H7 UART relay check (build with -DTEST_MODE -DTEST_RELAY, env relay_check).
// Runs on the bench ESP32: H7 USART2 TX PD5 -> ESP32 GPIO15 (UART2 RX) + GND.
// Uses the real traj_receiver (same parser, same 0xBB 14-byte packet as the
// main firmware). Every second it prints
//   RELAY t_ms=.. ok=.. bad=.. rate=../s src=UART|NONE age_ms=.. vx=.. vy=.. wz=..
// ok/bad = packets with good/bad XOR since boot (bad also counts lost sync).
// The RPi5 runs with a trajectory (run_hil.py --traj line ...); the H7 VCP line
// "Relay: sent=.. dropped=.." gives the sending side. Motors stay disabled.
// IMU init runs in its own task (every firmware has the IMU, QD 23; the bench
// ESP32 has no BNO085 before G2 and the library can block, bug ESP-BNOLIB-D).
// ─────────────────────────────────────────────────────────────────────────────

static void imu_init_task(void *pv) {
    imu_init();
    vTaskDelete(NULL);
}

void test_relay(void) {
    motor_init();                         /* EN = 0 */
    traj_receiver_init();
    xTaskCreate(imu_init_task, "imu_init", 4096, NULL, 3, NULL);
    printf("RELAY READY v1 uart=%d rx_gpio=%d baud=%d\n",
           TRAJ_UART_NUM, TRAJ_UART_RX, TRAJ_UART_BAUD);

    uint32_t ok_prev = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t ok, bad;
        int64_t last_us;
        traj_receiver_stats(&ok, &bad, &last_us);
        traj_setpoint_t sp = traj_receiver_get();      /* zero after 200 ms without a packet */
        traj_source_t src = traj_receiver_source();
        int64_t now = esp_timer_get_time();
        printf("RELAY t_ms=%lld ok=%lu bad=%lu rate=%lu/s src=%s age_ms=%lld vx=%.3f vy=%.3f wz=%.3f\n",
               (long long)(now / 1000), (unsigned long)ok, (unsigned long)bad,
               (unsigned long)(ok - ok_prev),
               src == TRAJ_SRC_UART ? "UART" : (src == TRAJ_SRC_NONE ? "NONE" : "WIFI"),
               last_us ? (long long)((now - last_us) / 1000) : -1LL,
               sp.vx, sp.vy, sp.wz);
        ok_prev = ok;
    }
}
