#include "traj_receiver.h"
#include "wifi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include <string.h>

static const char *TAG = "TRAJ";

/* ── Shared state ─────────────────────────────────────────────────────── */
/* Setpoint, source, receive time and counters are written by the UART and the
 * WiFi task and read by traj_task / led_task: all accesses go through g_mux so a
 * reader never gets fields of two packets (DUT group 1, Liam 29/10, Q5a). */
static portMUX_TYPE             g_mux       = portMUX_INITIALIZER_UNLOCKED;
static traj_setpoint_t          g_setpoint  = {0};
static int64_t                  g_last_us   = 0;   /* esp_timer time the last valid packet was received (µs) */
static traj_source_t            g_source    = TRAJ_SRC_NONE;
static uint32_t                 g_n_rx      = 0;   /* packets committed */
static uint32_t                 g_uart_ok   = 0;   /* UART packets with a good XOR */
static uint32_t                 g_uart_bad  = 0;   /* UART packets with a bad XOR  */
static int64_t                  g_uart_last_us = 0;
static uint32_t                 g_udp_ignored  = 0; /* good UDP packets dropped (TRAJ_UART_ONLY) */

/* ── Packet parser — dùng chung cho cả 2 nguồn ───────────────────────── */
/*
 * Parse 1 packet từ buf[0..TRAJ_PACKET_LEN-1].
 * Trả về true nếu hợp lệ, ghi vào out.
 */
static bool _parse_packet(const uint8_t *buf, traj_setpoint_t *out)
{
    if (buf[0] != TRAJ_START_BYTE) return false;

    /* XOR checksum: bytes [0..12] */
    uint8_t chk = 0;
    for (int i = 0; i < TRAJ_PACKET_LEN - 1; i++) chk ^= buf[i];
    if (chk != buf[TRAJ_PACKET_LEN - 1]) return false;

    memcpy(&out->vx, &buf[1], 4);
    memcpy(&out->vy, &buf[5], 4);
    memcpy(&out->wz, &buf[9], 4);
    return true;
}

/* ── Commit parsed setpoint (called from either task) ────────────────── */
/* t_rx_us: time the packet was read, taken before the lock */
static void _commit(const traj_setpoint_t *sp, traj_source_t src, int64_t t_rx_us)
{
    taskENTER_CRITICAL(&g_mux);
    g_setpoint = *sp;
    g_last_us  = t_rx_us;
    g_source   = src;
    g_n_rx++;
    if (src == TRAJ_SRC_UART) {
        g_uart_ok++;
        g_uart_last_us = t_rx_us;
    }
    taskEXIT_CRITICAL(&g_mux);
}

/* ── UART receiver task ───────────────────────────────────────────────── */
static void _uart_task(void *pv)
{
    uint8_t raw[TRAJ_UART_BUF];
    uint8_t pkt[TRAJ_PACKET_LEN];
    int     pkt_idx = 0;
    bool    syncing = true;

    while (1) {
        int n = uart_read_bytes(TRAJ_UART_NUM, raw, sizeof(raw),
                                pdMS_TO_TICKS(10));
        if (n <= 0) continue;

        for (int i = 0; i < n; i++) {
            uint8_t b = raw[i];

            if (syncing) {
                /* Cần tìm start byte */
                if (b == TRAJ_START_BYTE) {
                    pkt[0]  = b;
                    pkt_idx = 1;
                    syncing = false;
                }
                continue;
            }

            pkt[pkt_idx++] = b;

            if (pkt_idx == TRAJ_PACKET_LEN) {
                traj_setpoint_t sp;
                if (_parse_packet(pkt, &sp)) {
                    _commit(&sp, TRAJ_SRC_UART, esp_timer_get_time());
                } else {
                    taskENTER_CRITICAL(&g_mux);
                    g_uart_bad++;
                    taskEXIT_CRITICAL(&g_mux);
                    /* Checksum fail — resync */
                    ESP_LOGW(TAG, "UART checksum fail, resyncing");
                }
                pkt_idx = 0;
                syncing = true;
            }
        }
    }
}

/* ── WiFi UDP receiver task ───────────────────────────────────────────── */
static void _wifi_task(void *pv)
{
    /* Đợi WiFi connect */
    while (!wifi_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "UDP socket create failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(TRAJ_UDP_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "UDP bind failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    /* Receive timeout 200ms — không block mãi */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ESP_LOGI(TAG, "WiFi UDP listening on port %d", TRAJ_UDP_PORT);

    uint8_t buf[TRAJ_PACKET_LEN];
    while (1) {
        int n = recv(sock, buf, sizeof(buf), 0);
        if (n == TRAJ_PACKET_LEN) {
            traj_setpoint_t sp;
            if (_parse_packet(buf, &sp)) {
#ifdef TRAJ_UART_ONLY
                /* HIL bench (env hil_bench): setpoints come only from the H7
                 * relay; a UDP packet is counted and dropped (build difference
                 * HIL vs car, Outline QĐ 6) */
                taskENTER_CRITICAL(&g_mux);
                g_udp_ignored++;
                taskEXIT_CRITICAL(&g_mux);
#else
                _commit(&sp, TRAJ_SRC_WIFI, esp_timer_get_time());
                ESP_LOGI(TAG, "WiFi pkt: vx=%.2f vy=%.2f wz=%.2f", sp.vx, sp.vy, sp.wz);  // thêm
#endif
            } else {
                ESP_LOGW(TAG, "WiFi checksum fail");
            }
        }
        /* n < 0 = timeout hoặc error — loop lại */
    }
}

/* ── Public API ───────────────────────────────────────────────────────── */

void traj_receiver_init(void)
{
    /* UART init */
    uart_config_t cfg = {
        .baud_rate  = TRAJ_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(TRAJ_UART_NUM, TRAJ_UART_BUF * 2, 0, 0, NULL, 0);
    uart_param_config(TRAJ_UART_NUM, &cfg);
    uart_set_pin(TRAJ_UART_NUM, TRAJ_UART_TX, TRAJ_UART_RX, -1, -1);

    /* Spawn tasks */
    xTaskCreatePinnedToCore(_uart_task, "traj_uart", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(_wifi_task, "traj_wifi", 4096, NULL, 3, NULL, 0);

#ifdef TRAJ_UART_ONLY
    ESP_LOGI(TAG, "traj_receiver init done (UART=%d only; UDP %d packets dropped and counted)",
             TRAJ_UART_NUM, TRAJ_UDP_PORT);
#else
    ESP_LOGI(TAG, "traj_receiver init done (UART=%d, UDP=%d)",
             TRAJ_UART_NUM, TRAJ_UDP_PORT);
#endif
}

void traj_receiver_snapshot(traj_snapshot_t *out)
{
    taskENTER_CRITICAL(&g_mux);
    out->sp          = g_setpoint;
    out->src         = g_source;
    out->t_rx_us     = g_last_us;
    out->n_rx        = g_n_rx;
    out->udp_ignored = g_udp_ignored;
    taskEXIT_CRITICAL(&g_mux);
}

bool traj_receiver_is_fresh(const traj_snapshot_t *s, int64_t now_us)
{
    return s->t_rx_us > 0 && (now_us - s->t_rx_us) <= (int64_t)TRAJ_TIMEOUT_MS * 1000LL;
}

traj_setpoint_t traj_receiver_get(void)
{
    /* Fail-safe: nếu không có packet mới trong TRAJ_TIMEOUT_MS → trả về 0 */
    traj_snapshot_t s;
    traj_receiver_snapshot(&s);
    if (!traj_receiver_is_fresh(&s, esp_timer_get_time())) {
        return (traj_setpoint_t){0};
    }
    return s.sp;
}

void traj_receiver_stats(uint32_t *ok, uint32_t *bad, int64_t *last_us)
{
    taskENTER_CRITICAL(&g_mux);
    *ok      = g_uart_ok;
    *bad     = g_uart_bad;
    *last_us = g_uart_last_us;
    taskEXIT_CRITICAL(&g_mux);
}

traj_source_t traj_receiver_source(void)
{
    traj_snapshot_t s;
    traj_receiver_snapshot(&s);
    if (!traj_receiver_is_fresh(&s, esp_timer_get_time())) {
        return TRAJ_SRC_NONE;
    }
    return s.src;
}

void traj_receiver_send_status(float vx, float vy, float wz)
{
    /*
     * Status packet gửi về H7 (ESP32 TX → H7 RX).
     * Format đơn giản: [0xBC][vx(4)][vy(4)][wz(4)][XOR(1)] = 14 bytes
     * H7 có thể bỏ qua nếu chưa implement receive phía H7.
     */
    uint8_t buf[14];
    buf[0] = 0xBC;
    memcpy(&buf[1], &vx, 4);
    memcpy(&buf[5], &vy, 4);
    memcpy(&buf[9], &wz, 4);
    uint8_t chk = 0;
    for (int i = 0; i < 13; i++) chk ^= buf[i];
    buf[13] = chk;
    uart_write_bytes(TRAJ_UART_NUM, buf, 14);
}
