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
static volatile traj_setpoint_t g_setpoint  = {0};
static volatile int64_t         g_last_us   = 0;   /* timestamp of last valid packet (µs) */
static volatile traj_source_t   g_source    = TRAJ_SRC_NONE;

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
static void _commit(const traj_setpoint_t *sp, traj_source_t src)
{
    g_setpoint = *sp;
    g_last_us  = esp_timer_get_time();
    g_source   = src;
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
                    _commit(&sp, TRAJ_SRC_UART);
                } else {
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
                _commit(&sp, TRAJ_SRC_WIFI);
                ESP_LOGI(TAG, "WiFi pkt: vx=%.2f vy=%.2f wz=%.2f", sp.vx, sp.vy, sp.wz);  // thêm
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

    ESP_LOGI(TAG, "traj_receiver init done (UART=%d, UDP=%d)",
             TRAJ_UART_NUM, TRAJ_UDP_PORT);
}

traj_setpoint_t traj_receiver_get(void)
{
    /* Fail-safe: nếu không có packet mới trong TRAJ_TIMEOUT_MS → trả về 0 */
    int64_t now = esp_timer_get_time();
    if (g_last_us == 0 ||
        (now - g_last_us) > (int64_t)TRAJ_TIMEOUT_MS * 1000LL) {
        return (traj_setpoint_t){0};
    }
    return g_setpoint;
}

traj_source_t traj_receiver_source(void)
{
    int64_t now = esp_timer_get_time();
    if (g_last_us == 0 ||
        (now - g_last_us) > (int64_t)TRAJ_TIMEOUT_MS * 1000LL) {
        return TRAJ_SRC_NONE;
    }
    return g_source;
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
