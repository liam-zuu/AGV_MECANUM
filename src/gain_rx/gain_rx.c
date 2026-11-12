#include "gain_rx.h"
#include "../fw_id_gen.h"      // FW_GIT_SHA, FW_ENV
#include "wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include <stdio.h>
#include <string.h>

// Thiết kế: 03_CODE/docs/2026-11-12_Thiet_ke_doi_gain_qua_WiFi.md (L90, Liam 12/11).

static const char *TAG = "GAIN";

typedef struct {
    uint32_t   txn;
    int        err;
    gain_set_t cur;         // bộ gain đang chạy sau khi xử lý, chụp trong control_task
    uint32_t   applied_n;
} gain_result_t;

static QueueHandle_t s_req;   // 1 chỗ: gain_rx → control_task
static QueueHandle_t s_res;   // 1 chỗ: control_task → gain_rx
static uint32_t      s_boot;  // mã khởi động ngẫu nhiên (đổi mỗi lần boot): PC thấy board đã reset
static char          s_mac[18] = "??:??:??:??:??:??";

bool gain_rx_poll(gain_req_t *req) {
    return s_req != NULL && xQueueReceive(s_req, req, 0) == pdTRUE;
}

void gain_rx_done(const gain_req_t *req, int err) {
    if (s_res == NULL) return;
    gain_result_t r = { .txn = req->txn, .err = err };
    gain_set_current(&r.cur);
    r.applied_n = gain_set_applied_n();
    (void)xQueueOverwrite(s_res, &r);
}

// "#gain_ack,tx=..,req_id=..,ok=..,err=..,sha=..,env=..,mac=..,boot=..[,id=..,applied_n=..,ctrl=..,<7 cặp>]"
// về đúng nơi gửi. Phần gain chỉ có khi control_task đã xử lý yêu cầu (res != NULL): đó là bộ đang
// chạy sau yêu cầu, không phải bộ trong yêu cầu. Lỗi khuôn, busy, timeout: không kèm gain.
static void reply(int sock, const struct sockaddr_in *to, uint32_t tx, uint32_t req_id, int err,
                  const gain_result_t *res) {
    char buf[480];
    int k = snprintf(buf, sizeof(buf), "#gain_ack,tx=%lu,req_id=%lu,ok=%d,err=%s,sha=%s,env=%s,mac=%s,boot=%08lx",
                     (unsigned long)tx, (unsigned long)req_id, err == GAIN_OK, gain_err_str(err),
                     FW_GIT_SHA, FW_ENV, s_mac, (unsigned long)s_boot);
    if (k < 0 || k >= (int)sizeof(buf) - 2) return;
    if (res != NULL) {
        int m = gain_set_format(buf + k, sizeof(buf) - (size_t)k - 1, &res->cur, res->applied_n);
        if (m < 0) return;                                    // không gửi trả lời cụt
        k += m;
    }
    buf[k++] = '\n';
    (void)sendto(sock, buf, k, 0, (const struct sockaddr *)to, sizeof(*to));
}

// Chờ kết quả đúng mã giao dịch; quá hạn thì false
static bool wait_result(uint32_t txn, gain_result_t *out) {
    TickType_t t_end = xTaskGetTickCount() + pdMS_TO_TICKS(GAIN_REPLY_WAIT_MS);
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        int32_t left = (int32_t)(t_end - now);
        if (left <= 0) return false;
        if (xQueueReceive(s_res, out, (TickType_t)left) == pdTRUE && out->txn == txn) return true;
    }
}

static void gain_rx_task(void *pv) {
    while (!wifi_is_connected()) vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t mac[6];
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK)
        snprintf(s_mac, sizeof(s_mac), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "UDP socket create failed");
        vTaskDelete(NULL);
        return;
    }
    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(GAIN_UDP_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "UDP bind %d failed", GAIN_UDP_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "gain UDP listening on port %d, mac=%s boot=%08lx", GAIN_UDP_PORT, s_mac, (unsigned long)s_boot);

    // Bộ đệm dài hơn gói hợp lệ dài nhất: datagram dài hơn GAIN_MSG_MAX + 2 bị từ chối, không bị cắt
    // âm thầm thành gói hợp lệ (Codex L90-7)
    static char buf[GAIN_MSG_MAX + 64];
    uint32_t txn = 0;
    for (;;) {
        struct sockaddr_in src;
        socklen_t sl = sizeof(src);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&src, &sl);
        if (n <= 0) continue;
        int64_t t_rx = esp_timer_get_time();

        gain_req_t req = { .txn = ++txn, .t_rx_us = t_rx };
        uint32_t tx = 0;
        int err = (n > GAIN_MSG_MAX + 2) ? GAIN_ERR_FORMAT
                                         : gain_msg_parse(buf, (size_t)n, &req.kind, &req.set, &tx);
        uint32_t req_id = (err == GAIN_OK && req.kind == GAIN_MSG_SET) ? req.set.id : 0;
        gain_result_t res;
        bool have_res = false;
        if (err == GAIN_OK) {
            xQueueReset(s_res);                                // bỏ kết quả cũ (yêu cầu đã quá hạn)
            if (xQueueSend(s_req, &req, 0) != pdTRUE) {
                err = GAIN_ERR_BUSY;
            } else if (wait_result(req.txn, &res)) {
                have_res = true;
                err = res.err;
            } else {
                // Quá hạn: rút yêu cầu nếu control_task chưa lấy. Nếu nó đã lấy thì kết quả đến sau mang
                // mã giao dịch cũ và bị bỏ ở lần chờ kế (L90-3); PC thấy timeout, hỏi lại bằng #gain_get.
                xQueueReset(s_req);
                err = GAIN_ERR_TIMEOUT;
            }
        }
        // Log ở task này (lõi 0), không ở control_task: in serial chậm, không chen vòng 100 Hz
        if (req.kind == GAIN_MSG_GET && err == GAIN_OK) ESP_LOGI(TAG, "gain_get tx=%lu", (unsigned long)tx);
        else if (err == GAIN_OK) ESP_LOGI(TAG, "gain_set id=%lu tx=%lu applied", (unsigned long)req_id, (unsigned long)tx);
        else ESP_LOGW(TAG, "gain msg id=%lu tx=%lu rejected: %s", (unsigned long)req_id, (unsigned long)tx, gain_err_str(err));
        reply(sock, &src, tx, req_id, err, have_res ? &res : NULL);
    }
}

void gain_rx_init(void) {
    s_boot = esp_random();
    s_req = xQueueCreate(1, sizeof(gain_req_t));
    s_res = xQueueCreate(1, sizeof(gain_result_t));
    if (s_req == NULL || s_res == NULL) {
        ESP_LOGE(TAG, "queue create failed: gain over WiFi disabled");
        s_req = NULL;
        return;
    }
    xTaskCreatePinnedToCore(gain_rx_task, "gain_rx", 4096, NULL, 1, NULL, 0);
}
