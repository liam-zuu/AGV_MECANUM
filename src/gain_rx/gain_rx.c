#include "gain_rx.h"
#include "../fw_id_gen.h"      // FW_GIT_SHA, FW_ENV
#include "wifi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include <stdio.h>
#include <string.h>

// Thiết kế: 03_CODE/docs/2026-11-12_Thiet_ke_doi_gain_qua_WiFi.md (L90, Liam 12/11).

static const char *TAG = "GAIN";

typedef struct { uint32_t id; int err; } gain_result_t;

static QueueHandle_t s_req;   // 1 chỗ: gain_rx → control_task
static QueueHandle_t s_res;   // 1 chỗ: control_task → gain_rx

bool gain_rx_poll(gain_set_t *req) {
    return s_req != NULL && xQueueReceive(s_req, req, 0) == pdTRUE;
}

void gain_rx_done(uint32_t id, int err) {
    if (s_res == NULL) return;
    gain_result_t r = { .id = id, .err = err };
    (void)xQueueOverwrite(s_res, &r);
}

// "#gain_ack,req_id=..,ok=..,err=..,sha=..,env=..,id=..,applied_n=..,ctrl=..,<7 cặp>" về đúng nơi gửi.
// Gain trong gói trả lời là gain đang chạy (đọc lại sau khi áp), không phải gain trong yêu cầu.
static void reply(int sock, const struct sockaddr_in *to, uint32_t req_id, int err) {
    char buf[480];
    gain_set_t cur;
    gain_set_current(&cur);
    int k = snprintf(buf, sizeof(buf), "#gain_ack,req_id=%lu,ok=%d,err=%s,sha=%s,env=%s",
                     (unsigned long)req_id, err == GAIN_OK, gain_err_str(err), FW_GIT_SHA, FW_ENV);
    if (k < 0 || k >= (int)sizeof(buf) - 2) return;
    k += gain_set_format(buf + k, sizeof(buf) - (size_t)k - 1, &cur, gain_set_applied_n());
    buf[k++] = '\n';
    (void)sendto(sock, buf, k, 0, (const struct sockaddr *)to, sizeof(*to));
}

// Chờ kết quả đúng id; quá hạn thì GAIN_ERR_TIMEOUT
static int wait_result(uint32_t id) {
    TickType_t t_end = xTaskGetTickCount() + pdMS_TO_TICKS(GAIN_REPLY_WAIT_MS);
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        int32_t left = (int32_t)(t_end - now);
        if (left <= 0) return GAIN_ERR_TIMEOUT;
        gain_result_t r;
        if (xQueueReceive(s_res, &r, (TickType_t)left) == pdTRUE && r.id == id) return r.err;
    }
}

static void gain_rx_task(void *pv) {
    while (!wifi_is_connected()) vTaskDelay(pdMS_TO_TICKS(500));

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
    ESP_LOGI(TAG, "gain UDP listening on port %d", GAIN_UDP_PORT);

    char buf[330];
    for (;;) {
        struct sockaddr_in src;
        socklen_t sl = sizeof(src);
        int n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&src, &sl);
        if (n <= 0) continue;

        gain_set_t g;
        int err = gain_set_parse(buf, (size_t)n, &g);
        uint32_t req_id = err == GAIN_OK ? g.id : 0;
        if (err == GAIN_OK) {
            xQueueReset(s_res);                                // bỏ kết quả cũ (yêu cầu đã quá hạn)
            if (xQueueSend(s_req, &g, 0) != pdTRUE) {
                err = GAIN_ERR_BUSY;
            } else {
                err = wait_result(g.id);
                // Quá hạn: rút yêu cầu nếu control_task chưa lấy. Nếu nó đã lấy thì bộ gain có thể đã áp;
                // PC gửi lại cùng id, gói trả lời luôn mang gain đang chạy nên PC thấy được.
                if (err == GAIN_ERR_TIMEOUT) xQueueReset(s_req);
            }
        }
        // Log ở task này (lõi 0), không ở control_task: in serial chậm, không chen vòng 100 Hz
        if (err == GAIN_OK) ESP_LOGI(TAG, "gain_set id=%lu applied", (unsigned long)req_id);
        else                ESP_LOGW(TAG, "gain_set id=%lu rejected: %s", (unsigned long)req_id, gain_err_str(err));
        reply(sock, &src, req_id, err);
    }
}

void gain_rx_init(void) {
    s_req = xQueueCreate(1, sizeof(gain_set_t));
    s_res = xQueueCreate(1, sizeof(gain_result_t));
    if (s_req == NULL || s_res == NULL) {
        ESP_LOGE(TAG, "queue create failed: gain over WiFi disabled");
        return;
    }
    xTaskCreatePinnedToCore(gain_rx_task, "gain_rx", 4096, NULL, 1, NULL, 0);
}
