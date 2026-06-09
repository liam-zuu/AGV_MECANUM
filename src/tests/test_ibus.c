#include "test_ibus.h"
#include "ibus/ibus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TEST_IBUS";

// ─── In bảng tất cả 10 channel ────────────────────────────────────────────────
static void print_channels(const ibus_data_t *d) {
    ESP_LOGI(TAG,
        "RAW  CH1:%4d CH2:%4d CH3:%4d CH4:%4d CH5:%4d"
        "  CH6:%4d CH7:%4d CH8:%4d CH9:%4d CH10:%4d",
        d->channel[0], d->channel[1], d->channel[2], d->channel[3], d->channel[4],
        d->channel[5], d->channel[6], d->channel[7], d->channel[8], d->channel[9]);

    ESP_LOGI(TAG,
        "NORM CH1:%5d CH2:%5d CH3:%5d CH4:%5d",
        ibus_channel_normalized(d, 0),
        ibus_channel_normalized(d, 1),
        ibus_channel_normalized(d, 2),
        ibus_channel_normalized(d, 3));
}

// ─── Test 1: Đợi signal lần đầu (verify bind + hardware) ─────────────────────
// Pass: nhận được frame valid trong vòng 3 giây
// Fail: 3 giây không có gì → receiver chưa bind hoặc GPIO sai
static void test_1_signal_present(void) {
    ESP_LOGI(TAG, "=== TEST 1: Signal present ===");
    ESP_LOGI(TAG, "Bat remote + receiver, cho 3 giay...");

    ibus_data_t d;
    for (int i = 0; i < 30; i++) {           // 30 × 100ms = 3s
        if (ibus_get(&d) && d.valid) {
            ESP_LOGI(TAG, "[PASS] Test 1 — nhan duoc frame valid sau ~%dms", i * 100);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGE(TAG, "[FAIL] Test 1 — khong co data sau 3s. Kiem tra: bind chua? GPIO18? UART1?");
}

// ─── Test 2: Giá trị mid/min/max khi stick ở giữa ────────────────────────────
// Pass: CH1–CH4 raw trong khoảng [1480, 1520] khi tay không chạm
// Đây cũng verify checksum pass liên tục (nếu checksum fail thì valid=false)
static void test_2_mid_values(void) {
    ESP_LOGI(TAG, "=== TEST 2: Mid values — tha stick ra giua, cho 2 giay ===");
    vTaskDelay(pdMS_TO_TICKS(2000));

    ibus_data_t d;
    if (!ibus_get(&d)) {
        ESP_LOGE(TAG, "[FAIL] Test 2 — mat signal");
        return;
    }

    bool pass = true;
    for (int ch = 0; ch < 4; ch++) {
        int raw = d.channel[ch];
        if (raw < 1480 || raw > 1520) {
            ESP_LOGW(TAG, "  CH%d raw=%d (expect 1480-1520)", ch + 1, raw);
            pass = false;
        }
    }
    print_channels(&d);

    if (pass)
        ESP_LOGI(TAG, "[PASS] Test 2 — CH1-4 mid OK");
    else
        ESP_LOGW(TAG, "[WARN] Test 2 — mot so channel lech mid > 20 (check calibration tay cam)");
}

// ─── Test 3: Deadband ─────────────────────────────────────────────────────────
// Pass: normalized CH1–CH4 = 0 khi stick ở giữa (deadband đang hoạt động)
static void test_3_deadband(void) {
    ESP_LOGI(TAG, "=== TEST 3: Deadband — stick o giua, NORM phai = 0 ===");

    ibus_data_t d;
    int fail_count = 0;
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!ibus_get(&d)) continue;
        for (int ch = 0; ch < 4; ch++) {
            int norm = ibus_channel_normalized(&d, ch);
            if (norm != 0) {
                ESP_LOGW(TAG, "  CH%d norm=%d (expect 0, raw=%d)", ch + 1, norm, d.channel[ch]);
                fail_count++;
            }
        }
    }

    if (fail_count == 0)
        ESP_LOGI(TAG, "[PASS] Test 3 — deadband OK, tat ca NORM=0 khi stick giua");
    else
        ESP_LOGW(TAG, "[WARN] Test 3 — %d truong hop norm!=0. Tang IBUS_DEADBAND neu joystick bi ro", fail_count);
}

// ─── Test 4: Full range khi đẩy stick ────────────────────────────────────────
// Liam đẩy từng stick đến min và max, verify raw ~1000 và ~2000
// Không auto-pass/fail — chỉ print để Liam quan sát bằng mắt
static void test_4_full_range(void) {
    ESP_LOGI(TAG, "=== TEST 4: Full range — day stick toi da 4 huong, quan sat 10 giay ===");
    ESP_LOGI(TAG, "Xem RAW: phai thay 1000 va 2000 o CH1-CH4");
    ESP_LOGI(TAG, "Xem NORM: phai thay -1000 va +1000");

    ibus_data_t d;
    for (int i = 0; i < 50; i++) {           // 50 × 200ms = 10s
        vTaskDelay(pdMS_TO_TICKS(200));
        if (ibus_get(&d)) {
            print_channels(&d);
        } else {
            ESP_LOGW(TAG, "frame invalid (failsafe_count=%d)", g_ibus_failsafe_count);
        }
    }
    ESP_LOGI(TAG, "Test 4 done");
}

// ─── Test 5: Failsafe khi tắt remote ─────────────────────────────────────────
// Pass: valid=false và failsafe_count tăng sau khi tắt remote
static void test_5_failsafe(void) {
    ESP_LOGI(TAG, "=== TEST 5: Failsafe — TAT REMOTE NGAY BAY GIO ===");
    vTaskDelay(pdMS_TO_TICKS(1000));         // cho 1s để Liam tắt

    int triggered = 0;
    for (int i = 0; i < 30; i++) {           // theo dõi 3s
        vTaskDelay(pdMS_TO_TICKS(100));
        ibus_data_t d;
        bool got = ibus_get(&d);
        ESP_LOGI(TAG, "valid=%d failsafe_count=%d", got && d.valid, g_ibus_failsafe_count);
        if (g_ibus_failsafe_count >= IBUS_FAILSAFE_FRAMES) {
            triggered = 1;
        }
    }

    if (triggered)
        ESP_LOGI(TAG, "[PASS] Test 5 — failsafe triggered OK");
    else
        ESP_LOGW(TAG, "[WARN] Test 5 — failsafe chua trigger. Remote da tat chua?");

    ESP_LOGI(TAG, "Bat remote lai truoc khi chay test tiep theo...");
    vTaskDelay(pdMS_TO_TICKS(3000));
}

// ─── Entry point ──────────────────────────────────────────────────────────────
void test_ibus(void) {
    ibus_init();

    // ibus_task chạy riêng — đọc byte-by-byte background
    xTaskCreatePinnedToCore(ibus_task, "ibus_task", 2048, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "IBus test start. Dam bao remote + receiver da bat.");
    vTaskDelay(pdMS_TO_TICKS(500));          // cho task khởi động

    test_1_signal_present();
    test_2_mid_values();
    test_3_deadband();
    test_4_full_range();
    test_5_failsafe();

    ESP_LOGI(TAG, "=== ALL TESTS DONE ===");
    vTaskDelete(NULL);
}