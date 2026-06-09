#include "test_manual.h"
#include "manual_control/manual_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TEST_MANUAL";

// ─────────────────────────────────────────────────────────────────────────────
// test_manual: chạy xe bằng tay cầm FlySky i6X
//
// Checklist trước khi chạy:
//   [1] Bật remote TRƯỚC, sau đó cấp nguồn AGV (đúng thứ tự FlySky)
//   [2] LED receiver nhấp nháy → solid = bind OK
//   [3] SWC ở vị trí UP (motor OFF) khi khởi động
//   [4] VrA vặn về MIN (speed limit thấp nhất) khi test lần đầu
//   [5] Đặt AGV lên kê (bánh không chạm đất) cho lần chạy đầu tiên
//
// Verify sequence:
//   Bước 1 — SWC UP:   log "Motor DISABLED", không bánh nào quay
//   Bước 2 — SWC DOWN: log "Motor ENABLED",  VrA min → bánh quay chậm
//   Bước 3 — Right stick UP:   4 bánh cùng chiều tiến
//   Bước 4 — Right stick DOWN: 4 bánh cùng chiều lùi
//   Bước 5 — Right stick RIGHT: FL+RR ngược chiều FR+RL → xe đi ngang
//   Bước 6 — Left  stick RIGHT: FL+RL ngược FR+RR → xe xoay phải
//   Bước 7 — Right stick xéo 45°: xe đi xéo (không giật, mượt)
//   Bước 8 — SWC UP giữa chừng: motor dừng ngay lập tức
//   Bước 9 — Tắt remote: log FAILSAFE, motor dừng trong <1s
// ─────────────────────────────────────────────────────────────────────────────

void test_manual(void) {
    ESP_LOGI(TAG, "=== TEST MANUAL CONTROL ===");
    ESP_LOGI(TAG, "Checklist: remote ON? receiver bind? SWC UP? VrA MIN? AGV tren ke?");
    ESP_LOGI(TAG, "Cho 3 giay...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    manual_control_init();

    // manual_control_task tự spawn ibus_task bên trong
    // Priority 5, Core 1 — cùng core với control_task trong production
    xTaskCreatePinnedToCore(
        manual_control_task,
        "manual_ctrl",
        3072,
        NULL,
        5,
        NULL,
        1
    );

    ESP_LOGI(TAG, "Manual control task started");
    ESP_LOGI(TAG, "Dieu khien: Right stick = di chuyen | Left L/R = xoay | SWC = EN | VrA = speed");
    ESP_LOGI(TAG, "--- Thuc hien verify sequence tren serial monitor ---");

    // Task này không làm gì thêm — manual_control_task chạy mãi
    // Chỉ print reminder mỗi 10s
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "[REMINDER] SWC xuong=ON | VrA tang dan khi da verify huong banh");
    }
}