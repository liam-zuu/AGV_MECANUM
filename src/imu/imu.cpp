#include "imu.h"
#include "BNO08x.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "IMU";

// ─── GPIO config (khớp với GPIO map đã chốt) ─────────────────────────────────
#define IMU_MOSI    GPIO_NUM_11
#define IMU_MISO    GPIO_NUM_13
#define IMU_CLK     GPIO_NUM_12
#define IMU_CS      GPIO_NUM_10
#define IMU_INT     GPIO_NUM_14
#define IMU_RST     GPIO_NUM_1    // reset BNO085 (buzzer là GPIO17, không dùng chung)

// ─── Globals ──────────────────────────────────────────────────────────────────
static BNO08x          *g_imu       = nullptr;
static SemaphoreHandle_t g_mutex    = nullptr;
static imu_data_t        g_data     = {};
static volatile imu_report_hook_t g_hook = nullptr;

extern "C" void imu_set_report_hook(imu_report_hook_t hook) { g_hook = hook; }

// ─── Init ─────────────────────────────────────────────────────────────────────
extern "C" void imu_init(void) {
    g_mutex = xSemaphoreCreateMutex();
    configASSERT(g_mutex);

    // Config GPIO theo hardware đã chốt
    bno08x_config_t cfg;
    cfg.io_mosi    = IMU_MOSI;
    cfg.io_miso    = IMU_MISO;
    cfg.io_sclk    = IMU_CLK;
    cfg.io_cs      = IMU_CS;
    cfg.io_int     = IMU_INT;
    cfg.io_rst     = IMU_RST;
    cfg.sclk_speed = 3000000;   // 3MHz — safe starting point cho BNO085

    g_imu = new BNO08x(cfg);

    if (!g_imu->initialize()) {
        ESP_LOGE(TAG, "BNO085 init failed! Kiem tra: PS0/PS1 da bridge? Day noi dung chua?");
        return;
    }

    // Enable các report cần dùng, cùng chu kỳ IMU_REPORT_PERIOD_US
    g_imu->rpt.accelerometer.enable(IMU_REPORT_PERIOD_US);   // accelerometer (m/s²)
    g_imu->rpt.cal_gyro.enable(IMU_REPORT_PERIOD_US);        // gyro calibrated (rad/s) — heading control
    g_imu->rpt.rv_game.enable(IMU_REPORT_PERIOD_US);         // game rotation vector → euler angles

    // Callback chạy trong task callback của thư viện, mỗi lần đúng một report (rpt_id).
    // Chỉ cập nhật field của report đó và ghi thời điểm nhận phía host.
    g_imu->register_cb([](uint8_t rpt_id) {
        int64_t t_us = esp_timer_get_time();
        float v[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        uint32_t n = 0;

        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return;

        if (rpt_id == IMU_RPT_ACCEL) {
            auto a = g_imu->rpt.accelerometer.get();
            g_data.accel_x = a.x;
            g_data.accel_y = a.y;
            g_data.accel_z = a.z;
            g_data.accel_t_us = t_us;
            n = ++g_data.accel_n;
            v[0] = a.x; v[1] = a.y; v[2] = a.z;
        } else if (rpt_id == IMU_RPT_GYRO) {
            auto g = g_imu->rpt.cal_gyro.get();
            g_data.gyro_x = g.x;
            g_data.gyro_y = g.y;
            g_data.gyro_z = g.z;   // wz — dùng cho heading control
            g_data.gyro_t_us = t_us;
            n = ++g_data.gyro_n;
            v[0] = g.x; v[1] = g.y; v[2] = g.z;
        } else if (rpt_id == IMU_RPT_GAME_RV) {
            auto e = g_imu->rpt.rv_game.get_euler();
            g_data.roll  = e.x;
            g_data.pitch = e.y;
            g_data.yaw   = e.z;
            g_data.rv_t_us = t_us;
            n = ++g_data.rv_n;
            v[0] = e.x; v[1] = e.y; v[2] = e.z;
        } else {
            xSemaphoreGive(g_mutex);
            return;   // report khác (hoặc id 0 lúc task khởi động): bỏ qua
        }

        g_data.valid = true;
        xSemaphoreGive(g_mutex);

        imu_report_hook_t hook = g_hook;
        if (hook) hook(rpt_id, t_us, n, v);
    });

    ESP_LOGI(TAG, "BNO085 init OK — accel/gyro/euler @ %lu us", (unsigned long)IMU_REPORT_PERIOD_US);
}

#ifdef TEST_IMU_TRACE
// ─── Kịch bản trace (env imu_trace_*): đổi chu kỳ cả 3 report, xung RST ───────
extern "C" bool imu_trace_set_period(uint32_t period_us) {
    if (!g_imu) return false;
    bool ok = g_imu->rpt.accelerometer.enable(period_us);
    ok = g_imu->rpt.cal_gyro.enable(period_us) && ok;
    ok = g_imu->rpt.rv_game.enable(period_us) && ok;
    return ok;
}

extern "C" bool imu_trace_hard_reset(void) {
    return g_imu && g_imu->hard_reset();   // thư viện tự bật lại các report sau reset
}
#endif

// ─── Thread-safe getter ───────────────────────────────────────────────────────
extern "C" bool imu_get(imu_data_t *out) {
    if (!out || !g_mutex) return false;
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;
    *out = g_data;
    xSemaphoreGive(g_mutex);
    return out->valid;
}
