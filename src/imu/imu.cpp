#include "imu.h"
#include "BNO08x.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "IMU";

// ─── GPIO config (khớp với GPIO map đã chốt) ─────────────────────────────────
#define IMU_MOSI    GPIO_NUM_11
#define IMU_MISO    GPIO_NUM_13
#define IMU_CLK     GPIO_NUM_12
#define IMU_CS      GPIO_NUM_10
#define IMU_INT     GPIO_NUM_14
#define IMU_RST     GPIO_NUM_1    // share với buzzer — thư viện dùng để hard reset BNO085

// ─── Globals ──────────────────────────────────────────────────────────────────
static BNO08x          *g_imu       = nullptr;
static SemaphoreHandle_t g_mutex    = nullptr;
static imu_data_t        g_data     = {};

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

    // Enable các report cần dùng
    // 50ms interval = 20Hz — đủ cho AGV
    g_imu->rpt.accelerometer.enable(50000UL);   // accelerometer (m/s²)
    g_imu->rpt.cal_gyro.enable(50000UL);        // gyro calibrated (rad/s) — heading control
    g_imu->rpt.rv_game.enable(50000UL);         // game rotation vector → euler angles

    // Callback: cập nhật từng field riêng lẻ — KHÔNG zero out toàn bộ g_data
    // Vì mỗi report có rate riêng, callback fire nhiều lần, mỗi lần chỉ có 1 report có data mới
    g_imu->register_cb([](uint8_t rpt_id) {
        if (xSemaphoreTakeFromISR(g_mutex, nullptr) != pdTRUE) return;

        if (g_imu->rpt.accelerometer.has_new_data()) {
            auto a = g_imu->rpt.accelerometer.get();
            g_data.accel_x = a.x;
            g_data.accel_y = a.y;
            g_data.accel_z = a.z;
        }

        if (g_imu->rpt.cal_gyro.has_new_data()) {
            auto g = g_imu->rpt.cal_gyro.get();
            g_data.gyro_x = g.x;
            g_data.gyro_y = g.y;
            g_data.gyro_z = g.z;   // wz — dùng cho heading control
        }

        if (g_imu->rpt.rv_game.has_new_data()) {
            auto e = g_imu->rpt.rv_game.get_euler();
            g_data.roll  = e.x;
            g_data.pitch = e.y;
            g_data.yaw   = e.z;
        }

        g_data.valid = true;
        xSemaphoreGiveFromISR(g_mutex, nullptr);
    });

    ESP_LOGI(TAG, "BNO085 init OK — accel/gyro/euler @ 20Hz");
}

// ─── Thread-safe getter ───────────────────────────────────────────────────────
extern "C" bool imu_get(imu_data_t *out) {
    if (!out || !g_mutex) return false;
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;
    *out = g_data;
    xSemaphoreGive(g_mutex);
    return out->valid;
}