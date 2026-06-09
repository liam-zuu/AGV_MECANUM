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
#define IMU_RST     GPIO_NUM_NC   // kéo lên 3.3V qua 10K, không dùng GPIO

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
    cfg.spi_clk_speed = 3000000;   // 3MHz — safe starting point cho BNO085

    g_imu = new BNO08x(cfg);

    if (!g_imu->initialize()) {
        ESP_LOGE(TAG, "BNO085 init failed! Kiem tra: PS0/PS1 da bridge? Dây nối đúng chưa?");
        return;
    }

    // Enable các report cần dùng
    // 50ms interval = 20Hz — đủ cho AGV
    g_imu->rpt.cal_accel.enable(50000UL);   // accelerometer (m/s²)
    g_imu->rpt.cal_gyro.enable(50000UL);    // gyro (rad/s) — quan trọng nhất cho heading
    g_imu->rpt.rv_game.enable(50000UL);     // game rotation vector → euler angles

    // Callback: tự động cập nhật g_data mỗi khi có data mới
    g_imu->register_cb([](uint8_t rpt_id) {
        imu_data_t tmp = {};

        if (g_imu->rpt.cal_accel.has_new_data()) {
            bno08x_accel_t a = g_imu->rpt.cal_accel.get();
            tmp.accel_x = a.x;
            tmp.accel_y = a.y;
            tmp.accel_z = a.z;
        }

        if (g_imu->rpt.cal_gyro.has_new_data()) {
            bno08x_gyro_t g = g_imu->rpt.cal_gyro.get();
            tmp.gyro_x = g.x;
            tmp.gyro_y = g.y;
            tmp.gyro_z = g.z;   // wz — dùng cho heading control
        }

        if (g_imu->rpt.rv_game.has_new_data()) {
            bno08x_euler_angle_t e = g_imu->rpt.rv_game.get_euler();
            tmp.roll  = e.x;
            tmp.pitch = e.y;
            tmp.yaw   = e.z;
        }

        tmp.valid = true;

        if (xSemaphoreTakeFromISR(g_mutex, nullptr) == pdTRUE) {
            g_data = tmp;
            xSemaphoreGiveFromISR(g_mutex, nullptr);
        }
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