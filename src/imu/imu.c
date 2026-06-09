#include "imu.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "IMU";

static spi_device_handle_t spi_handle;
static shtp_t shtp;

// Q point cho fixed-point conversion
#define Q(n) (1 << n)

static float fixed_to_float(int16_t val, int q) {
    return (float)val / (float)Q(q);
}

void imu_init(void) {
    // SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = IMU_MOSI,
        .miso_io_num = IMU_MISO,
        .sclk_io_num = IMU_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,
    };
    spi_bus_initialize(IMU_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

    // SPI device
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = IMU_SPI_FREQ,
        .mode           = 3,
        .spics_io_num   = IMU_CS,
        .queue_size     = 4,
        .cs_ena_pretrans = 2,
    };
    spi_bus_add_device(IMU_SPI_HOST, &dev_cfg, &spi_handle);

    // INT pin
    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << IMU_INT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&int_cfg);

    // Init SHTP
    shtp_init(&shtp, spi_handle);
    vTaskDelay(pdMS_TO_TICKS(200)); // Đợi BNO085 boot

    // Enable reports: 10ms interval (100Hz)
    shtp_enable_report(&shtp, SENSOR_REPORT_ACCEL,           10000);
    vTaskDelay(pdMS_TO_TICKS(10));
    shtp_enable_report(&shtp, SENSOR_REPORT_GYRO,            10000);
    vTaskDelay(pdMS_TO_TICKS(10));
    shtp_enable_report(&shtp, SENSOR_REPORT_ROTATION_VECTOR, 10000);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "IMU init done");
}

bool imu_read(imu_data_t *data) {
    // Đợi INT pin xuống LOW
    if (gpio_get_level(IMU_INT)) return false;

    uint8_t channel;
    uint8_t buf[SHTP_MAX_PACKET_SIZE];
    uint16_t len;

    if (!shtp_receive(&shtp, &channel, buf, &len)) return false;
    if (channel != SHTP_CHAN_REPORTS || len < 1) return false;

    uint8_t report_id = buf[0];

    switch (report_id) {
    case SENSOR_REPORT_ACCEL:
        // Q point = 8
        data->ax = fixed_to_float((int16_t)(buf[4] | buf[5] << 8), 8);
        data->ay = fixed_to_float((int16_t)(buf[6] | buf[7] << 8), 8);
        data->az = fixed_to_float((int16_t)(buf[8] | buf[9] << 8), 8);
        break;

    case SENSOR_REPORT_GYRO:
        // Q point = 9
        data->gx = fixed_to_float((int16_t)(buf[4] | buf[5] << 8), 9);
        data->gy = fixed_to_float((int16_t)(buf[6] | buf[7] << 8), 9);
        data->gz = fixed_to_float((int16_t)(buf[8] | buf[9] << 8), 9);
        break;

    case SENSOR_REPORT_ROTATION_VECTOR:
        // Q point = 14 cho quaternion
        data->qi = fixed_to_float((int16_t)(buf[4]  | buf[5]  << 8), 14);
        data->qj = fixed_to_float((int16_t)(buf[6]  | buf[7]  << 8), 14);
        data->qk = fixed_to_float((int16_t)(buf[8]  | buf[9]  << 8), 14);
        data->qr = fixed_to_float((int16_t)(buf[10] | buf[11] << 8), 14);

        // Quaternion → Euler
        float qw = data->qr, qx = data->qi, qy = data->qj, qz = data->qk;
        data->yaw   = atan2f(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz)) * 180.0f / M_PI;
        data->pitch = asinf(2*(qw*qy - qz*qx)) * 180.0f / M_PI;
        data->roll  = atan2f(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)) * 180.0f / M_PI;
        break;

    default:
        return false;
    }

    data->valid = true;
    return true;
}