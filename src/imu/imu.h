#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "shtp.h"

// SPI pins
#define IMU_CS      10
#define IMU_MOSI    11
#define IMU_CLK     12
#define IMU_MISO    13
#define IMU_INT     14

#define IMU_SPI_HOST    SPI2_HOST
#define IMU_SPI_FREQ    3000000  // 3MHz — BNO085 max SPI 3MHz

typedef struct {
    float ax, ay, az;       // m/s²
    float gx, gy, gz;       // rad/s
    float yaw, pitch, roll; // degrees
    float qr, qi, qj, qk;
    bool valid;
} imu_data_t;

void imu_init(void);
bool imu_read(imu_data_t *data);

#endif