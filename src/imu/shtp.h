#ifndef SHTP_H
#define SHTP_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"

#define SHTP_MAX_PACKET_SIZE    256

// SHTP Channel numbers
#define SHTP_CHAN_COMMAND        0
#define SHTP_CHAN_EXECUTABLE     1
#define SHTP_CHAN_CONTROL        2
#define SHTP_CHAN_REPORTS        3
#define SHTP_CHAN_WAKE_REPORTS   4
#define SHTP_CHAN_GYRO           5

// Report IDs
#define SHTP_REPORT_PRODUCT_ID_REQ      0xF9
#define SHTP_REPORT_PRODUCT_ID_RESP     0xF8
#define SHTP_REPORT_SET_FEATURE         0xFD

// Sensor Report IDs
#define SENSOR_REPORT_ACCEL             0x01
#define SENSOR_REPORT_GYRO              0x02
#define SENSOR_REPORT_ROTATION_VECTOR   0x05

typedef struct {
    spi_device_handle_t spi;
    uint8_t seq[6];  // sequence number per channel
} shtp_t;

void shtp_init(shtp_t *shtp, spi_device_handle_t spi);
bool shtp_receive(shtp_t *shtp, uint8_t *channel, uint8_t *buf, uint16_t *len);
bool shtp_send(shtp_t *shtp, uint8_t channel, uint8_t *buf, uint16_t len);
bool shtp_enable_report(shtp_t *shtp, uint8_t report_id, uint32_t interval_us);

#endif