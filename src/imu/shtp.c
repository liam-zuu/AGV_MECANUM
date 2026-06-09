#include "shtp.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// static const char *TAG = "SHTP";

void shtp_init(shtp_t *shtp, spi_device_handle_t spi) {
    shtp->spi = spi;
    memset(shtp->seq, 0, sizeof(shtp->seq));
}

static bool spi_transfer(shtp_t *shtp, uint8_t *tx, uint8_t *rx, uint16_t len) {
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(shtp->spi, &t) == ESP_OK;
}

bool shtp_receive(shtp_t *shtp, uint8_t *channel, uint8_t *buf, uint16_t *len) {
    uint8_t header[4] = {0};
    uint8_t tx[4]     = {0};

    // Đọc 4-byte SHTP header trước
    if (!spi_transfer(shtp, tx, header, 4)) return false;

    uint16_t packet_len = (header[0] | (header[1] << 8)) & 0x7FFF;
    if (packet_len == 0 || packet_len > SHTP_MAX_PACKET_SIZE) return false;

    *channel = header[2];
    // header[3] là sequence number

    // Đọc payload
    uint16_t payload_len = packet_len - 4;
    if (payload_len == 0) {
        *len = 0;
        return true;
    }

    uint8_t tx_buf[SHTP_MAX_PACKET_SIZE] = {0};
    if (!spi_transfer(shtp, tx_buf, buf, payload_len)) return false;

    *len = payload_len;
    return true;
}

bool shtp_send(shtp_t *shtp, uint8_t channel, uint8_t *buf, uint16_t len) {
    uint8_t packet[SHTP_MAX_PACKET_SIZE] = {0};
    uint16_t total_len = len + 4;

    // SHTP header
    packet[0] = total_len & 0xFF;
    packet[1] = (total_len >> 8) & 0x7F;
    packet[2] = channel;
    packet[3] = shtp->seq[channel]++;

    memcpy(&packet[4], buf, len);

    uint8_t rx[SHTP_MAX_PACKET_SIZE] = {0};
    return spi_transfer(shtp, packet, rx, total_len);
}

bool shtp_enable_report(shtp_t *shtp, uint8_t report_id, uint32_t interval_us) {
    uint8_t cmd[17] = {0};
    cmd[0]  = SHTP_REPORT_SET_FEATURE;
    cmd[1]  = report_id;
    cmd[2]  = 0;                            // feature flags
    cmd[3]  = 0;                            // change sensitivity LSB
    cmd[4]  = 0;                            // change sensitivity MSB
    cmd[5]  = (interval_us >> 0)  & 0xFF;   // report interval LSB
    cmd[6]  = (interval_us >> 8)  & 0xFF;
    cmd[7]  = (interval_us >> 16) & 0xFF;
    cmd[8]  = (interval_us >> 24) & 0xFF;   // report interval MSB
    cmd[9]  = 0;                            // batch interval
    cmd[10] = 0;
    cmd[11] = 0;
    cmd[12] = 0;
    cmd[13] = 0;                            // sensor specific config
    cmd[14] = 0;
    cmd[15] = 0;
    cmd[16] = 0;

    return shtp_send(shtp, SHTP_CHAN_CONTROL, cmd, sizeof(cmd));
}