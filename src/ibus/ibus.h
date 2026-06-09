#ifndef IBUS_H
#define IBUS_H

#include "driver/uart.h"

#define IBUS_UART_NUM       UART_NUM_1
#define IBUS_GPIO_RX        18
#define IBUS_BAUD           115200
#define IBUS_PACKET_LEN     32
#define IBUS_NUM_CHANNELS   10

// Giá trị raw từ receiver (1000-2000us)
#define IBUS_MIN            1000
#define IBUS_MID            1500
#define IBUS_MAX            2000

typedef struct {
    uint16_t channel[IBUS_NUM_CHANNELS];
    bool valid;
} ibus_data_t;

void ibus_init(void);
bool ibus_read(ibus_data_t *data);

// Helper: map channel về -1000 đến +1000
int ibus_channel_normalized(ibus_data_t *data, int ch);

#endif