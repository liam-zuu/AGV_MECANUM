#ifndef RASP_UART_H
#define RASP_UART_H

#include "driver/uart.h"
#include <stdint.h>
#include <stdbool.h>

#define RASP_UART_NUM       UART_NUM_2
#define RASP_GPIO_TX        16
#define RASP_GPIO_RX        15
#define RASP_BAUD           115200
#define RASP_BUF_SIZE       256

// Packet format: [START][TYPE][LEN][DATA...][CHECKSUM]
#define RASP_START_BYTE     0xAA

typedef enum {
    RASP_MSG_SETPOINT = 0x01,   // RPi5 → ESP32: trajectory setpoint
    RASP_MSG_STATUS   = 0x02,   // ESP32 → RPi5: status report
} rasp_msg_type_t;

typedef struct {
    float vx, vy, wz;   // setpoint từ RPi5
    bool valid;
} rasp_setpoint_t;

void rasp_uart_init(void);
bool rasp_uart_read_setpoint(rasp_setpoint_t *setpoint);
void rasp_uart_send_status(float vx, float vy, float wz);

#endif