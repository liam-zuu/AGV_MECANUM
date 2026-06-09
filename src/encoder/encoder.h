#ifndef ENCODER_H
#define ENCODER_H

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

// Encoder GPIO
#define FL_ENC_A    4
#define FL_ENC_B    5
#define FR_ENC_A    2
#define FR_ENC_B    42
#define RL_ENC_A    6
#define RL_ENC_B    7
#define RR_ENC_A    20
#define RR_ENC_B    19

#define PPR         1320    // Pulses per revolution

typedef enum {
    ENC_FL = 0,
    ENC_FR,
    ENC_RL,
    ENC_RR,
    ENC_COUNT
} enc_id_t;

void encoder_init(void);
int32_t encoder_get_count(enc_id_t id);
void encoder_clear(enc_id_t id);
float encoder_get_rpm(enc_id_t id);    // Gọi định kỳ 10ms

#endif