#ifndef MOTOR_H
#define MOTOR_H

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include <stdbool.h>

// PWM config
#define PWM_FREQ_HZ     20000
#define PWM_RESOLUTION  1000    // 0 - 1000 cho MCPWM
#define PWM_MAX         1000

// EN chung
#define GPIO_EN_LEFT    17   // FL + RL
#define GPIO_EN_RIGHT   45   // FR + RR

// Motor pins (RPWM, LPWM)
#define FL_RPWM         8
#define FL_LPWM         3
#define FR_RPWM         40
#define FR_LPWM         41
#define RL_RPWM         46
#define RL_LPWM         9
#define RR_RPWM         47
#define RR_LPWM         21

typedef enum {
    MOTOR_FL = 0,
    MOTOR_FR,
    MOTOR_RL,
    MOTOR_RR,
    MOTOR_COUNT
} motor_id_t;

void motor_init(void);
void motor_set(motor_id_t id, int speed); // -1000 đến +1000
void motor_stop_all(void);
void motor_enable(bool en);

#endif