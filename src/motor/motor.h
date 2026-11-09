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
#define GPIO_EN         45

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

// Thông tin một lần ghi compare (env đo S06): thời điểm trước/sau, compare theo GPIO
typedef struct {
    int64_t  t_before, t_after;   // esp_timer [µs]
    int      gpio[2];             // GPIO nối ra H7
    uint32_t cmp[2];              // compare đã ghi (0..PWM_RESOLUTION) cho gpio[0], gpio[1]
} motor_set_info_t;
void motor_set_info(motor_id_t id, int speed, motor_set_info_t *info);   // info có thể NULL
void motor_stop_all(void);
void motor_enable(bool en);

#endif