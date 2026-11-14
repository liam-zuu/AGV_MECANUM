#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

// GPIO
#define ULTRASONIC_TRIG     39
#define ULTRASONIC_ECHO_0   35
#define ULTRASONIC_ECHO_1   36
#define ULTRASONIC_ECHO_2   37
#define ULTRASONIC_ECHO_3   38

#define ULTRASONIC_COUNT    4
#define ULTRASONIC_TIMEOUT_US   12000   // 25ms ~ 4m max range
#define ULTRASONIC_MIN_CM       2
#define ULTRASONIC_MAX_CM       400

typedef enum {
    US_FRONT = 0,
    US_BACK,
    US_LEFT,
    US_RIGHT,
} us_id_t;

typedef struct {
    float distance_cm[ULTRASONIC_COUNT];
    bool  valid[ULTRASONIC_COUNT];
} us_data_t;

void ultrasonic_init(void);
void ultrasonic_read_all(us_data_t *data);

#ifdef US_DIAG
// CODE_US (us_diag.h): mẫu chẩn đoán của lần ultrasonic_read_all gần nhất; chỉ gọi từ ultrasonic_task
#include "us_diag.h"
void ultrasonic_diag_get(us_diag_sample_t out[ULTRASONIC_COUNT]);
#endif

#endif