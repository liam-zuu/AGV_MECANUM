#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────
// Config — TODO: đổi IP laptop
// ─────────────────────────────────────────
#define LOGGER_PC_IP        "192.168.10.114"
#define LOGGER_UDP_PORT     5005
#define LOGGER_QUEUE_SIZE   32      // buffer 32 frame ở 100Hz = 320ms headroom

// ─────────────────────────────────────────
// Log data struct
// ─────────────────────────────────────────
typedef struct {
    int64_t timestamp_us;

    // Encoder
    int32_t fl_count, fr_count, rl_count, rr_count;
    float   fl_rpm,   fr_rpm,   rl_rpm,   rr_rpm;

    // Velocity actual (forward kinematics)
    float vx, vy, wz;

    // Setpoint (từ IBus hoặc RPi5)
    float sp_vx, sp_vy, sp_wz;

    // PWM command (-1000 ~ +1000)
    int pwm_fl, pwm_fr, pwm_rl, pwm_rr;

    // IMU — accelerometer (m/s²)
    float ax, ay, az;

    // IMU — gyroscope (rad/s)
    float gx, gy, gz;

    // IMU — euler (degrees)
    float yaw, pitch, roll;
} log_data_t;

// ─────────────────────────────────────────
// API
// ─────────────────────────────────────────
void logger_init(void);
void logger_push(const log_data_t *data);   // non-blocking, gọi từ control_task
void logger_task(void *pv);                 // tạo task trong app_main, core 0

#endif // LOGGER_H
