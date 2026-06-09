#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdbool.h>

#define LOGGER_UDP_PORT     5005
// TODO: điền IP laptop
#define LOGGER_PC_IP        "192.168.1.100"

#define LOGGER_QUEUE_SIZE   20

typedef struct {
    int64_t timestamp_us;

    // Encoder
    int32_t fl_count, fr_count, rl_count, rr_count;
    float   fl_rpm,   fr_rpm,   rl_rpm,   rr_rpm;

    // Velocity
    float vx, vy, wz;

    // Setpoint
    float sp_vx, sp_vy, sp_wz;

    // IMU
    float ax, ay, az;
    float gx, gy, gz;
    float yaw, pitch, roll;
} log_data_t;

void logger_init(void);
void logger_push(const log_data_t *data);  // Gọi từ main task
void logger_task(void *pv);                // Chạy task riêng

#endif