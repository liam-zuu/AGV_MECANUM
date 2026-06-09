#ifndef IMU_H
#define IMU_H

#include <stdbool.h>

// ─── Data struct (C-compatible) ───────────────────────────────────────────────
typedef struct {
    float accel_x;   // m/s²
    float accel_y;
    float accel_z;
    float gyro_x;    // rad/s
    float gyro_y;
    float gyro_z;    // wz — dùng cho heading control
    float roll;      // độ
    float pitch;
    float yaw;
    bool  valid;
} imu_data_t;

// ─── API (C-compatible, implement bên trong .cpp) ─────────────────────────────
#ifdef __cplusplus
extern "C" {
#endif

void imu_init(void);
bool imu_get(imu_data_t *out);   // thread-safe copy data ra ngoài

#ifdef __cplusplus
}
#endif

#endif // IMU_H