#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

#include <stdbool.h>

// ─── Channel mapping (FS-i6X) ─────────────────────────────────────────────────
#define MC_CH_VY        0    // CH1 — Right stick L/R  → strafe
#define MC_CH_VX        1    // CH2 — Right stick U/D  → tiến/lùi
#define MC_CH_WZ        3    // CH4 — Left  stick L/R  → xoay
#define MC_CH_ENABLE    4    // CH5 — SWC              → motor EN
#define MC_CH_SPEED     5    // CH6 — VrA              → speed limit

// SWC: chỉ vị trí DOWN (~2000) mới enable motor
#define MC_SWC_ON_THRESHOLD     1800

// Speed limit: map VrA [1000,2000] → [MIN%, 100%]
#define MC_SPEED_MIN_PCT        20   // % tối thiểu khi VrA hết về 0
#define MC_SPEED_MAX_PCT        100

// Tốc độ tịnh tiến tối đa gửi vào kinematics (m/s)
// W_MAX=34.56 rad/s × r=0.0485m ≈ 1.67 m/s, dùng 1.2 m/s cho an toàn khi test
#define MC_VXY_MAX              1.2f   // m/s

// Tốc độ xoay tối đa (rad/s)
// Giới hạn 2.0 rad/s ≈ 115°/s — đủ nhanh, không quá giật
#define MC_WZ_MAX               2.0f   // rad/s

// ─── API ──────────────────────────────────────────────────────────────────────
void manual_control_init(void);
void manual_control_task(void *pvParam);   // xTaskCreate stack ≥ 3072

#endif // MANUAL_CONTROL_H