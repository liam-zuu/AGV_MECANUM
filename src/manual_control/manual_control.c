#include "manual_control.h"
#include "ibus/ibus.h"
#include "motor/motor.h"
#include "kinematics/kinematics.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MANUAL";

// ─── Init ─────────────────────────────────────────────────────────────────────
void manual_control_init(void) {
    motor_init();
    ibus_init();
    ESP_LOGI(TAG, "Manual control init done");
}

// ─── Helpers ──────────────────────────────────────────────────────────────────

// Map ibus normalized [-1000,+1000] → float [-scale, +scale]
static inline float norm_to_float(int norm, float scale) {
    return (float)norm / 1000.0f * scale;
}

// Map VrA raw [1000,2000] → speed multiplier [MIN_PCT/100, 1.0]
static float get_speed_limit(int vra_raw) {
    // Clamp
    if (vra_raw < 1000) vra_raw = 1000;
    if (vra_raw > 2000) vra_raw = 2000;

    float pct = MC_SPEED_MIN_PCT
              + (float)(vra_raw - 1000) / 1000.0f
              * (MC_SPEED_MAX_PCT - MC_SPEED_MIN_PCT);
    return pct / 100.0f;   // 0.20 – 1.00
}

// ─── Task ─────────────────────────────────────────────────────────────────────
void manual_control_task(void *pvParam) {
    // Spawn ibus_task trước
    xTaskCreatePinnedToCore(ibus_task, "ibus_task", 2048, NULL, 4, NULL, 0);
    ESP_LOGI(TAG, "Manual control task running");
    ESP_LOGI(TAG, "SWC xuong = motor ON | VrA = speed limit | Right stick = di chuyen | Left L/R = xoay");

    bool motor_on = false;

    while (1) {
        ibus_data_t rc;
        bool got = ibus_get(&rc);

        // ── Mất tín hiệu hoặc failsafe ───────────────────────────────────────
        if (!got || g_ibus_failsafe_count >= IBUS_FAILSAFE_FRAMES) {
            if (motor_on) {
                motor_stop_all();
                motor_enable(false);
                motor_on = false;
                ESP_LOGW(TAG, "FAILSAFE — motor disabled");
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // ── CH5 SWC: enable / disable motor ──────────────────────────────────
        bool sw_on = (rc.channel[MC_CH_ENABLE] >= MC_SWC_ON_THRESHOLD);
        if (sw_on != motor_on) {
            motor_on = sw_on;
            motor_enable(motor_on);
            if (!motor_on) motor_stop_all();
            ESP_LOGI(TAG, "Motor %s (SWC)", motor_on ? "ENABLED" : "DISABLED");
        }

        if (!motor_on) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // ── Đọc stick values ─────────────────────────────────────────────────
        int n_vy = ibus_channel_normalized(&rc, MC_CH_VY);   // Right L/R
        int n_vx = ibus_channel_normalized(&rc, MC_CH_VX);   // Right U/D
        int n_wz = ibus_channel_normalized(&rc, MC_CH_WZ);   // Left  L/R

        // ── Speed limit từ VrA ───────────────────────────────────────────────
        float speed_lim = get_speed_limit(rc.channel[MC_CH_SPEED]);

        // ── Normalize → m/s, rad/s, áp speed limit ───────────────────────────
        agv_velocity_t vel = {
            .vx = norm_to_float(n_vx, MC_VXY_MAX) * speed_lim,
            .vy = norm_to_float(n_vy, MC_VXY_MAX) * speed_lim,
            .wz = norm_to_float(n_wz, MC_WZ_MAX)  * speed_lim,
        };

        // ── Inverse kinematics → wheel rad/s ─────────────────────────────────
        wheel_velocity_t w = kinematics_inverse(vel);

        // ── rad/s → PWM [-1000, +1000] ────────────────────────────────────────
        int pwm_fl = kinematics_radps_to_pwm(w.fl);
        int pwm_fr = kinematics_radps_to_pwm(w.fr);
        int pwm_rl = kinematics_radps_to_pwm(w.rl);
        int pwm_rr = kinematics_radps_to_pwm(w.rr);

        // ── Set motor ─────────────────────────────────────────────────────────
        motor_set(MOTOR_FL, pwm_fl);
        motor_set(MOTOR_FR, pwm_fr);
        motor_set(MOTOR_RL, pwm_rl);
        motor_set(MOTOR_RR, pwm_rr);

        // ── Log mỗi 500ms để không spam ──────────────────────────────────────
        static TickType_t last_log = 0;
        TickType_t now = xTaskGetTickCount();
        if (now - last_log >= pdMS_TO_TICKS(500)) {
            last_log = now;
            ESP_LOGI(TAG,
                "Vx:%.2f Vy:%.2f Wz:%.2f spd:%.0f%%  PWM FL:%d FR:%d RL:%d RR:%d",
                vel.vx, vel.vy, vel.wz, speed_lim * 100,
                pwm_fl, pwm_fr, pwm_rl, pwm_rr);
        }

        vTaskDelay(pdMS_TO_TICKS(20));   // 50Hz control loop
    }
}