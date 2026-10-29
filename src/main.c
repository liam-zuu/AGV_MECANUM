#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

#include "motor/motor.h"
#include "encoder/encoder.h"
#include "ibus/ibus.h"
#include "kinematics/kinematics.h"
#include "logger/logger.h"
#include "wifi/wifi.h"
#include "imu/imu.h"
#include "led_buzzer/led_buzzer.h"
#include "traj_receiver/traj_receiver.h"
#include "ultrasonic/ultrasonic.h"
#include "manual_control/manual_control.h"
#include "controller/vel_ctrl.h"
#include "tests/test_runner.h"

static const char *TAG = "MAIN";

// ─── Controller mode: CTRL_MODE trong controller/vel_ctrl.h ──────────────────

// ─── IBus channel (tay cầm đếm từ 1, index = channel - 1) ───────────────────
#define CH_LX   0   // channel 1 — left stick X  → vy
#define CH_LY   1   // channel 2 — left stick Y  → vx
#define CH_RY   2   // channel 3 — right stick Y (unused)
#define CH_RX   3   // channel 4 — right stick X → wz
#define CH_SWC  4   // channel 5 — nấc 3 (>1700) → MANUAL
#define CH_VRA  5   // channel 6 — speed scale
#define CH_SWA  6   // channel 7 — motor enable
#define CH_SWD  7   // channel 8 — buzzer
#define CH_SWB  8   // channel 9 — tắt/bật ultrasonic obstacle

// ─── Threshold ────────────────────────────────────────────────────────────────
#define OBSTACLE_CM      20.0f      // tăng lên 20cm — US-015 không ổn định dưới 10cm
#define TRAJ_TIMEOUT_US  200000LL   // 200ms tính bằng µs

// ─── System mode ─────────────────────────────────────────────────────────────
typedef enum {
    MODE_WAITING = 0,
    MODE_MANUAL,
    MODE_AUTO,
} system_mode_t;

// ─── Shared state ─────────────────────────────────────────────────────────────
static volatile system_mode_t  g_mode            = MODE_WAITING;
static volatile bool           g_emergency_stop  = false;
static volatile bool           g_motor_enabled   = false;
// g_setpoint (3 trường) và g_last_traj_us (64 bit) ghi ở lõi 0, đọc ở lõi 1: chỉ đọc/ghi
// trong g_traj_mux để không đọc rách (Codex ctrl-imu #6, Liam duyệt 28/10)
static portMUX_TYPE             g_traj_mux        = portMUX_INITIALIZER_UNLOCKED;
static int64_t                  g_last_traj_us    = 0;

static traj_setpoint_t          g_setpoint        = {0};
static volatile float            g_enc_rpm[4]     = {0};
static volatile int32_t          g_enc_count[4]   = {0};
static volatile us_data_t        g_us_data        = {0};
static SemaphoreHandle_t         g_us_mutex;

static volatile bool g_obstacle_enabled = true;
static volatile bool g_imu_fail         = false;   // imu_init() lỗi: motor tắt, LED/còi báo (§7)

// ─────────────────────────────────────────
// CORE 0 - traj_task (priority 3)
// ─────────────────────────────────────────
static void traj_task(void *pv) {
    while (1) {
        /* Một bản chụp của gói cuối (setpoint, nguồn, thời điểm nhận). g_last_traj_us lấy
         * thời điểm NHẬN gói, không phải lúc task này thấy nó: control_task tính hết hạn
         * TRAJ_TIMEOUT_US từ gói thật cuối cùng, hết timeout kép ~400 ms (DUT nhóm 1,
         * Liam duyệt Q5a 29/10) */
        traj_snapshot_t s;
        traj_receiver_snapshot(&s);
        bool fresh = traj_receiver_is_fresh(&s, esp_timer_get_time());
        if (fresh) {
            ESP_LOGI(TAG, "traj fresh src=%d vx=%.2f", s.src, s.sp.vx);  // thêm
            taskENTER_CRITICAL(&g_traj_mux);
            g_setpoint     = s.sp;
            g_last_traj_us = s.t_rx_us;
            taskEXIT_CRITICAL(&g_traj_mux);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─────────────────────────────────────────
// CORE 0 - ultrasonic_task (priority 1)
// ─────────────────────────────────────────
static void ultrasonic_task(void *pv) {
    us_data_t buf[3];
    int idx = 0;

    for (int i = 0; i < 3; i++) ultrasonic_read_all(&buf[i]);

    while (1) {
        ultrasonic_read_all(&buf[idx]);
        idx = (idx + 1) % 3;

        us_data_t filtered;
        for (int i = 0; i < ULTRASONIC_COUNT; i++) {
            int valid_count = 0;
            float vals[3];
            for (int j = 0; j < 3; j++) {
                if (buf[j].valid[i]) vals[valid_count++] = buf[j].distance_cm[i];
            }

            if (valid_count == 0) {
                filtered.valid[i]       = false;
                filtered.distance_cm[i] = -1.0f;
            } else if (valid_count == 1) {
                filtered.valid[i]       = true;
                filtered.distance_cm[i] = vals[0];
            } else if (valid_count == 2) {
                filtered.valid[i]       = true;
                filtered.distance_cm[i] = (vals[0] + vals[1]) / 2.0f;
            } else {
                if (vals[0] > vals[1]) { float t = vals[0]; vals[0] = vals[1]; vals[1] = t; }
                if (vals[1] > vals[2]) { float t = vals[1]; vals[1] = vals[2]; vals[2] = t; }
                if (vals[0] > vals[1]) { float t = vals[0]; vals[0] = vals[1]; vals[1] = t; }
                filtered.valid[i]       = true;
                filtered.distance_cm[i] = vals[1];
            }
        }

        if (xSemaphoreTake(g_us_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_us_data = filtered;
            xSemaphoreGive(g_us_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ─────────────────────────────────────────
// CORE 0 - led_task (priority 2)
// ─────────────────────────────────────────
static void led_task(void *pv) {
    float hue = 0.0f;
    while (1) {
        system_mode_t mode = g_mode;
        bool emg = g_emergency_stop;

        if (g_imu_fail) {
            // IMU khởi tạo lỗi (Liam 27–28/10): LED xanh dương NHÁY + còi 1 dài 5 ngắn, lặp mỗi 3 s
            // [ĐỀ XUẤT: dài 600 ms, nghỉ 200 ms, ngắn 120/120 ms]. Khác AUTO-UART (xanh dương sáng liên tục).
            led_set_color(COLOR_BLUE);
            buzzer_beep(2700, 600);
            led_set_color(COLOR_OFF);
            vTaskDelay(pdMS_TO_TICKS(200));
            for (int i = 0; i < 5; i++) {
                led_set_color(COLOR_BLUE);
                buzzer_beep(2700, 120);
                led_set_color(COLOR_OFF);
                vTaskDelay(pdMS_TO_TICKS(120));
            }
            vTaskDelay(pdMS_TO_TICKS(1000));   // 600 + 200 + 5 × 240 + 1000 = 3000 ms
            continue;
        }

        if (emg) {
            led_set_color(COLOR_RED);
            buzzer_beep(2700, 100);
            vTaskDelay(pdMS_TO_TICKS(150));
            led_set_color(COLOR_OFF);
            buzzer_off();
            vTaskDelay(pdMS_TO_TICKS(150));
            continue;
        }

        switch (mode) {
        case MODE_WAITING:
            led_set_color(COLOR_WHITE);
            buzzer_beep(2700, 50);
            vTaskDelay(pdMS_TO_TICKS(500));
            led_set_color(COLOR_OFF);
            buzzer_off();
            vTaskDelay(pdMS_TO_TICKS(500));
            break;

        case MODE_MANUAL: {
            float h = hue / 60.0f;
            int   i = (int)h;
            float f = h - i;
            float q = 1.0f - f;
            uint8_t r = 0, g = 0, b = 0;
            switch (i % 6) {
                case 0: r=255; g=(uint8_t)(255*f); b=0;           break;
                case 1: r=(uint8_t)(255*q); g=255; b=0;           break;
                case 2: r=0;   g=255; b=(uint8_t)(255*f);         break;
                case 3: r=0;   g=(uint8_t)(255*q); b=255;         break;
                case 4: r=(uint8_t)(255*f); g=0;   b=255;         break;
                case 5: r=255; g=0;   b=(uint8_t)(255*q);         break;
            }
            led_set_color((rgb_color_t){r, g, b});
            hue += 3.0f;
            if (hue >= 360.0f) hue = 0.0f;
            vTaskDelay(pdMS_TO_TICKS(30));
            break;
        }

        case MODE_AUTO:
            /* Màu theo source: UART=xanh dương, WiFi=tím */
            if (traj_receiver_source() == TRAJ_SRC_UART)
                led_set_color((rgb_color_t){0, 100, 255});
            else
                led_set_color((rgb_color_t){180, 0, 255});
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }
}

// ─────────────────────────────────────────
// CORE 0 - logger_task: định nghĩa trong logger.c
// ─────────────────────────────────────────

// ─────────────────────────────────────────
// CORE 1 - encoder_task (priority 6)
// ─────────────────────────────────────────
static void encoder_task(void *pv) {
    while (1) {
        for (int i = 0; i < ENC_COUNT; i++) {
            g_enc_rpm[i]   = encoder_get_rpm((enc_id_t)i);
            g_enc_count[i] = encoder_get_count((enc_id_t)i);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─────────────────────────────────────────
// Log 100 Hz (motor bật) / 10 Hz (motor tắt): cột cũ + cột IMU, mất log (spec controller–IMU v2.2 §8)
// ─────────────────────────────────────────
static void push_log(int64_t t_now, agv_velocity_t vel, const vel_ctrl_out_t *vc,
                     const int pwm[4], const imu_data_t *imu, bool motor_on) {
    const imu_fusion_out_t *m = &vc->imu;
    const heading_out_t    *h = &vc->head;
    log_data_t d = {
        .timestamp_us = t_now,
        .fl_count = g_enc_count[ENC_FL],
        .fr_count = g_enc_count[ENC_FR],
        .rl_count = g_enc_count[ENC_RL],
        .rr_count = g_enc_count[ENC_RR],
        .fl_rpm   = g_enc_rpm[ENC_FL],
        .fr_rpm   = g_enc_rpm[ENC_FR],
        .rl_rpm   = g_enc_rpm[ENC_RL],
        .rr_rpm   = g_enc_rpm[ENC_RR],
        .vx       = vc->actual.vx,
        .vy       = vc->actual.vy,
        .wz       = vc->actual.wz,
        .sp_vx    = vel.vx,
        .sp_vy    = vel.vy,
        .sp_wz    = vel.wz,
        .pwm_fl   = pwm[0],
        .pwm_fr   = pwm[1],
        .pwm_rl   = pwm[2],
        .pwm_rr   = pwm[3],
        .ax       = imu->accel_x,
        .ay       = imu->accel_y,
        .az       = imu->accel_z,
        .gx       = imu->gyro_x,
        .gy       = imu->gyro_y,
        .gz       = imu->gyro_z,
        .yaw      = imu->yaw,
        .pitch    = imu->pitch,
        .roll     = imu->roll,
        .tick     = vc->tick,
        .dpwm_fl  = vc->dpwm_sum[0], .dpwm_fr = vc->dpwm_sum[1],
        .dpwm_rl  = vc->dpwm_sum[2], .dpwm_rr = vc->dpwm_sum[3],
        .sat_fl   = vc->sat_ticks[0], .sat_fr = vc->sat_ticks[1],
        .sat_rl   = vc->sat_ticks[2], .sat_rr = vc->sat_ticks[3],

        .kind     = LOG_KIND_DATA,
        .motor_on = motor_on,
        .imu_init = (int8_t)imu_init_status(),
        .wz_meas  = m->wz_meas,
        .wz_cmd   = vc->wz_cmd,
        .gz_fused = m->gz,
        .yaw_rad  = m->yaw,
        .age_g_ms = m->age_g_ms,
        .age_r_ms = m->age_r_ms,
        .gyro_n   = imu->gyro_n,
        .rv_n     = imu->rv_n,
        .reset_n  = imu->reset_n,
        .g_valid  = m->g_valid,
        .r_valid  = m->r_valid,
        .wz_src   = (uint8_t)m->wz_src,
        .src_switch_count = m->src_switch_count,
        .psi_ref  = h->psi_ref,
        .e_psi    = h->e_psi,
        .head_on  = h->on,
        .psi_reset_count = h->psi_reset_count,
        .slip_flag = m->slip_flag,
        .slip_on_count = m->slip_on_count,
        .r        = m->r,
        .bias_gz  = m->bias_gz,
        .src_switch_dwz = m->src_switch_dwz,
        .bias_updates   = m->bias_updates,
        .slip_eval      = m->slip_eval,
    };
    logger_push(&d);
}

// Hết lượt (motor bật → tắt): dòng tổng kết gửi 3 lần; tick mang log_seq của dòng dữ liệu cuối
static void push_run_end(void) {
    log_data_t d = { .kind = LOG_KIND_RUN_END, .tick = logger_seq() };
    for (int i = 0; i < 3; i++) logger_push(&d);
}

// ─────────────────────────────────────────
// CORE 1 - control_task (priority 5)
// ─────────────────────────────────────────
// IMU: control_task tự copy imu_get() mỗi tick (có mutex), thay cho imu_task cũ
// (bản sao volatile cũ có thể bị đọc lệch giữa chừng khi imu_task ưu tiên cao hơn chen vào).
static void control_task(void *pv) {
    const float dt = 0.01f;
    int64_t t_prev = esp_timer_get_time();
    bool    swa_prev = false;
    static imu_data_t s_imu;
    bool    have_imu = false;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz

        int64_t t_now   = esp_timer_get_time();
        float dt_actual = (t_now - t_prev) * 1e-6f;
        t_prev = t_now;
        if (dt_actual <= 0.0f || dt_actual > 0.1f) dt_actual = dt;

        // ── 1. Đọc IBus ──────────────────────────────────────────────────────
        ibus_data_t rc;
        bool has_rc = ibus_get(&rc);

        // ── 2. SWA — motor enable/disable ────────────────────────────────────
        // bool swa_on = has_rc && (rc.channel[CH_SWA] > 1500);
        // if (swa_on != swa_prev) {
        //     g_emergency_stop = false;
        //     swa_prev         = swa_on;
        // }
        // if (g_motor_enabled != swa_on) {
        //     g_motor_enabled = swa_on;
        //     motor_enable(swa_on);
        // }


        // ── 2. SWA — motor enable/disable ────────────────────────────────────
        // HIL_BYPASS_MOTOR_ENABLE comes only from env hil_bench (HIL bench has no RC
        // receiver). Vehicle firmware: SWA enables the motors, and RC loss (ibus
        // valid = false after 20 ms without a byte) disables them (spec
        // controller-IMU v2 §7b, Liam approved 27/10).
        #ifdef HIL_BYPASS_MOTOR_ENABLE
                bool swa_on = true;
        #else
                bool swa_on = has_rc && (rc.channel[CH_SWA] > 1500);
        #endif
                // IMU khởi tạo lỗi: không bao giờ bật motor (spec controller–IMU v2.2 §7)
                if (g_imu_fail) swa_on = false;
                if (swa_on != swa_prev) {
                    g_emergency_stop = false;
                    swa_prev         = swa_on;
                }
                if (g_motor_enabled != swa_on) {
                    g_motor_enabled = swa_on;
                    motor_enable(swa_on);
                }


        // ── 2b. SWB — tắt/bật obstacle detection (chỉ hoạt động ở MANUAL) ───
        if (has_rc && g_mode == MODE_MANUAL) {
            g_obstacle_enabled = (rc.channel[CH_SWB] < 1500);
        } else {
            g_obstacle_enabled = true;
        }

        // ── 3. SWD — buzzer (TODO: task riêng) ───────────────────────────────

        // ── 4. Emergency stop — ultrasonic ───────────────────────────────────
        static bool    s_obstacle      = false;
        static int64_t s_last_update   = 0;
        static int     obstacle_count  = 0;
        static int     clear_count     = 0;
        static bool    s_front_blocked = false;
        static bool    s_back_blocked  = false;
        static bool    s_left_blocked  = false;
        static bool    s_right_blocked = false;

        static us_data_t us_snap = {0};
        bool us_data_fresh = false;

        if (xSemaphoreTake(g_us_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            us_snap = g_us_data;
            xSemaphoreGive(g_us_mutex);
            us_data_fresh = true;
        }

        if (us_data_fresh && (t_now - s_last_update > 50000LL)) {
            bool any_obstacle = false;
            bool all_clear    = true;

            if (g_obstacle_enabled) {
                s_front_blocked = us_snap.valid[US_FRONT] && us_snap.distance_cm[US_FRONT] < OBSTACLE_CM;
                s_back_blocked  = us_snap.valid[US_BACK]  && us_snap.distance_cm[US_BACK]  < OBSTACLE_CM;
                s_left_blocked  = us_snap.valid[US_LEFT]  && us_snap.distance_cm[US_LEFT]  < OBSTACLE_CM;
                s_right_blocked = us_snap.valid[US_RIGHT] && us_snap.distance_cm[US_RIGHT] < OBSTACLE_CM;
            } else {
                s_front_blocked = false;
                s_back_blocked  = false;
                s_left_blocked  = false;
                s_right_blocked = false;
            }

            any_obstacle = s_front_blocked || s_back_blocked ||
                           s_left_blocked  || s_right_blocked;
            all_clear    = !any_obstacle;

            if (any_obstacle) {
                clear_count = 0;
                obstacle_count++;
                if (obstacle_count >= 2) s_obstacle = true;
            } else {
                obstacle_count = 0;
                clear_count++;
                if (clear_count >= 3 && all_clear) s_obstacle = false;
            }
            s_last_update = t_now;
        }
        g_emergency_stop = s_obstacle;

        // ── 5. Xác định mode ─────────────────────────────────────────────────
        static int s_rc_lost_counter = 0;
        bool swc_manual  = has_rc && (rc.channel[CH_SWC] > 1700);
        traj_setpoint_t sp_snap;
        int64_t         traj_us;
        taskENTER_CRITICAL(&g_traj_mux);         // một bản chụp nhất quán (Codex ctrl-imu #6)
        sp_snap = g_setpoint;
        traj_us = g_last_traj_us;
        taskEXIT_CRITICAL(&g_traj_mux);
        bool traj_fresh  = (traj_us > 0) &&
                           ((t_now - traj_us) < TRAJ_TIMEOUT_US);

        if (swc_manual) {
            g_mode = MODE_MANUAL;
            s_rc_lost_counter = 0;
        } else if (traj_fresh) {
            g_mode = MODE_AUTO;
            s_rc_lost_counter = 0;
        } else {
            if (g_mode == MODE_MANUAL && s_rc_lost_counter < 5) {
                s_rc_lost_counter++;
            } else {
                g_mode = MODE_WAITING;
            }
        }

        // ── 6. Motor off: xử lý sau bước 8–10 (vel_ctrl_step chạy mỗi tick để
        //       theo dõi hợp lệ IMU, bias khi đứng yên; spec controller–IMU v2.2 §9)

        // ── 7. Setpoint theo mode ─────────────────────────────────────────────
        agv_velocity_t vel = {0};
        switch (g_mode) {
        case MODE_MANUAL:
            if (has_rc) {
                float speed_scale = (float)(rc.channel[CH_VRA] - 1000) / 1000.0f;
                if (speed_scale < 0.0f) speed_scale = 0.0f;
                if (speed_scale > 1.0f) speed_scale = 1.0f;
                vel.vx =  ibus_channel_normalized(&rc, CH_LY) / 500.0f * speed_scale;
                vel.vy = -ibus_channel_normalized(&rc, CH_LX) / 500.0f * speed_scale;
                vel.wz = -ibus_channel_normalized(&rc, CH_RX) / 500.0f * speed_scale;
            }
            break;
        case MODE_AUTO:
            vel.vx = sp_snap.vx;
            vel.vy = sp_snap.vy;
            vel.wz = sp_snap.wz;
            break;
        case MODE_WAITING:
        default:
            break;
        }

        // ── 7b. Lưu intent (trước khi interlock) ─────────────────────────────
        bool intent_fwd   = vel.vx > 0.0f;
        bool intent_bwd   = vel.vx < 0.0f;
        bool intent_left  = vel.vy > 0.0f;
        bool intent_right = vel.vy < 0.0f;

        // ── 7c. Interlock ─────────────────────────────────────────────────────
        if (s_front_blocked && vel.vx > 0.0f) vel.vx = 0.0f;
        if (s_back_blocked  && vel.vx < 0.0f) vel.vx = 0.0f;
        if (s_left_blocked  && vel.vy > 0.0f) vel.vy = 0.0f;
        if (s_right_blocked && vel.vy < 0.0f) vel.vy = 0.0f;

        // ── 8–10. FK → hợp nhất IMU + giữ hướng → controller → IK → PWM ─────
        //         (controller/vel_ctrl.c, imu_fusion.c, heading.c)
        if (imu_get(&s_imu)) have_imu = true;
        vel_ctrl_in_t vin = {
            .sp         = vel,
            .enc_rpm    = { g_enc_rpm[ENC_FL], g_enc_rpm[ENC_FR],
                            g_enc_rpm[ENC_RL], g_enc_rpm[ENC_RR] },
            .enc_count  = { g_enc_count[ENC_FL], g_enc_count[ENC_FR],
                            g_enc_count[ENC_RL], g_enc_count[ENC_RR] },
            .imu        = have_imu ? &s_imu : NULL,
            .now_us     = t_now,
            .dt         = dt_actual,
            .motor_on   = g_motor_enabled,
            .run_active = g_motor_enabled && g_mode == MODE_AUTO && traj_fresh,
#ifdef RUN_WHEELS_LIFTED
            .wheels_lifted = true,
#else
            .wheels_lifted = false,
#endif
        };
        vel_ctrl_out_t vc = vel_ctrl_step(&vin);

        static bool s_motor_prev = false;
        static int  s_idle_div   = 0;
        if (!g_motor_enabled) {
            motor_stop_all();
            traj_receiver_send_status(0, 0, 0);
            if (s_motor_prev) push_run_end();                 // hết lượt: tổng kết 3 lần (§8)
            s_motor_prev = false;
            if (++s_idle_div >= 10) {                         // dòng nhịp thưa 10 Hz khi motor tắt
                s_idle_div = 0;
                const int pwm0[4] = {0, 0, 0, 0};
                push_log(t_now, vel, &vc, pwm0, &s_imu, false);
            }
            continue;
        }
        s_motor_prev = true;
        s_idle_div   = 0;
        agv_velocity_t actual_vel = vc.actual;
        int pwm_fl = vc.pwm[0];
        int pwm_fr = vc.pwm[1];
        int pwm_rl = vc.pwm[2];
        int pwm_rr = vc.pwm[3];

        // ── 10b. Hard cutoff PWM ──────────────────────────────────────────────
        if (g_obstacle_enabled &&
           ((s_front_blocked && intent_fwd)   ||
            (s_back_blocked  && intent_bwd)   ||
            (s_left_blocked  && intent_left)  ||
            (s_right_blocked && intent_right))) {
            pwm_fl = 0; pwm_fr = 0; pwm_rl = 0; pwm_rr = 0;
        }

        motor_set(MOTOR_FL, pwm_fl);
        motor_set(MOTOR_FR, pwm_fr);
        motor_set(MOTOR_RL, pwm_rl);
        motor_set(MOTOR_RR, pwm_rr);

        // ── 11. Gửi status về H7 ─────────────────────────────────────────────
        traj_receiver_send_status(actual_vel.vx, actual_vel.vy, actual_vel.wz);

        // ── 12. Push log ──────────────────────────────────────────────────────
        const int pwm_out[4] = { pwm_fl, pwm_fr, pwm_rl, pwm_rr };
        vel_ctrl_applied(pwm_out, &vc);   // đứng yên + bộ đếm theo PWM thực áp (Codex ctrl-imu #12)
        push_log(t_now, vel, &vc, pwm_out, &s_imu, true);
    }
}

// ─────────────────────────────────────────
// app_main
// ─────────────────────────────────────────
void app_main(void) {

#ifdef TEST_MODE
    run_tests();
    return;
#endif

    ESP_LOGI(TAG, "AGV starting...");

    // LED + buzzer trước imu_init để báo được lỗi IMU (RST BNO085 là GPIO1, buzzer GPIO17: không dùng chung)
    led_buzzer_init();
    led_set_color(COLOR_YELLOW);
    buzzer_beep(2700, 100);

    // Init peripherals
    wifi_init();
    motor_init();
    encoder_init();
    ibus_init();
    int imu_rc = imu_init();   // chặn tối đa IMU_INIT_TIMEOUT_MS
    if (imu_rc != IMU_INIT_OK) {
        g_imu_fail = true;     // motor không bao giờ bật; led_task báo; log mang imu_init (§7, §8)
        ESP_LOGE(TAG, "IMU_INIT_FAIL %d (%s): motors stay disabled", imu_rc, imu_init_err_str(imu_rc));
    }
    ultrasonic_init();
    traj_receiver_init();
    logger_init();

    // Init controller
    vel_ctrl_init();
    ESP_LOGI(TAG, "Controller: %s", CTRL_MODE == CTRL_PID ? "PID" : "ADRC");

    motor_enable(false);
    g_us_mutex = xSemaphoreCreateMutex();

    if (!g_imu_fail) {
        buzzer_beep(2700, 100);
        vTaskDelay(pdMS_TO_TICKS(100));
        buzzer_beep(2700, 100);
        led_set_color(COLOR_WHITE);
        ESP_LOGI(TAG, "Boot done — WAITING");
    } else {
        ESP_LOGE(TAG, "Boot done — IMU FAIL STATE");
    }

    xTaskCreatePinnedToCore(ibus_task,       "ibus",       4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(traj_task,       "traj",       4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(led_task,        "led",        4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(ultrasonic_task, "ultrasonic", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(logger_task,     "logger",     4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(encoder_task,    "encoder",    4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(control_task,    "control",    4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "All tasks started");
}