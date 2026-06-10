#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "motor/motor.h"
#include "encoder/encoder.h"
#include "ibus/ibus.h"
#include "kinematics/kinematics.h"
#include "logger/logger.h"
#include "wifi/wifi.h"
#include "imu/imu.h"
#include "led_buzzer/led_buzzer.h"
#include "rasp_uart/rasp_uart.h"
#include "ultrasonic/ultrasonic.h"
#include "manual_control/manual_control.h"
#include "controller/pid.h"
#include "controller/adrc.h"
#include "tests/test_runner.h"

static const char *TAG = "MAIN";

// ─── Controller mode ──────────────────────────────────────────────────────────
// Đổi để chọn controller: CTRL_PID hoặc CTRL_ADRC
#define CTRL_PID   0
#define CTRL_ADRC  1
#define CTRL_MODE  CTRL_PID   // ← đổi ở đây

// Shared data
static volatile rasp_setpoint_t g_setpoint     = {0};
static volatile float           g_enc_rpm[4]   = {0};
static volatile int32_t         g_enc_count[4] = {0};
static volatile imu_data_t      g_imu_data     = {0};
static volatile us_data_t       g_us_data      = {0};

// Controllers — chỉ dùng một, chọn bằng CTRL_MODE
static pid_controller_t  g_pid;
static adrc_controller_t g_adrc;

// ─────────────────────────────────────────
// CORE 0 - rasp_task (priority 3)
// ─────────────────────────────────────────
static void rasp_task(void *pv) {
    rasp_setpoint_t sp;
    while (1) {
        if (rasp_uart_read_setpoint(&sp)) {
            g_setpoint = sp;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

// ─────────────────────────────────────────
// CORE 0 - ultrasonic_task (priority 2)
// ─────────────────────────────────────────
static void ultrasonic_task(void *pv) {
    us_data_t data;
    while (1) {
        ultrasonic_read_all(&data);
        g_us_data = data;
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
    }
}

// ─────────────────────────────────────────
// CORE 0 - logger_task (priority 1)
// ─────────────────────────────────────────
// định nghĩa trong logger.c

// ─────────────────────────────────────────
// CORE 1 - encoder_task (priority 6)
// ─────────────────────────────────────────
static void encoder_task(void *pv) {
    while (1) {
        for (int i = 0; i < ENC_COUNT; i++) {
            g_enc_rpm[i]   = encoder_get_rpm((enc_id_t)i);
            g_enc_count[i] = encoder_get_count((enc_id_t)i);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

// ─────────────────────────────────────────
// CORE 1 - imu_task (priority 6)
// ─────────────────────────────────────────
static void imu_task(void *pv) {
    imu_data_t data;
    while (1) {
        if (imu_get(&data)) {
            g_imu_data = data;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

// ─────────────────────────────────────────
// CORE 1 - control_task (priority 5)
// ─────────────────────────────────────────
static void control_task(void *pv) {
    const float dt = 0.01f;   // 100Hz → 10ms
    int64_t t_prev = esp_timer_get_time();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz

        // dt thực tế — tránh spike lần đầu
        int64_t t_now = esp_timer_get_time();
        float dt_actual = (t_now - t_prev) * 1e-6f;
        t_prev = t_now;
        if (dt_actual <= 0.0f || dt_actual > 0.1f) dt_actual = dt;

        // 1. Đọc setpoint — ưu tiên RPi5, fallback IBus
        agv_velocity_t vel = {0};

        if (g_setpoint.valid) {
            vel.vx = g_setpoint.vx;
            vel.vy = g_setpoint.vy;
            vel.wz = g_setpoint.wz;
        } else {
            
            ibus_data_t rc;
                if (ibus_get(&rc)) {
                    // VRA = channel 5, range [1000, 2000] → scale [0.0, 1.0]
                    float speed_scale = (float)(rc.channel[5] - 1000) / 1000.0f;
                    if (speed_scale < 0.0f) speed_scale = 0.0f;
                    if (speed_scale > 1.0f) speed_scale = 1.0f;

                    vel.vx = ibus_channel_normalized(&rc, 1) / 500.0f * speed_scale;
                    vel.vy = -ibus_channel_normalized(&rc, 0) / 500.0f * speed_scale;
                    vel.wz = -ibus_channel_normalized(&rc, 3) / 500.0f * speed_scale;
                }
        }

        // 2. Forward kinematics từ encoder → actual velocity
        wheel_velocity_t enc_wheels = {
            .fl = g_enc_rpm[ENC_FL] * 2.0f * 3.14159f / 60.0f,
            .fr = g_enc_rpm[ENC_FR] * 2.0f * 3.14159f / 60.0f,
            .rl = g_enc_rpm[ENC_RL] * 2.0f * 3.14159f / 60.0f,
            .rr = g_enc_rpm[ENC_RR] * 2.0f * 3.14159f / 60.0f,
        };
        agv_velocity_t actual_vel = kinematics_forward(enc_wheels);

        // 3. Controller — closed-loop velocity control
        //    Input:  vel (setpoint), actual_vel (feedback từ encoder)
        //    Output: corrected velocity → inverse kinematics → PWM
        agv_velocity_t ctrl_out = {0};

#if CTRL_MODE == CTRL_PID
        ctrl_out.vx = vel.vx + pid_update_vx(&g_pid, vel.vx, actual_vel.vx, dt_actual);
        ctrl_out.vy = vel.vy + pid_update_vy(&g_pid, vel.vy, actual_vel.vy, dt_actual);
        ctrl_out.wz = vel.wz + pid_update_wz(&g_pid, vel.wz, actual_vel.wz, dt_actual);
#elif CTRL_MODE == CTRL_ADRC
        ctrl_out.vx = vel.vx + adrc_update_vx(&g_adrc, vel.vx, actual_vel.vx, dt_actual);
        ctrl_out.vy = vel.vy + adrc_update_vy(&g_adrc, vel.vy, actual_vel.vy, dt_actual);
        ctrl_out.wz = vel.wz + adrc_update_wz(&g_adrc, vel.wz, actual_vel.wz, dt_actual);
#endif

        // 4. Inverse kinematics → wheel speed → PWM
        wheel_velocity_t wheels = kinematics_inverse(ctrl_out);

        int pwm_fl = kinematics_radps_to_pwm(wheels.fl);
        int pwm_fr = kinematics_radps_to_pwm(wheels.fr);
        int pwm_rl = kinematics_radps_to_pwm(wheels.rl);
        int pwm_rr = kinematics_radps_to_pwm(wheels.rr);

        motor_set(MOTOR_FL, pwm_fl);
        motor_set(MOTOR_FR, pwm_fr);
        motor_set(MOTOR_RL, pwm_rl);
        motor_set(MOTOR_RR, pwm_rr);

        // 5. Gửi status về RPi5
        rasp_uart_send_status(actual_vel.vx, actual_vel.vy, actual_vel.wz);

        // 6. Push log
        log_data_t log = {
            .timestamp_us = t_now,
            .fl_count = g_enc_count[ENC_FL],
            .fr_count = g_enc_count[ENC_FR],
            .rl_count = g_enc_count[ENC_RL],
            .rr_count = g_enc_count[ENC_RR],
            .fl_rpm   = g_enc_rpm[ENC_FL],
            .fr_rpm   = g_enc_rpm[ENC_FR],
            .rl_rpm   = g_enc_rpm[ENC_RL],
            .rr_rpm   = g_enc_rpm[ENC_RR],
            .vx       = actual_vel.vx,
            .vy       = actual_vel.vy,
            .wz       = actual_vel.wz,
            .sp_vx    = vel.vx,
            .sp_vy    = vel.vy,
            .sp_wz    = vel.wz,
            .pwm_fl   = pwm_fl,
            .pwm_fr   = pwm_fr,
            .pwm_rl   = pwm_rl,
            .pwm_rr   = pwm_rr,
            .ax       = g_imu_data.accel_x,
            .ay       = g_imu_data.accel_y,
            .az       = g_imu_data.accel_z,
            .gx       = g_imu_data.gyro_x,
            .gy       = g_imu_data.gyro_y,
            .gz       = g_imu_data.gyro_z,
            .yaw      = g_imu_data.yaw,
            .pitch    = g_imu_data.pitch,
            .roll     = g_imu_data.roll,
        };
        logger_push(&log);
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

    // Status LED: khởi động
    led_buzzer_init();
    led_set_color(COLOR_YELLOW);

    // Init peripherals
    wifi_init();
    motor_init();
    encoder_init();
    ibus_init();
    imu_init();
    ultrasonic_init();
    rasp_uart_init();
    logger_init();

    // Init controller
#if CTRL_MODE == CTRL_PID
    pid_init(&g_pid);
    ESP_LOGI(TAG, "Controller: PID");
#elif CTRL_MODE == CTRL_ADRC
    adrc_init(&g_adrc);
    ESP_LOGI(TAG, "Controller: ADRC");
#endif

    // Enable motor
    motor_enable(true);

    // Status LED: ready
    led_set_color(COLOR_GREEN);
    buzzer_beep(2700, 200);

    // Tạo tasks
    xTaskCreatePinnedToCore(ibus_task,        "ibus",       4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(rasp_task,        "rasp",       4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(ultrasonic_task,  "ultrasonic", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(logger_task,      "logger",     4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(encoder_task,     "encoder",    4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(imu_task,         "imu",        4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(control_task,     "control",    4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "All tasks started");
}