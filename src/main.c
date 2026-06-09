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
#include "imu/shtp.h"
#include "led_buzzer/led_buzzer.h"
#include "rasp_uart/rasp_uart.h"
#include "ultrasonic/ultrasonic.h"
#include "tests/test_runner.h"

static const char *TAG = "MAIN";

// Shared data
static volatile ibus_data_t     g_ibus_data    = {0};
static volatile rasp_setpoint_t g_setpoint     = {0};
static volatile float           g_enc_rpm[4]   = {0};
static volatile int32_t         g_enc_count[4] = {0};
static volatile imu_data_t      g_imu_data     = {0};
static volatile us_data_t       g_us_data      = {0};

// ─────────────────────────────────────────
// CORE 0 - ibus_task (priority 4)
// ─────────────────────────────────────────
static void ibus_task(void *pv) {
    ibus_data_t data;
    while (1) {
        if (ibus_read(&data)) {
            g_ibus_data = data;
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
    }
}

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
        if (imu_read(&data)) {
            g_imu_data = data;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

// ─────────────────────────────────────────
// CORE 1 - control_task (priority 5)
// ─────────────────────────────────────────
static void control_task(void *pv) {
    while (1) {
        // 1. Đọc setpoint — ưu tiên RPi5, fallback IBus
        agv_velocity_t vel = {0};

        if (g_setpoint.valid) {
            vel.vx = g_setpoint.vx;
            vel.vy = g_setpoint.vy;
            vel.wz = g_setpoint.wz;
        } else if (g_ibus_data.valid) {
            vel.vx = ibus_channel_normalized((ibus_data_t*)&g_ibus_data, 1) / 1000.0f;
            vel.vy = ibus_channel_normalized((ibus_data_t*)&g_ibus_data, 0) / 1000.0f;
            vel.wz = ibus_channel_normalized((ibus_data_t*)&g_ibus_data, 3) / 1000.0f;
        }

        // 2. Inverse kinematics → wheel speed
        wheel_velocity_t wheels = kinematics_inverse(vel);

        // 3. Set motor
        motor_set(MOTOR_FL, kinematics_radps_to_pwm(wheels.fl));
        motor_set(MOTOR_FR, kinematics_radps_to_pwm(wheels.fr));
        motor_set(MOTOR_RL, kinematics_radps_to_pwm(wheels.rl));
        motor_set(MOTOR_RR, kinematics_radps_to_pwm(wheels.rr));

        // 4. Forward kinematics từ encoder
        wheel_velocity_t enc_wheels = {
            .fl = g_enc_rpm[ENC_FL] * 2.0f * 3.14159f / 60.0f,
            .fr = g_enc_rpm[ENC_FR] * 2.0f * 3.14159f / 60.0f,
            .rl = g_enc_rpm[ENC_RL] * 2.0f * 3.14159f / 60.0f,
            .rr = g_enc_rpm[ENC_RR] * 2.0f * 3.14159f / 60.0f,
        };
        agv_velocity_t actual_vel = kinematics_forward(enc_wheels);

        // 5. Gửi status về RPi5
        rasp_uart_send_status(actual_vel.vx, actual_vel.vy, actual_vel.wz);

        // 6. Push log
        log_data_t log = {
            .timestamp_us = esp_timer_get_time(),
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
            .ax       = g_imu_data.ax,
            .ay       = g_imu_data.ay,
            .az       = g_imu_data.az,
            .gx       = g_imu_data.gx,
            .gy       = g_imu_data.gy,
            .gz       = g_imu_data.gz,
            .yaw      = g_imu_data.yaw,
            .pitch    = g_imu_data.pitch,
            .roll     = g_imu_data.roll,
        };
        logger_push(&log);

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
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