#include "test_imu.h"
#include "imu/imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>

static const char *TAG = "TEST_IMU";

// ─── Countdown helper ─────────────────────────────────────────────────────────
static void _countdown(const char *msg, int seconds) {
    ESP_LOGI(TAG, "%s", msg);
    for (int i = seconds; i > 0; i--) {
        ESP_LOGI(TAG, "  %d...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ─── Test 1: Init OK ──────────────────────────────────────────────────────────
static void test_1_init(void) {
    _countdown(">>> TEST 1: Init — khong can lam gi, cho tu dong...", 3);
    imu_data_t d = {};
    for (int i = 0; i < 30; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (imu_get(&d) && d.valid) {
            ESP_LOGI(TAG, "[PASS] Test 1 — init OK, data valid sau ~%dms", i * 100);
            return;
        }
    }
    ESP_LOGE(TAG, "[FAIL] Test 1 — khong co data sau 3s");
    ESP_LOGE(TAG, "  Kiem tra: PS0/PS1 da bridge? Day noi dung GPIO? RST co 3.3V?");
}

// ─── Test 2: Gyro static ──────────────────────────────────────────────────────
static void test_2_gyro_static(void) {
    _countdown(">>> TEST 2: Gyro static — DAT XE/BOARD PHANG, KHONG DONG CHAM", 5);
    ESP_LOGI(TAG, "  Do 2 giay...");

    imu_data_t d = {};
    int fail = 0;
    for (int i = 0; i < 20; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!imu_get(&d)) continue;
        if (fabsf(d.gyro_z) > 0.05f) {
            fail++;
            ESP_LOGW(TAG, "  gyro_z=%.4f (expect <0.05)", d.gyro_z);
        }
    }
    if (fail == 0)
        ESP_LOGI(TAG, "[PASS] Test 2 — gyro_z stable khi static");
    else
        ESP_LOGW(TAG, "[WARN] Test 2 — %d/20 sample vuot nguong", fail);
}

// ─── Test 3: Accel gravity ────────────────────────────────────────────────────
static void test_3_accel_gravity(void) {
    _countdown(">>> TEST 3: Accel gravity — DAT XE PHANG, KHONG NGHIENG", 5);
    ESP_LOGI(TAG, "  Doc accel...");
    vTaskDelay(pdMS_TO_TICKS(500));

    imu_data_t d = {};
    imu_get(&d);
    ESP_LOGI(TAG, "  accel_x=%.3f accel_y=%.3f accel_z=%.3f (expect z~9.81)",
             d.accel_x, d.accel_y, d.accel_z);

    float g = sqrtf(d.accel_x*d.accel_x + d.accel_y*d.accel_y + d.accel_z*d.accel_z);
    if (fabsf(g - 9.81f) < 0.5f)
        ESP_LOGI(TAG, "[PASS] Test 3 — |accel|=%.3f m/s2 (trong khoang 9.31-10.31)", g);
    else
        ESP_LOGE(TAG, "[FAIL] Test 3 — |accel|=%.3f m/s2, sai qua", g);
}

// ─── Test 4: Gyro rotation ────────────────────────────────────────────────────
static void test_4_gyro_rotation(void) {
    _countdown(">>> TEST 4: Gyro rotation — CHUAN BI CAM XE/BOARD DE XOAY", 5);
    ESP_LOGI(TAG, "  XOAY MANH TRAI/PHAI TRONG 5 GIAY!");

    float max_gz = 0.0f;
    imu_data_t d = {};
    for (int i = 0; i < 50; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!imu_get(&d)) continue;
        float gz = fabsf(d.gyro_z);
        if (gz > max_gz) max_gz = gz;
        ESP_LOGI(TAG, "  gyro_z=%.3f rad/s (%.1f deg/s)", d.gyro_z, d.gyro_z * 57.3f);
    }

    if (max_gz > 0.5f)
        ESP_LOGI(TAG, "[PASS] Test 4 — max gyro_z=%.3f rad/s khi xoay", max_gz);
    else
        ESP_LOGW(TAG, "[WARN] Test 4 — max gyro_z=%.3f qua nho, co xoay chua?", max_gz);
}

// ─── Test 5: Euler yaw ────────────────────────────────────────────────────────
static void test_5_euler_yaw(void) {
    _countdown(">>> TEST 5: Euler yaw — CHUAN BI XOAY XE/BOARD ~90 DO", 5);

    imu_data_t d = {};
    imu_get(&d);
    float yaw_start = d.yaw;
    ESP_LOGI(TAG, "  Yaw ban dau: %.1f deg", yaw_start);
    ESP_LOGI(TAG, "  XOAY ~90 DO ROI GIU YEN — do sau 5 giay...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    imu_get(&d);
    float yaw_end = d.yaw;
    float delta = fabsf(yaw_end - yaw_start);
    if (delta > 180.0f) delta = 360.0f - delta;

    ESP_LOGI(TAG, "  Yaw sau: %.1f deg — delta=%.1f deg", yaw_end, delta);
    if (delta > 60.0f && delta < 120.0f)
        ESP_LOGI(TAG, "[PASS] Test 5 — euler yaw tracking OK");
    else
        ESP_LOGW(TAG, "[WARN] Test 5 — delta=%.1f (expect ~90)", delta);
}

// ─── Test 6: Noise variance ───────────────────────────────────────────────────
#define T6_N_SAMPLES  600   /* 30s x 20Hz */

static void test_6_noise_variance(void) {
    _countdown(">>> TEST 6: Noise variance — DAT XE PHANG, KHONG DONG CHAM 30 GIAY", 5);
    ESP_LOGI(TAG, "  Thu thap %d samples (30s)...", T6_N_SAMPLES);

    double mean_gz = 0, M2_gz = 0;
    double mean_ax = 0, M2_ax = 0;
    double mean_ay = 0, M2_ay = 0;
    double mean_az = 0, M2_az = 0;
    int n = 0;

    for (int i = 0; i < T6_N_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
        imu_data_t d = {};
        if (!imu_get(&d) || !d.valid) continue;

        n++;
        double delta;
        delta = d.gyro_z  - mean_gz; mean_gz += delta/n; M2_gz += delta*(d.gyro_z  - mean_gz);
        delta = d.accel_x - mean_ax; mean_ax += delta/n; M2_ax += delta*(d.accel_x - mean_ax);
        delta = d.accel_y - mean_ay; mean_ay += delta/n; M2_ay += delta*(d.accel_y - mean_ay);
        delta = d.accel_z - mean_az; mean_az += delta/n; M2_az += delta*(d.accel_z - mean_az);

        if ((i + 1) % 100 == 0)
            ESP_LOGI(TAG, "  [%ds/%ds] gz=%.4f ax=%.4f ay=%.4f az=%.4f",
                     (i+1)/20, T6_N_SAMPLES/20,
                     d.gyro_z, d.accel_x, d.accel_y, d.accel_z);
    }

    if (n < 10) {
        ESP_LOGE(TAG, "[FAIL] Test 6 — qua it sample (%d)", n);
        return;
    }

    double sig_gz = sqrt(M2_gz / (n - 1));
    double sig_ax = sqrt(M2_ax / (n - 1));
    double sig_ay = sqrt(M2_ay / (n - 1));
    double sig_az = sqrt(M2_az / (n - 1));

    /* In kết quả lần đầu */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "=== TEST 6 RESULT — %d samples ===", n);
    ESP_LOGI(TAG, "  gyro_z  : mean=%.5f rad/s  std=%.5f", mean_gz, sig_gz);
    ESP_LOGI(TAG, "  accel_x : mean=%.4f m/s2   std=%.5f", mean_ax, sig_ax);
    ESP_LOGI(TAG, "  accel_y : mean=%.4f m/s2   std=%.5f", mean_ay, sig_ay);
    ESP_LOGI(TAG, "  accel_z : mean=%.4f m/s2   std=%.5f", mean_az, sig_az);
    ESP_LOGI(TAG, "--- Copy vao imu_model.m ---");
    ESP_LOGI(TAG, "  sigma_gyro  = %.5f;  %% rad/s", sig_gz);
    ESP_LOGI(TAG, "  sigma_accel = %.5f;  %% m/s2 (accel_z)", sig_az);
    ESP_LOGI(TAG, "  bias_gyro_z = %.5f;  %% rad/s", mean_gz);
    ESP_LOGI(TAG, "========================================");

    if (sig_gz < 0.01f)
        ESP_LOGI(TAG, "[PASS] Test 6 — gyro noise OK");
    else
        ESP_LOGW(TAG, "[WARN] Test 6 — sig_gz=%.5f > 0.01", sig_gz);
}

// ─── Test 7: Live monitor — nhúc nhích xe xem số thay đổi ────────────────────
static void test_7_live_monitor(void) {
    _countdown(">>> TEST 7: Live monitor — NHIN TERMINAL, NHUC NHICH XE TU TU", 3);
    ESP_LOGI(TAG, "  In moi 100ms — reset de thoat");
    ESP_LOGI(TAG, "  gz=gyro_z | ax/ay/az=accel | yaw/pitch/roll=euler");

    imu_data_t d = {};
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!imu_get(&d)) continue;
        ESP_LOGI(TAG,
            "gz=%6.3f  ax=%6.3f  ay=%6.3f  az=%6.3f  yaw=%7.2f  pitch=%5.2f  roll=%5.2f",
            d.gyro_z,
            d.accel_x, d.accel_y, d.accel_z,
            d.yaw, d.pitch, d.roll);
    }
}

// ─── Entry point ──────────────────────────────────────────────────────────────
extern "C" void test_imu(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "=== BNO085 IMU TEST START ===");
    ESP_LOGI(TAG, "  Test 1-6: co countdown 5 giay de chuan bi");
    ESP_LOGI(TAG, "  Test 7: live monitor — nhuc nhich xe xem so");
    ESP_LOGI(TAG, "========================================");

    imu_init();
    vTaskDelay(pdMS_TO_TICKS(1000));

    test_1_init();
    test_2_gyro_static();
    test_3_accel_gravity();
    test_4_gyro_rotation();
    test_5_euler_yaw();
    test_6_noise_variance();
    test_7_live_monitor();  /* loop mãi — reset để thoát */

    vTaskDelete(NULL);
}