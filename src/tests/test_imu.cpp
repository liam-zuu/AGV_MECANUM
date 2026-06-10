#include "test_imu.h"
#include "imu/imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>

static const char *TAG = "TEST_IMU";

// ─── Test 1: Init OK ──────────────────────────────────────────────────────────
// Pass: imu_init không crash, data valid trong 3s
static void test_1_init(void) {
    ESP_LOGI(TAG, "=== TEST 1: Init ===");
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

// ─── Test 2: Gyro wz khi board đứng yên ──────────────────────────────────────
// Pass: |gyro_z| < 0.05 rad/s (board đứng yên)
static void test_2_gyro_static(void) {
    ESP_LOGI(TAG, "=== TEST 2: Gyro static — dat board len ban, khong dong cham ===");
    vTaskDelay(pdMS_TO_TICKS(1000));

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
        ESP_LOGW(TAG, "[WARN] Test 2 — %d/20 sample vuot nguong (co the chua calibrate)", fail);
}

// ─── Test 3: Accel gravity khi board phẳng ───────────────────────────────────
// Pass: accel_z ≈ 9.81 m/s² (trục Z hướng lên)
//       accel_x, accel_y ≈ 0
static void test_3_accel_gravity(void) {
    ESP_LOGI(TAG, "=== TEST 3: Accel gravity — dat board phang, mat tren huong len ===");
    vTaskDelay(pdMS_TO_TICKS(1000));

    imu_data_t d = {};
    imu_get(&d);
    ESP_LOGI(TAG, "  accel_x=%.3f accel_y=%.3f accel_z=%.3f (expect z~9.81)", 
             d.accel_x, d.accel_y, d.accel_z);

    float g = sqrtf(d.accel_x*d.accel_x + d.accel_y*d.accel_y + d.accel_z*d.accel_z);
    if (fabsf(g - 9.81f) < 0.5f)
        ESP_LOGI(TAG, "[PASS] Test 3 — |accel|=%.3f m/s² (trong khoang 9.31-10.31)", g);
    else
        ESP_LOGE(TAG, "[FAIL] Test 3 — |accel|=%.3f m/s², sai qua (BNO085 co dang hoat dong?)", g);
}

// ─── Test 4: Gyro wz khi xoay board ─────────────────────────────────────────
// Liam xoay board bằng tay → verify gyro_z tăng rõ ràng
static void test_4_gyro_rotation(void) {
    ESP_LOGI(TAG, "=== TEST 4: Gyro rotation — XOAY BOARD TRAI/PHAI TRONG 5 GIAY ===");

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

// ─── Test 5: Euler yaw thay đổi khi xoay ─────────────────────────────────────
static void test_5_euler_yaw(void) {
    ESP_LOGI(TAG, "=== TEST 5: Euler yaw — ghi nho yaw ban dau, xoay 90 do, check ===");

    imu_data_t d = {};
    imu_get(&d);
    float yaw_start = d.yaw;
    ESP_LOGI(TAG, "  Yaw ban dau: %.1f deg — XOAY ~90 DO ROI GIU TRONG 3 GIAY", yaw_start);
    vTaskDelay(pdMS_TO_TICKS(5000));

    imu_get(&d);
    float yaw_end = d.yaw;
    float delta = fabsf(yaw_end - yaw_start);
    if (delta > 180.0f) delta = 360.0f - delta;  // wrap around

    ESP_LOGI(TAG, "  Yaw sau: %.1f deg — delta=%.1f deg", yaw_end, delta);
    if (delta > 60.0f && delta < 120.0f)
        ESP_LOGI(TAG, "[PASS] Test 5 — euler yaw tracking OK");
    else
        ESP_LOGW(TAG, "[WARN] Test 5 — delta=%.1f (expect ~90). Co xoay du 90 do khong?", delta);
}

// ─── Test 6: Noise variance — input cho imu_model.m ──────────────────────────
// Board đứng yên hoàn toàn, 30s × 20Hz = 600 samples
// Output: mean + variance của gyro_z, accel_x, accel_y, accel_z
// Copy số này thẳng vào imu_model.m (sigma_gyro, sigma_accel)
#define T6_N_SAMPLES  600   /* 30s × 20Hz */

static void test_6_noise_variance(void) {
    ESP_LOGI(TAG, "=== TEST 6: Noise variance — DAT BOARD PHANG, KHONG CHAM 30 GIAY ===");
    ESP_LOGI(TAG, "  Thu thap %d samples (30s)...", T6_N_SAMPLES);
    vTaskDelay(pdMS_TO_TICKS(2000));  /* cho người dùng đặt board xuống */

    /* Welford online algorithm — tránh tích lũy float lớn */
    double mean_gz = 0, M2_gz = 0;
    double mean_ax = 0, M2_ax = 0;
    double mean_ay = 0, M2_ay = 0;
    double mean_az = 0, M2_az = 0;
    int n = 0;

    for (int i = 0; i < T6_N_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));  /* 20Hz */
        imu_data_t d = {};
        if (!imu_get(&d) || !d.valid) continue;

        n++;
        /* Welford update */
        double delta;
        delta = d.gyro_z  - mean_gz; mean_gz += delta/n; M2_gz += delta*(d.gyro_z  - mean_gz);
        delta = d.accel_x - mean_ax; mean_ax += delta/n; M2_ax += delta*(d.accel_x - mean_ax);
        delta = d.accel_y - mean_ay; mean_ay += delta/n; M2_ay += delta*(d.accel_y - mean_ay);
        delta = d.accel_z - mean_az; mean_az += delta/n; M2_az += delta*(d.accel_z - mean_az);

        /* Progress mỗi 5 giây */
        if ((i + 1) % 100 == 0)
            ESP_LOGI(TAG, "  [%ds] gz=%.4f ax=%.4f ay=%.4f az=%.4f",
                     (i+1)/20, d.gyro_z, d.accel_x, d.accel_y, d.accel_z);
    }

    if (n < 10) {
        ESP_LOGE(TAG, "[FAIL] Test 6 — qua it sample (%d), IMU co dang chay?", n);
        return;
    }

    double var_gz = M2_gz / (n - 1);
    double var_ax = M2_ax / (n - 1);
    double var_ay = M2_ay / (n - 1);
    double var_az = M2_az / (n - 1);

    double sig_gz = sqrt(var_gz);
    double sig_ax = sqrt(var_ax);
    double sig_ay = sqrt(var_ay);
    double sig_az = sqrt(var_az);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 6 RESULT — %d samples ===", n);
    ESP_LOGI(TAG, "  gyro_z  : mean=%.5f rad/s   std=%.5f   var=%.7f",
             mean_gz, sig_gz, var_gz);
    ESP_LOGI(TAG, "  accel_x : mean=%.4f m/s2    std=%.5f   var=%.7f",
             mean_ax, sig_ax, var_ax);
    ESP_LOGI(TAG, "  accel_y : mean=%.4f m/s2    std=%.5f   var=%.7f",
             mean_ay, sig_ay, var_ay);
    ESP_LOGI(TAG, "  accel_z : mean=%.4f m/s2    std=%.5f   var=%.7f",
             mean_az, sig_az, var_az);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- Copy vao imu_model.m ---");
    ESP_LOGI(TAG, "  sigma_gyro  = %.5f;  %% rad/s (gyro_z std)", sig_gz);
    ESP_LOGI(TAG, "  sigma_accel = %.5f;  %% m/s2  (accel xy, lay trung binh)");
    ESP_LOGI(TAG, "    accel_x std=%.5f  accel_y std=%.5f", sig_ax, sig_ay);
    ESP_LOGI(TAG, "  bias_gyro_z = %.5f;  %% rad/s (mean offset)", mean_gz);

    /* Pass/warn dựa trên datasheet BNO085 (typical noise floor) */
    if (sig_gz < 0.01f)
        ESP_LOGI(TAG, "[PASS] Test 6 — gyro noise OK (sig_gz < 0.01 rad/s)");
    else
        ESP_LOGW(TAG, "[WARN] Test 6 — sig_gz=%.5f > 0.01, co the chua warm up / co rung dong", sig_gz);
}

// ─── Entry point ──────────────────────────────────────────────────────────────
extern "C" void test_imu(void) {
    ESP_LOGI(TAG, "=== BNO085 IMU TEST START ===");
    ESP_LOGI(TAG, "Dam bao: PS0/PS1 da bridge, day noi dung, RST co 3.3V");
    vTaskDelay(pdMS_TO_TICKS(500));

    imu_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    test_1_init();
    test_2_gyro_static();
    test_3_accel_gravity();
    test_4_gyro_rotation();
    test_5_euler_yaw();
    test_6_noise_variance();

    ESP_LOGI(TAG, "=== ALL IMU TESTS DONE ===");
    vTaskDelete(NULL);
}