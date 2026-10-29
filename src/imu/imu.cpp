#include "imu.h"
#include "BNO08x.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>

static const char *TAG = "IMU";

// ─── GPIO config (khớp với GPIO map đã chốt) ─────────────────────────────────
#define IMU_MOSI    GPIO_NUM_11
#define IMU_MISO    GPIO_NUM_13
#define IMU_CLK     GPIO_NUM_12
#define IMU_CS      GPIO_NUM_10
#define IMU_INT     GPIO_NUM_14
#define IMU_RST     GPIO_NUM_1    // reset BNO085 (buzzer là GPIO17, không dùng chung)

// ─── Globals ──────────────────────────────────────────────────────────────────
static BNO08x          *g_imu       = nullptr;
static SemaphoreHandle_t g_mutex    = nullptr;
static imu_data_t        g_data     = {};
static volatile imu_report_hook_t g_hook = nullptr;
static volatile imu_event_hook_t  g_ev_hook = nullptr;   // env đo S06 (spec S06 v3.4)
// Số thông báo reset hoàn tất đã nhận: chỉ task dịch vụ SH2 HAL ghi (imu_note_chip_reset) và đọc
// (sensor_event_cb đóng dấu report lúc nhận), nên không cần khóa. Khác g_data.reset_n (có thể trễ
// một nhịp khi mutex bận, fold_pending_reset).
static volatile uint32_t s_hal_reset_n = 0;
// Report bị bỏ theo tầng (Y13); mỗi bộ đếm chỉ một task ghi
static volatile uint32_t s_drop_rx = 0, s_drop_cbq = 0, s_drop_mutex = 0;

// Khởi tạo có hạn: task thư viện báo 0 = đang chạy, 1 = xong, −1 = initialize() lỗi
static volatile int      g_lib_state   = 0;
static volatile int      g_init_status = -1;
static volatile uint32_t g_init_ms     = 0;
// Reset chip báo lúc không lấy được mutex: cộng dồn ở lần lấy được kế tiếp.
// Hai biến pending được bảo vệ bằng spinlock riêng (Codex ctrl-imu #3: ++ và = 0 đua nhau).
static portMUX_TYPE      g_pending_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t          g_reset_pending = 0;
static int64_t           g_reset_pending_t_us = 0;
// Tiêu chí khởi tạo (§7, Codex #13): report dùng được kể từ lần reset chip gần nhất (giữ g_mutex)
static uint32_t          g_init_gyro_ok = 0, g_init_rv_ok = 0;

extern "C" void imu_set_report_hook(imu_report_hook_t hook) { g_hook = hook; }
extern "C" void imu_set_event_hook(imu_event_hook_t hook)   { g_ev_hook = hook; }
extern "C" uint32_t imu_hal_reset_count(void)               { return s_hal_reset_n; }

extern "C" void imu_note_lib_drop(int layer) {
    if (layer == 1) s_drop_rx = s_drop_rx + 1;          // task dịch vụ SH2 HAL
    else if (layer == 2) s_drop_cbq = s_drop_cbq + 1;    // data_proc_task
}

extern "C" void imu_get_drops(imu_drops_t *out) {
    if (!out) return;
    out->rx_queue = s_drop_rx;
    out->cb_queue = s_drop_cbq;
    out->cb_mutex = s_drop_mutex;
}

static void fold_pending_reset(void) {   // gọi khi đang giữ g_mutex
    taskENTER_CRITICAL(&g_pending_mux);
    uint32_t n    = g_reset_pending;
    int64_t  t_us = g_reset_pending_t_us;
    g_reset_pending = 0;
    taskEXIT_CRITICAL(&g_pending_mux);
    if (n) {
        g_data.reset_n   += n;
        g_data.reset_t_us = t_us;
        g_init_gyro_ok = g_init_rv_ok = 0;
    }
}

// Thư viện gọi trong task dịch vụ SH2 HAL khi chip báo SH2_RESET (kể cả lần reset lúc khởi động)
extern "C" void imu_note_chip_reset(void) {
    int64_t t_us = esp_timer_get_time();
    uint32_t hal_n = s_hal_reset_n + 1;
    s_hal_reset_n = hal_n;     // trước mọi việc khác: report nhận sau dòng này mang epoch mới
    if (g_mutex && xSemaphoreTake(g_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        fold_pending_reset();
        g_data.reset_n++;
        g_data.reset_t_us = t_us;
        g_init_gyro_ok = g_init_rv_ok = 0;
        xSemaphoreGive(g_mutex);
    } else {
        taskENTER_CRITICAL(&g_pending_mux);
        g_reset_pending_t_us = t_us;
        g_reset_pending++;
        taskEXIT_CRITICAL(&g_pending_mux);
    }
    imu_event_hook_t ev = g_ev_hook;      // spec S06 v3.4 V4-1: thời điểm nhận từng thông báo
    if (ev) {
        imu_event_t e = {};
        e.kind = IMU_EVT_RESET;
        e.t_us = t_us;
        e.n    = hal_n;
        ev(&e);
    }
}

// ─── Khởi tạo thư viện (task riêng: initialize() có thể treo, G0 v1 R1, R2, R10) ─
static void imu_lib_init_task(void *pv) {
    // Config GPIO theo hardware đã chốt
    bno08x_config_t cfg;
    cfg.io_mosi    = IMU_MOSI;
    cfg.io_miso    = IMU_MISO;
    cfg.io_sclk    = IMU_CLK;
    cfg.io_cs      = IMU_CS;
    cfg.io_int     = IMU_INT;
    cfg.io_rst     = IMU_RST;
    cfg.sclk_speed = 3000000;   // 3MHz — safe starting point cho BNO085

    g_imu = new BNO08x(cfg);

    if (!g_imu->initialize()) {
        ESP_LOGE(TAG, "BNO085 init failed! Kiem tra: PS0/PS1 da bridge? Day noi dung chua?");
        g_lib_state = -1;
        vTaskDelete(NULL);
        return;
    }

    // Enable các report cần dùng, cùng chu kỳ IMU_REPORT_PERIOD_US
    g_imu->rpt.accelerometer.enable(IMU_REPORT_PERIOD_US);   // accelerometer (m/s²)
    g_imu->rpt.cal_gyro.enable(IMU_REPORT_PERIOD_US);        // gyro calibrated (rad/s) — heading control
    g_imu->rpt.rv_game.enable(IMU_REPORT_PERIOD_US);         // game rotation vector → euler angles

    // Callback chạy trong task callback của thư viện, mỗi lần đúng một report (rpt_id).
    // Chỉ cập nhật field của report đó và ghi thời điểm nhận phía host.
    // Dữ liệu và danh tính report (số thứ tự SH-2, epoch) đọc cùng một lần khóa của thư viện
    // (get(meta), spec S06 v3.4 V3-8): thư viện chỉ giữ report mới nhất mỗi loại, nên nếu hai report
    // tới trước khi callback chạy thì callback đọc report sau, và seq/lib_n cho thấy điều đó.
    g_imu->register_cb([](uint8_t rpt_id) {
        int64_t t_us = esp_timer_get_time();
        float v[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        uint32_t n = 0;
        bno08x_rpt_meta_t meta = {};

        if (rpt_id == IMU_RPT_ACCEL) {
            auto a = g_imu->rpt.accelerometer.get(meta);
            v[0] = a.x; v[1] = a.y; v[2] = a.z;
        } else if (rpt_id == IMU_RPT_GYRO) {
            auto g = g_imu->rpt.cal_gyro.get(meta);
            v[0] = g.x; v[1] = g.y; v[2] = g.z;
        } else if (rpt_id == IMU_RPT_GAME_RV) {
            auto e = g_imu->rpt.rv_game.get_euler(meta);   // độ (mặc định in_degrees = true)
            v[0] = e.x; v[1] = e.y; v[2] = e.z;
        } else {
            return;   // report khác (hoặc id 0 lúc task khởi động): bỏ qua
        }

        bool applied = false;
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            fold_pending_reset();
            if (rpt_id == IMU_RPT_ACCEL) {
                g_data.accel_x = v[0];
                g_data.accel_y = v[1];
                g_data.accel_z = v[2];
                g_data.accel_t_us = t_us;
                n = ++g_data.accel_n;
            } else if (rpt_id == IMU_RPT_GYRO) {
                g_data.gyro_x = v[0];
                g_data.gyro_y = v[1];
                g_data.gyro_z = v[2];   // wz — dùng cho heading control
                g_data.gyro_t_us = t_us;
                n = ++g_data.gyro_n;
                if (std::isfinite(v[2]) && std::fabs(v[2]) < IMU_GYRO_SAT_RADPS) g_init_gyro_ok++;
            } else {
                g_data.roll  = v[0];
                g_data.pitch = v[1];
                g_data.yaw   = v[2];
                g_data.rv_t_us = t_us;
                n = ++g_data.rv_n;
                if (std::isfinite(v[2])) g_init_rv_ok++;
            }
            g_data.valid = true;
            xSemaphoreGive(g_mutex);
            applied = true;
        } else {
            s_drop_mutex = s_drop_mutex + 1;   // trước 29/10 bỏ im lặng (spec S06 v3.4 Y13)
        }

        imu_report_hook_t hook = g_hook;
        if (hook && applied) hook(rpt_id, t_us, n, v);

        imu_event_hook_t ev = g_ev_hook;
        if (ev) {
            imu_event_t e = {};
            e.kind    = rpt_id;
            e.applied = applied;
            e.seq     = meta.sequence;
            e.t_us    = t_us;
            e.n       = n;
            e.epoch   = meta.epoch;
            e.lib_n   = meta.n_updates;
            e.v[0] = v[0]; e.v[1] = v[1]; e.v[2] = v[2];
            ev(&e);
        }
    });

    g_lib_state = 1;
    vTaskDelete(NULL);
}

// ─── Init có hạn (spec controller–IMU v2.2 §7) ────────────────────────────────
extern "C" int imu_init(void) {
    if (!g_mutex) g_mutex = xSemaphoreCreateMutex();
    configASSERT(g_mutex);

    int64_t t0 = esp_timer_get_time();
    // Cùng ưu tiên với task gọi (Codex ctrl-imu #4): thư viện có quay vòng không nhường CPU
    // thì time slicing vẫn cho vòng kiểm hạn T_init bên dưới chạy.
    xTaskCreate(imu_lib_init_task, "imu_init", 6144, NULL, uxTaskPriorityGet(NULL), NULL);

    int rc;
    uint32_t gyro_n = 0, rv_n = 0;
    while (1) {
        uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            fold_pending_reset();
            gyro_n = g_init_gyro_ok;   // report dùng được sau reset gần nhất (§7)
            rv_n   = g_init_rv_ok;
            xSemaphoreGive(g_mutex);
        }
        if (g_lib_state < 0) { rc = IMU_INIT_ERR_LIB; break; }
        if (g_lib_state > 0 && gyro_n >= IMU_INIT_MIN_REPORTS && rv_n >= IMU_INIT_MIN_REPORTS) {
            rc = IMU_INIT_OK;
            break;
        }
        if (ms >= IMU_INIT_TIMEOUT_MS) {
            rc = (g_lib_state == 0) ? IMU_INIT_ERR_HANG : IMU_INIT_ERR_REPORTS;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    g_init_ms     = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    g_init_status = rc;

    if (rc == IMU_INIT_OK)
        ESP_LOGI(TAG, "BNO085 init OK in %lu ms (gyro %lu, rv %lu usable reports since last reset) — accel/gyro/euler @ %lu us",
                 (unsigned long)g_init_ms, (unsigned long)gyro_n, (unsigned long)rv_n,
                 (unsigned long)IMU_REPORT_PERIOD_US);
    else
        ESP_LOGE(TAG, "IMU_INIT_FAIL %d (%s) after %lu ms: gyro %lu, rv %lu usable reports since last reset",
                 rc, imu_init_err_str(rc), (unsigned long)g_init_ms,
                 (unsigned long)gyro_n, (unsigned long)rv_n);
    return rc;
}

extern "C" int      imu_init_status(void) { return g_init_status; }
extern "C" uint32_t imu_init_ms(void)     { return g_init_ms; }

extern "C" const char *imu_init_err_str(int code) {
    switch (code) {
    case IMU_INIT_OK:          return "ok";
    case IMU_INIT_ERR_LIB:     return "library init failed";
    case IMU_INIT_ERR_HANG:    return "library init did not finish";
    case IMU_INIT_ERR_REPORTS: return "too few gyro/game RV reports";
    default:                   return "?";
    }
}

// ─── Thread-safe getter ───────────────────────────────────────────────────────
extern "C" bool imu_get(imu_data_t *out) {
    if (!out || !g_mutex) return false;
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;
    fold_pending_reset();
    *out = g_data;
    xSemaphoreGive(g_mutex);
    return out->valid;
}
