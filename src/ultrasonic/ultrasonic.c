#include "ultrasonic.h"
#include "us_guard.h"          // US_K_*
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// CODE_US: đo bằng ngắt cạnh, xem ultrasonic.h. Bản vòng chờ bận cũ (5bde4c5) và bản chẩn đoán của
// nó (feat/us-diag f841c43) nằm trong lịch sử git.

static const char *TAG = "ULTRASONIC";

// Thứ tự theo us_id_t: trước, sau, trái, phải (như 5bde4c5)
static const int echo_pins[ULTRASONIC_COUNT] = {
    ULTRASONIC_ECHO_2,
    ULTRASONIC_ECHO_0,
    ULTRASONIC_ECHO_1,
    ULTRASONIC_ECHO_3,
};

#define ST_IDLE   0   // không nghe (sau khi quyết)
#define ST_ARMED  1   // đã phát, chờ ECHO lên
#define ST_HIGH   2   // ECHO đang cao
#define ST_DONE   3   // ECHO đã xuống
#define ST_BUSY   4   // ECHO đã cao lúc phát (cảm biến bỏ qua lần phát này)

typedef struct {
    volatile uint8_t  st;
    volatile uint16_t edges;
    volatile int64_t  t_rise;
    volatile int64_t  t_fall;
} echo_state_t;

static echo_state_t  s_echo[ULTRASONIC_COUNT];
static portMUX_TYPE  s_mux = portMUX_INITIALIZER_UNLOCKED;
static int64_t       s_t_trig;
static uint8_t       s_pre[ULTRASONIC_COUNT];
static uint8_t       s_decided[ULTRASONIC_COUNT];
static uint8_t       s_prev_kind[ULTRASONIC_COUNT];   // kết quả lần phát trước (cho ECHO còn cao lúc phát)

#define RISE_MAX_US 5000   // như 5bde4c5: ECHO lên muộn hơn 5 ms sau khi phát thì không tin

// Ngắt hai cạnh của ECHO i. Dịch vụ ngắt GPIO cài không cờ IRAM (thư viện BNO085, BNO08x.cpp:598),
// nên hàm này không cần IRAM_ATTR.
static void echo_isr(void *arg) {
    int i = (int)(intptr_t)arg;
    int lvl = gpio_get_level(echo_pins[i]);
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_mux);
    echo_state_t *e = &s_echo[i];
    if (e->edges < 0xFFFF) e->edges++;
    if (e->st == ST_ARMED && lvl) {
        e->t_rise = now;
        e->st = ST_HIGH;
    } else if (e->st == ST_HIGH && !lvl) {
        e->t_fall = now;
        e->st = ST_DONE;
    }
    portEXIT_CRITICAL_ISR(&s_mux);
}

void ultrasonic_init(void) {
    // TRIG pin
    gpio_config_t trig_cfg = {
        .pin_bit_mask = (1ULL << ULTRASONIC_TRIG),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig_cfg);
    gpio_set_level(ULTRASONIC_TRIG, 0);

    // ECHO pins: kéo xuống như 5bde4c5, ngắt hai cạnh
    uint64_t echo_mask = 0;
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        echo_mask |= (1ULL << echo_pins[i]);
        s_echo[i].st = ST_IDLE;
        s_prev_kind[i] = US_K_UNKNOWN;
    }
    gpio_config_t echo_cfg = {
        .pin_bit_mask = echo_mask,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&echo_cfg);

    // Dịch vụ ngắt thường đã do imu_init() cài; ESP_ERR_INVALID_STATE nghĩa là đã có
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(err));
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        err = gpio_isr_handler_add(echo_pins[i], echo_isr, (void *)(intptr_t)i);
        if (err != ESP_OK) ESP_LOGE(TAG, "isr add GPIO%d: %s", echo_pins[i], esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Ultrasonic init done (edge capture, ping %d ms)", US_PING_MS);
}

int64_t ultrasonic_ping(void) {
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        int lvl = gpio_get_level(echo_pins[i]);
        s_pre[i]       = (uint8_t)(lvl ? 1 : 0);
        s_decided[i]   = 0;
        s_echo[i].edges  = 0;
        s_echo[i].t_rise = 0;
        s_echo[i].t_fall = 0;
        s_echo[i].st     = lvl ? ST_BUSY : ST_ARMED;
    }
    portEXIT_CRITICAL(&s_mux);

    s_t_trig = esp_timer_get_time();
    gpio_set_level(ULTRASONIC_TRIG, 1);
    esp_rom_delay_us(20);
    gpio_set_level(ULTRASONIC_TRIG, 0);
    return s_t_trig;
}

uint8_t ultrasonic_collect(us_meas_t out[ULTRASONIC_COUNT], bool final) {
    echo_state_t snap[ULTRASONIC_COUNT];
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        snap[i].st     = s_echo[i].st;
        snap[i].edges  = s_echo[i].edges;
        snap[i].t_rise = s_echo[i].t_rise;
        snap[i].t_fall = s_echo[i].t_fall;
    }
    portEXIT_CRITICAL(&s_mux);
    int64_t now = esp_timer_get_time();

    uint8_t mask = 0;
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        if (s_decided[i]) continue;
        us_meas_t m = { .kind = US_K_UNKNOWN, .cm = -1.0f, .pre = s_pre[i], .stage = final ? 2 : 1,
                        .edges = snap[i].edges, .rise_us = 0, .w_us = 0 };
        bool decide = false;
        if (snap[i].st == ST_BUSY) {
            // Cảm biến bỏ qua lần phát này. Lần trước là FAR (còn cao vì chưa nghe vọng nào): vẫn FAR, mẫu
            // cũ tối đa một chu kỳ. Khác: không biết.
            m.kind  = (s_prev_kind[i] == US_K_FAR) ? US_K_FAR : US_K_UNKNOWN;
            m.stage = 0;
            decide  = true;
        } else if (snap[i].st == ST_DONE) {
            m.rise_us = (uint32_t)(snap[i].t_rise - s_t_trig);
            m.w_us    = (uint32_t)(snap[i].t_fall - snap[i].t_rise);
            float cm  = (float)m.w_us * 0.0343f / 2.0f;
            if (m.rise_us > RISE_MAX_US || cm < ULTRASONIC_MIN_CM) {
                m.kind = US_K_UNKNOWN;
            } else if (m.w_us > US_W_MAX_US) {
                m.kind = US_K_FAR;
            } else {
                m.kind = US_K_DIST;
                m.cm   = cm;
            }
            decide = true;
        } else if (snap[i].st == ST_HIGH) {
            m.rise_us = (uint32_t)(snap[i].t_rise - s_t_trig);
            m.w_us    = (uint32_t)(now - snap[i].t_rise);
            if (m.rise_us > RISE_MAX_US) {
                if (final) { m.kind = US_K_UNKNOWN; decide = true; }
            } else if (m.w_us > US_W_MAX_US || final) {
                m.kind = US_K_FAR;          // chưa có vọng trong tầm
                decide = true;
            }
        } else if (final) {
            m.kind = US_K_UNKNOWN;          // ECHO không lên (ST_ARMED)
            decide = true;
        }
        if (!decide) continue;
        s_decided[i]   = 1;
        s_prev_kind[i] = m.kind;
        out[i] = m;
        mask |= (uint8_t)(1u << i);
    }
    if (final) {
        portENTER_CRITICAL(&s_mux);
        for (int i = 0; i < ULTRASONIC_COUNT; i++)
            if (s_echo[i].st != ST_HIGH) s_echo[i].st = ST_IDLE;   // bỏ cạnh muộn
        portEXIT_CRITICAL(&s_mux);
    }
    return mask;
}

void ultrasonic_read_all(us_data_t *data) {
    us_meas_t m[ULTRASONIC_COUNT];
    ultrasonic_ping();
    vTaskDelay(pdMS_TO_TICKS(US_COLLECT2_TICKS * 10));
    ultrasonic_collect(m, false);
    ultrasonic_collect(m, true);
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        data->valid[i]       = (m[i].kind == US_K_DIST);
        data->distance_cm[i] = data->valid[i] ? m[i].cm : -1.0f;
    }
}
