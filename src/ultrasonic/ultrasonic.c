#include "ultrasonic.h"
#include "us_guard.h"          // US_K_*
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// CODE_US: đo bằng ngắt cạnh, xem ultrasonic.h. Bản vòng chờ bận cũ (5bde4c5) và bản chẩn đoán của
// nó (feat/us-diag f841c43) nằm trong lịch sử git. Sửa theo Codex Sol vòng 1 (Liam 15/11 "Sửa cả 14 +
// vòng 2"): US-1 (FAR dùng lại một lần), US-2 (giờ đọc cùng bản chụp), US-5 (xung ngắn nghe tiếp, cạnh
// thừa là UNKNOWN), US-7 (đặt trạng thái + phát trong một vùng găng), US-8 (ngân sách cạnh, tắt ngắt
// ngoài cửa sổ), US-16 (giờ quyết từng mẫu).

static const char *TAG = "ULTRASONIC";

// Thứ tự theo us_id_t: trước, sau, trái, phải (như 5bde4c5)
static const int echo_pins[ULTRASONIC_COUNT] = {
    ULTRASONIC_ECHO_2,
    ULTRASONIC_ECHO_0,
    ULTRASONIC_ECHO_1,
    ULTRASONIC_ECHO_3,
};

#define ST_IDLE   0   // không nghe (ngoài cửa sổ đo)
#define ST_ARMED  1   // đã phát, chờ ECHO lên
#define ST_HIGH   2   // ECHO đang cao
#define ST_DONE   3   // ECHO đã xuống (xung đủ dài)
#define ST_BUSY   4   // ECHO đã cao lúc phát (cảm biến bỏ qua lần phát này)
#define ST_OVER   5   // quá US_EDGE_MAX cạnh trong một lần đo: ngắt chân này tắt tới lần phát sau

#define RISE_MAX_US 5000u  // như 5bde4c5: ECHO lên muộn hơn 5 ms sau khi phát thì không tin
#define W_MIN_US    117u   // 2 cm (ULTRASONIC_MIN_CM): xung ngắn hơn là nhiễu, nghe tiếp (US-5)

typedef struct {
    volatile uint8_t  st;
    volatile uint16_t edges;    // số lần ngắt trong lần đo
    volatile uint8_t  glitch;   // số xung < W_MIN_US đã bỏ (quay về ARMED)
    volatile uint8_t  extra;    // số cạnh sau khi đã xong xung (ST_DONE)
    volatile int64_t  t_rise;
    volatile int64_t  t_fall;
} echo_state_t;

static echo_state_t  s_echo[ULTRASONIC_COUNT];
static portMUX_TYPE  s_mux = portMUX_INITIALIZER_UNLOCKED;
static int64_t       s_t_trig;
static uint8_t       s_pre[ULTRASONIC_COUNT];
static uint8_t       s_decided[ULTRASONIC_COUNT];
static uint8_t       s_prev_kind[ULTRASONIC_COUNT];   // kết quả đo MỚI gần nhất (không tính FAR dùng lại)
static uint8_t       s_reused[ULTRASONIC_COUNT];      // lần trước đã là FAR dùng lại (US-1)

// Ngắt hai cạnh của ECHO i. Dịch vụ ngắt GPIO cài không cờ IRAM (thư viện BNO085, BNO08x.cpp:598),
// nên hàm này không cần IRAM_ATTR.
static void echo_isr(void *arg) {
    int i = (int)(intptr_t)arg;
    int lvl = gpio_get_level(echo_pins[i]);
    int64_t now = esp_timer_get_time();
    bool over = false;
    portENTER_CRITICAL_ISR(&s_mux);
    echo_state_t *e = &s_echo[i];
    if (e->st != ST_IDLE && e->st != ST_OVER) {
        if (e->edges < 0xFFFF) e->edges++;
        if (e->edges > US_EDGE_MAX) {
            e->st = ST_OVER;                                  // US-8: dây ECHO nhiễu, ngừng phục vụ
            over = true;
        } else if (e->st == ST_ARMED && lvl) {
            e->t_rise = now;
            e->st = ST_HIGH;
        } else if (e->st == ST_HIGH && !lvl) {
            if ((uint64_t)(now - e->t_rise) < W_MIN_US) {     // US-5: xung nhiễu ngắn, nghe tiếp
                e->st = ST_ARMED;
                if (e->glitch < 0xFF) e->glitch++;
            } else {
                e->t_fall = now;
                e->st = ST_DONE;
            }
        } else if (e->st == ST_DONE) {
            if (e->extra < 0xFF) e->extra++;                  // US-5: cạnh thừa sau xung
        }
    }
    portEXIT_CRITICAL_ISR(&s_mux);
    if (over) gpio_intr_disable(echo_pins[i]);
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

    // ECHO pins: kéo xuống như 5bde4c5, ngắt hai cạnh (chỉ bật trong cửa sổ đo, US-8)
    uint64_t echo_mask = 0;
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        echo_mask |= (1ULL << echo_pins[i]);
        s_echo[i].st = ST_IDLE;
        s_prev_kind[i] = US_K_UNKNOWN;
        s_reused[i] = 0;
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
        gpio_intr_disable(echo_pins[i]);
    }

    ESP_LOGI(TAG, "Ultrasonic init done (edge capture, ping %d ms)", US_PING_MS);
}

int64_t ultrasonic_ping(void) {
    for (int i = 0; i < ULTRASONIC_COUNT; i++) gpio_intr_enable(echo_pins[i]);
    // US-7: đặt trạng thái, ghi mốc và phát TRIG trong cùng một vùng găng (khoảng 20 µs): không cạnh
    // nào được xử lý giữa lúc ARMED và lúc phát, không task nào chen làm lệch mốc.
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        int lvl = gpio_get_level(echo_pins[i]);
        s_pre[i]         = (uint8_t)(lvl ? 1 : 0);
        s_decided[i]     = 0;
        s_echo[i].edges  = 0;
        s_echo[i].glitch = 0;
        s_echo[i].extra  = 0;
        s_echo[i].t_rise = 0;
        s_echo[i].t_fall = 0;
        s_echo[i].st     = lvl ? ST_BUSY : ST_ARMED;
    }
    s_t_trig = esp_timer_get_time();
    gpio_set_level(ULTRASONIC_TRIG, 1);
    esp_rom_delay_us(20);
    gpio_set_level(ULTRASONIC_TRIG, 0);
    portEXIT_CRITICAL(&s_mux);
    return s_t_trig;
}

uint8_t ultrasonic_collect(us_meas_t out[ULTRASONIC_COUNT], bool final) {
    echo_state_t snap[ULTRASONIC_COUNT];
    int64_t now;
    portENTER_CRITICAL(&s_mux);
    now = esp_timer_get_time();                     // US-2: giờ cùng bản chụp
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        snap[i].st     = s_echo[i].st;
        snap[i].edges  = s_echo[i].edges;
        snap[i].glitch = s_echo[i].glitch;
        snap[i].extra  = s_echo[i].extra;
        snap[i].t_rise = s_echo[i].t_rise;
        snap[i].t_fall = s_echo[i].t_fall;
    }
    portEXIT_CRITICAL(&s_mux);

    uint8_t mask = 0;
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        if (s_decided[i]) continue;
        us_meas_t m = { .kind = US_K_UNKNOWN, .why = US_WHY_NONE, .cm = -1.0f, .pre = s_pre[i],
                        .stage = final ? 2 : 1, .edges = snap[i].edges, .glitch = snap[i].glitch,
                        .rise_us = 0, .w_us = 0, .dec_us = (uint32_t)(now - s_t_trig) };
        bool decide = false;
        bool fresh  = true;                          // kết quả đo mới (không phải FAR dùng lại)
        bool rise_ok = snap[i].t_rise >= s_t_trig &&
                       (uint64_t)(snap[i].t_rise - s_t_trig) <= RISE_MAX_US;
        if (snap[i].t_rise >= s_t_trig) m.rise_us = (uint32_t)(snap[i].t_rise - s_t_trig);
        if (snap[i].st == ST_BUSY) {
            // Cảm biến bỏ qua lần phát này. Lần trước là FAR đo mới (còn cao vì chưa nghe vọng nào): dùng
            // lại FAR MỘT lần (mẫu cũ tối đa một chu kỳ). Lần BUSY thứ hai liền: không biết (US-1).
            if (s_prev_kind[i] == US_K_FAR && !s_reused[i]) {
                m.kind = US_K_FAR;
                m.why  = US_WHY_REUSED;
                fresh  = false;
            } else {
                m.why = US_WHY_BUSY;
            }
            m.stage = 0;
            decide  = true;
        } else if (snap[i].st == ST_OVER) {
            m.why  = US_WHY_EDGES;
            decide = true;
        } else if (snap[i].st == ST_DONE) {
            m.w_us   = (uint32_t)(snap[i].t_fall - snap[i].t_rise);
            float cm = (float)m.w_us * 0.0343f / 2.0f;
            if (!rise_ok) {
                m.why = US_WHY_LATE;
            } else if (snap[i].extra > 0) {
                m.why = US_WHY_EXTRA;                // US-5: có cạnh thừa sau xung, không tin
            } else if (m.w_us > US_W_MAX_US) {
                m.kind = US_K_FAR;
            } else {
                m.kind = US_K_DIST;
                m.cm   = cm;
            }
            decide = true;
        } else if (snap[i].st == ST_HIGH) {
            m.w_us = (uint32_t)(now - snap[i].t_rise);
            if (!rise_ok) {
                if (final) { m.why = US_WHY_LATE; decide = true; }
            } else if (m.w_us > US_W_MAX_US || final) {
                m.kind = US_K_FAR;                   // chưa có vọng trong tầm
                decide = true;
            }
        } else if (final) {
            m.why  = US_WHY_NORISE;                  // ECHO không lên (ST_ARMED)
            decide = true;
        }
        if (!decide) continue;
        s_decided[i] = 1;
        if (fresh) { s_prev_kind[i] = m.kind; s_reused[i] = 0; }
        else       { s_reused[i] = 1; }
        out[i] = m;
        mask |= (uint8_t)(1u << i);
    }
    if (final) {
        // US-8: hết cửa sổ đo, tắt ngắt; chân còn cao (FAR) vẫn được xét bằng mức GPIO lúc phát kế
        portENTER_CRITICAL(&s_mux);
        for (int i = 0; i < ULTRASONIC_COUNT; i++) s_echo[i].st = ST_IDLE;
        portEXIT_CRITICAL(&s_mux);
        for (int i = 0; i < ULTRASONIC_COUNT; i++) gpio_intr_disable(echo_pins[i]);
    }
    return mask;
}

void ultrasonic_read_all(us_data_t *data) {
    us_meas_t m[ULTRASONIC_COUNT];
    ultrasonic_ping();
    vTaskDelay(pdMS_TO_TICKS(US_COLLECT2_US / 1000 + 10));
    ultrasonic_collect(m, false);
    ultrasonic_collect(m, true);
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        data->valid[i]       = (m[i].kind == US_K_DIST);
        data->distance_cm[i] = data->valid[i] ? m[i].cm : -1.0f;
    }
}
