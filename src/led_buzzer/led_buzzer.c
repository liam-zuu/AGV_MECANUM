#include "led_buzzer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include <string.h>

static const char *TAG = "LED_BUZZER";

// RMT
static rmt_channel_handle_t rmt_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

// WS2812 timing (ns)
#define WS2812_T0H_NS   400
#define WS2812_T0L_NS   850
#define WS2812_T1H_NS   800
#define WS2812_T1L_NS   450
#define WS2812_RESET_NS 55000

// RMT resolution
#define RMT_RESOLUTION_HZ   10000000  // 10MHz → 100ns per tick

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    rmt_symbol_word_t reset_code;
    int state;
} ws2812_encoder_t;

static size_t ws2812_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                             const void *primary_data, size_t data_size,
                             rmt_encode_state_t *ret_state) {
    ws2812_encoder_t *ws2812 = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (ws2812->state) {
    case 0:
        encoded_symbols += ws2812->bytes_encoder->encode(
            ws2812->bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws2812->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            *ret_state = RMT_ENCODING_MEM_FULL;
            return encoded_symbols;
        }
        // fall through
    case 1:
        encoded_symbols += ws2812->copy_encoder->encode(
            ws2812->copy_encoder, channel,
            &ws2812->reset_code, sizeof(ws2812->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws2812->state = RMT_ENCODING_RESET;
            *ret_state = RMT_ENCODING_COMPLETE;
        }
        break;
    }
    return encoded_symbols;
}

static esp_err_t ws2812_del(rmt_encoder_t *encoder) {
    ws2812_encoder_t *ws2812 = __containerof(encoder, ws2812_encoder_t, base);
    rmt_del_encoder(ws2812->bytes_encoder);
    rmt_del_encoder(ws2812->copy_encoder);
    free(ws2812);
    return ESP_OK;
}

static esp_err_t ws2812_reset(rmt_encoder_t *encoder) {
    ws2812_encoder_t *ws2812 = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encoder_reset(ws2812->bytes_encoder);
    rmt_encoder_reset(ws2812->copy_encoder);
    ws2812->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static void ws2812_encoder_create(void) {
    ws2812_encoder_t *ws2812 = calloc(1, sizeof(ws2812_encoder_t));
    ws2812->base.encode  = ws2812_encode;
    ws2812->base.del     = ws2812_del;
    ws2812->base.reset   = ws2812_reset;

    uint32_t ticks = RMT_RESOLUTION_HZ / 1000000; // ticks per us

    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit0 = {
            .level0    = 1,
            .duration0 = WS2812_T0H_NS * ticks / 1000,
            .level1    = 0,
            .duration1 = WS2812_T0L_NS * ticks / 1000,
        },
        .bit1 = {
            .level0    = 1,
            .duration0 = WS2812_T1H_NS * ticks / 1000,
            .level1    = 0,
            .duration1 = WS2812_T1L_NS * ticks / 1000,
        },
        .flags.msb_first = 1,
    };
    rmt_new_bytes_encoder(&bytes_cfg, &ws2812->bytes_encoder);

    rmt_copy_encoder_config_t copy_cfg = {};
    rmt_new_copy_encoder(&copy_cfg, &ws2812->copy_encoder);

    ws2812->reset_code = (rmt_symbol_word_t) {
        .level0    = 0,
        .duration0 = WS2812_RESET_NS * ticks / 1000,
        .level1    = 0,
        .duration1 = 0,
    };

    led_encoder = &ws2812->base;
}

void led_buzzer_init(void) {
    // RMT cho WS2812
    rmt_tx_channel_config_t rmt_cfg = {
        .gpio_num          = RGB_LED_GPIO,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    rmt_new_tx_channel(&rmt_cfg, &rmt_chan);
    ws2812_encoder_create();
    rmt_enable(rmt_chan);

    // LEDC cho buzzer
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = BUZZER_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz         = BUZZER_FREQ_DEFAULT,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .gpio_num   = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = BUZZER_LEDC_CH,
        .timer_sel  = BUZZER_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch);

    led_set_color(COLOR_OFF);
    ESP_LOGI(TAG, "LED Buzzer init done");
}

void led_set_color(rgb_color_t color) {
    // WS2812 order: GRB
    uint8_t grb[3] = {color.g, color.r, color.b};
    rmt_transmit_config_t tx_cfg = {.loop_count = 0};
    rmt_transmit(rmt_chan, led_encoder, grb, sizeof(grb), &tx_cfg);
    rmt_tx_wait_all_done(rmt_chan, pdMS_TO_TICKS(10));
}

void buzzer_beep(uint32_t freq_hz, uint32_t duration_ms) {
    ledc_set_freq(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_TIMER, freq_hz);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CH, 512); // 50% duty
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CH);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    buzzer_off();
}

void buzzer_off(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CH, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CH);
}