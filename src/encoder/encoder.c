#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ENCODER";

static pcnt_unit_handle_t pcnt_units[ENC_COUNT];

static const struct {
    int gpio_a;
    int gpio_b;
} enc_cfg[ENC_COUNT] = {
    [ENC_FL] = {FL_ENC_B, FL_ENC_A},
    [ENC_FR] = {FR_ENC_A, FR_ENC_B}, 
    [ENC_RL] = {RL_ENC_B, RL_ENC_A},
    [ENC_RR] = {RR_ENC_A, RR_ENC_B},
};

// ─── Accumulated count để tránh PCNT wrap-around ─────────────────────────────
// PCNT hardware chỉ đếm ±32767 rồi wrap về 0.
// Dùng accumulator int32_t để track tổng count thật sự.
static int32_t last_raw[ENC_COUNT]     = {0};  /* raw PCNT lần trước */
static int32_t accum_count[ENC_COUNT]  = {0};  /* tổng tích lũy */
static int64_t last_time_us[ENC_COUNT] = {0};
static int32_t last_accum[ENC_COUNT]   = {0};  /* accum lần trước để tính delta */

#define PCNT_RANGE  65536   /* 32767 - (-32768) + 1 */
#define WRAP_THRESH 30000   /* > half range → đã wrap */

void encoder_init(void) {
    for (int i = 0; i < ENC_COUNT; i++) {
        pcnt_unit_config_t unit_cfg = {
            .high_limit = 32767,
            .low_limit  = -32768,
        };
        pcnt_new_unit(&unit_cfg, &pcnt_units[i]);

        pcnt_chan_config_t chan_a = {
            .edge_gpio_num  = enc_cfg[i].gpio_a,
            .level_gpio_num = enc_cfg[i].gpio_b,
        };
        pcnt_channel_handle_t ch_a;
        pcnt_new_channel(pcnt_units[i], &chan_a, &ch_a);
        pcnt_channel_set_edge_action(ch_a,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(ch_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        pcnt_chan_config_t chan_b = {
            .edge_gpio_num  = enc_cfg[i].gpio_b,
            .level_gpio_num = enc_cfg[i].gpio_a,
        };
        pcnt_channel_handle_t ch_b;
        pcnt_new_channel(pcnt_units[i], &chan_b, &ch_b);
        pcnt_channel_set_edge_action(ch_b,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(ch_b,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP);

        pcnt_glitch_filter_config_t filter = {
            .max_glitch_ns = 1000,
        };
        pcnt_unit_set_glitch_filter(pcnt_units[i], &filter);

        pcnt_unit_enable(pcnt_units[i]);
        pcnt_unit_clear_count(pcnt_units[i]);
        pcnt_unit_start(pcnt_units[i]);

        last_raw[i]     = 0;
        accum_count[i]  = 0;
        last_accum[i]   = 0;
        last_time_us[i] = esp_timer_get_time();
    }

    ESP_LOGI(TAG, "Encoder init done");
}

// ─── Internal: update accumulator ────────────────────────────────────────────
static void _update_accum(enc_id_t id) {
    int raw = 0;
    pcnt_unit_get_count(pcnt_units[id], &raw);
    int32_t raw32 = (int32_t)raw;

    int32_t delta = raw32 - last_raw[id];

    /* Detect wrap-around */
    if (delta > WRAP_THRESH) {
        delta -= PCNT_RANGE;   /* wrapped positive → thực ra là âm */
    } else if (delta < -WRAP_THRESH) {
        delta += PCNT_RANGE;   /* wrapped negative → thực ra là dương */
    }

    accum_count[id] += delta;
    last_raw[id] = raw32;
}

int32_t encoder_get_count(enc_id_t id) {
    if (id >= ENC_COUNT) return 0;
    _update_accum(id);
    return accum_count[id];
}

void encoder_clear(enc_id_t id) {
    if (id >= ENC_COUNT) return;
    pcnt_unit_clear_count(pcnt_units[id]);
    last_raw[id]    = 0;
    accum_count[id] = 0;
    last_accum[id]  = 0;
    last_time_us[id] = esp_timer_get_time();
}

float encoder_get_rpm(enc_id_t id) {
    if (id >= ENC_COUNT) return 0.0f;

    int64_t now_us = esp_timer_get_time();
    _update_accum(id);

    int32_t delta_count = accum_count[id] - last_accum[id];
    float   delta_time_s = (float)(now_us - last_time_us[id]) / 1e6f;

    last_accum[id]   = accum_count[id];
    last_time_us[id] = now_us;

    if (delta_time_s <= 0.0f) return 0.0f;

    /* RPM = (delta_count / PPR) / dt * 60 */
    float rpm = ((float)delta_count / (float)PPR) / delta_time_s * 60.0f;
    return rpm;
}