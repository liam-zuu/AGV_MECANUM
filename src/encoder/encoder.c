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

// Lưu count và timestamp lần trước để tính RPM
static int32_t last_count[ENC_COUNT] = {0};
static int64_t last_time_us[ENC_COUNT] = {0};

void encoder_init(void) {
    for (int i = 0; i < ENC_COUNT; i++) {
        // Config PCNT unit
        pcnt_unit_config_t unit_cfg = {
            .high_limit = 32767,
            .low_limit  = -32768,
        };
        pcnt_new_unit(&unit_cfg, &pcnt_units[i]);

        // Config channel A
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

        // Config channel B
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

        // Glitch filter 1us
        pcnt_glitch_filter_config_t filter = {
            .max_glitch_ns = 1000,
        };
        pcnt_unit_set_glitch_filter(pcnt_units[i], &filter);

        pcnt_unit_enable(pcnt_units[i]);
        pcnt_unit_clear_count(pcnt_units[i]);
        pcnt_unit_start(pcnt_units[i]);

        last_time_us[i] = esp_timer_get_time();
    }

    ESP_LOGI(TAG, "Encoder init done");
}

int32_t encoder_get_count(enc_id_t id) {
    if (id >= ENC_COUNT) return 0;
    int count = 0;
    pcnt_unit_get_count(pcnt_units[id], &count);
    return (int32_t)count;
}

void encoder_clear(enc_id_t id) {
    if (id >= ENC_COUNT) return;
    pcnt_unit_clear_count(pcnt_units[id]);
    last_count[id] = 0;
    last_time_us[id] = esp_timer_get_time();
}

// float encoder_get_rpm(enc_id_t id) {
//     if (id >= ENC_COUNT) return 0.0f;

//     int64_t now_us = esp_timer_get_time();
//     int32_t now_count = encoder_get_count(id);

//     int32_t delta_count = now_count - last_count[id];
//     float delta_time_s = (float)(now_us - last_time_us[id]) / 1e6f;

//     last_count[id] = now_count;
//     last_time_us[id] = now_us;

//     if (delta_time_s <= 0.0f) return 0.0f;

//     // RPM = (delta_count / PPR) / delta_time_s * 60
//     float rpm = ((float)delta_count / PPR) / delta_time_s * 60.0f;
//     return rpm;
// }

float encoder_get_rpm(enc_id_t id) {
    if (id >= ENC_COUNT) return 0.0f;

    int64_t now_us = esp_timer_get_time();
    int32_t now_count = encoder_get_count(id);

    int32_t delta_count = now_count - last_count[id];
    float delta_time_s = (float)(now_us - last_time_us[id]) / 1e6f;

    ESP_LOGI("ENC_DEBUG", "id=%d raw_count=%ld delta=%ld dt=%.3f",
        id, now_count, delta_count, delta_time_s);

    last_count[id] = now_count;
    last_time_us[id] = now_us;

    if (delta_time_s <= 0.0f) return 0.0f;

    float rpm = ((float)delta_count / PPR) / delta_time_s * 60.0f;
    return rpm;
}