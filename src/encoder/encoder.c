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

// ─── Count tích lũy ──────────────────────────────────────────────────────────
// Bộ đếm PCNT phần cứng về 0 khi chạm high_limit/low_limit (không tràn kiểu
// int16). Bản cũ bù tràn ±65536 nên cứ 32767 count (~24.8 vòng) lại đọc ra
// một nhịp tốc độ cực lớn (log xe thật 10/06: FL 32716 → 41, rpm −49559).
// Sửa 21/10/2026: bật accum_count, driver tự cộng dồn khi chạm giới hạn
// (cần đăng ký 2 giới hạn làm watch point), pcnt_unit_get_count trả tổng.
static int32_t accum_count[ENC_COUNT]  = {0};  /* tổng tích lũy */
static int64_t last_time_us[ENC_COUNT] = {0};
static int32_t last_accum[ENC_COUNT]   = {0};  /* accum lần trước để tính delta */

#define PCNT_HIGH_LIMIT  32767
#define PCNT_LOW_LIMIT  -32768

void encoder_init(void) {
    for (int i = 0; i < ENC_COUNT; i++) {
        pcnt_unit_config_t unit_cfg = {
            .high_limit = PCNT_HIGH_LIMIT,
            .low_limit  = PCNT_LOW_LIMIT,
            .flags.accum_count = true,
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

        pcnt_unit_add_watch_point(pcnt_units[i], PCNT_HIGH_LIMIT);
        pcnt_unit_add_watch_point(pcnt_units[i], PCNT_LOW_LIMIT);

        pcnt_unit_enable(pcnt_units[i]);
        pcnt_unit_clear_count(pcnt_units[i]);
        pcnt_unit_start(pcnt_units[i]);

        accum_count[i]  = 0;
        last_accum[i]   = 0;
        last_time_us[i] = esp_timer_get_time();
    }

    ESP_LOGI(TAG, "Encoder init done");
}

// ─── Internal: update accumulator ────────────────────────────────────────────
static void _update_accum(enc_id_t id) {
    int count = 0;
    pcnt_unit_get_count(pcnt_units[id], &count);   /* đã gồm phần driver cộng dồn */
    accum_count[id] = (int32_t)count;
}

void encoder_read_all(int32_t out[ENC_COUNT]) {
    for (int i = 0; i < ENC_COUNT; i++) {
        int count = 0;
        pcnt_unit_get_count(pcnt_units[i], &count);
        out[i] = (int32_t)count;
    }
}

int32_t encoder_get_count(enc_id_t id) {
    if (id >= ENC_COUNT) return 0;
    _update_accum(id);
    return accum_count[id];
}

void encoder_clear(enc_id_t id) {
    if (id >= ENC_COUNT) return;
    pcnt_unit_clear_count(pcnt_units[id]);
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