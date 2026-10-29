#include "motor.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static const char *TAG = "MOTOR";

// MCPWM handles
static mcpwm_timer_handle_t timers[4];
static mcpwm_oper_handle_t operators[4];
static mcpwm_cmpr_handle_t comparators[MOTOR_COUNT][2]; // [motor][rpwm, lpwm]
static mcpwm_gen_handle_t generators[MOTOR_COUNT][2];

static const struct {
    int rpwm_gpio;
    int lpwm_gpio;
    int group_id;
} motor_cfg[MOTOR_COUNT] = {
    [MOTOR_FL] = {FL_LPWM, FL_RPWM, 0},  // đổi chỗ
    [MOTOR_FR] = {FR_LPWM, FR_RPWM, 0},  // đổi chỗ
    [MOTOR_RL] = {RL_LPWM, RL_RPWM, 1},  // đổi chỗ
    [MOTOR_RR] = {RR_LPWM, RR_RPWM, 1},  // đổi chỗ
};

// Chiều lệnh từng bánh: +1 giữ nguyên, -1 đảo. Bù cho dây motor đấu ngược.
// Spec S01 chạy thử 20/10/2026: với +1, PWM dương làm bánh FL quay LÙI (count âm),
// do commit 6b794c2 đảo logic PWM mà vẫn giữ phép đổi chỗ chân ở motor_cfg.
// Chốt 21/10/2026 theo S01 run01 (05_DATA/results/S01/2026-10-20_run01): cả 4 bánh quay lùi ở +PWM khi dir = +1.
// Nhớ đặt cực tính dây tương ứng cho plant HIL (rpi5/params.py motor_polarity,
// matlab params_mecanum.m params.motor_polarity) để HIL mô hình đúng xe thật.
static const int8_t motor_dir[MOTOR_COUNT] = { -1, -1, -1, -1 };

void motor_init(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // Timer
        mcpwm_timer_config_t timer_cfg = {
            .group_id      = motor_cfg[i].group_id,
            .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = PWM_FREQ_HZ * PWM_RESOLUTION,
            .period_ticks  = PWM_RESOLUTION,
            .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        };
        mcpwm_new_timer(&timer_cfg, &timers[i]);

        // Operator
        mcpwm_operator_config_t oper_cfg = {
            .group_id = motor_cfg[i].group_id,
        };
        mcpwm_new_operator(&oper_cfg, &operators[i]);
        mcpwm_operator_connect_timer(operators[i], timers[i]);

        // Comparator RPWM
        mcpwm_comparator_config_t cmpr_cfg = {
            .flags.update_cmp_on_tez = true,
        };
        mcpwm_new_comparator(operators[i], &cmpr_cfg, &comparators[i][0]);
        mcpwm_new_comparator(operators[i], &cmpr_cfg, &comparators[i][1]);

        // Generator RPWM
        mcpwm_generator_config_t gen_cfg = {
            .gen_gpio_num = motor_cfg[i].rpwm_gpio,
        };
        mcpwm_new_generator(operators[i], &gen_cfg, &generators[i][0]);
        mcpwm_generator_set_action_on_timer_event(generators[i][0],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(generators[i][0],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                comparators[i][0], MCPWM_GEN_ACTION_LOW));

        // Generator LPWM
        gen_cfg.gen_gpio_num = motor_cfg[i].lpwm_gpio;
        mcpwm_new_generator(operators[i], &gen_cfg, &generators[i][1]);
        mcpwm_generator_set_action_on_timer_event(generators[i][1],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(generators[i][1],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                comparators[i][1], MCPWM_GEN_ACTION_LOW));

        mcpwm_comparator_set_compare_value(comparators[i][0], PWM_RESOLUTION);
        mcpwm_comparator_set_compare_value(comparators[i][1], PWM_RESOLUTION);

        // Start timer
        mcpwm_timer_enable(timers[i]);
        mcpwm_timer_start_stop(timers[i], MCPWM_TIMER_START_NO_STOP);
    }

    // Dòng ra GPIO46 (RL_RPWM) lên mức 3: ở mức mặc định (2), H7 đọc sai xung từ chân này khi băm duty 70 %
    // (05_DATA/results/PWM_CAPTURE/2026-10-27_run06, run07; cơ chế chưa rõ). Liam duyệt 27/10, chỉ chân này.
    gpio_set_drive_capability(RL_RPWM, GPIO_DRIVE_CAP_3);

    // EN chung
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << GPIO_EN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&en_cfg);
    gpio_set_level(GPIO_EN, 0);

    ESP_LOGI(TAG, "Motor init done (MCPWM)");
}

// void motor_set(motor_id_t id, int speed) {
//     if (id >= MOTOR_COUNT) return;

//     if (speed >  PWM_MAX) speed =  PWM_MAX;
//     if (speed < -PWM_MAX) speed = -PWM_MAX;

//     uint32_t rpwm = 0, lpwm = 0;
//     if (speed > 0) {
//         rpwm = (uint32_t)speed;
//     } else if (speed < 0) {
//         lpwm = (uint32_t)(-speed);
//     }

//     mcpwm_comparator_set_compare_value(comparators[id][0], rpwm);
//     mcpwm_comparator_set_compare_value(comparators[id][1], lpwm);
// }

void motor_set(motor_id_t id, int speed) {
    motor_set_info(id, speed, NULL);
}

// Như motor_set, ghi thêm esp_timer ngay trước lệnh ghi compare đầu và ngay sau lệnh cuối, và
// compare theo GPIO (env đo S06: mốc "áp duty" [L7-13, V2-8], duty theo tên GPIO). Compare có hiệu
// lực ở lần bộ đếm về 0 kế tiếp (update_cmp_on_tez), tức trong [t_before; t_after + 50 µs].
void motor_set_info(motor_id_t id, int speed, motor_set_info_t *info) {
    if (id >= MOTOR_COUNT) return;

    if (speed >  PWM_MAX) speed =  PWM_MAX;
    if (speed < -PWM_MAX) speed = -PWM_MAX;

    speed *= motor_dir[id];

    uint32_t rpwm = PWM_RESOLUTION;  // compare = period: HIGH cả chu kỳ (2 chân HIGH = hãm)
    uint32_t lpwm = PWM_RESOLUTION;

    if (speed > 0) {
        rpwm = PWM_RESOLUTION - (uint32_t)speed;  // duty = speed/1000
        lpwm = PWM_RESOLUTION;                     // LPWM = 0%
    } else if (speed < 0) {
        rpwm = PWM_RESOLUTION;                     // RPWM = 0%
        lpwm = PWM_RESOLUTION - (uint32_t)(-speed);
    }

    int64_t t0 = info ? esp_timer_get_time() : 0;
    mcpwm_comparator_set_compare_value(comparators[id][0], rpwm);
    mcpwm_comparator_set_compare_value(comparators[id][1], lpwm);
    if (info) {
        info->t_after  = esp_timer_get_time();
        info->t_before = t0;
        info->gpio[0]  = motor_cfg[id].rpwm_gpio;   // GPIO của comparators[id][0]
        info->gpio[1]  = motor_cfg[id].lpwm_gpio;
        info->cmp[0]   = rpwm;
        info->cmp[1]   = lpwm;
    }
}

void motor_stop_all(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_set(i, 0);
    }
}

void motor_enable(bool en) {
    gpio_set_level(GPIO_EN, en ? 1 : 0);
    ESP_LOGI(TAG, "Motor %s", en ? "enabled" : "disabled");
}