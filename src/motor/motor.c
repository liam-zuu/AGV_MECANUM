#include "motor.h"
#include "esp_log.h"
#include "driver/gpio.h"

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

        // Set duty = 0
        mcpwm_comparator_set_compare_value(comparators[i][0], 0);
        mcpwm_comparator_set_compare_value(comparators[i][1], 0);

        // Start timer
        mcpwm_timer_enable(timers[i]);
        mcpwm_timer_start_stop(timers[i], MCPWM_TIMER_START_NO_STOP);
    }

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

void motor_set(motor_id_t id, int speed) {
    if (id >= MOTOR_COUNT) return;

    if (speed >  PWM_MAX) speed =  PWM_MAX;
    if (speed < -PWM_MAX) speed = -PWM_MAX;

    uint32_t rpwm = 0, lpwm = 0;
    if (speed > 0) {
        rpwm = (uint32_t)speed;
    } else if (speed < 0) {
        lpwm = (uint32_t)(-speed);
    }

    mcpwm_comparator_set_compare_value(comparators[id][0], rpwm);
    mcpwm_comparator_set_compare_value(comparators[id][1], lpwm);
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