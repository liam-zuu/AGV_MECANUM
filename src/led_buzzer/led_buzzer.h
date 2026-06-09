#ifndef LED_BUZZER_H
#define LED_BUZZER_H

#include <stdint.h>
#include "driver/ledc.h"
#include "driver/rmt_tx.h"

// GPIO
#define RGB_LED_GPIO        48
#define BUZZER_GPIO         1

// LEDC cho buzzer
#define BUZZER_LEDC_TIMER   LEDC_TIMER_1
#define BUZZER_LEDC_CH      LEDC_CHANNEL_0
#define BUZZER_FREQ_DEFAULT 2700  // Hz, tần số buzzer thường nghe rõ nhất

// RGB color
typedef struct {
    uint8_t r, g, b;
} rgb_color_t;

#define COLOR_OFF       (rgb_color_t){0,   0,   0  }
#define COLOR_RED       (rgb_color_t){255, 0,   0  }
#define COLOR_GREEN     (rgb_color_t){0,   255, 0  }
#define COLOR_BLUE      (rgb_color_t){0,   0,   255}
#define COLOR_YELLOW    (rgb_color_t){255, 255, 0  }
#define COLOR_WHITE     (rgb_color_t){255, 255, 255}

void led_buzzer_init(void);

// RGB
void led_set_color(rgb_color_t color);

// Buzzer
void buzzer_beep(uint32_t freq_hz, uint32_t duration_ms);
void buzzer_off(void);

#endif