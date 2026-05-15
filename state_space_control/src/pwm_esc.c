#include "pwm_esc.h"

#include "config.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

static inline uint32_t clamp_uint32(uint32_t value, uint32_t min, uint32_t max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static inline float clamp_float(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

void pwm_esc_init(pwm_esc_t *esc, uint8_t gpio_pin,
                  uint16_t min_us, uint16_t nominal_us, uint16_t max_us) {
    esc->gpio_pin = gpio_pin;
    esc->min_us = min_us;
    esc->nominal_us = nominal_us;
    esc->max_us = max_us;
    esc->current_us = nominal_us;
    esc->period_us = (uint16_t)ESC_PWM_PERIOD_US;
    esc->pwm_slice = pwm_gpio_to_slice_num(gpio_pin);
    esc->pwm_channel = pwm_gpio_to_channel(gpio_pin);

    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);

    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint16_t wrap = (uint16_t)(ESC_PWM_PERIOD_US - 1u);
    float clock_div = (float)clock_freq / 1000000.0f;

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, clock_div);
    pwm_config_set_wrap(&cfg, wrap);
    pwm_init(esc->pwm_slice, &cfg, true);

#if ESC_PWM_INVERT_OUTPUT
    pwm_set_output_polarity(esc->pwm_slice,
                            esc->pwm_channel == PWM_CHAN_A,
                            esc->pwm_channel == PWM_CHAN_B);
#endif

    pwm_esc_set_speed_us(esc, nominal_us);
}

void pwm_esc_set_speed(pwm_esc_t *esc, float speed) {
    speed = clamp_float(speed, -1.0f, 1.0f);
    uint16_t pulse_us;

    if (speed < 0.0f) {
        float range = (float)esc->nominal_us - (float)esc->min_us;
        pulse_us = (uint16_t)((float)esc->nominal_us - range * (-speed));
    } else {
        float range = (float)esc->max_us - (float)esc->nominal_us;
        pulse_us = (uint16_t)((float)esc->nominal_us + range * speed);
    }

    pwm_esc_set_speed_us(esc, pulse_us);
}

uint32_t pwm_esc_pwm_frequency_hz(void) {
    return ESC_PWM_FREQUENCY_HZ;
}

uint32_t pwm_esc_pwm_period_us(void) {
    return ESC_PWM_PERIOD_US;
}

void pwm_esc_set_speed_us(pwm_esc_t *esc, uint16_t pulse_us) {
    uint32_t max_allowed = esc->max_us;
    if (max_allowed > esc->period_us) {
        max_allowed = esc->period_us;
    }
    pulse_us = (uint16_t)clamp_uint32(pulse_us, esc->min_us, max_allowed);
    esc->current_us = pulse_us;
    pwm_set_chan_level(esc->pwm_slice, esc->pwm_channel, pulse_us);
}

uint16_t pwm_esc_get_current_us(pwm_esc_t *esc) {
    return esc->current_us;
}

void pwm_esc_stop(pwm_esc_t *esc) {
    pwm_esc_set_speed_us(esc, esc->min_us);
}

void pwm_esc_deinit(pwm_esc_t *esc) {
    pwm_set_enabled(esc->pwm_slice, false);
    gpio_set_function(esc->gpio_pin, GPIO_FUNC_NULL);
}
