#include "pwm_esc.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <math.h>

/* PWM configuration for ESC control
 * Standard ESC expects 50Hz frequency (20ms period)
 * Pulse width: 1000-2000 microseconds
 */
#define ESC_PWM_FREQUENCY 50  // Hz
#define ESC_PWM_PERIOD_US 20000  // microseconds (1/50Hz)

static inline uint32_t clamp_uint32(uint32_t value, uint32_t min, uint32_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static inline float clamp_float(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void pwm_esc_init(pwm_esc_t *esc, uint8_t gpio_pin, 
                  uint16_t min_us, uint16_t nominal_us, uint16_t max_us) {
    
    // Store configuration
    esc->gpio_pin = gpio_pin;
    esc->min_us = min_us;
    esc->nominal_us = nominal_us;
    esc->max_us = max_us;
    esc->current_us = nominal_us;
    
    // Determine PWM slice and channel from GPIO pin
    esc->pwm_slice = pwm_gpio_to_slice_num(gpio_pin);
    esc->pwm_channel = pwm_gpio_to_channel(gpio_pin);
    
    // Enable PWM on the GPIO pin
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    
    // Configure PWM
    // Clock frequency: 125MHz (standard Pico)
    // We need 50Hz frequency with 16-bit resolution
    // Wrap value determines the period
    // Period = (wrap + 1) * clock_div / clock_freq
    // For 50Hz: 20000us = (wrap + 1) * clock_div / 125MHz
    
    uint32_t clock_freq = clock_get_hz(clk_sys);  // 125MHz
    
    // Calculate wrap and clock divisor
    // We want: period = (wrap + 1) * clock_div / 125MHz = 20000us
    // Let's use clock_div = 25, then:
    // 20000us = (wrap + 1) * 25 / 125MHz
    // wrap + 1 = 20000 * 125MHz / (25 * 1000000) = 100000
    // wrap = 99999
    
    // But that's too large for 16-bit. Let's use different approach:
    // clock_div = 125 gives us: period = (wrap + 1) * 125 / 125MHz = (wrap + 1) / 1MHz
    // For 20000us: wrap + 1 = 20000, so wrap = 19999
    
    uint16_t wrap = 19999;  // 20000 ticks at 1MHz = 20000us
    float clock_div = (float)clock_freq / 1000000.0f;  // 125MHz / 1MHz = 125
    
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, clock_div);
    pwm_config_set_wrap(&cfg, wrap);
    
    pwm_init(esc->pwm_slice, &cfg, true);
    
    // Set initial speed
    pwm_esc_set_speed_us(esc, nominal_us);
}

void pwm_esc_set_speed(pwm_esc_t *esc, float speed) {
    // Clamp speed to [-1.0, 1.0]
    speed = clamp_float(speed, -1.0f, 1.0f);
    
    // Map normalized speed to microsecond range
    // -1.0 -> min_us, 0.0 -> nominal_us, 1.0 -> max_us
    uint16_t pulse_us;
    
    if (speed < 0.0f) {
        // Negative: from nominal to min
        float range = (float)esc->nominal_us - (float)esc->min_us;
        pulse_us = (uint16_t)((float)esc->nominal_us - range * (-speed));
    } else {
        // Positive: from nominal to max
        float range = (float)esc->max_us - (float)esc->nominal_us;
        pulse_us = (uint16_t)((float)esc->nominal_us + range * speed);
    }
    
    pwm_esc_set_speed_us(esc, pulse_us);
}

void pwm_esc_set_speed_us(pwm_esc_t *esc, uint16_t pulse_us) {
    // Clamp to valid range
    pulse_us = (uint16_t)clamp_uint32(pulse_us, esc->min_us, esc->max_us);
    esc->current_us = pulse_us;
    
    // Convert microseconds to PWM counter level
    // PWM runs at 1MHz (with our clock_div=125)
    // So 1 microsecond = 1 counter unit
    uint16_t level = pulse_us;
    
    // Set the PWM level (duty cycle)
    pwm_set_chan_level(esc->pwm_slice, esc->pwm_channel, level);
}

uint16_t pwm_esc_get_current_us(pwm_esc_t *esc) {
    return esc->current_us;
}

void pwm_esc_stop(pwm_esc_t *esc) {
    pwm_esc_set_speed_us(esc, esc->min_us);
}

void pwm_esc_deinit(pwm_esc_t *esc) {
    // Disable PWM on this slice
    pwm_set_enabled(esc->pwm_slice, false);
    // Set pin back to general GPIO
    gpio_set_function(esc->gpio_pin, GPIO_FUNC_NULL);
}
