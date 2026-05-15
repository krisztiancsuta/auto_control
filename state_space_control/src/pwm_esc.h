#ifndef PWM_ESC_H
#define PWM_ESC_H

#include <stdint.h>

typedef struct {
    uint8_t gpio_pin;
    uint8_t pwm_slice;
    uint8_t pwm_channel;
    uint16_t min_us;
    uint16_t nominal_us;
    uint16_t max_us;
    uint16_t period_us;
    uint16_t current_us;
} pwm_esc_t;

void pwm_esc_init(pwm_esc_t *esc, uint8_t gpio_pin,
                  uint16_t min_us, uint16_t nominal_us, uint16_t max_us);
uint32_t pwm_esc_pwm_frequency_hz(void);
uint32_t pwm_esc_pwm_period_us(void);
void pwm_esc_set_speed(pwm_esc_t *esc, float speed);
void pwm_esc_set_speed_us(pwm_esc_t *esc, uint16_t pulse_us);
uint16_t pwm_esc_get_current_us(pwm_esc_t *esc);
void pwm_esc_stop(pwm_esc_t *esc);
void pwm_esc_deinit(pwm_esc_t *esc);

#endif
