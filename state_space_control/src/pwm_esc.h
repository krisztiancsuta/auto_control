#ifndef PWM_ESC_H
#define PWM_ESC_H

#include <stdint.h>

/**
 * PWM ESC Control Module
 * 
 * Controls an Electronic Speed Controller (ESC) via PWM signal.
 * Standard ESC control range: 1000-2000 microseconds (1-2ms pulse width)
 *   - 1000 µs: minimum speed (stop)
 *   - 1500 µs: nominal/mid speed
 *   - 2000 µs: maximum speed
 */

typedef struct {
    uint8_t gpio_pin;           // GPIO pin for PWM output
    uint8_t pwm_slice;          // PWM slice number (determined from gpio_pin)
    uint8_t pwm_channel;        // PWM channel (A=0 or B=1, determined from gpio_pin)
    
    uint16_t min_us;            // Minimum PWM pulse width in microseconds (default: 1000)
    uint16_t nominal_us;        // Nominal PWM pulse width in microseconds (default: 1500)
    uint16_t max_us;            // Maximum PWM pulse width in microseconds (default: 2000)
    
    uint16_t current_us;        // Current PWM pulse width in microseconds
} pwm_esc_t;

/**
 * Initialize PWM ESC control on the specified GPIO pin.
 * 
 * @param esc Pointer to pwm_esc_t structure
 * @param gpio_pin GPIO pin number to use for PWM output
 * @param min_us Minimum speed pulse width (microseconds)
 * @param nominal_us Nominal speed pulse width (microseconds)
 * @param max_us Maximum speed pulse width (microseconds)
 * 
 * Default parameters: min=1000, nominal=1500, max=2000
 * PWM frequency is set to 50Hz (20ms period) for ESC compatibility
 */
void pwm_esc_init(pwm_esc_t *esc, uint8_t gpio_pin, 
                  uint16_t min_us, uint16_t nominal_us, uint16_t max_us);

/**
 * Set ESC speed using normalized value (-1.0 to 1.0).
 * 
 * @param esc Pointer to pwm_esc_t structure
 * @param speed Normalized speed:
 *              -1.0 = minimum speed (min_us)
 *               0.0 = nominal speed (nominal_us)
 *              +1.0 = maximum speed (max_us)
 * 
 * Values outside [-1.0, 1.0] are clamped to the range.
 */
void pwm_esc_set_speed(pwm_esc_t *esc, float speed);

/**
 * Set ESC speed using raw microsecond value.
 * 
 * @param esc Pointer to pwm_esc_t structure
 * @param pulse_us Pulse width in microseconds (will be clamped to [min_us, max_us])
 */
void pwm_esc_set_speed_us(pwm_esc_t *esc, uint16_t pulse_us);

/**
 * Get the current PWM pulse width in microseconds.
 * 
 * @param esc Pointer to pwm_esc_t structure
 * @return Current pulse width in microseconds
 */
uint16_t pwm_esc_get_current_us(pwm_esc_t *esc);

/**
 * Stop the ESC by setting speed to minimum.
 * 
 * @param esc Pointer to pwm_esc_t structure
 */
void pwm_esc_stop(pwm_esc_t *esc);

/**
 * Deinitialize PWM output.
 * 
 * @param esc Pointer to pwm_esc_t structure
 */
void pwm_esc_deinit(pwm_esc_t *esc);

#endif // PWM_ESC_H
