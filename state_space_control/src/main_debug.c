#include <stdio.h>

#include "config.h"
#include "pwm_esc.h"
#include "serial_cli.h"

static pwm_esc_t g_esc;

int main(void) {
    serial_stdio_setup();
    printf("Waiting for USB serial...\r\n");
    fflush(stdout);
    serial_wait_usb();

    const uint32_t pwm_hz = pwm_esc_pwm_frequency_hz();

    printf("PWM debug: %lu Hz, GPIO %u\r\n", (unsigned long)pwm_hz, (unsigned int)PIN_PWM_ESC);
    printf("High-time %u-%u us, then Enter\r\n",
           (unsigned int)PWM_SERIAL_DEBUG_MIN_US, (unsigned int)PWM_SERIAL_DEBUG_MAX_US);

    pwm_esc_init(&g_esc, PIN_PWM_ESC, PWM_SERIAL_DEBUG_MIN_US, PWM_SERIAL_DEBUG_NEUTRAL_US,
                 PWM_SERIAL_DEBUG_MAX_US);
    printf("Initial pwm_us=%u\r\n", (unsigned int)pwm_esc_get_current_us(&g_esc));
    fflush(stdout);

    char line[SERIAL_LINE_BUF_SIZE];
    while (true) {
        serial_read_line(line, sizeof(line));

        uint16_t pwm_us;
        if (!serial_parse_pulse_us(line, &pwm_us, PWM_SERIAL_DEBUG_MIN_US,
                                   PWM_SERIAL_DEBUG_MAX_US)) {
            printf("ERR: integer %u-%u\r\n",
                   (unsigned int)PWM_SERIAL_DEBUG_MIN_US, (unsigned int)PWM_SERIAL_DEBUG_MAX_US);
            fflush(stdout);
            continue;
        }

        pwm_esc_set_speed_us(&g_esc, pwm_us);
        printf("OK pwm_us=%u\r\n", (unsigned int)pwm_esc_get_current_us(&g_esc));
        fflush(stdout);
    }

    return 0;
}
