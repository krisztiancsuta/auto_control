
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pwm_esc.h"

#if LIB_PICO_STDIO_USB
#include "pico/stdio_usb.h"
#endif

#define PIN_PWM_ESC 6

static pwm_esc_t g_esc;

static bool parse_pwm_us_line(const char *line, uint16_t *out_us, uint32_t max_us) {
    while (*line != '\0' && isspace((unsigned char)*line)) {
        line++;
    }

    if (*line == '\0') {
        return false;
    }

    char *end = NULL;
    long value = strtol(line, &end, 10);
    if (end == line) {
        return false;
    }

    while (*end != '\0' && isspace((unsigned char)*end)) {
        end++;
    }
    if (*end != '\0') {
        return false;
    }

    if (value < 0 || value > (long)max_us) {
        return false;
    }

    *out_us = (uint16_t)value;
    return true;
}

static void serial_read_line(char *line, size_t line_size) {
    size_t idx = 0;

    printf("> ");
    fflush(stdout);

    while (idx + 1 < line_size) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) {
            tight_loop_contents();
            continue;
        }

        if (c == '\r' || c == '\n') {
            line[idx] = '\0';
            putchar_raw('\n');
            fflush(stdout);
            return;
        }

        if (c == 127 || c == 8) {
            if (idx > 0) {
                idx--;
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        }

        if (c >= ' ' && c <= '~') {
            line[idx++] = (char)c;
            putchar_raw((char)c);
            fflush(stdout);
        }
    }

    line[line_size - 1] = '\0';
}

int main(void) {
    stdio_init_all();
    setbuf(stdout, NULL);

#if LIB_PICO_STDIO_USB
    printf("Waiting for USB serial...\r\n");
    fflush(stdout);
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
#endif
    sleep_ms(500);

    const uint32_t pwm_period_us = pwm_esc_pwm_period_us();
    const uint32_t pwm_hz = pwm_esc_pwm_frequency_hz();

    printf("PWM debug: %lu Hz on GPIO %u\r\n", (unsigned long)pwm_hz, (unsigned int)PIN_PWM_ESC);
    printf("Send high-time 0-%lu us, then Enter\r\n", (unsigned long)pwm_period_us);

    pwm_esc_init(&g_esc, PIN_PWM_ESC, 0u, (uint16_t)(pwm_period_us / 2u),
                 (uint16_t)pwm_period_us);
    printf("Initial pwm_us=%u\r\n", (unsigned int)pwm_esc_get_current_us(&g_esc));
    fflush(stdout);

    char line[32];
    while (true) {
        serial_read_line(line, sizeof(line));

        uint16_t pwm_us;
        if (!parse_pwm_us_line(line, &pwm_us, pwm_period_us)) {
            printf("ERR: send integer 0-%lu\r\n", (unsigned long)pwm_period_us);
            fflush(stdout);
            continue;
        }

        pwm_esc_set_speed_us(&g_esc, pwm_us);
        printf("OK pwm_us=%u\r\n", (unsigned int)pwm_esc_get_current_us(&g_esc));
        fflush(stdout);
    }

    return 0;
}

#if 0
/* -------------------------------------------------------------------------- */
/* Full state-space / micro-ROS firmware (src/main.c) — disabled in debug build */
/* -------------------------------------------------------------------------- */

#include <math.h>
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include "pico_uart_transports.h"
#include "quadrature_encoder.pio.h"
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "state_space_control.h"
#include "std_msgs/msg/float64.h"
#include "std_msgs/msg/int64.h"

#define PIN_AB 2
#define CONTROL_PERIOD_MS 20
#define CONTROL_DT_S 0.02f
#define PI_F 3.14159265358979323846f
#define ENCODER_COUNTS_PER_REV 127.0f
#define WHEEL_RADIUS_M 0.055f
#define GEAR_RATIO_SHAFT_PER_WHEEL 4.0f
#define REF_SPEED_MPS_DEFAULT 0.0f
#define REF_SPEED_TOPIC "ref_speed"
#define MICRO_ROS_AGENT_PING_TIMEOUT_MS 1000
#define MICRO_ROS_AGENT_PING_ATTEMPTS 120u
#define DEBUG false
#define CTRL_FORCE_MIN_N -18.5f
#define CTRL_FORCE_MAX_N 20.0f
#define ESC_MIN_US 1100u
#define ESC_NEUTRAL_US 1300u
#define ESC_MAX_US 1380u
#define ESC_DEADZONE_LOW_US 1300u
#define ESC_DEADZONE_HIGH_US 1300u

/* ... remainder of src/main.c ... */

#endif
