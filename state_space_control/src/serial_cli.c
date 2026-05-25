#include "serial_cli.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"

#if LIB_PICO_STDIO_USB
#include "pico/stdio_usb.h"
#endif

void serial_stdio_setup(void) {
    stdio_init_all();
    setbuf(stdout, NULL);
}

bool serial_usb_connected(void) {
#if LIB_PICO_STDIO_USB
    return stdio_usb_connected();
#else
    return true;
#endif
}

void serial_wait_usb(void) {
#if LIB_PICO_STDIO_USB
    while (!stdio_usb_connected()) {
        sleep_ms(USB_SERIAL_WAIT_STEP_MS);
    }
#endif
    sleep_ms(500);
}

void serial_wait_usb_optional(uint32_t max_wait_ms) {
#if LIB_PICO_STDIO_USB
    uint32_t waited_ms = 0;
    while (!stdio_usb_connected() && waited_ms < max_wait_ms) {
        sleep_ms(USB_SERIAL_WAIT_STEP_MS);
        waited_ms += USB_SERIAL_WAIT_STEP_MS;
    }
    if (stdio_usb_connected()) {
        sleep_ms(500);
    }
#else
    (void)max_wait_ms;
    sleep_ms(500);
#endif
}

bool serial_parse_pulse_us(const char *line, uint16_t *out_us, uint32_t min_us, uint32_t max_us) {
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
    if (*end != '\0' || value < (long)min_us || value > (long)max_us) {
        return false;
    }

    *out_us = (uint16_t)value;
    return true;
}

void serial_read_line(char *line, size_t line_size) {
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
