#include "pico_uart_transports.h"

#include <time.h>

#include "config.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

void usleep(uint64_t us) {
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp) {
    (void)unused;
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

static uart_inst_t *const g_uart = uart0;
static bool g_uart_open = false;

bool pico_serial_transport_open(struct uxrCustomTransport *transport) {
    (void)transport;
    if (g_uart_open) {
        return true;
    }

    uart_init(g_uart, MICRO_ROS_UART_BAUD_RATE);
    gpio_set_function(MICRO_ROS_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(MICRO_ROS_UART_RX_PIN, GPIO_FUNC_UART);
    g_uart_open = true;
    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport *transport) {
    (void)transport;
    if (!g_uart_open) {
        return true;
    }

    uart_deinit(g_uart);
    g_uart_open = false;
    return true;
}

size_t pico_serial_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf,
                                   size_t len, uint8_t *errcode) {
    (void)transport;
    uart_write_blocking(g_uart, buf, len);
    *errcode = 0;
    return len;
}

size_t pico_serial_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len,
                                  int timeout, uint8_t *errcode) {
    (void)transport;
    uint64_t start_time_us = time_us_64();

    for (size_t i = 0; i < len; i++) {
        int64_t remaining_us =
            (int64_t)timeout * 1000 - (int64_t)(time_us_64() - start_time_us);
        if (remaining_us < 0) {
            *errcode = 1;
            return i;
        }

        if (!uart_is_readable_within_us(g_uart, (uint32_t)remaining_us)) {
            *errcode = 1;
            return i;
        }

        buf[i] = uart_getc(g_uart);
    }

    *errcode = 0;
    return len;
}
