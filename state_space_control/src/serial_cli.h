#ifndef SERIAL_CLI_H
#define SERIAL_CLI_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

void serial_stdio_setup(void);
void serial_wait_usb(void);
bool serial_parse_pulse_us(const char *line, uint16_t *out_us, uint32_t min_us, uint32_t max_us);
void serial_read_line(char *line, size_t line_size);

#endif
