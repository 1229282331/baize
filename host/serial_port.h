#ifndef BAIZE_SERIAL_PORT_H
#define BAIZE_SERIAL_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef _WIN32
#include <windows.h>
typedef HANDLE SerialPort;
#define SERIAL_INVALID INVALID_HANDLE_VALUE
#else
typedef int SerialPort;
#define SERIAL_INVALID (-1)
#endif

SerialPort serial_open(const char *name);
int serial_read(SerialPort port, uint8_t *data, size_t size);
bool serial_write(SerialPort port, const uint8_t *data, size_t size);
void serial_close(SerialPort port);
uint64_t monotonic_ms(void);

#endif
