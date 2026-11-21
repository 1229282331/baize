#ifndef BAIZE_SERIAL_PORT_H
#define BAIZE_SERIAL_PORT_H

#include "baize_protocol.h"

#ifdef _WIN32
#include <windows.h>
typedef HANDLE SerialPort;
#define SERIAL_INVALID INVALID_HANDLE_VALUE
#else
typedef int SerialPort;
#define SERIAL_INVALID (-1)
#endif

#ifdef __cplusplus
extern "C" {
#endif

SerialPort serial_open(const char *name);
int serial_read(SerialPort port, uint8_t *data, size_t size);
bool serial_write(SerialPort port, const uint8_t *data, size_t size);
bool serial_write_frame(SerialPort port, const BaizeFrame *frame);
void serial_close(SerialPort port);
uint64_t monotonic_ms(void);

#ifdef __cplusplus
}
#endif
#endif
