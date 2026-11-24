#ifndef _WIN32
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include "serial_port.h"

#include <stdio.h>
#include <string.h>

#ifdef _WIN32
SerialPort serial_open(const char *name)
{
    char path[64];
    if (_strnicmp(name, "COM", 3) != 0 || name[3] == '\0' ||
        strspn(name + 3, "0123456789") != strlen(name + 3) || strlen(name) + 5 > sizeof path) {
        fprintf(stderr, "Expected a Windows port name such as COM7\n");
        return SERIAL_INVALID;
    }
    snprintf(path, sizeof path, "\\\\.\\%s", name);
    HANDLE port = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (port == SERIAL_INVALID) {
        fprintf(stderr, "Could not open %s (Windows error %lu)\n", name, GetLastError());
        return SERIAL_INVALID;
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof dcb;
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.fBinary = TRUE;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.XonChar = 0x11;
    dcb.XoffChar = 0x13;
    COMMTIMEOUTS timeouts = {MAXDWORD, MAXDWORD, 20, 0, 100};
    if (!SetCommState(port, &dcb) || !SetCommTimeouts(port, &timeouts) ||
        !PurgeComm(port, PURGE_RXCLEAR | PURGE_TXCLEAR)) {
        fprintf(stderr, "Could not configure %s (Windows error %lu)\n", name, GetLastError());
        CloseHandle(port);
        return SERIAL_INVALID;
    }
    return port;
}

int serial_read(SerialPort port, uint8_t *data, size_t size)
{
    DWORD received;
    if (!ReadFile(port, data, (DWORD)size, &received, NULL)) return -1;
    return (int)received;
}

int serial_available(SerialPort port)
{
    COMSTAT status;
    DWORD errors;
    return ClearCommError(port, &errors, &status) ? (int)status.cbInQue : -1;
}

bool serial_write(SerialPort port, const uint8_t *data, size_t size)
{
    DWORD written;
    return WriteFile(port, data, (DWORD)size, &written, NULL) && written == size;
}

void serial_close(SerialPort port) { CloseHandle(port); }
uint64_t monotonic_ms(void) { return GetTickCount64(); }
#else
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

SerialPort serial_open(const char *name)
{
    int port = open(name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (port == SERIAL_INVALID) {
        perror(name);
        return SERIAL_INVALID;
    }
    if (flock(port, LOCK_EX | LOCK_NB) < 0) {
        perror(name);
        close(port);
        return SERIAL_INVALID;
    }
    struct termios settings;
    if (tcgetattr(port, &settings) == 0) {
        cfmakeraw(&settings);
        settings.c_cflag &= ~(tcflag_t)(CSIZE | PARENB | CSTOPB | CRTSCTS);
        settings.c_cflag |= CLOCAL | CREAD | CS8;
        settings.c_iflag &= ~(tcflag_t)(IXON | IXOFF | IXANY);
        if (cfsetispeed(&settings, B115200) == 0 && cfsetospeed(&settings, B115200) == 0 &&
            tcsetattr(port, TCSANOW, &settings) == 0 && tcflush(port, TCIOFLUSH) == 0) return port;
    }
    perror(name);
    close(port);
    return SERIAL_INVALID;
}

int serial_read(SerialPort port, uint8_t *data, size_t size)
{
    struct pollfd pfd = {.fd = port, .events = POLLIN};
    int ready = poll(&pfd, 1, 20);
    if (ready < 0 && errno == EINTR) return 0;
    if (ready <= 0) return ready;
    if (!(pfd.revents & POLLIN)) return -1;
    ssize_t received = read(port, data, size);
    if (received < 0 && (errno == EAGAIN || errno == EINTR)) return 0;
    return received > 0 ? (int)received : -1;
}

int serial_available(SerialPort port)
{
    struct pollfd pfd = {.fd = port, .events = POLLIN};
    int ready = poll(&pfd, 1, 0);
    if (ready < 0 && errno == EINTR) return 0;
    if (ready <= 0) return ready;
    if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) return -1;
    int bytes;
    return ioctl(port, FIONREAD, &bytes) == 0 ? bytes : -1;
}

bool serial_write(SerialPort port, const uint8_t *data, size_t size)
{
    while (size > 0) {
        struct pollfd pfd = {.fd = port, .events = POLLOUT};
        if (poll(&pfd, 1, 100) <= 0 || !(pfd.revents & POLLOUT)) return false;
        ssize_t n = write(port, data, size);
        if (n <= 0) return false;
        data += n;
        size -= (size_t)n;
    }
    return true;
}

void serial_close(SerialPort port) { close(port); }

uint64_t monotonic_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t)now.tv_sec * 1000 + (uint64_t)now.tv_nsec / 1000000;
}
#endif

bool serial_write_frame(SerialPort port, const BaizeFrame *frame)
{
    uint8_t wire[BAIZE_WIRE_MAX];
    size_t size = baize_encode(frame, wire);
    return size != 0 && serial_write(port, wire, size);
}
