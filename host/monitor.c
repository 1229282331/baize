#include "baize_session.h"
#include "serial_port.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void print_status(uint64_t host_ms, const BaizeStatus *status)
{
    printf("%" PRIu64 ",%" PRIu32 ",%u,%u,", host_ms, status->sample_ms,
           (unsigned)status->drive_state, (unsigned)status->stop_reason);
    if (status->supply_mv != UINT16_MAX) printf("%.3f", status->supply_mv / 1000.0);
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) printf(",%.3f", (double)status->angle_mrad[i] / 1000.0);
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) printf(",%.3f", status->velocity_mrad_s[i] / 1000.0);
    putchar('\n');
}

static int monitor(SerialPort port, unsigned long frame_limit)
{
    BaizeSession session;
    baize_session_open(&session, 250);
    uint64_t start = monotonic_ms();
    uint64_t last_status = start;
    uint64_t next_send = start;
    unsigned long rows = 0;
    puts("host_elapsed_ms,board_ms,drive_state,stop_reason,supply_v,"
         "fl_rad,fr_rad,rl_rad,rr_rad,fl_rad_s,fr_rad_s,rl_rad_s,rr_rad_s");

    for (;;) {
        uint64_t now = monotonic_ms();
        if (now >= next_send) {
            BaizeFrame command;
            if (baize_session_next(&session, (uint32_t)now, &command)) {
                uint8_t wire[BAIZE_WIRE_MAX];
                size_t size = baize_encode(&command, wire);
                if (!serial_write(port, wire, size)) {
                    fputs("Serial write failed\n", stderr);
                    return 1;
                }
            }
            next_send = now + 50;
        }

        uint8_t bytes[256];
        int count = serial_read(port, bytes, sizeof bytes);
        if (count < 0) {
            fputs("Serial read failed\n", stderr);
            return 1;
        }
        now = monotonic_ms();
        for (int i = 0; i < count; ++i) {
            BaizeFrame frame;
            if (baize_session_receive(&session, bytes[i], (uint32_t)now, &frame) && frame.type == BAIZE_STATUS) {
                print_status(now - start, &frame.status);
                if (fflush(stdout) == EOF || ferror(stdout)) return 1;
                last_status = now;
                if (frame_limit && ++rows == frame_limit) return 0;
            }
        }
        if (now - last_status >= 1000) {
            fputs("No valid chassis status for 1000 ms\n", stderr);
            return 1;
        }
    }
}

int main(int argc, char **argv)
{
    unsigned long limit = 0;
    if (argc == 3) {
        errno = 0;
        limit = strtoul(argv[2], NULL, 10);
        if (errno || !limit || strspn(argv[2], "0123456789") != strlen(argv[2])) {
            fputs("Frame count must be a positive integer\n", stderr);
            return 2;
        }
    } else if (argc != 2) {
        fputs("Usage: baize-monitor PORT [FRAME_COUNT]\n", stderr);
        return 2;
    }
    SerialPort port = serial_open(argv[1]);
    if (port == SERIAL_INVALID) return 1;
    int result = monitor(port, limit);
    serial_close(port);
    return result;
}
