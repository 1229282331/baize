#include "baize_session.h"
#include "jog.h"
#include "serial_port.h"

#include <errno.h>
#include <inttypes.h>
#include <signal.h>
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

static void print_imu(FILE *out, uint64_t host_ms, const BaizeImu *imu)
{
    fprintf(out, "%" PRIu64 ",%" PRIu32, host_ms, imu->sample_ms);
    for (size_t i = 0; i < 3; ++i) fprintf(out, ",%.3f", imu->acceleration_mm_s2[i] / 1000.0);
    for (size_t i = 0; i < 3; ++i) fprintf(out, ",%.3f", imu->gyro_mrad_s[i] / 1000.0);
    fputc('\n', out);
}

static volatile sig_atomic_t interrupted;
static void interrupt_jog(int sig) { (void)sig; interrupted = 1; }

static int monitor(SerialPort port, unsigned long frame_limit, FILE *imu_csv, BaizeJog *jog)
{
    BaizeSession session;
    baize_session_open(&session, 250);
    uint64_t start = monotonic_ms();
    uint64_t last_status = start;
    uint64_t last_imu = start;
    bool imu_stale = false;
    uint64_t next_send = start;
    unsigned long rows = 0;
    if (jog) jog->phase_ms = start;
    puts("host_elapsed_ms,board_ms,drive_state,stop_reason,supply_v,"
         "fl_rad,fr_rad,rl_rad,rr_rad,fl_rad_s,fr_rad_s,rl_rad_s,rr_rad_s");
    if (imu_csv) fputs("host_elapsed_ms,board_ms,ax_m_s2,ay_m_s2,az_m_s2,gx_rad_s,gy_rad_s,gz_rad_s\n", imu_csv);

    for (;;) {
        uint64_t now = monotonic_ms();
        if (jog && interrupted && jog->phase != JOG_STOPPING && jog->phase != JOG_DONE) {
            baize_jog_cancel(jog, now);
            next_send = now;
        }
        if (now >= next_send) {
            BaizeFrame command;
            bool send = jog ? baize_jog_next(jog, &session, now, &command)
                            : baize_session_next(&session, (uint32_t)now, &command);
            if (send && !serial_write_frame(port, &command)) {
                fputs("Serial write failed\n", stderr);
                return 1;
            }
            next_send = now + (jog ? 20u : 50u);
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
            if (!baize_session_receive(&session, bytes[i], (uint32_t)now, &frame)) continue;
            if (frame.type == BAIZE_STATUS) {
                print_status(now - start, &frame.status);
                if (fflush(stdout) == EOF || ferror(stdout)) return 1;
                if (jog) baize_jog_status(jog, &frame.status, now);
                last_status = now;
                if (frame_limit && ++rows == frame_limit) return 0;
            } else if (frame.type == BAIZE_IMU && imu_csv) {
                print_imu(imu_csv, now - start, &frame.imu);
                if (fflush(imu_csv) == EOF || ferror(imu_csv)) return 1;
                if (imu_stale) fputs("IMU telemetry resumed\n", stderr);
                imu_stale = false;
                last_imu = now;
            }
        }
        if (jog && jog->phase == JOG_DONE) {
            if (jog->failed) fputs("Jog interrupted or handshake failed\n", stderr);
            return jog->failed ? 1 : 0;
        }
        if (now - last_status >= 1000) {
            fputs("No valid chassis status for 1000 ms\n", stderr);
            return 1;
        }
        if (imu_csv && !imu_stale && now - last_imu >= 1000) {
            fputs("No valid IMU sample for 1000 ms. Wheel logging continues.\n", stderr);
            imu_stale = true;
        }
    }
}

int main(int argc, char **argv)
{
    unsigned long limit = 0;
    BaizeJog trial;
    BaizeJog *jog = NULL;
    const char *imu_path = NULL;
    if (argc >= 4 && strcmp(argv[argc - 2], "--imu-csv") == 0) {
        imu_path = argv[argc - 1];
        argc -= 2;
    }
    if (argc == 8 && strcmp(argv[2], "--jog") == 0) {
        if (!baize_jog_parse(&trial, &argv[3])) {
            fputs("Jog requires 1..2000 ms and four finite speeds within +/-2.0 rad/s\n", stderr);
            return 2;
        }
        jog = &trial;
        signal(SIGINT, interrupt_jog);
#ifdef SIGPIPE
        signal(SIGPIPE, SIG_IGN);
#endif
    } else if (argc == 3) {
        errno = 0;
        limit = strtoul(argv[2], NULL, 10);
        if (errno || !limit || strspn(argv[2], "0123456789") != strlen(argv[2])) {
            fputs("Frame count must be a positive integer\n", stderr);
            return 2;
        }
    } else if (argc != 2) {
        fputs("Usage: baize-monitor PORT [FRAME_COUNT | --jog MS FL FR RL RR] [--imu-csv FILE]\n", stderr);
        return 2;
    }
    SerialPort port = serial_open(argv[1]);
    if (port == SERIAL_INVALID) return 1;
    FILE *imu_csv = imu_path ? fopen(imu_path, "w") : NULL;
    if (imu_path && !imu_csv) {
        perror(imu_path);
        serial_close(port);
        return 1;
    }
    int result = monitor(port, limit, imu_csv, jog);
    if (jog && result) serial_write_frame(port, &(BaizeFrame){.type = BAIZE_STOP});
    if (imu_csv && fclose(imu_csv) != 0) result = 1;
    serial_close(port);
    return result;
}
