#include "jog.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void print_drive(uint64_t host_ms, const baize_drive_sample *drive)
{
    printf("%" PRIu64 ",%" PRIu32 ",%u,%u,", host_ms, drive->sample_ms,
           (unsigned)drive->state, (unsigned)drive->stop_reason);
    if (isfinite(drive->supply_v)) printf("%.3f", drive->supply_v);
    for (size_t i = 0; i < BAIZE_WHEEL_COUNT; ++i) printf(",%.3f", drive->angle_rad[i]);
    for (size_t i = 0; i < BAIZE_WHEEL_COUNT; ++i) printf(",%.3f", drive->velocity_rad_s[i]);
    putchar('\n');
}

static void print_imu(FILE *out, uint64_t host_ms, const baize_imu_sample *imu)
{
    fprintf(out, "%" PRIu64 ",%" PRIu32, host_ms, imu->sample_ms);
    for (size_t i = 0; i < 3; ++i) fprintf(out, ",%.3f", imu->acceleration_m_s2[i]);
    for (size_t i = 0; i < 3; ++i) fprintf(out, ",%.3f", imu->angular_velocity_rad_s[i]);
    fputc('\n', out);
}

static volatile sig_atomic_t interrupted;
static void interrupt_jog(int sig) { (void)sig; interrupted = 1; }

static int monitor(baize_chassis *chassis, unsigned long frame_limit, FILE *imu_csv, Jog *jog)
{
    uint64_t start = monotonic_ms();
    uint64_t last_wheel_rx_ms = start, last_imu_rx_ms = start, next_send_ms = start;
    bool imu_stale = false;
    unsigned long rows = 0;
    if (jog) jog->phase_ms = start;
    puts("host_elapsed_ms,board_ms,drive_state,stop_reason,supply_v,"
         "fl_rad,fr_rad,rl_rad,rr_rad,fl_rad_s,fr_rad_s,rl_rad_s,rr_rad_s");
    if (imu_csv) fputs("host_elapsed_ms,board_ms,ax_m_s2,ay_m_s2,az_m_s2,gx_rad_s,gy_rad_s,gz_rad_s\n", imu_csv);

    for (;;) {
        uint64_t now_ms = monotonic_ms();
        if (jog && interrupted && !jog_stop(jog, chassis, now_ms, true)) return 1;
        if (jog && now_ms >= next_send_ms) {
            if (!jog_step(jog, chassis, now_ms)) return 1;
            next_send_ms = now_ms + 20;
        }
        FeedbackSample samples[FEEDBACK_BATCH];
        int count = chassis_poll(chassis, samples);
        if (count < 0) {
            fputs("Serial I/O failed\n", stderr);
            return 1;
        }
        now_ms = monotonic_ms();
        for (int i = 0; i < count; ++i) {
            if (samples[i].kind == BAIZE_STATUS) {
                print_drive(now_ms - start, &samples[i].drive);
                if (fflush(stdout) == EOF || ferror(stdout)) return 1;
                last_wheel_rx_ms = now_ms;
                if (frame_limit && ++rows == frame_limit) return 0;
            } else if (imu_csv) {
                print_imu(imu_csv, now_ms - start, &samples[i].imu);
                if (fflush(imu_csv) == EOF || ferror(imu_csv)) return 1;
                if (imu_stale) fputs("IMU telemetry resumed\n", stderr);
                imu_stale = false;
                last_imu_rx_ms = now_ms;
            }
        }
        if (jog && jog->phase == JOG_DONE) {
            if (jog->failed) fputs("Jog interrupted or handshake failed\n", stderr);
            return jog->failed ? 1 : 0;
        }
        if (now_ms - last_wheel_rx_ms >= 1000) {
            fputs("No valid chassis status for 1000 ms\n", stderr);
            return 1;
        }
        if (imu_csv && !imu_stale && now_ms - last_imu_rx_ms >= 1000) {
            fputs("No valid IMU sample for 1000 ms. Wheel logging continues.\n", stderr);
            imu_stale = true;
        }
    }
}

int main(int argc, char **argv)
{
    unsigned long limit = 0;
    Jog trial;
    Jog *jog = NULL;
    const char *imu_path = NULL;
    if (argc >= 4 && strcmp(argv[argc - 2], "--imu-csv") == 0) {
        imu_path = argv[argc - 1];
        argc -= 2;
    }
    if (argc == 8 && strcmp(argv[2], "--jog") == 0) {
        if (!jog_parse(&trial, &argv[3])) {
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
    baize_chassis *chassis = chassis_connect(argv[1]);
    if (!chassis) return 1;
    FILE *imu_csv = imu_path ? fopen(imu_path, "w") : NULL;
    if (imu_path && !imu_csv) {
        perror(imu_path);
        baize_chassis_close(chassis);
        return 1;
    }
    int result = monitor(chassis, limit, imu_csv, jog);
    if (imu_csv && fclose(imu_csv) != 0) result = 1;
    baize_chassis_close(chassis);
    return result;
}
