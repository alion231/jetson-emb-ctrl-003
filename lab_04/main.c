// CPE 4700 Lab 04 - Motor Step Response Characterization
// Runs: deadband sweep -> step response -> triangular speed control
// Build: make     Run: sudo ./robotest     (robot lifted, wheels free)

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <math.h>
#include <signal.h>

#include "encoder.h"
#include "pwm.h"
#include "speed_control.h"

#define PWM_CH 0
#define ENC    0

atomic_bool running = true;
static struct timespec next_time;

void handle_sigint(int sig)
{
    (void)sig;
    atomic_store(&running, false);
}

static void wait_1ms(void)
{
    next_time.tv_nsec += 1000000;
    if (next_time.tv_nsec >= 1000000000) {
        next_time.tv_nsec -= 1000000000;
        next_time.tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
}

static void log_row(FILE *fp, int t_ms, double duty)
{
    fprintf(fp, "%d,%f,%f,%ld\n", t_ms, duty, encoder_get_speed(ENC), encoder_get_count(ENC));
    wait_1ms();
}

static double find_deadband(FILE *fp, int dir, int *t_ms)
{
    double found = -1.0;
    for (int level = 1; level <= 30 && running && found < 0; level++) {
        double duty = dir * level * 0.02;
        pwm_set_duty(PWM_CH, (float)duty);
        long start = encoder_get_count(ENC);
        for (int i = 0; i < 500 && running; i++)
            log_row(fp, (*t_ms)++, duty);
        long moved = labs(encoder_get_count(ENC) - start);
        printf("duty %+.2f : moved %ld counts\n", duty, moved);
        if (moved >= 10)
            found = fabs(duty);
    }
    pwm_set_duty(PWM_CH, 0.0f);
    for (int i = 0; i < 1000 && running; i++)
        log_row(fp, (*t_ms)++, 0.0);
    return found;
}

int main(void)
{
    struct sched_param param = { .sched_priority = 80 };
    signal(SIGINT, handle_sigint);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1 || sched_setscheduler(0, SCHED_FIFO, &param) == -1)
        perror("real-time setup failed, run with sudo");

    if (encoder_init() || pwm_init()) {
        fprintf(stderr, "Failed to initialize encoders or PWM\n");
        atomic_store(&running, false);
        return 1;
    }
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    FILE *fp = fopen("deadband.csv", "w");
    fprintf(fp, "time_ms,duty_cycle,speed,position\n");
    int t_ms = 0;
    double db_pos = find_deadband(fp, +1, &t_ms);
    double db_neg = find_deadband(fp, -1, &t_ms);
    fclose(fp);
    if (db_pos < 0 || db_neg < 0) {
        if (running)
            fprintf(stderr, "Wheel did not move on encoder %d. Try ENC 0.\n", ENC);
        goto cleanup;
    }
    printf("positive_deadband = %+.2f\nnegative_deadband = %+.2f\n", db_pos, -db_neg);

    const double step_seq[8] = {0.0, 0.60, 0.0, 0.0, -0.60, 0.60, 0.0, 0.0};
    fp = fopen("step_response.csv", "w");
    fprintf(fp, "time_ms,duty_cycle,speed,position\n");
    for (int t = 0; t < 8000 && running; t++) {
        if (t % 1000 == 0)
            pwm_set_duty(PWM_CH, (float)step_seq[t / 1000]);
        log_row(fp, t, step_seq[t / 1000]);
    }
    fclose(fp);
    if (!running)
        goto cleanup;

    speed_control_init();
    FILE *speed_log = fopen("speed_control.csv", "w");
    fprintf(speed_log, "time_ms,target_speed,measured_speed\n");
    int t;
    for (t = 0; t < 6000 && running; t++) {
        double cycle_time = (t % 2000) / 1000.0;
        double target_speed = (cycle_time < 1.0) ? 100.0 - 200.0 * cycle_time
                                                 : -100.0 + 200.0 * (cycle_time - 1.0);
        double measured_speed = encoder_get_speed(ENC);
        speed_control(PWM_CH, target_speed);
        fprintf(speed_log, "%d,%f,%f\n", t, target_speed, measured_speed);
        wait_1ms();
    }
    fclose(speed_log);
    printf("completed %d cycles\n", t / 2000);
    speed_control(PWM_CH, 0.0);

cleanup:
    speed_control_cleanup();
    atomic_store(&running, false);
    encoder_cleanup();
    pwm_cleanup();
    munlockall();
    return 0;
}