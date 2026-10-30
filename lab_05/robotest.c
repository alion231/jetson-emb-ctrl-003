#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <math.h>
#include <signal.h>
#include <pthread.h>

#include "encoder.h"
#include "pwm.h"
#include "imu.h"
#include "balance_loop.h"
#include "gui.h"

#define INTERVAL_NS 1000000 // 1 millisecond in nanoseconds
#define PRIORITY 80         // Real-time priority (1-99)

atomic_bool running = true;

void handle_sigint(int sig)
{
    (void)sig;
    atomic_store(&running, false);
}

static void *balance_thread_func(void *arg)
{
    (void)arg;

    struct sched_param param;
    struct timespec next_time;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        perror("mlockall failed");
        return NULL;
    }

    memset(&param, 0, sizeof(param));
    param.sched_priority = PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed (run as root)");
        return NULL;
    }

    if (clock_gettime(CLOCK_MONOTONIC, &next_time) == -1)
    {
        perror("clock_gettime failed");
        return NULL;
    }

    printf("Real-time balance loop started at 1ms intervals.\n");

    for (unsigned int loop_count = 0; atomic_load(&running) && loop_count < 1000000; ++loop_count)
    {
        next_time.tv_nsec += INTERVAL_NS;
        if (next_time.tv_nsec >= 1000000000)
        {
            next_time.tv_sec += 1;
            next_time.tv_nsec -= 1000000000;
        }

        if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL) != 0)
        {
            perror("clock_nanosleep failed");
            break;
        }

        balance_loop_step(loop_count);
    }

    munlockall();
    return NULL;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, handle_sigint);

    if (imu_init())
    {
        fprintf(stderr, "Failed to initialize IMU\n");
        exit(EXIT_FAILURE);
    }

    if (encoder_init())
    {
        fprintf(stderr, "Failed to initialize encoders\n");
        exit(EXIT_FAILURE);
    }

    if (pwm_init())
    {
        fprintf(stderr, "Failed to initialize pwms\n");
        exit(EXIT_FAILURE);
    }

    balance_loop_init();

    pthread_t balance_thread;
    if (pthread_create(&balance_thread, NULL, balance_thread_func, NULL) != 0)
    {
        perror("pthread_create failed");
        exit(EXIT_FAILURE);
    }

    /* GTK4 runs in the main thread */
    gui_init(argc, argv);

    /* When GTK main loop exits, stop balance thread */
    atomic_store(&running, false);
    pthread_join(balance_thread, NULL);

    balance_loop_cleanup();
    encoder_cleanup();
    pwm_cleanup();
    imu_cleanup();

    return 0;
}
