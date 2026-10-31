/* 
* Bob Turney
* compile command
*          $ gcc -o game_controller game_controller.c
*/

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sched.h>
#include <sys/mman.h>
#include <stdatomic.h>
#include "pwm.h"
#include "speed_control.h"
#include "encoder.h"
#include "imu.h"

#define CONTROLLER_DEVICE "/dev/input/event12"

#define PAD_MODE BTN_X
#define PAD_STOP BTN_B
#define PAD_EXIT BTN_START

#define SLOW_SPEED 25.0
#define FAST_SPEED 50.0
#define RAMP_STEP  0.2
#define STEER_DUTY 0.12
#define PWN_LIMIT  0.60

#define FB_CH    0
#define LEFT_CH  0
#define RIGHT_CH 1

static const double fwd_sign[2] = { -1.0, 1.0 };

#define LOOP_NS  1000000L
#define PRIORITY 80

atomic_bool running = 1;

static int dpad_up, dpad_down, dpad_left, dpad_right;
static int hat_x, hat_y;
static int mode;
static int stopped;
static int quit;

static const char *button_name(unsigned short code)
{
    switch(code)
    {
        case BTN_A:        return "A";
        case BTN_B:        return "B";
        case BTN_X:        return "X";
        case BTN_Y:        return "Y";

        case BTN_TL:       return "LEFT_BUMPER";
        case BTN_TR:       return "RIGHT_BUMPER";

        case BTN_TL2:      return "LEFT_TRIGGER";
        case BTN_TR2:      return "RIGHT_TRIGGER";

        case BTN_SELECT:   return "SELECT";
        case BTN_START:    return "START";
        case BTN_MODE:     return "HOME";

        case BTN_THUMBL:   return "LEFT_STICK_BUTTON";
        case BTN_THUMBR:   return "RIGHT_STICK_BUTTON";

        case BTN_DPAD_UP:     return "DPAD_UP";
        case BTN_DPAD_DOWN:   return "DPAD_DOWN";
        case BTN_DPAD_LEFT:   return "DPAD_LEFT";
        case BTN_DPAD_RIGHT:  return "DPAD_RIGHT";

        default:
            return "UNKNOWN_BUTTON";
    }
}

static void handle_sigint(int sig)
{
    (void)sig;
    atomic_store(&running, 0);
}

static void resume_if_stopped(void)
{
    if (stopped)
    {
        stopped = 0;
        printf("Resumed\n");
    }
}

static void handle_button(unsigned short code, int value)
{
    int pressed = (value == 1);

    if (value == 2)
        return;

    switch (code)
    {
        case BTN_DPAD_UP:    dpad_up    = pressed; break;
        case BTN_DPAD_DOWN:  dpad_down  = pressed; break;
        case BTN_DPAD_LEFT:  dpad_left  = pressed; break;
        case BTN_DPAD_RIGHT: dpad_right = pressed; break;

        case PAD_MODE:
            if (pressed)
            {
                mode = !mode;
                printf("Mode: %s\n", mode ? "fast" : "slow");
            }
            return;

        case PAD_STOP:
            if (pressed && !stopped)
            {
                stopped = 1;
                printf("Stopped\n");
            }
            return;

        case PAD_EXIT:
            if (pressed)
                quit = 1;
            return;

        default:
            return;
    }
    if (pressed)
        resume_if_stopped();
}

static void handle_axis(unsigned short code, int value)
{
    if (code != ABS_HAT0X && code != ABS_HAT0Y)
        return;

    int v = (value > 0) - (value < 0);
    int *hat = (code == ABS_HAT0X) ? &hat_x : &hat_y;

    if (v != 0 && v != *hat)
        resume_if_stopped();
    *hat = v;
}

static double clamp(double u)
{
    if (u > PWN_LIMIT)
        return PWN_LIMIT;
    if (u < -PWN_LIMIT)
        return -PWN_LIMIT;
    return u;
}

int main(void)
{
    int fd = open(CONTROLLER_DEVICE, O_RDONLY | O_NONBLOCK);

    if (fd < 0)
    {
        perror("open");
        return EXIT_FAILURE;
    }

    printf("Monitoring controller: %s\n", CONTROLLER_DEVICE);
    printf("Press buttons on the controller...\n\n");

    char log_name[64];
    time_t now_t = time(NULL);
    strftime(log_name, sizeof(log_name), "lab06_%Y%m%d_%H%M%S.csv", localtime(&now_t));

    FILE *log = fopen(log_name, "w");
    if (log == NULL)
    {
        perror(log_name);
        close(fd);
        return EXIT_FAILURE;
    }
    fprintf(log, "time_ms,target_speed,measured_speed,left_pwm,right_pwm,mode\n");
    printf("Logging to %s\n", log_name);

    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        perror("mlockall failed");
        return EXIT_FAILURE;
    }
    struct sched_param param;
    memset(&param, 0, sizeof(param));
    param.sched_priority = PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
        return EXIT_FAILURE;
    }

    if (encoder_init())
    {
        fprintf(stderr, "Failed to initialize encoders\n");
        return EXIT_FAILURE;
    }
    if (pwm_init())
    {
        fprintf(stderr, "Failed to initialize PWM\n");
        return EXIT_FAILURE;
    }
    speed_control_init();

    struct timespec start, next, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    next = start;
    long next_log_ms = 20;
    double target_speed = 0.0;
    int read_error = 0;

    struct input_event ev;

    while (atomic_load(&running))
    {
        
        while (1)
        {
            ssize_t bytes = read(fd, &ev, sizeof(ev));

            if (bytes < 0)
            {
                if (errno != EAGAIN && errno != EINTR)
                    read_error = errno;
                break;
            }

            if (bytes == 0)
            {
                read_error = ENODEV;
                break;
            }

            if (bytes != sizeof(ev))
                break;

            if (ev.type == EV_SYN && ev.code == SYN_DROPPED)
            {
                dpad_up = dpad_down = dpad_left = dpad_right = 0;
                hat_x = hat_y = 0;
            }

            if (ev.type == EV_KEY)
            {
                const char *name = button_name(ev.code);

                if (ev.value == 1)
                {
                    printf("BUTTON PRESSED  : %s (code=%u)\n",
                        name, ev.code);
                    fflush(stdout);
                }
                else if (ev.value == 0)
                {
                    printf("BUTTON RELEASED : %s (code=%u)\n",
                        name, ev.code);
                    fflush(stdout);
                }
                handle_button(ev.code, ev.value);
            }

            if (ev.type == EV_ABS)
            {
                printf("AXIS %u VALUE %d\n",
                    ev.code,
                    ev.value);
                fflush(stdout);

                handle_axis(ev.code, ev.value);
            }
        }

        if (read_error || quit)
            break;

        int drive = (dpad_up || hat_y < 0) - (dpad_down || hat_y > 0);
        int turn = (dpad_left || hat_x < 0) - (dpad_right || hat_x > 0);

        double speed_cmd = 0.0, u_steer = 0.0;
        if (!stopped)
        {
            speed_cmd = drive * (mode ? FAST_SPEED : SLOW_SPEED);
            u_steer = turn * STEER_DUTY;
        }

        if (stopped)
            target_speed = 0.0;
        else if (target_speed < speed_cmd - RAMP_STEP)
            target_speed += RAMP_STEP;
        else if (target_speed > speed_cmd + RAMP_STEP)
            target_speed -= RAMP_STEP;
        else
            target_speed = speed_cmd;

        double measured_speed = -fwd_sign[FB_CH] * speed_control_measure(FB_CH);
        double u_base = 0.0;
        if (target_speed != 0.0)
            u_base = fwd_sign[FB_CH] * 
                     speed_control_compute(-fwd_sign[FB_CH] * target_speed, PWN_LIMIT - fabs(u_steer));
        else
            speed_control_reset();

        double u_left = clamp(u_base - u_steer);
        double u_right = clamp(u_base + u_steer);

        float duty[2];
        duty[LEFT_CH] = (float)(fwd_sign[LEFT_CH] * u_left);
        duty[RIGHT_CH] = (float)(fwd_sign[RIGHT_CH] * u_right);
        pwm_set_duty(0, duty[0]);
        pwm_set_duty(1, duty[1]);

        float left_pwm = duty[LEFT_CH];
        float right_pwm = duty[RIGHT_CH];

        clock_gettime(CLOCK_MONOTONIC, &now);
        long time_ms = ((now.tv_sec - start.tv_sec) * 1000000000L + (now.tv_nsec - start.tv_nsec)) / 1000000L;

        if (time_ms >= next_log_ms)
        {
            fprintf(log, "%ld,%.3f,%.3f,%.3f,%.3f,%d\n",
                    time_ms, target_speed, measured_speed,
                    left_pwm, right_pwm, mode);
            next_log_ms += 20;
        }

        next.tv_nsec += LOOP_NS;
        if (next.tv_nsec >= 1000000000L)
        {
            next.tv_sec += 1;
            next.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    pwm_set_duty(0, 0.0f);
    pwm_set_duty(1, 0.0f);
    fclose(log);

    atomic_store(&running, 0);
    speed_control_cleanup();
    encoder_cleanup();
    pwm_cleanup();
    munlockall();

    close(fd);

    if (read_error)
    {
        fprintf(stderr, "read: %s (controller lost, motors were stopped)\n", strerror(read_error));
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}