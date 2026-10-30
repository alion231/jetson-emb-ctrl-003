#include <stdio.h>
#include <math.h>
#include <stdatomic.h>
#include <time.h>

#include "imu.h"
#include "encoder.h"
#include "pwm.h"
#include "balance_loop.h"

#define MAX_DUTY 0.75f
#define RATE_SPAN 4
#define D_ALPHA 0.16 

_Atomic double g_pitch_deg;
_Atomic double g_motor_rpm[2];
_Atomic double g_kp;
_Atomic double g_ki;
_Atomic double g_kd;

static double kp = 0.05;
static double ki = 0.0;
static double kd = 0.0;

static double integral = 0.0;
static double derivative = 0.0;

static double tilt_hist[RATE_SPAN + 1];
static double time_hist[RATE_SPAN + 1];
static int hist_n = 0;
static double tilt_rate = 0.0;
static double rate_f1 = 0.0;
static double rate_f2 = 0.0;

static FILE *log_file = NULL;
static double log_t0 = 0.0;

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

void balance_loop_set_gains(double new_kp, double new_ki, double new_kd)
{
    kp = new_kp;
    ki = new_ki;
    kd = new_kd;

    atomic_store(&g_kp, kp);
    atomic_store(&g_ki, ki);
    atomic_store(&g_kd, kd);
}

void balance_loop_init(void)
{
    integral = 0.0;

    atomic_store(&g_kp, kp);
    atomic_store(&g_ki, ki);
    atomic_store(&g_kd, kd);

    char name[64];
    time_t now = time(NULL);
    strftime(name, sizeof(name), "log_%Y%m%d_%H%M%S.csv", localtime(&now));
    log_file = fopen(name, "w");
    if (log_file != NULL) {
        fprintf(log_file, "time_s,tilt_deg,rpm0,rpm1,u,kp,ki,kd\n");
        printf("Logging to %s\n", name);
    }
    log_t0 = now_s();
}

void balance_loop_step(unsigned int loop_count)
{
    (void)loop_count;

    /* Read sensors */
    double pitch = get_imu_pitch();     /* degrees */
    double rpm0 = encoder_get_speed(0); /* RPM */
    double rpm1 = encoder_get_speed(1); /* RPM */
    double tilt = pitch - 180.0; /* degrees, 180 = upright, 0 = upside down */

    if(tilt > 180.0)
        tilt -= 360.0;
    else if(tilt < -180.0)
        tilt += 360.0;

    atomic_store(&g_pitch_deg, tilt);
    atomic_store(&g_motor_rpm[0], rpm0);
    atomic_store(&g_motor_rpm[1], rpm1);

    /* Target pitch is upright (0 deg) */
    double theta_target = -0.3;
    double error = theta_target - tilt;

    /* TODO: Implement basic P/PI/PID control here.
     *
     * Example:
     *   integral += error * 0.001; // 1 ms
     *   double derivative = (error - prev_error) / 0.001;
     *   double u = kp * error + ki * integral + kd * derivative;
     *
     * Use u to compute motor commands for both motors.
     */

     double kp_now = atomic_load(&g_kp);
     double ki_now = atomic_load(&g_ki);
     double kd_now = atomic_load(&g_kd);

    if (loop_count % 10 == 0) {
        for (int i = 0; i < RATE_SPAN; i++) {
            tilt_hist[i] = tilt_hist[i + 1];
            time_hist[i] = time_hist[i + 1];
        }
        tilt_hist[RATE_SPAN] = tilt;
        time_hist[RATE_SPAN] = now_s();
        if (hist_n < RATE_SPAN + 1) hist_n++;

        if (hist_n == RATE_SPAN + 1) {
            double raw_rate = (tilt_hist[RATE_SPAN] - tilt_hist[0]) / (time_hist[RATE_SPAN] - time_hist[0]);
            rate_f1 += D_ALPHA * (raw_rate - rate_f1);
            rate_f2 += D_ALPHA * (rate_f1 - rate_f2);
            tilt_rate = rate_f2;
        }
    }
    derivative = -tilt_rate;

     integral += error * 0.001; // Update integral every 1 ms
     if (ki_now > 0.0) {
        integral = fmax(fmin(integral, MAX_DUTY / ki_now), -MAX_DUTY / ki_now); // Anti-windup
     } else {
        integral = 0.0; // Reset integral if Ki is zero
     }
    
     double u = kp_now*error + ki_now*integral + kd_now*derivative;
     
     if (fabs(error) > 5.0) {
        u = 0.0; // Stop motors if tilt exceeds 0.6 degrees
        integral = 0.0; // Reset integral term to prevent windup
     }

    /* Simple placeholder: zero command until TODO is implemented */
    float duty0 = -(float)u;
    float duty1 = (float)u;

    /* Clamp duty to +/- MAX_DUTY */
    if (duty0 > MAX_DUTY)
        duty0 = MAX_DUTY;
    if (duty0 < -MAX_DUTY)
        duty0 = -MAX_DUTY;
    if (duty1 > MAX_DUTY)
        duty1 = MAX_DUTY;
    if (duty1 < -MAX_DUTY)
        duty1 = -MAX_DUTY;

    pwm_set_duty(0, duty0);
    pwm_set_duty(1, duty1);

    if (log_file != NULL && loop_count % 10 == 0) {
        fprintf(log_file, "%.3f,%.3f,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f\n",
                now_s() - log_t0, tilt, rpm0, rpm1, u, kp_now, ki_now, kd_now);
        if (loop_count % 1000 == 0)
            fflush(log_file);
    }

}

void balance_loop_cleanup(void)
{
    /* Nothing special yet; placeholder for future extensions */
    if (log_file != NULL) {
        fclose(log_file);
        log_file = NULL;
    }
}
