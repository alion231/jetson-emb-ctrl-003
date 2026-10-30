#ifndef BALANCE_LOOP_H
#define BALANCE_LOOP_H

#include <stdatomic.h>

void balance_loop_init(void);
void balance_loop_step(unsigned int loop_count);
void balance_loop_set_gains(double kp, double ki, double kd);
void balance_loop_cleanup(void);

/* Shared state for GUI (read-only from GUI thread) */
extern _Atomic double g_pitch_deg;
extern _Atomic double g_motor_rpm[2];
extern _Atomic double g_kp;
extern _Atomic double g_ki;
extern _Atomic double g_kd;

#endif /* BALANCE_LOOP_H */
