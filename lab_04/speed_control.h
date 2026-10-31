#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

void speed_control_init(void);
void speed_control(int motor, double target_speed);
double speed_control_measure(int motor);
double speed_control_compute(double target_speed, double limit);
void speed_control_reset(void);
void speed_control_cleanup(void);

#endif