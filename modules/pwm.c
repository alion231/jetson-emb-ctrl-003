#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <gpiod.h>
#include "pwm.h"

// add definitions for the PWM chips and channels
static const int PWM_CHIP2 = 2;
static const int PWM_CHIP3 = 3;
static const int PWM0 = 0;

// PWM definitions
#define PWM_PERIOD_NS 100000 // 10 kHz
#define PWM_CH0_CHIP_PATH "/sys/class/pwm/pwmchip2"
#define PWM_CH1_CHIP_PATH "/sys/class/pwm/pwmchip3"
#define PWM_CH0_PATH PWM_CH0_CHIP_PATH "/pwm0"
#define PWM_CH1_PATH PWM_CH1_CHIP_PATH "/pwm0"
#define PWM_CHANNEL "0"

// GPIO direction line definitions ( gpiochip0 )
#define GPIO_CHIP "/dev/gpiochip0"
#define DIR_LINE_OFFSET 106 // channel 0 direction
#define DIR_LINE_OFFSET2 113 // channel 1 direction

static struct gpiod_chip * chip;
static struct gpiod_line * dir_line [2];

// Helper function to write values to sysfs files
static int write_sysfs (const char *path, const char *value)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    if (write (fd, value, strlen(value)) < 0) {
        close (fd) ;
        return -1;
    }
    close (fd) ;
    return 0;
}

int pwm_init (void)
{
    // Use the definitions above to initialize the PWM channels
    // Example of exporting a PWM channel :
    // Note that exporting takes significant background time , so
    // a delay is often necessary after writing to the export
    // file .
    //
    // if ( access ( PWM_PATH , F_OK ) != 0)
    // {
    // write_sysfs ( PWM_CHIP_PATH "/ export " , PWM_CHANNEL ) ;
    // usleep (100000) ;
    // }
    // TODO : export channels
    if (access(PWM_CH0_PATH, F_OK) != 0) {
        write_sysfs(PWM_CH0_CHIP_PATH "/export", PWM_CHANNEL);
        usleep(100000);     
    }
    if (access(PWM_CH1_PATH, F_OK) != 0) {
        write_sysfs(PWM_CH1_CHIP_PATH "/export", PWM_CHANNEL);
        usleep(100000);     
    }

    // set the PWM period in nanoseconds . (10 kHz max )
    // Write a string representing the period in nanoseconds to
    // the period file .
    // TODO : configure period
    char period_str[20];
    snprintf(period_str, sizeof(period_str), "%d", PWM_PERIOD_NS);
    write_sysfs(PWM_CH0_PATH "/period", period_str);
    write_sysfs(PWM_CH1_PATH "/period", period_str);

    // Initialize the duty cycle to 0 for all channels . Write
    // "0" to the duty_cycle file .
    // TODO : set initial duty cycle
    write_sysfs(PWM_CH0_PATH "/duty_cycle", "0");
    write_sysfs(PWM_CH1_PATH "/duty_cycle", "0");

    // Enable the PWM channels by writing "1" to the enable
    // file for each channel .
    // TODO : enable channels
    write_sysfs(PWM_CH0_PATH "/enable", "1");
    write_sysfs(PWM_CH1_PATH "/enable", "1");

    // TODO : open the GPIO chip with
    // gpiod_chip_open ( GPIO_CHIP )
    chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) {
        perror("gpio_chip_open");
        return -1;
    }

    // TODO : get the two direction lines with
    // gpiod_chip_get_line () using
    // DIR_LINE_OFFSET and DIR_LINE_OFFSET2 , storing
    // them in dir_line [0] and dir_line [1]
    dir_line[0] = gpiod_chip_get_line(chip, DIR_LINE_OFFSET);
    dir_line[1] = gpiod_chip_get_line(chip, DIR_LINE_OFFSET2);
    if (!dir_line[0] || !dir_line[1]) {
        fprintf(stderr, "Failed to get direction lines\n");
        gpiod_chip_close(chip);
        return -1;
    }

    // TODO : request each line as an output with
    // gpiod_line_request_output ()
    if (gpiod_line_request_output(dir_line[0], "pwm", 0)) {
        fprintf(stderr, "Failed to request direction line 0\n");
        gpiod_chip_close(chip);
        return -1;
    }
    if (gpiod_line_request_output(dir_line[1], "pwm", 0)) {
        fprintf(stderr, "Failed to request direction line 1\n");
        gpiod_chip_close(chip);
        return -1;
    }

    return 0;
}
// duty cycle setter . duty_cycle is a float between -1.0 and 1.0
// Positive values should drive the motor in the positive direction .
int pwm_set_duty (int channel, float duty_cycle )
{
    // TODO : set the direction output for this channel based on
    // the sign of duty_cycle using
    // gpiod_line_set_value ( dir_line [ channel ] , ...) .
    // Drive the line high (1) for positive duty_cycle ,
    // low (0) for negative .
    int dir = (duty_cycle >= 0) ? 1 : 0;
    gpiod_line_set_value(dir_line[channel], dir);

    // TODO : take the absolute value of duty_cycle so that only
    // a positive value is ever written to the PWM
    // duty_cycle file .
    if (duty_cycle < 0) {
        duty_cycle = -duty_cycle;
    }

    // TODO : use one of the two sysfs paths based on channel
    const char *path = (channel == 0) ? PWM_CH0_PATH "/duty_cycle" : PWM_CH1_PATH "/duty_cycle";

    // TODO : write duty cycle value
    // Example :
    // char duty_cycle_str [20];
    // snprintf ( duty_cycle_str , sizeof ( duty_cycle_str ) , "% d " ,
    // ( int ) ( duty_cycle * PWM_PERIOD_NS ) ) ;
    char duty_cycle_str[20];
    snprintf(duty_cycle_str, sizeof(duty_cycle_str), "%d", (int)(duty_cycle * PWM_PERIOD_NS));

    // TODO : write duty_cycle_str to the appropriate sysfs
    // duty_cycle file
    if (write_sysfs(path, duty_cycle_str) < 0) {
        return -1;
    }

    return 0;
}

int pwm_cleanup (void)
{
    // TODO : disable channels
    write_sysfs(PWM_CH0_PATH "/enable", "0");
    write_sysfs(PWM_CH1_PATH "/enable", "0");

    // TODO : unexport channels
    write_sysfs(PWM_CH0_CHIP_PATH "/unexport", PWM_CHANNEL);
    write_sysfs(PWM_CH1_CHIP_PATH "/unexport", PWM_CHANNEL);

    // TODO : release the direction lines with
    // gpiod_line_release () and close the chip with
    // gpiod_chip_close ( chip )
    gpiod_line_release(dir_line[0]);
    gpiod_line_release(dir_line[1]);
    gpiod_chip_close(chip);

    return 0;
}