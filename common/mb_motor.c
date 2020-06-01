/*******************************************************************************
* mb_motors.c
*
* Control up to 2 DC motor drivers
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <rc/motor.h>
#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pwm.h>
#include <rc/adc.h>
#include "mb_motor.h"
#include "mb_defs.h"

// preposessor macros
#define unlikely(x) __builtin_expect (!!(x), 0)

// global initialized flag
static int init_flag = 0;


/*******************************************************************************
* int mb_motor_init()
*
* initialize mb_motor with default frequency
*******************************************************************************/
int mb_motor_init()
{
    // define motor PWM output channel 
    rc_pwm_init(1,PWM_FREQ);

    // define motor direction pin
    rc_gpio_init(MDIR1_CHIP, MDIR1_PIN, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(MDIR2_CHIP, MDIR2_PIN, GPIOHANDLE_REQUEST_OUTPUT);
    
    // define motor brake pin
    rc_gpio_init(MOT_BRAKE_EN, GPIOHANDLE_REQUEST_OUTPUT);

    init_flag = 1;

    return mb_motor_init_freq(MB_MOTOR_DEFAULT_PWM_FREQ);
}


/*******************************************************************************
* int mb_motor_init_freq()
*
* set up pwm channels, gpio assignments and make sure motors are left off.
*******************************************************************************/
int mb_motor_init_freq(int pwm_freq_hz)
{
    return 0;
}


/*******************************************************************************
* mb_motor_cleanup()
*
*******************************************************************************/
int mb_motor_cleanup()
{
    if (unlikely(!init_flag)) {
        fprintf(stderr, "ERROR: trying cleanup before motors have been initialized\n");
        return -1;
    }
    mb_motor_disable();

    return 0;
}


/*******************************************************************************
* mb_motor_brake()
*
* allows setting the brake function on the motor drivers
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_motor_brake(int brake_en) {


    if (unlikely(!init_flag)) {
        fprintf(stderr,"ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }

    if (brake_en == 0) {
        rc_gpio_set_value(MOT_BRAKE_EN, LOW);
    } else {
        rc_gpio_set_value(MOT_BRAKE_EN, HIGH);
    }

   return 0;
}


/*******************************************************************************
* int mb_disable_motors()
*
* disables PWM output signals
* returns 0 on success
*******************************************************************************/
int mb_motor_disable()
{
    if (unlikely(!init_flag)) {
        fprintf(stderr,"ERROR: trying to disable motors before motors have been initialized\n");
        return -1;
    }

    mb_motor_set(LEFT_MOTOR, 0.0);
    mb_motor_set(RIGHT_MOTOR, 0.0);

    return 0;
}


/*******************************************************************************
* int mb_motor_set(int motor, double duty)
*
* set a motor direction and power
* motor is from 1 to 2, duty is from -1.0 to +1.0
* uses the defines in mb_defs.h
* returns 0 on success
*******************************************************************************/
int mb_motor_set(int motor, double duty)
{    
    if (unlikely(!init_flag)) {
        fprintf(stderr, "ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }

    if (motor == 1) {
        if (duty >= 0.0) {
         rc_gpio_set_value(MDIR1_CHIP, MDIR1_PIN, HIGH);
        } else {
         rc_gpio_set_value(MDIR1_CHIP, MDIR1_PIN, LOW);
        }

        rc_pwm_set_duty(1, 'A', fabs(duty));

    } else {
        if (duty >= 0.0) {
         rc_gpio_set_value(MDIR2_CHIP, MDIR2_PIN, LOW);
        } else {
         rc_gpio_set_value(MDIR2_CHIP, MDIR2_PIN, HIGH);
        }

        rc_pwm_set_duty(1, 'B', fabs(duty));
    }

    return 0;
}


/*******************************************************************************
* int mb_motor_set_all(double duty)
*
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_motor_set_all(double duty)
{
    if (unlikely(!init_flag)) {
        fprintf(stderr, "ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }

    return 0;
}


/*******************************************************************************
* int mb_motor_read_current(int motor)
*
* returns the measured current in A
*******************************************************************************/
double mb_motor_read_current(int motor) {
    // DRV8801 driver board CS pin puts out 500mV/A
    // adc only take channel number between 0 and 7.   
    // left motor = 1 -> channel = 0, rihgt motor = 2 -> channel = 1
    
    return rc_adc_read_volt(motor - 1) * 2.0;
}


