/*******************************************************************************
* mb_defs.h
*
*   defines for your bot
*   You will need to fill this in based on the data sheets, schematics, etc.
*      and your specific configuration...
*
*******************************************************************************/
#ifndef MB_DEFS_H
#define MB_DEFS_H

#define M_PI                        3.14159265358979323846264338327950288
#define DEFAULT_PWM_FREQ        25000           // period of motor drive pwm
#define PWM_FREQ                25000
#define LEFT_MOTOR                  1           // id of left motor
#define RIGHT_MOTOR                 2           // id of right motor
#define MDIR1_CHIP                  1           // chip of MDIR1 gpio pin
#define MDIR1_PIN                  28           // MDIRR1 gpio(CHIP.PIN) P9.12
#define MDIR2_CHIP                  1           // chip of MDIR2 gpio pin
#define MDIR2_PIN                  16           // MDIRR2 gpio(CHIP.PIN) P9.15
#define MOT_BRAKE_EN             0,20           // gpio0.20  P9.41
#define MOT_1_POL                   1           // polarity of motor 1
#define MOT_2_POL                  -1           // polarity of motor 2
#define ENC_1_POL                  -1           // polarity of encoder 1
#define ENC_2_POL                   1           // polarity of encoder 2
#define MOT_1_CS                   39           // analog in of motor 1 current sense
#define MOT_2_CS                   40           // analog in of motor 2 current sense
#define GEAR_RATIO                 20.4         // gear ratio of motor
#define ENCODER_RES                48.0         // encoder counts per motor shaft revolution
#define WHEEL_DIAMETER              0.08        // diameter of wheel in meters

#define CL                          1.000236    // correction factor based on UMBmark for left wheel
#define CR                          0.999764    // correction factor based on UMBmark for right wheel
#define WHEEL_BASE                  0.20495     // actual wheel separation distance in meters (corrected base on UMBmark)

#define FWD_VEL_SENSITIVITY         0.1         // sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY        0.1         // sensitivity of RC control for turning
#define SAMPLE_RATE_HZ            100           // main filter and control loop speed
#define DT                          0.01        // 1/sample_rate
#define PRINTF_HZ                  10           // rate of print loop
#define RC_CTL_HZ                  30           // rate of RC data update
#define ENC_LEFT_POL               -1
#define ENC_RIGHT_POL               1
#define HIGH                        1
#define LOW                         0
#define VOLTAGE                    12.0
#define NOMINAL_VOLTAGE             4.3   

#define LEFT_TAU_S                 -0.4968
#define LEFT_W_NL                 -43.7975
#define RIGHT_TAU_S                 0.7144
#define RIGHT_W_NL                 43.3766


#define GPIO_INT_PIN_CHIP           3
#define GPIO_INT_PIN_PIN           21
#define FEEDBACK_HZ               100
#define I2C_BUS                     2

#endif
