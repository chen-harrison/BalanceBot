#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H

#include "mb_structs.h"

// pid files
#define CFG_PATH        "/home/debian/balancebot-w20/bin/pid.csv"

// log files 
#define LOG_DIR		    "/home/debian/balancebot-w20/mb_logs/"
#define MAX_LOG_FILES   	500
#define BUF_LEN		        20
#define LOG_MANAGER_HZ		20
#define LOG_MANAGER_TOUT	2.0
#define LOG_MANAGER_PRI     52

// user control parameters
#define MANUAL_CONTROL      1       // 1 = manual control, 0 = autonomous 
#define MAX_PITCH_SETPOINT	0.2     // rad
#define SOFT_START_SECONDS	1.0     // controller soft start seconds
#define VERICAL_PITCH       0.012   // rad, desired vertical pitch position

// deadband and punch,  controller absolute limits
#define MAX_PITCH_COMPONENT 0.523   // max Torque [Nm]
#define DEADBAND_VAL        0.05
#define PUNCH_VAL           0.02  

// dsm control 
#define DSM_CH_KILL             1
#define DSM_CH_MODE             5
#define DSM_CH_HEADING          4
#define DSM_CH_THROTTLE         3

// heading state filtering thresholds
#define STEADY_THRESH       0.005
#define OBSTACLE_THRESH     0.05

// manual mode control 
#define MAX_FORWARD_VELOCITY    2.0     // m/s
#define MAX_TURN_VELOCITY       7.5     // rad/s  

//autonomous mode control 
#define AUTO_FORWARD_VELOCITY   0.2     // m/s
#define AUTO_PHI_INCREMENT      6.5
#define SEGMENT_LENGTH          1
#define HEADING_THRESHOLD       0.03    // 2.5 deg
#define DISTANCE_BUFFER         0.075   // m   
#define COMPENSATION_LENGTH     0.05

// PID control 
int mb_controller_init();
double* mb_controller_load_config(char*);
int mb_controller_update(mb_state_t* mb_state);
int mb_controller_cleanup();

// actuator control   
int mb_direct_PWM(double);
int mb_combined_PWM(double,double);
double mb_deadband_punch(double u);

// state estimation 
float mb_heading_go_march();
float mb_heading_kf_march();

// autonomous setpoints setting 
void mb_auto_update_setpoint_openloop();
void mb_auto_update_psi_closeloop();
void mb_auto_update_setpoint_closeloop();
void mb_auto_debug();
void mb_drag_race(); 

int sgn(double v); 

#endif

