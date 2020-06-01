#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

#include <stdint.h>     // for uint64_t
#include <stdbool.h>    // for bool

typedef struct mb_state mb_state_t;
struct mb_state {

    uint64_t time_run_ns;	///< time since boot>
    uint64_t time_init; 

    int arm_state;

    // raw sensor inputs
    float   theta;             // body angle (rad)
    float   phi;               // average wheel angle (rad)    
    float   left_phi;
    float   right_phi;
    
    // hading angle filter
    float   psi;
    float   psi_imu;
    float   psi_odometry;
    float   psi_kf;
    float   psi_go;
    float   dpsi;
    float   dpsi_imu;
    float   dpsi_odometry;

    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading
    int     encoder_pre[2];
    float   omega[2];

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    float opti_x;
    float opti_y;
    float x;
    float y;
    float opti_roll;
    float opti_pitch;
    float opti_yaw;

    // control input 
    double u[2];
    double motor_PWM[2]; 

    // autonomous movement segmentation
    int counter;
    
    float temp;
};

mb_state_t mb_state;   // global 

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints {
    float x;
    float y; 

    float theta_d;
    float phi_d;
    float psi_d;
    float psi_instant_d;
    float phi_ref;
    
    int pass[3];
    int psi_flag;
    int phi_flag;

    float fwd_velocity;     // fwd velocity in m/s
    float turn_velocity;    // turn velocity in rad/s
    
    int manual_ctl;  
};

mb_setpoints_t mb_setpoints;  // global 

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry {  
    float x;        // x position from initialization in m
    float y;        // y position from initialization in m
    float psi;      // orientation from initialization in rad
};

mb_odometry_t mb_odometry;  //global

typedef struct mb_dsminput mb_dsminput_t;
struct mb_dsminput {
    int   enter_auto;
    float kill_switch;
    float stick_throttle;   // x velocity from initialization in m
    float stick_heading;   // y velocity from initialization in m
    float stick_mode;      // dsm control / autonomous mode
};

mb_dsminput_t mb_dsminput;  //global

#endif