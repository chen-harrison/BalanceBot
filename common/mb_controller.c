#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <ctype.h>

#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/quaternion.h>
#include <rc/math/other.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/servo.h>
#include <rc/time.h>

#include "mb_controller.h"
#include "mb_defs.h"
#include "mb_motor.h"
#include "log_manager.h"
#include "mb_odometry.h"


// filters
static rc_filter_t PID_theta= RC_FILTER_INITIALIZER;
static rc_filter_t PID_phi	= RC_FILTER_INITIALIZER;
static rc_filter_t PID_phi_auto	= RC_FILTER_INITIALIZER;
static rc_filter_t PID_psi  = RC_FILTER_INITIALIZER;
static rc_kalman_t kf_psi  = RC_KALMAN_INITIALIZER;


static int __heading_init(void)
{
    rc_matrix_t F   = RC_MATRIX_INITIALIZER;
    rc_matrix_t G   = RC_MATRIX_INITIALIZER;
    rc_matrix_t H   = RC_MATRIX_INITIALIZER;
    rc_matrix_t Q   = RC_MATRIX_INITIALIZER;
    rc_matrix_t R   = RC_MATRIX_INITIALIZER;
    rc_matrix_t Pi  = RC_MATRIX_INITIALIZER;

    const int Nx = 1;
	const int Ny = 1;
	const int Nu = 1;

	// allocate appropirate memory for system
	rc_matrix_zeros(&F, Nx, Nx);
	rc_matrix_zeros(&G, Nx, Nu);
	rc_matrix_zeros(&H, Ny, Nx);
	rc_matrix_zeros(&Q, Nx, Nx);
	rc_matrix_zeros(&R, Ny, Ny);
	rc_matrix_zeros(&Pi, Nx, Nx);

	// define system -DT; // accel bias
	F.d[0][0] = 1.0;
	G.d[0][0] = 1.0;
	H.d[0][0] = 1.0;

    // TODO: determine initial Q,R,Pi
	// covariance matrices
	Q.d[0][0] = 0.000000001;

    R.d[0][0] = 1000000.0;

	// initial P, cloned from converged P while running
	Pi.d[0][0] = 0.0001;

	// initialize the kalman filter
	if (rc_kalman_alloc_lin(&kf_psi, F, G, H, Q, R, Pi) == -1) {
        return -1;
    }
	rc_matrix_free(&F);
	rc_matrix_free(&G);
	rc_matrix_free(&H);
	rc_matrix_free(&Q);
	rc_matrix_free(&R);
	rc_matrix_free(&Pi);

	return 0;
}


float mb_heading_kf_march(void)
{
	static rc_vector_t u = RC_VECTOR_INITIALIZER;
	static rc_vector_t y = RC_VECTOR_INITIALIZER;

	// do first-run filter setup
	if (kf_psi.step == 0) {
        rc_vector_zeros(&u, 1);
        rc_vector_zeros(&y, 1);
		kf_psi.x_est.d[0] = mb_state.psi_odometry;
	}

	u.d[0] = mb_state.dpsi_odometry;
	y.d[0] = mb_state.psi_imu;

	rc_kalman_update_lin(&kf_psi, u, y);

	// heading angle estimate
	mb_state.psi_kf  = mb_clamp_radians(kf_psi.x_est.d[0]);

	return mb_state.psi_kf;
}


float mb_heading_go_march()
{
    if (fabs(mb_state.dpsi_imu - mb_state.dpsi_odometry) < STEADY_THRESH) {
        mb_state.dpsi = mb_state.dpsi_odometry;
    } else {
        mb_state.dpsi = mb_state.dpsi_imu;
    }

    mb_state.psi_go += mb_state.dpsi;
    mb_state.psi_go = mb_clamp_radians(mb_state.psi_go);

    return mb_state.psi_go;
}


/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
*
* TODO: figure out High Frequency roll off time constant, Tf, in the PID controller
* TODO: position controller
*
*
* return 0 on success
*
*******************************************************************************/
int mb_controller_init()
{
	double* gain;

    printf("Loading gain table file\n");
    gain = mb_controller_load_config(CFG_PATH);

    // initialize theta PID controller
    if (rc_filter_pid(&PID_theta,gain[0], gain[1], gain[2], 4 * DT, DT)) {
    	printf("ERROR: failed to alloc pitch control PD filter\n");
		return -1;
	}

    rc_filter_enable_saturation(&PID_theta,	-1, 1);
    rc_filter_enable_soft_start(&PID_theta, SOFT_START_SECONDS);
    printf("Pitch control: kp = %f, ki = %f, kd = %f \n", gain[0], gain[1], gain[2]);

    // initialize phi PID controller for manual mode
    if (rc_filter_pid(&PID_phi, gain[3], gain[4], gain[5], 4 * DT, DT)) {
    	printf("ERROR: failed to alloc pitch control PD filter\n");
		return -1;
	}

    rc_filter_enable_saturation(&PID_phi, -MAX_PITCH_COMPONENT, MAX_PITCH_COMPONENT);
    rc_filter_enable_soft_start(&PID_phi, SOFT_START_SECONDS);
    printf("Phi control: kp = %f, ki = %f, kd = %f \n", gain[4],gain[5],gain[6]);

    // initialize phi PID controller for autonomous mode
    if (rc_filter_pid(&PID_phi_auto,gain[9], gain[10], gain[11], 4 * DT, DT)) {
    	printf("ERROR: failed to alloc pitch control PD filter\n");
		return -1;
	}

    rc_filter_enable_saturation(&PID_phi_auto, -MAX_PITCH_COMPONENT, MAX_PITCH_COMPONENT);
    rc_filter_enable_soft_start(&PID_phi_auto, SOFT_START_SECONDS);
    printf("Phi Auto control: kp = %f, ki = %f, kd = %f\n", gain[9], gain[10], gain[11]);

    // initialize heading PID controller
    if (rc_filter_pid(&PID_psi, gain[6], gain[7], gain[8], 4 * DT, DT)) {
    	printf("ERROR: failed to alloc pitch control PD filter\n");
		return -1;
	}

    rc_filter_enable_saturation(&PID_psi, -1, 1);
    rc_filter_enable_soft_start(&PID_psi, SOFT_START_SECONDS);
    printf("Heading controll: kp = %f, ki = %f, kd = %f\n", gain[6], gain[7], gain[8]);

    // initialize heading kalman filter (state estimation of psi)
    __heading_init();

    // initialize mb_state parameters
    mb_state.encoder_pre[0] = 0;
    mb_state.encoder_pre[1] = 0;
    mb_state.omega[0] = 0;
    mb_state.omega[1] = 0;

    mb_state.phi = 0.0;

    // initialize mb_setpoints parameters
    mb_setpoints.phi_d = mb_state.phi;
    mb_setpoints.psi_d = mb_state.psi;
    mb_setpoints.theta_d = VERICAL_PITCH;   // want the bot to maintain vertical stance

    mb_dsminput.enter_auto = 0;
    mb_setpoints.pass[0] = 0;
    mb_setpoints.pass[1] = 0;
    mb_setpoints.pass[2] = 0;
    mb_setpoints.pass[3] = 0;

    return 0;
}


/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
double* mb_controller_load_config(char* path)
{
    FILE* file = fopen(path, "r");
    static double M[6];

    if (file == NULL) {
        printf("Error opening %s\n", path);
    }

    fscanf(file, "%lf%*c %lf%*c %lf%*c %lf%*c %lf%*c %lf%*c %lf%*c %lf%*c %lf%*c %lf%*c %lf%*c %lf%*c",
           &M[0], &M[1], &M[2], &M[3], &M[4], &M[5], &M[6], &M[7], &M[8], &M[9], &M[10], &M[11]);

    fclose(file);
    return M;
}


/*******************************************************************************
* int mb_controller_update()
*
*
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/
int mb_controller_update(mb_state_t* mb_state)
{
    // heading control (error psi -> PWM)
    if (fabs(mb_setpoints.psi_d - mb_state->psi) > M_PI) {
        mb_state->u[0] = rc_filter_march(&PID_psi, (mb_setpoints.psi_d - mb_state->psi) + sgn(mb_state->psi)* 2.0 * M_PI);
    } else {
        mb_state->u[0] = rc_filter_march(&PID_psi, mb_setpoints.psi_d - mb_state->psi);
    }

    mb_setpoints.theta_d = rc_filter_march(&PID_phi, mb_setpoints.phi_d - mb_state->phi);

    // inner loop control (error theta -> PWM)
    mb_state->u[1] = rc_filter_march(&PID_theta, -mb_setpoints.theta_d - mb_state->theta);

    //send PWM signal to motor
    mb_combined_PWM(mb_state->u[0], mb_state->u[1]);

    // write mb_state to the log
    log_manager_add_new();

    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
*
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/
int mb_controller_cleanup()
{
    rc_filter_free(&PID_theta);
    rc_filter_free(&PID_phi);
    rc_filter_free(&PID_phi_auto);
    rc_filter_free(&PID_psi);

    return 0;
}


int mb_direct_PWM(double u)
{
    u = 0 * mb_deadband_punch(u);

    mb_state.motor_PWM[0] = u;
    mb_state.motor_PWM[1] = u;

    mb_motor_set(LEFT_MOTOR, mb_state.motor_PWM[0]);
    mb_motor_set(RIGHT_MOTOR, mb_state.motor_PWM[1]);

    return 0;
}


int mb_combined_PWM(double u_psi, double u_theta)
{
    mb_state.motor_PWM[0] = u_theta + u_psi;
    mb_state.motor_PWM[1] = u_theta - u_psi;

    rc_saturate_double(&mb_state.motor_PWM[0], -1, 1);
    rc_saturate_double(&mb_state.motor_PWM[1], -1, 1);

    mb_motor_set(LEFT_MOTOR, mb_state.motor_PWM[0]);
    mb_motor_set(RIGHT_MOTOR, mb_state.motor_PWM[1]);

    return 0;
}


double mb_deadband_punch(double u)
{
    if (fabs(u) <= DEADBAND_VAL) {
        u = 0;
    } else if (u > DEADBAND_VAL) {
        u = u + PUNCH_VAL;
    } else {
        u = u - PUNCH_VAL;
    }

    return u;
}


void mb_auto_update_setpoint_openloop()
{
    switch (mb_state.counter) {
    case 0:
    // segment 1
        if ((mb_setpoints.psi_d - (-M_PI / 2.0) > 0.04) && (mb_setpoints.psi_flag == 0)) {
            mb_setpoints.psi_d += -0.02;
        } else if ((mb_setpoints.psi_d - (-M_PI / 2.0) < -0.04) && (mb_setpoints.psi_flag == 0)) {
            mb_setpoints.psi_d += 0.02;
        } else if (mb_setpoints.psi_flag == 0) {
            mb_setpoints.psi_flag = 1;
        }

        if ((mb_setpoints.phi_ref - mb_setpoints.phi_d > 0.1) && (mb_setpoints.psi_flag == 1)) {
            mb_setpoints.phi_d += 0.02;
        } else if ((mb_setpoints.phi_ref - mb_setpoints.phi_d < -0.1) && (mb_setpoints.psi_flag == 1)) {
            mb_setpoints.phi_d += -0.02;
        } else if (mb_setpoints.psi_flag == 1) {
            mb_setpoints.phi_ref += 25.0;
            mb_setpoints.psi_flag = 0;
            mb_state.counter ++;
        }
        break;

    case 1:
    // segment 2
        if (mb_clamp_radians((mb_setpoints.psi_d - (-M_PI)) > 0.04) && (mb_setpoints.psi_flag == 0)) {
            mb_setpoints.psi_d += -0.02;
        } else if (mb_clamp_radians((mb_setpoints.psi_d - (-M_PI)) < -0.04) && (mb_setpoints.psi_flag == 0)) {
            mb_setpoints.psi_d += 0.02;
        } else if (mb_setpoints.psi_flag == 0) {
            mb_setpoints.psi_flag = 1;
        }

        if ((mb_setpoints.phi_ref - mb_setpoints.phi_d > 0.1) && (mb_setpoints.psi_flag == 1)) {
            mb_setpoints.phi_d += 0.03;
        } else if ((mb_setpoints.phi_ref - mb_setpoints.phi_d < -0.1) && (mb_setpoints.psi_flag == 1)) {
            mb_setpoints.phi_d += -0.03;
        } else if (mb_setpoints.psi_flag == 1) {
            mb_setpoints.phi_ref += 25.0;
            mb_setpoints.psi_flag = 0;
            mb_state.counter ++;
        }
        break;

    case 2:
    // segment 3
        if (mb_clamp_radians((mb_setpoints.psi_d - (M_PI / 2.0)) > 0.04) && (mb_setpoints.psi_flag == 0)) {
            mb_setpoints.psi_d += -0.02;
        } else if (mb_clamp_radians((mb_setpoints.psi_d - (M_PI / 2.0)) < -0.04) && (mb_setpoints.psi_flag == 0)) {
            mb_setpoints.psi_d += 0.02;
        } else if (mb_setpoints.psi_flag == 0) {
            mb_setpoints.psi_flag = 1;
        }

        if ((mb_setpoints.phi_ref - mb_setpoints.phi_d > 0.1) && (mb_setpoints.psi_flag == 1)) {
                mb_setpoints.phi_d += 0.03;
        } else if ((mb_setpoints.phi_ref - mb_setpoints.phi_d < -0.1) && (mb_setpoints.psi_flag == 1)) {
            mb_setpoints.phi_d += -0.03;
        } else if (mb_setpoints.psi_flag == 1) {
            mb_setpoints.phi_ref += 25.0;
            mb_setpoints.psi_flag = 0;
            mb_state.counter ++;
        }
        break;

    case 3:
    // segment 4
        if (mb_clamp_radians((mb_setpoints.psi_d - (0.0)) > 0.04) && (mb_setpoints.psi_flag == 0)) {
            mb_setpoints.psi_d += -0.02;
        } else if (mb_clamp_radians((mb_setpoints.psi_d - (0.0)) < -0.04) && (mb_setpoints.psi_flag == 0)) {
            mb_setpoints.psi_d += 0.02;
        } else if (mb_setpoints.psi_flag == 0) {
            mb_setpoints.psi_flag = 1;
        }

        if ((mb_setpoints.phi_ref - mb_setpoints.phi_d > 0.1) && (mb_setpoints.psi_flag == 1)) {
                mb_setpoints.phi_d += 0.03;
        } else if ((mb_setpoints.phi_ref - mb_setpoints.phi_d < -0.1) && (mb_setpoints.psi_flag == 1)) {
            mb_setpoints.phi_d += -0.03;
        } else if (mb_setpoints.psi_flag == 1) {
            // mb_setpoints.phi_ref += 15.0;
            mb_setpoints.psi_flag = 0;
            mb_state.counter ++;
            printf("open loop completed \n");
        }
        break;
    }
}


void mb_auto_update_setpoint_closeloop()
{
    float dis;
    if ((mb_setpoints.psi_flag == 0) && (mb_setpoints.psi_instant_d - mb_state.psi > HEADING_THRESHOLD)) {
        mb_setpoints.psi_d += -0.02;
        mb_setpoints.psi_instant_d = atan2(-(mb_state.y - mb_setpoints.y), -(mb_state.x - mb_setpoints.x));
    } else if (mb_clamp_radians((mb_setpoints.psi_d - (0.0)) < -0.04) && (mb_setpoints.psi_flag == 0)) {
        mb_setpoints.psi_d += 0.02;
        mb_setpoints.psi_instant_d = atan2(-(mb_state.y - mb_setpoints.y), -(mb_state.x - mb_setpoints.x));
    } else if (mb_setpoints.psi_flag == 0) {
        // update segment indicator
        mb_state.counter++;
        mb_setpoints.psi_flag = 1;
    }

    if ((mb_setpoints.psi_flag == 1) && (sqrt(pow(mb_state.y - mb_setpoints.y, 2) + pow(mb_state.x - mb_setpoints.x, 2)) < DISTANCE_BUFFER)) {
        switch (mb_state.counter)
        {
            case 1:     // finished the first segment
        	    mb_setpoints.y = mb_state.y - SEGMENT_LENGTH;
			    mb_setpoints.psi_d =  atan2(-(mb_state.y - mb_setpoints.y), -(mb_state.x - mb_setpoints.x));
                mb_setpoints.psi_flag = 0;
                break;
            case 2:     // finished the second segment
                mb_setpoints.x = mb_state.x- SEGMENT_LENGTH;
			    mb_setpoints.psi_d =  atan2(-(mb_state.y - mb_setpoints.y), -(mb_state.x - mb_setpoints.x));
                mb_setpoints.psi_flag = 0;
                break;
            case 3:     // finished the third segment
        	    mb_setpoints.y = mb_state.y + SEGMENT_LENGTH;
			    mb_setpoints.psi_d = atan2(-(mb_state.y - mb_setpoints.y), -(mb_state.x - mb_setpoints.x));
                mb_setpoints.psi_flag = 0;
                break;
            default:    // finished all segments
                mb_motor_set(LEFT_MOTOR, 0);
                mb_motor_set(RIGHT_MOTOR, 0);
                break;

            printf("changed heading setpoint \n");
        }
	}
}


void mb_auto_update_psi_closeloop()
{
    float dis;
    if ((mb_setpoints.psi_flag == 0) && (fabs(mb_setpoints.psi_d - mb_state.psi) <= HEADING_THRESHOLD)) {
        dis = sqrt(pow(mb_state.y - mb_setpoints.y, 2) + pow(mb_state.x - mb_setpoints.x, 2)) / (WHEEL_DIAMETER / 2.0);
        mb_setpoints.phi_d = dis + mb_state.phi;
        mb_setpoints.psi_flag = 1;
        printf("changed coordinate setpoint\n");

        // update segment indicator
        mb_state.counter++;
    }
}


void mb_auto_debug()
{
    if ((mb_setpoints.psi_flag == 0) && (mb_setpoints.psi_instant_d - mb_state.psi > 0.04)) {
        if ((mb_setpoints.psi_instant_d*mb_setpoints.psi_d) < 0) {
            mb_setpoints.psi_d += -0.02;
        } else {
            mb_setpoints.psi_d += 0.02;
        }
        mb_setpoints.psi_instant_d = atan2(-(mb_state.y - mb_setpoints.y), -(mb_state.x - mb_setpoints.x));
    } else if ((mb_setpoints.psi_flag == 0) && (mb_setpoints.psi_instant_d - mb_state.psi < -0.04)) {
        mb_setpoints.psi_d += -0.02;
        mb_setpoints.psi_instant_d = atan2(-(mb_state.y - mb_setpoints.y), -(mb_state.x - mb_setpoints.x));
    } else if (mb_setpoints.psi_flag == 0) {
        mb_setpoints.psi_d = mb_setpoints.psi_instant_d;

        // update segment indicator
        mb_setpoints.psi_flag = 1;
        mb_setpoints.phi_flag = 0;
    }

    if ((mb_setpoints.psi_flag == 1) && (sqrt(pow(mb_state.y - mb_setpoints.y, 2) + pow(mb_state.x - mb_setpoints.x, 2)) > DISTANCE_BUFFER) && (mb_setpoints.phi_flag == 0)) {
        mb_setpoints.phi_d += 0.05;
        mb_setpoints.psi_d = atan2(-(mb_state.y-mb_setpoints.y), -(mb_state.x-mb_setpoints.x));
    } else if ((mb_setpoints.psi_flag == 1) && (sqrt(pow(mb_state.y - mb_setpoints.y, 2) + pow(mb_state.x - mb_setpoints.x, 2)) < DISTANCE_BUFFER) && (mb_setpoints.phi_flag == 0)) {
        switch (mb_state.counter % 4)
        {
        case 0:
            mb_setpoints.y = mb_setpoints.y - SEGMENT_LENGTH;
            break;
        case 1:
            mb_setpoints.x = mb_setpoints.x - SEGMENT_LENGTH;
            break;
        case 2:
            mb_setpoints.y = mb_setpoints.y + SEGMENT_LENGTH;
            break;
        case 3:
            mb_setpoints.x = mb_setpoints.x + SEGMENT_LENGTH;
            break;
        }
        mb_setpoints.psi_instant_d = atan2(-(mb_state.y - mb_setpoints.y), -(mb_state.x - mb_setpoints.x));
        mb_setpoints.psi_flag = 0;
        mb_setpoints.phi_flag = 1;
        mb_state.counter ++;
    }
}


void mb_drag_race()
{
    float temp;
    if ((11.5 / (WHEEL_DIAMETER / 2) - mb_state.phi < 0.25 / (WHEEL_DIAMETER / 2)) && (mb_setpoints.phi_flag == 0)) {
        mb_setpoints.phi_flag = 1;
    } else if (!mb_setpoints.phi_flag) {
        if (mb_state.phi / (11.5 / (WHEEL_DIAMETER / 2)) < 0.3) {
            mb_setpoints.phi_d += 0.24;
        } else if (mb_state.phi / (11.5 / (WHEEL_DIAMETER / 2)) < 0.85) {
            mb_setpoints.phi_d += 0.28;    
        } else {
            mb_setpoints.phi_d += 0.15;
        }  
        mb_setpoints.psi_d += -0.00015;
    }
}


int sgn(double v)
{
    return (v < 0) ? -1 : ((v > 0) ? 1 : 0);
}
