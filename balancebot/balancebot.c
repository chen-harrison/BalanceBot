/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
* 
*******************************************************************************/

#include <math.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>

#include "balancebot.h"
#include "../common/log_manager.h"



int main()
{
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if (rc_kill_existing_process(2.0) < -2) return -1;

	// start signal handler so we can exit cleanly
    if (rc_enable_signal_handler()==-1) {
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if (rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0) {
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }

	// initialize enocders
    if (rc_encoder_eqep_init()==-1) {
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if (rc_adc_init()==-1) {
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

	// initialize dsm receiver
    if (rc_dsm_init()==-1) {
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}
	
	printf("initializing xbee... \n");
	// initalize XBee Radio
	int baudrate = BAUDRATE;
	if (XBEE_init(baudrate)==-1) {
		fprintf(stderr,"Error initializing XBee\n");
		return -1;
	};

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, NULL, SCHED_OTHER, 0);

	// start dsm control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, NULL, SCHED_FIFO, 50);


	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;

	// now set up the imu for dmp interrupt operation
	if (rc_mpu_initialize_dmp(&mpu_data, mpu_config)) {
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	rc_nanosleep(3E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_controller_init();

	printf("initializing motors...\n");
	mb_motor_init();

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0,0.0);

	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("initializing log manager... \n");
	log_manager_init();

	// disarm brake in the motor
	mb_motor_brake(1);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 
	
	// keep looping until state changes to EXITING
	while (rc_get_state()!=EXITING) {

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}
	
	// exit cleanly
	rc_mpu_power_off();
	mb_controller_cleanup();
	mb_motor_cleanup();
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST 
	return 0;
}


/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*
*******************************************************************************/
void balancebot_controller()
{	

	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	
	// read time since boot
	mb_state.time_run_ns = rc_nanos_since_boot();
	// read IMU
	mb_state.theta = mpu_data.dmp_TaitBryan[0] + VERICAL_PITCH;
	mb_state.dpsi_imu = -mpu_data.dmp_TaitBryan[2] - mb_state.psi_imu;
	mb_state.psi_imu = -mpu_data.dmp_TaitBryan[2];
	
	// read encoders
	mb_state.left_encoder = rc_encoder_eqep_read(1);
	mb_state.right_encoder = rc_encoder_eqep_read(2);

	// calculate phi 
	mb_state.left_phi =  CL * ((mb_state.left_encoder)/ ENCODER_RES ) * 2.0 * M_PI / GEAR_RATIO;
    mb_state.right_phi = CR * ((mb_state.right_encoder)/ ENCODER_RES ) * 2.0 * M_PI / GEAR_RATIO;
    mb_state.phi = (mb_state.right_phi + (-mb_state.left_phi))/2.0;
	
	// calculate wheel speed 
	mb_state.omega[0] = ((mb_state.left_encoder-mb_state.encoder_pre[0])/ ENCODER_RES ) * 2.0 * M_PI / DT / GEAR_RATIO;
	mb_state.omega[1] = -( (mb_state.right_encoder-mb_state.encoder_pre[1])/ ENCODER_RES ) * 2.0 * M_PI / DT / GEAR_RATIO;
	
	// Update odometry 
	mb_odometry_update(&mb_odometry, &mb_state);

	// update heading angle psi with filtering the sensor reading 
    //mb_state.psi = mb_heading_go_march();       // Gyrodometry Filtering
    mb_state.psi = mb_heading_kf_march();       // Kalman Filtering 
	//mb_state.psi = mb_state.psi_imu;

	mb_state.encoder_pre[0] = mb_state.left_encoder;
	mb_state.encoder_pre[1] = mb_state.right_encoder;

	if (rc_get_state()!=EXITING) {	// controller engages only when the state is at RUNNING 
    	
		// Calculate controller outputs
    	if (!mb_setpoints.manual_ctl) {
			//mb_auto_update_setpoint_openloop();
			//mb_auto_update_psi_closeloop();
			//mb_auto_update_setpoint_closeloop();
			mb_auto_debug();
			//mb_drag_race();
		}

    	if (mb_setpoints.manual_ctl) {
    		// do nothing just chill
   		}

		mb_controller_update(&mb_state);
		
	}

/*
	XBEE_getData();
	double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
	double tb_array[3] = {0, 0, 0};
	rc_quaternion_to_tb_array(q_array, tb_array);
	mb_state.opti_x = xbeeMsg.x;
	mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_roll = tb_array[0];
	mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
*/	
	
   	//unlock state mutex
    pthread_mutex_unlock(&state_mutex);
}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr)
{
	while (true) {
		if (rc_dsm_is_new_data()) {
				// TODO: handle the DSM data from the Spektrum radio reciever
				// you may should implement switching between manual and autonomous mode
				// using channel 5 of the DSM data.

				mb_dsminput.stick_mode = rc_dsm_ch_normalized(DSM_CH_MODE);
				mb_dsminput.kill_switch = rc_dsm_ch_normalized(DSM_CH_KILL);
				mb_dsminput.stick_throttle = rc_dsm_ch_normalized(DSM_CH_THROTTLE);
				mb_dsminput.stick_heading = rc_dsm_ch_normalized(DSM_CH_HEADING) / 130.0;
		}		
		
		if (fabs(mb_dsminput.stick_mode) < 0.5) {		// manual control 		
			
			// setpoint updated by dsm stick input  
			mb_setpoints.manual_ctl = 1;
				
			if (fabs(mb_dsminput.stick_throttle) > 0.05) {
				mb_setpoints.fwd_velocity = mb_dsminput.stick_throttle * MAX_FORWARD_VELOCITY;
			} else {
				mb_setpoints.fwd_velocity = 0;
			}

			mb_setpoints.phi_d = mb_setpoints.phi_d + mb_setpoints.fwd_velocity * DT / (WHEEL_DIAMETER / 2.0);
			

			if (fabs(mb_dsminput.stick_heading) > 0.05) {
				mb_setpoints.turn_velocity = -mb_dsminput.stick_heading * MAX_TURN_VELOCITY;	
			} else {
				mb_setpoints.turn_velocity = 0;
			}
			mb_setpoints.psi_d = mb_setpoints.psi_d + mb_setpoints.turn_velocity * DT;
			mb_setpoints.psi_d = mb_clamp_radians(mb_setpoints.psi_d);
		} 
			
		if ((mb_dsminput.stick_mode < -0.5) && (mb_dsminput.enter_auto == 0)) {
			// autonomous motion
			mb_setpoints.manual_ctl = 0;
			mb_state.time_init = rc_nanos_since_boot();

			mb_setpoints.phi_d = mb_state.phi;
			mb_setpoints.psi_d = mb_state.psi;

		
			mb_setpoints.psi_flag = 0;
			mb_setpoints.phi_flag = 0;
			mb_setpoints.phi_ref = mb_state.phi + 25.0;
			
			// update desired coordinate 
			mb_state.counter = 0;		
			
			mb_setpoints.x = (0.0 + SEGMENT_LENGTH);
			
			// UNCOMMENT IF DRAG RACE
			//mb_setpoints.x += 10.5;				
			mb_setpoints.y = 0.0;

			//mb_setpoints.y = 0.0; 
			//printf("UMBmark start here\n");
			//mb_state.u[0] = 0.2;
			//mb_state.u[1] = 0.7;
			mb_setpoints.psi_instant_d = atan2(-(mb_state.y - mb_setpoints.y), -(mb_state.x - mb_setpoints.x));

			mb_dsminput.enter_auto = 1;
			
			//rc_encoder_eqep_write(1, 0);
			//rc_encoder_eqep_write(2, 0);
		} 	
			
		if (mb_dsminput.stick_mode > 0.5) {
			rc_set_state(EXITING); 
		}

		//printf("mode: %f | forward: %f | turn: %f \n", mode,forward,turn);
	

		//mb_setpoints.theta_d = VERICAL_PITCH; 
		//printf("\n");
		rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	
	return NULL;
}


/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr)
{
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING) {
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if (new_state==RUNNING && last_state!=RUNNING) {
			printf("\nRUNNING: Hold upright to balance.\n");
			//printf("                 SENSORS               |            MOCAP            | tau |");
			printf("\n");
			//printf("ARM STATE|");
			printf(" θ(deg)  |");
			printf("SP θ(deg)|");
			//printf(" L φ(rad)|");
			//printf(" R φ(rad)|");
			printf("  φ(rad) |");
			printf("Sp φ(rad)|");
			printf("  ψ(deg) |");
			printf("Sp ψ(deg)|");
			//printf("  L Enc  |");
			//printf("  R Enc  |");
			//printf("  L W    |");
			//printf("  R W    |");
			printf("   X(m)  |");
			printf(" SP X(m) |");
			printf("   Y(m)  |");
			printf(" SP Y(m) |");
			printf("  L u  |");
			printf("  R u  |");
			printf("phi flg|");
			printf("psi flg|");
			//printf("  PWM_L  |");
			//printf("  PWM_R  |");
			//printf("IMU yaw(deg)|");

			printf("\n");
		}
		else if (new_state==PAUSED && last_state!=PAUSED) {
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if (new_state == RUNNING) {
			printf("\r");
			// add print statements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%8.3f  |", mb_state.theta * 180 / M_PI);
			printf("%8.3f  |", mb_setpoints.theta_d * 180 / M_PI);
			//printf("%8.3f  |", mb_state.left_phi);
			//printf("%8.3f  |", mb_state.right_phi);
			printf("%8.3f  |", mb_state.phi);
			printf("%8.3f  |", mb_setpoints.phi_d);
			printf("%8.3f  |", mb_odometry.psi * 180 / M_PI);
			printf("%8.3f  |", mb_setpoints.psi_d * 180 / M_PI);
			//printf("%7d  |", mb_state.left_encoder);
			//printf("%7d  |", mb_state.right_encoder);
			//printf("%7.3f  |", mb_state.omega[0]);
			//printf("%7.3f  |", mb_state.omega[1]);
			printf("%7.3f  |", mb_odometry.x);
			printf("%7.3f  |", mb_setpoints.x);
			printf("%7.3f  |", mb_odometry.y);
			printf("%7.3f  |", mb_setpoints.y);
			printf("%7.2f|", mb_state.u[1]+ mb_state.u[0]);
			printf("%7.2f|", mb_state.u[1]- mb_state.u[0]);
			printf("%3d   |", mb_setpoints.phi_flag);
			printf("%3d   |", mb_setpoints.psi_flag);
			printf("%7f   |",rc_adc_batt());
			//printf("%9.2f|", mb_state.motor_PWM[0]);
			//printf("%9.2f|", mb_state.motor_PWM[1]);
			//printf("%12.2f|", mb_state.psi*180/M_PI);
			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
} 
