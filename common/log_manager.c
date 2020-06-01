/**
 * @file log_manager.c
 */


#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <string.h>
#include <math.h>

// to allow printf macros for multi-architecture portability
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>

#include "log_manager.h"
#include "mb_controller.h"
#include "mb_structs.h"

// #define FALSE 0
// #define TRUE 1

static uint64_t num_entries;	// number of entries logged so far
static int buffer_pos;		// position in current buffer
static int current_buf;	// 0 or 1 to indicate which buffer is being filled
static int needs_writing;	// flag set to 1 if a buffer is full
static FILE* fd;		// file descriptor for the log file

// array of two buffers so one can fill while writing the other to file
static log_entry_t buffer[2][BUF_LEN];

// background thread and running flag
static pthread_t pthread;
static int logging_enabled; // set to 0 to exit the write_thread


static int __write_header(FILE* fd)
{	
	fprintf(fd,"last_step_ns");
	fprintf(fd,",Theta(deg)");
	fprintf(fd,",SP_Theta(deg)");
	fprintf(fd,",Phi(rad)");
	fprintf(fd,",SP_Phi(rad)");

	fprintf(fd,",Psi_imu(deg)");
	fprintf(fd,",Psi_odom(deg)");
	fprintf(fd,",Psi_go(deg)");
	fprintf(fd,",Psi_kf(deg)");
	fprintf(fd,",Psi(deg)");
	fprintf(fd,",SP_Psi(deg)");
	fprintf(fd,",SP_instant_Psi(deg)");

	fprintf(fd,",L Enc");
	fprintf(fd,",R Enc");
	fprintf(fd,",L W");
	fprintf(fd,",R W");

	fprintf(fd,",L PWM");
	fprintf(fd,",R PWM");

	fprintf(fd,",X");
	fprintf(fd,",Y");
	
	fprintf(fd,",psi error");
	

	fprintf(fd, "\n");
	return 0;

}


static int __write_log_entry(FILE* fd, log_entry_t e)
{	
	fprintf(fd, "%" PRIu64 "", e.last_step_ns);

	fprintf(fd, ",%.4F", e.theta*180/M_PI);
	fprintf(fd, ",%.4F", e.theta_d*180/M_PI);
	
	fprintf(fd, ",%.4F", e.phi);
	fprintf(fd, ",%.4F", e.phi_d);

	fprintf(fd, ",%.4F", e.psi_imu*180/M_PI);
	fprintf(fd, ",%.4F", e.psi_odometry*180/M_PI);
	fprintf(fd, ",%.4F", e.psi_go*180/M_PI);
	fprintf(fd, ",%.4F", e.psi_kf*180/M_PI);
	fprintf(fd, ",%.4F", e.psi*180/M_PI);
	fprintf(fd, ",%.4F", e.psi_d*180/M_PI);
	fprintf(fd, ",%.4F", e.psi_instant_d*180/M_PI);

	fprintf(fd,	",%d", e.left_encoder);
	fprintf(fd, ",%d", e.right_encoder);
	fprintf(fd, ",%.4F", e.left_speed);
	fprintf(fd, ",%.4F", e.right_speed);

	fprintf(fd, ",%.4F", e.left_PWM);
	fprintf(fd, ",%.4F", e.right_PWM);

	fprintf(fd, ",%.4F", e.opti_x);
	fprintf(fd, ",%.4F", e.opti_y);

	fprintf(fd, ",%.4F", e.temp/M_PI*180);
	
	fprintf(fd, "\n");
	return 0;
}


static void* __log_manager_func(__attribute__ ((unused)) void* ptr)
{
	int i, buf_to_write;
	// while logging enabled and not exiting, write full buffers to disk
	while (rc_get_state()!=EXITING && logging_enabled) {
		if (needs_writing) {
			// buffer to be written is opposite of one currently being filled
			if (current_buf == 0) {
				buf_to_write = 1;
			} else {
				buf_to_write = 0;
			}
			// write the full buffer to disk;
			for (i = 0; i < BUF_LEN; i++) {
				__write_log_entry(fd, buffer[buf_to_write][i]);
			}
			fflush(fd);
			needs_writing = 0;
		}
		rc_usleep(1000000 / LOG_MANAGER_HZ);
	}

	// if program is exiting or logging got disabled, write out the rest of
	// the logs that are in the buffer current being filled
	for (i = 0; i < buffer_pos; i++) {
		__write_log_entry(fd, buffer[current_buf][i]);
	}
	fflush(fd);
	fclose(fd);

	// zero out state
	logging_enabled = 0;
	num_entries = 0;
	buffer_pos = 0;
	current_buf = 0;
	needs_writing = 0;
	return NULL;
}


int log_manager_init()
{
	int i;
	char path[100];
	struct stat st = {0};

	// if the thread if running, stop before starting a new log file
	if (logging_enabled) {
		//fprintf(stderr,"ERROR: in start_log_manager, log manager already running.\n");
		//return -1;
		log_manager_cleanup();
	}

	// first make sure the directory exists, make it if not
	if (stat(LOG_DIR, &st) == -1) {
		mkdir(LOG_DIR, 0755);
	}

	// search for existing log files to determine the next number in the series
	for (i = 1; i <= MAX_LOG_FILES + 1; i++) {
		memset(&path, 0, sizeof(path));
		sprintf(path, LOG_DIR "%d.csv", i);
		// if file exists, move onto the next index
		if (stat(path, &st) == 0) {
			continue;
		} else {
			break;
		}
	}
	// limit number of log files
	if (i == MAX_LOG_FILES + 1) {
		fprintf(stderr,"ERROR: log file limit exceeded\n");
		fprintf(stderr,"delete old log files before continuing\n");
		return -1;
	}
	// create and open new file for writing
	fd = fopen(path, "w+");
	if (fd == 0) {
		printf("ERROR: can't open log file for writing\n");
		return -1;
	}

	// write header
	__write_header(fd);

	// start thread
	logging_enabled = 1;
	num_entries = 0;
	buffer_pos = 0;
	current_buf = 0;
	needs_writing = 0;

	// start logging thread
	if (rc_pthread_create(&pthread, __log_manager_func, NULL, SCHED_FIFO, LOG_MANAGER_PRI) < 0) {
		fprintf(stderr,"ERROR in start_log_manager, failed to start thread\n");
		return -1;
	}
	rc_usleep(1000);
	return 0;
}


static log_entry_t __construct_new_entry()
{
	log_entry_t l;
	
	l.last_step_ns	= mb_state.time_run_ns;


	l.theta =  mb_state.theta;   
	l.theta_d = mb_setpoints.theta_d;        
    l.phi = mb_state.phi;   
	l.phi_d = mb_setpoints.phi_d;   

	// heading angle psi
	l.psi = mb_state.psi;
	l.psi_d = mb_setpoints.psi_d;
	l.psi_instant_d = mb_setpoints.psi_instant_d;
	l.psi_imu = mb_state.psi_imu;
	l.psi_odometry = mb_state.psi_odometry;
	l.psi_go = mb_state.psi_go;
	l.psi_kf = mb_state.psi_kf;
	 

    l.left_encoder = mb_state.left_encoder;      
    l.right_encoder = mb_state.right_encoder;    

    //outputs
    l.left_cmd = mb_state.left_cmd;  
    l.right_cmd = mb_state.right_cmd; 

    l.left_speed = mb_state.omega[0];
	l.right_speed = mb_state.omega[1];
	
	l.opti_x = mb_odometry.x;
    l.opti_y = mb_odometry.y;
    l.opti_roll = mb_state.opti_roll;
    l.opti_pitch = mb_state.opti_pitch;
    l.opti_yaw = mb_state.opti_yaw;

    l.u_psi = mb_state.u[0];
	l.u_theta = mb_state.u[1];

	l.left_PWM = mb_state.motor_PWM[0];
    l.right_PWM = mb_state.motor_PWM[1];

	l.temp = mb_state.temp;

	return l;
}


int log_manager_add_new()
{
	if (!logging_enabled) {
		fprintf(stderr,"ERROR: trying to log entry while logger isn't running\n");
		return -1;
	}
	if (needs_writing && buffer_pos >= BUF_LEN) {
		fprintf(stderr,"WARNING: logging buffer full, skipping log entry\n");
		return -1;
	}
	// add to buffer and increment counters
	buffer[current_buf][buffer_pos] = __construct_new_entry();
	buffer_pos++;
	num_entries++;
	// check if we've filled a buffer
	if (buffer_pos >= BUF_LEN) {
		buffer_pos = 0;		// reset buffer position to 0
		needs_writing = 1;	// flag the writer to dump to disk
		// swap buffers
		if (current_buf==0) current_buf=1;
		else current_buf=0;
	}
	return 0;
}


int log_manager_cleanup()
{
	// just return if not logging
	if (logging_enabled==0) return 0;

	// disable logging so the thread can stop and start multiple times
	// thread also exits on rc_get_state()==EXITING
	logging_enabled=0;
	int ret = rc_pthread_timed_join(pthread,NULL,LOG_MANAGER_TOUT);
	if (ret==1) fprintf(stderr,"WARNING: log_manager_thread exit timeout\n");
	else if (ret==-1) fprintf(stderr,"ERROR: failed to join log_manager thread\n");
	return ret;
}
