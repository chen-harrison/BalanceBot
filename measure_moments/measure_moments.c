/*******************************************************************************
* measure_moments.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the moments of inertia of your Balancebot
*
* TODO: capture the gyro data and timestamps to a file to determine the period.
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/mpu.h>
#include "../common/mb_defs.h"

FILE* fp;
FILE* f1;
static rc_mpu_data_t mpu_data;


// declear callback funciton
static void __print__data(void);


// write function
int writeMatrixToFile(FILE *fp, double* matrix, int height, int width) {
    if (fp == NULL) {
        return 1;
    }

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (j > 0) {
                fputc(',', fp);
            }
            fprintf(fp, "%lf", matrix[i*width +j]);
        }
        fputs("\r\n", fp);
    }

    return 0;
}


// define callback function
static void __print__data(void) {
    rc_mpu_read_accel(&mpu_data);
    rc_mpu_read_gyro(&mpu_data);


    printf("%6.2f %6.2f %6.2f |", mpu_data.dmp_TaitBryan[1] * 180 / M_PI, -mpu_data.dmp_TaitBryan[0] * 180 / M_PI, -mpu_data.dmp_TaitBryan[2] * 180 / M_PI);
    printf("%6.2f %6.2f %6.2f", -mpu_data.accel[1], -mpu_data.accel[0], -mpu_data.accel[2]);
    printf("%6.1f %6.1f %6.1f\r", mpu_data.gyro[1], mpu_data.gyro[0], mpu_data.gyro[2]);

    double buffer[9] = {mpu_data.dmp_TaitBryan[1] * 180 / M_PI, -mpu_data.dmp_TaitBryan[0] * 180 / M_PI, -mpu_data.dmp_TaitBryan[2] * 180 / M_PI,
                        -mpu_data.accel[1], -mpu_data.accel[0], -mpu_data.accel[2],
                        mpu_data.gyro[1], mpu_data.gyro[0], mpu_data.gyro[2]};

    writeMatrixToFile(fp, buffer, 1, 9);

}


/*******************************************************************************
* int main()
*
*******************************************************************************/
int main() {

    fp = fopen("data_log.csv", "w+");

	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if (rc_kill_existing_process(2.0)<-2) return -1;

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

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

    // initialize dmp configeration
    rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;

	// now set up the imu for dmp interrupt operation
	if (rc_mpu_initialize_dmp(&mpu_data, mpu_config)) {
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

    rc_mpu_set_dmp_callback(&__print__data);
    rc_set_state(RUNNING);

    while (rc_get_state()!=EXITING) {
    	rc_nanosleep(1E9);
    }

	// exit cleanly
    fclose(fp);
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}
