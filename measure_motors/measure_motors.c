/*******************************************************************************
* measure_motors.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the parameters for your motors
*
* TODO: Option A: Capture encoder readings, current readings, timestamps etc.
*       to a file to analyze and determine motor parameters
*
*       Option B: Capture the same information within get_motor_params and follow
*       on its structure for obtaining the parameters and printing them in your
*       terminal.
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <rc/cpu.h>
#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/encoder.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"

double test_dur = 5.0;
FILE* f1;


int get_motor_params(int motor, int polarity, float resistance, float dtime_s);


/*******************************************************************************
* int main()
*
*******************************************************************************/
int main() {

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

    // initialize motor
    if (mb_motor_init()<0) {
        fprintf(stderr,"ERROR: failed to initialze mb_motors\n");
        return -1;
    }
    
    mb_motor_brake(0);

    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();

    rc_nanosleep(2E9);
    rc_set_state(RUNNING);

    /**********************************************************************
    [OPTION A] TODO : Loop and Save data to a file and migrate that info
                      to python or matlab

    while (rc_get_state()!=EXITING) {
        rc_nanosleep(1E9);
        //get data
        //save to file
    }
    // close file
    **********************************************************************/

    /**********************************************************************
    [OPTION B] TODO : Follow on the guide within get_motor_params and
                      construct it accordingly. Then run it for each motor
                      given you know its resistance.
    **********************************************************************/
    int pass_mot1, pass_mot2;
    float dtime_s = 5;  // 5 sec is usually enough but you can change
    pass_mot1 = get_motor_params(LEFT_MOTOR, MOT_1_POL, 6.1, dtime_s);
    rc_nanosleep(2E9);
    pass_mot2 = get_motor_params(RIGHT_MOTOR, MOT_2_POL, 4.5, dtime_s);


    printf("test completed \n");
    //mb_motor_disable();

    // exit cleanly
    rc_adc_cleanup();
    rc_encoder_eqep_cleanup();
    rc_remove_pid_file();   // remove pid file LAST
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// [OPTION B] TODO : Fill in get_motor_params to obtain motor parameters

int get_motor_params(int motor, int polarity, float resistance, float dtime_s) {

    // Parameters to feed in:
    /***************************************************************************************************************
    > motor : Motor number referring to right or left motor
    > polarity : Encoder polarity, you can remove this as an argument and just use the global variable from the defs
    > resistance : The resistance value you measured off the motors as we will use this in the calculations
    > dtime_s : The time to complete the transient measurements
    ***************************************************************************************************************/

    // pass flag you can manipulate to return success or fail of motor measurement
    int pass_flag = -1;

    // variable defs;

    float duty = 0.99;                      // running at full duty cycle
    float encoder_ticks, speed, noload_speed, mot_constant, stall_torque, shaft_fric, shaft_inertia;
    double noload_current;
    double dt, start_time, time_elapse, prevtime, time_const;
    int got_time_const = 0;

    // first run for steady state data and obtain all info attainable from that.

    rc_encoder_write(motor, 0);             // reset the enocder
    mb_motor_set(motor, duty);              // command the duty cycle we provided
    rc_nanosleep(test_dur * pow(10, 9));    // sleep for 5s [changeable] to guarantee we reach steady state

    // TODO: steady state measurements and calculations
    encoder_ticks = rc_encoder_read(motor);             // get your encoder counts accumulated for 5s
    noload_speed =  (encoder_ticks / ENCODER_RES) * 2.0 * M_PI / test_dur / GEAR_RATIO;        // use your accumulated encoder counts + sleep time to get a noload speed measurement
    noload_current = mb_motor_read_current(motor);      // read from the analog pin to get the no-load current

    // things you would be able to calculate from the three recorded values above
    mot_constant = (VOLTAGE - resistance * noload_current) / noload_speed;
    stall_torque = mot_constant * VOLTAGE / resistance;
    shaft_fric = ((mot_constant * VOLTAGE / noload_speed) - pow(mot_constant, 2)) / resistance;

    // turn off the motor after steady state calcs are done
    mb_motor_set(motor, 0.0);
    rc_nanosleep(2E9);

    // TODO: transient state measurements and calculations
    rc_encoder_write(motor, 0);             // reset the encoder
    mb_motor_set(motor, duty);              // set the motor again at max dc

    // we need to time the transient run now in order to obtain the time constant.
    // we will keep monitoring our spin speed in loops and once it exceeds or matches 63%
    // of our no load speed calculated above, we record the time as the time constant.

    start_time = (double)(rc_nanos_since_epoch())*1.0E-9;
    time_elapse = 0.0;                      // current loop time
    prevtime = 0.0;                         // prev loop time, needed to get dt

    // our while loop termination condition is the max run time dtime_s we provide as an argument to the function
    while (time_elapse < dtime_s) {

        time_elapse = (double)(rc_nanos_since_epoch()) * 1.0E-9 - start_time;
        dt = time_elapse - prevtime;
        prevtime = time_elapse;
        encoder_ticks = rc_encoder_read(motor);
        speed = (encoder_ticks / ENCODER_RES) * 2.0 * M_PI / test_dur / GEAR_RATIO;


        if (!got_time_const && speed > (0.63 * noload_speed)) {
            printf("Got time constant\n");
            got_time_const = 1;
            time_const = time_elapse;
        }
        rc_nanosleep(1E7);
    }

    shaft_inertia = shaft_fric * time_const;

    // finally, print all the info you obtained
    printf("[ No Load Speed (rad/s) : %3.4f, No Load Current (A) : %3.4lf,  Stall Torque (N.m) : %3.4f ]\n", noload_speed, noload_current, stall_torque);
    printf("[ Motor Constant K : %3.4f, Shaft Friction : %3.4f,  Shaft Inertia (Kg.m^2) : %1.4e ]\n\n", mot_constant, shaft_fric, shaft_inertia);

    mb_motor_set(motor,0);
    pass_flag = 1;

    return pass_flag;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
