/**
 * <log_manager.h>
 *
 * @brief      Functions to start, stop, and interact with the log manager
 *             thread.
 *
 */

#ifndef LOG_MANAGER_H
#define LOG_MANAGER_H


/**
 * Struct containing all possible values that could be writen to the log. For
 * each log entry you wish to create, fill in an instance of this and pass to
 * add_log_entry(void). You do not need to populate all parts of the struct.
 * Currently feedback.c populates all values and log_manager.c only writes the
 * values enabled in the settings file.
 */
typedef struct log_entry_t{
	uint64_t last_step_ns;	// <time since boot>

    // raw sensor inputs
    float   theta;             // body angle (rad)
    float   phi;               // average wheel angle (rad)
    float   psi;
    float   psi_imu;
    float   psi_odometry;
    float   psi_go;
    float   psi_kf;
    float   left_phi;
    float   right_phi;
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]
    float   left_speed;
    float   right_speed;

    
    float opti_x;
    float opti_y;
    float opti_roll;
    float opti_pitch;
    float opti_yaw;

    double u_psi;
    double u_theta;
    double left_PWM;
    double right_PWM;

    // setpoints
    double psi_d;
    double phi_d;
    double theta_d;
    double psi_instant_d;

    float temp; 

} log_entry_t;


/**
 * @brief      creates a new csv log file and starts the background thread.
 *
 * @return     0 on success, -1 on failure
 */
int log_manager_init(void);


/**
 * @brief      quickly add new data to local buffer
 *
 * This is called after feedback_march after signals have been sent to
 * the motors.
 *
 * @return     0 on success, -1 on failure
 */
int log_manager_add_new();


/**
 * @brief      Finish writing remaining data to log and close thread.
 *
 *             Used in log_manager.c
 *
 * @return     0 on sucess and clean exit, -1 on exit timeout/force close.
 */
int log_manager_cleanup(void);

#endif // LOG_MANAGER_H
