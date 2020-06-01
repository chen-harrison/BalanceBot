/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality
*
*******************************************************************************/
#include <math.h>
#include "mb_odometry.h"


void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y)
{
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->psi = mb_state.psi;
}


float mb_clamp_radians(float angle)
{
    float temp;
    temp = fmod(angle, (2.0*M_PI));

    if (temp > M_PI) temp = temp - 2.0*M_PI;  
    if (temp < -M_PI) temp = temp + 2.0*M_PI;

    return temp;
}


void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state)
{
    double delta_sL = (mb_state->left_encoder - mb_state->encoder_pre[0]) / ENCODER_RES * 2.0 * M_PI / GEAR_RATIO * WHEEL_DIAMETER / 2.0;
    double delta_sR = -(mb_state->right_encoder - mb_state->encoder_pre[1]) / ENCODER_RES * 2.0 * M_PI / GEAR_RATIO * WHEEL_DIAMETER / 2.0;
    
    double delta_s = (delta_sL + delta_sR) / 2.0;
    double alpha = (delta_sR - delta_sL) / WHEEL_BASE;

    // x and y are reversed with respect to the body frame
    double delta_x = -delta_s * cos(mb_odometry->psi + alpha / 2.0);
    double delta_y = -delta_s * sin(mb_odometry->psi + alpha / 2.0);

    mb_odometry->x += delta_x;
    mb_odometry->y += delta_y;
    mb_odometry->psi += alpha;

    mb_state->dpsi_odometry = alpha;

    // clamp the odometry reading of psi between [-pi, pi]
    mb_odometry->psi = mb_clamp_radians(mb_odometry->psi);
    
    // parse to mb_state 
    mb_state->psi_odometry = mb_odometry->psi;
    mb_state->x = mb_odometry->x;
    mb_state->y = mb_odometry->y;
}
