#include "pid.h"
#include "imu.h"
#include "cph_config.h"
#include "ap.h"


clock_time_t time = 0;
clock_time_t previous_time = 0;
clock_time_t elapsed_time = 0;
float pid = 0.0f;
float error = 0.0f;
float previous_error = 0.0f;
float pid_p = 0.0f;
float pid_i = 0.0f;
float pid_d = 0.0f;

float desired_angle = 0.0f;


void pid_init(void)
{
    time = 0;
    previous_time = 0;
    elapsed_time = 0;
    pid = 0.0f;
    error = 0.0f;
    previous_error = 0.0f;
    pid_p = 0.0f;
    pid_i = 0.0f;
    pid_d = 0.0f;
}


// float pid_tick(void)
// {
//     previous_error = time;
//     time = cph_get_millis();
//     elapsed_time = (time - previous_time)/1000;

//     error = ap.imu.x_axis - desired_angle;
//     pid_p = config.pid_kp*error;

//     // if (-3.0f < error < 3.0f) {
//     //     pid_i = pid_i +(config.pid_ki*error);
//     // }

//     pid_i = pid_i +(config.pid_kp*error);

//     pid_d = config.pid_kd * ((error-previous_error)/elapsed_time);

//     pid = pid_p + pid_i + pid_d;

//     previous_error = error;

//     return pid;
// }

float kp = 1.0f;
float ki = 0.0f;
float kd = 0.0;

float pid_tick(void)
{
    previous_error = time;
    time = cph_get_millis();
    elapsed_time = (time - previous_time)/1000;

    error = ap.imu.y_axis - AP.desired_angle_x;
    pid_p = kp*error;

    // if (-3.0f < error < 3.0f) {
    //     pid_i = pid_i +(ki*error);
    // }

    pid_i = pid_i +(ki*error);

    pid_d = kd * ((error-previous_error)/elapsed_time);

    pid = pid_p + pid_i + pid_d;

    previous_error = error;

    return pid;
}