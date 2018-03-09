#include "pid.h"
#include "imu.h"
#include "cph_config.h"
#include "ap.h"


volatile clock_time_t time = 0;
volatile clock_time_t previous_time = 0;
volatile clock_time_t elapsed_time = 0;
float pid = 0.0f;
float error = 0.0f;
float previous_error = 0.0f;
float pid_p = 0.0f;
float pid_i = 0.0f;
float pid_d = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;

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
    integral = 0.0f;
}


float pid_tick(void)
{
    time = cph_get_millis();
    elapsed_time = (time - previous_time);

    error = desired_angle - ap.imu.x_axis;

    integral = integral + (error * elapsed_time);

    derivative = ((error-previous_error)/elapsed_time);

    pid_p = (config.pid_kp * error);
    pid_i = (config.pid_ki * integral);
    pid_d = (config.pid_kd * derivative);

    pid = pid_p + pid_i + pid_d;

    previous_error = error;
    previous_time = time;    

    return pid;
}