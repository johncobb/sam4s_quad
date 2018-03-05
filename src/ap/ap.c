#include "ap.h"


ap_manager_t AP;


void ap_init(void)
{
    AP.state = AP_INITIALIZING;
    AP.desired_angle_x = 0.0f;
    AP.desired_angle_y = 0.0f;
    AP.motor_power = 0;
}