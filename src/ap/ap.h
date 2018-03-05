#ifndef CPH_AP_H_
#define CPH_AP_H_

#include <asf.h>


// command line interface parsing results
enum ap_state {
 AP_INITIALIZING = 0,
 AP_ARMED,
 AP_MANUAL,
 AP_HOLD
};

typedef struct
{
	uint8_t state;
    float desired_angle_x;
    float desired_angle_y;
    uint32_t motor_power;

} ap_manager_t;

extern ap_manager_t AP;

void ap_init(void);

#endif