#ifndef SERVO_H_
#define SERVO_H_

#include <asf.h>
#include "cph_millis.h"


#define ANGLE_MIN -90
#define ANGLE_MAX 90
#define ANGLE_MID 0
#define PWM_CLOCKSOURCE_FREQ 1000000
#define PWM_FREQ 50
#define PWM_PERIOD_TICKS PWM_CLOCKSOURCE_FREQ/PWM_FREQ

// #define PWM_MIN 900
// #define PWM_MID 1500
// #define PWM_MAX 2100


// #define PWM_MIN 		700
#define PWM_MIN 		1150
#define PWM_MID 		1350
#define PWM_MAX 		2000
#define PWM_STEP		10

void servo_init(void);
void servo_init2(void);
void servo_tick(void);
void servo_set_angle(float angle);
void servo_min(void);
void servo_max(void);
void servo_mid(void);
void servo_increment(void);
void servo_decrement(void);



#endif
