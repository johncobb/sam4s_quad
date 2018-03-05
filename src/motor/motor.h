#ifndef MOTOR_H_
#define MOTOR_H_

#include <asf.h>
#include "cph_millis.h"

#define NUM_MOTORS 4

#define AP_ANGLE_MIN -90
#define AP_ANGLE_MAX 90
#define AP_ANGLE_MID 0
#define MOTOR_PWM_CLOCKSOURCE_FREQ 1000000
#define MOTOR_PWM_FREQ 50
#define MOTOR_PWM_PERIOD_TICKS MOTOR_PWM_CLOCKSOURCE_FREQ/MOTOR_PWM_FREQ

// #define PWM_MIN 900
// #define PWM_MID 1500
// #define PWM_MAX 2100


// #define PWM_MIN 		700
#define MOTOR_PWM_MIN 		1150
#define MOTOR_PWM_MID 		1350
#define MOTOR_PWM_MAX 		2000
#define MOTOR_PWM_STEP		10

pwm_clock_t motor_clock_setting;

typedef struct
{
	Pwm *p_pwm;
	uint32_t ul_pin; // EXT1_PIN_PWM_0
	uint32_t ul_flag; // PIO_TYPE_PIO_PERIPH_B
	uint32_t ul_periph_clkid; // ID_PWM
	uint32_t ul_channel; // EXT1_PWM_CHANNEL;
	pwm_clock_t clock_setting;
	pwm_channel_t pwm_channel;
    
} motor_config_t;

typedef struct
{
    motor_config_t config;
    long angle_min;
    long angle_max;
    clock_time_t timeout;
    
} motor_t;

motor_t motors[NUM_MOTORS];

void motor_init(void);
void motor_tick(void);
void motor_set_angle(float angle);
void motor_min(motor_t motor);
void motor_max(motor_t motor);
void motor_mid(motor_t motor);
void motor_increment(motor_t motor);
void motor_decrement(motor_t motor);
void motor_set_power(motor_t motor, uint32_t power);






#endif