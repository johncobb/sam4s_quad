#include "cph_config.h"

config_t config;

void config_init(void)
{
	float pid_kp = 1.0f;
    float pid_ki = 0.0f;
    float pid_kd = 0.0f;
    clock_time_t pid_elapsed = 0;
	long motor_offset = 200;
	bool motor_armed = false;
	// config.gyro_pitch_kp = 20000;
	// config.gyro_pitch_ki = 10000;
	// config.gyro_pitch_kd = 40000;
	// config.gyro_roll_kp = 20000;
	// config.gyro_roll_ki = 8000;
	// config.gyro_roll_kd = 30000;
	// config.acc_time_constant = 7;
	// config.angle_offset_pitch = 0;
	// config.angle_offset_roll = 0;
	// config.dir_motor_pitch = 1;
	// config.dir_motor_roll = -1;
	// config.motor_number_pitch = 0;
	// config.motor_number_roll = 1;
	// config.max_pwm_motor_pitch = 80;
	// config.max_pwm_motor_roll = 80;
	// config.ref_voltage_bat = 800;
	// config.cuttoff_voltage = 600;
	// config.motor_power_scale = 0;
	// config.enable_gyro = true;
	// config.enable_acc = true;
	// config.axis_reverse_z = true;
	// config.axis_swap_xy = false;
	// config.fpv_freeze_pitch = false;
	// config.fpv_freeze_roll = false;
	// config.max_pwm_fpv_pitch = 80;
	// config.max_pwm_fpv_roll = 80;
    config.gyro_calibrate = true;
	config.accel_calibrate = false;
	config.imu_calibrate = true;
	config.log_motor = false;
	config.log_imu = false;
}