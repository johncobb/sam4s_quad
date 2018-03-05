#ifndef CPH_CONFIG_H_
#define CPH_CONFIG_H_

#include <asf.h>
#include "cph_millis.h"


typedef struct {
    float pid_kp;
    float pid_ki;
    float pid_kd;
    clock_time_t pid_elapsed;
    long motor_offset;
    bool motor_armed;
    bool gyro_calibrate;
    bool accel_calibrate;
    bool imu_calibrate;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;
    bool log_motor;
    bool log_imu;
} config_t;

extern config_t config;


void config_init(void);

#endif