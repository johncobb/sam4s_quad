#ifndef IMU_H_
#define IMU_H_

#include <asf.h>

#define IMU_TWI_ID                      ID_TWI0
#define IMU_ADDRESS                     0x68
#define TWI_WAIT_TIME   10
/** TWI Bus Clock 400kHz */
#define TWI_CLK     400000
// #define TWI_CLK     100000L

//#define GRAVITY 16384.0f
#define GRAVITY	15500.0f;
#define GYRO_SAMPLES        50

#define MAG_READ_RATE       100 // ms between reads


typedef struct fp_vector
{
    float x_axis;
    float y_axis;
    float z_axis;
} t_fp_vector;



typedef struct ap_vectors
{
    t_fp_vector imu;
    t_fp_vector mag;
    t_fp_vector setpoint;
    t_fp_vector command;
} t_ap_vectors;

typedef struct bool_activity
{
    bool is_overflow;
    bool is_freefall;
    bool is_inactivity;
    bool is_activity;
    bool is_pos_activity_on_x;
    bool is_pos_activity_on_y;
    bool is_pos_activity_on_z;
    bool is_neg_activity_on_x;
    bool is_neg_activity_on_y;
    bool is_neg_activity_on_z;
    bool is_data_ready;
} t_bool_activity;

// Raw variables used to read data
// from i2c registers from mpu
uint8_t mpu_buffer[16];
int16_t ax, ay, az; // accel
int16_t gx, gy, gz; // gyro
int16_t mx, my, mz; // mag

bool use_calibrate;
float actual_threshold;
float dps_per_digit;
float range_per_digit;

// Raw vectors
t_fp_vector raw_gyro;
t_fp_vector raw_accel;
t_fp_vector raw_mag;

// Normalized vectors
t_fp_vector norm_gyro;
t_fp_vector norm_accel;
t_fp_vector norm_mag;

// Delta vectors
t_fp_vector threshold_gyro;
t_fp_vector delta_gyro;

// Threshold
t_fp_vector threshold;
t_bool_activity mpu_activities;

extern t_fp_vector imu_complementary;
extern t_ap_vectors ap;


bool imu_init(void);
void imu_calibrate(void);
void imu_tick(void);


#endif /* IMU_H_ */