#include "imu.h"
#include "cph_config.h"
#include "cph_millis.h"
#include "mpu6050.h"

uint8_t mpu_buffer[16] = {0};
int16_t ax, ay, az;
int16_t gx, gy, gz;

bool use_calibrate = false;
float actual_threshold = 0.0f;
float dps_per_digit = 0.0f;
float range_per_digit = 0.0f;

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

t_fp_vector imu_complementary;
t_ap_vectors ap;


clock_time_t last_time_read = 0;
clock_time_t f_timeout = 0;
volatile clock_time_t f_mag_timeout = 0;


const float alpha = 0.96f;
float last_angle_x = 0.0f;
float last_angle_y = 0.0f;
float last_angle_z = 0.0f;

float last_gyro_angle_x = 0.0f;
float last_gyro_angle_y = 0.0f;
float last_gyro_angle_z = 0.0f;

bool imu_init(void)
{
    ap.imu.x_axis = 0.0f;
    ap.imu.y_axis = 0.0f;
    ap.imu.z_axis = 0.0f;

    ap.setpoint.x_axis = 0.0f;
    ap.setpoint.y_axis = 0.0f;
    ap.setpoint.z_axis = 0.0f;

    ap.command.x_axis = 0.0f;
    ap.command.y_axis = 0.0f;
    ap.command.z_axis = 0.0f;

    return (mpu_init() & mpu_probe() & mpu_begin(MPU6050_GYRO_FS_250, MPU6050_ACCEL_FS_2));
}

void imu_calibrate(void)
{
    printf("calibrating imu...\r\n");
    mpu_calibrate_gyro(GYRO_SAMPLES);
    mpu_set_threshold(0);
    mpu_log_settings();
    printf("calibration complete.\r\n");
}

void imu_tick(void)
{
    clock_time_t t_now = cph_get_millis();

    mpu_read_normalized_gyro();
    mpu_read_normalized_acceleration();

    float gyro_x = (norm_gyro.x_axis*M_PI)/180.0f;
    float gyro_y = (norm_gyro.y_axis*M_PI)/180.0f;
    float gyro_z = (norm_gyro.z_axis*M_PI)/180.0f;


    float mag_x = 0.0f;
    float mag_y = 0.0f;
    float mag_z = 0.0f;


    // TODO: Need to tune
    // if (cph_get_millis() >= f_mag_timeout) {
    //     f_mag_timeout = cph_get_millis() + MAG_READ_RATE;

    //     mpu_read_normalized_mag();
        
    //     mag_x = ((float) norm_mag.x_axis)*M_PI/180.0f;
    //     mag_y = ((float) norm_mag.y_axis)*M_PI/180.0f;
    //     mag_z = ((float) norm_mag.z_axis)*M_PI/180.0f;
    // }



    float accel_angle_y = atan2(norm_accel.x_axis, sqrt( pow(norm_accel.y_axis, 2) + pow(norm_accel.z_axis, 2))) * 180.0f / M_PI;
    float accel_angle_x = atan2(norm_accel.y_axis, sqrt( pow(norm_accel.x_axis, 2) + pow(norm_accel.z_axis, 2))) * 180.0f / M_PI;
    float accel_angle_z = 0;

    // Compute filtered angles
    clock_time_t delta_t = (t_now-last_time_read);
    float dt = ((float)delta_t/1000.0f);

    float gyro_angle_x = gyro_x * dt + last_angle_x;
    float gyro_angle_y = gyro_y * dt + last_angle_y;
    float gyro_angle_z = gyro_z * dt + last_angle_z;

    // Compute drifting gyro angles
    float unfiltered_gyro_angle_x = gyro_x * dt + last_gyro_angle_x;
    float unfiltered_gyro_angle_y = gyro_y * dt + last_gyro_angle_y;
    float unfiltered_gyro_angle_z = gyro_z * dt + last_gyro_angle_z;

    float angle_x = alpha * gyro_angle_x + (1.0f - alpha) * accel_angle_x;
    float angle_y = alpha * gyro_angle_y + (1.0f - alpha) * accel_angle_y;
    float angle_z = alpha * gyro_angle_z + (1.0f - alpha) * accel_angle_z;


    last_time_read = t_now;
    last_angle_x = angle_x;
    last_angle_y = angle_y;
    last_angle_z = angle_z;

    imu_complementary.x_axis = angle_x;
    imu_complementary.y_axis = angle_y;
    imu_complementary.z_axis = angle_z;

    ap.imu.x_axis = angle_x;
    ap.imu.y_axis = angle_y;
    ap.imu.z_axis = angle_z;
    ap.mag.x_axis = mag_x;
    ap.mag.y_axis = mag_y;
    ap.mag.z_axis = mag_z;


}