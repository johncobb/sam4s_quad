#ifndef MPU6050_H_
#define MPU6050_H_


#define MPU9150_RA_MAG_ADDRESS		    0x0C
#define MPU9150_RA_MAG_XOUT_L		    0x03
#define MPU9150_RA_MAG_XOUT_H		    0x04
#define MPU9150_RA_ZA_OFFS_H            0x0A


#define MPU6050_RA_WHO_AM_I             0x75
#define MPU6050_WHO_AM_I_BIT            6
#define MPU6050_WHO_AM_I_LENGTH         6

#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL            0x6A
#define MPU6050_RA_PWR_MGMT_1           0x6B
#define MPU6050_RA_GYRO_CONFIG          0x1B
#define MPU6050_RA_ACCEL_CONFIG         0x1C
#define MPU6050_RA_INT_PIN_CFG          0x37
#define MPU6050_RA_INT_ENABLE           0x38
#define MPU6050_RA_FF_THR               0x1D
#define MPU6050_RA_FF_DUR               0x1E
#define MPU6050_RA_MOT_THR              0x1F
#define MPU6050_RA_MOT_DUR              0x20
#define MPU6050_RA_ZRMOT_THR            0x21
#define MPU6050_RA_ZRMOT_DUR            0x22
#define MPU6050_RA_INT_STATUS           0x3A
#define MPU6050_RA_ACCEL_XOUT_H         0x3B
#define MPU6050_RA_GYRO_XOUT_H          0x43



#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_GYRO_FS_250             0x00

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42

#define IMU_TWI  TWI0

bool mpu_init(void);
bool mpu_probe(void);
bool mpu_begin(uint8_t scale, uint8_t range);
uint8_t mpu_who_am_i(void);
void mpu_set_clock_source(uint8_t source);
int16_t mpu_get_temperature(void);
uint8_t mpu_get_clock_source(void);
void mpu_set_full_scale_gyro_range(uint8_t range);
void mpu_set_gyro_scale(uint8_t scale);
uint8_t mpu_get_gyro_scale(void);
void mpu_set_accel_range(uint8_t range);
uint8_t mpu_get_accel_range(void);

void mpu_set_dlpf_mode(uint8_t mode);
uint8_t mpu_get_dlpf_mode(void);

void mpu_set_sleep_enabled(bool state);
bool mpu_get_sleep_enabled(void);

void mpu_set_int_zero_motion_enabled(bool state);
bool mpu_get_int_zero_motion_enabled(void);

void mpu_set_int_motion_enabled(bool state);
bool mpu_get_int_motion_enabled(void);

void mpu_set_int_freefall_enabled(bool state);
bool mpu_get_int_freefall_enabled(void);

void mpu_set_motion_detection_threshold(uint8_t threshold);
uint8_t mpu_get_motion_detection_threshold(void);

void mpu_set_motion_detection_duration(uint8_t duration);
uint8_t mpu_get_motion_detection_duration(void);

void mpu_set_zero_motion_detection_threshold(uint8_t threshold);
uint8_t mpu_get_zero_motion_detection_threshold(void);

void mpu_set_zero_motion_detection_duration(uint8_t duration);
uint8_t mpu_get_zero_motion_detection_duration(void);

void mpu_set_freefall_detection_threshold(uint8_t threshold);
uint8_t mpu_get_freefall_detection_threshold(void);

void mpu_set_freefall_detection_duration(uint8_t duration);
uint8_t mpu_get_freefall_detection_duration(void);

void mpu_set_i2c_master_mode_enabled(bool state);
bool mpu_get_i2c_master_mode_enabled(void);

void mpu_set_i2c_bypass_enabled(bool state);
bool mpu_get_i2c_bypass_enabled(void);

void mpu_set_accel_power_on_delay(uint8_t delay);
uint8_t mpu_get_accel_power_on_delay(void);

uint8_t mpu_get_int_status(void);

void mpu_calibrate_gyro(uint8_t samples);
void mpu_read_activities(t_bool_activity *a);
void mpu_log_settings(void);


void mpu_read_gyro(int16_t *x, int16_t *y, int16_t *z);
void mpu_read_raw_gyro(void);
t_fp_vector mpu_read_normalized_gyro(void);

// void mpu_read_rotation(t_fp_vector *vect);
void mpu_read_acceleration(int16_t *x, int16_t *y, int16_t *z);
void mpu_read_raw_acceleration(void);
void mpu_read_normalized_acceleration(void);
t_fp_vector mpu_read_scaled_acceleration(void);


void mpu_read_mag(int16_t *x, int16_t *y, int16_t *z);
void mpu_read_raw_mag(void);
void mpu_read_normalized_mag(void);

void mpu_set_threshold(uint8_t multiple);
uint8_t mpu_get_threshold(void);

#endif