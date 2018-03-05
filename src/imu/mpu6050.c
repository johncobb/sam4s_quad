#include <asf.h>
#include <stdio.h>
#include <string.h>
#include "imu.h"
#include "mpu6050.h"


/*
https://github.com/jarzebski/Arduino-MPU6050/blob/master/MPU6050.cpp
http://community.atmel.com/forum/samc21-printf-not-printing-float-values

*/

// static uint8_t mpu_buffer[16] = {0};
// int16_t ax, ay, az;
// int16_t gx, gy, gz;

// bool use_calibrate = false;
// float actual_threshold = 0;
// float dps_per_digit = 0;
// float range_per_digit;

// // Raw vectors
// t_fp_vector raw_gyro;
// t_fp_vector raw_accel;

// // Normalized vectors
// t_fp_vector norm_gyro;
// t_fp_vector norm_accel;

// // Delta vectors
// t_fp_vector threshold_gyro;
// t_fp_vector delta_gyro;

// // Threshold
// t_fp_vector threshold;

// t_bool_activity mpu_activities;


void write_register8(uint8_t address, uint8_t reg, uint8_t value);
void write_register16(uint8_t address, uint8_t reg, int16_t value);
uint8_t read_register8(uint8_t address, uint8_t reg);
void write_register_bit(uint8_t address, uint8_t reg, uint8_t pos, bool state);
bool read_register_bit(uint8_t address, uint8_t reg, uint8_t pos);
int16_t read_register16(uint8_t address, uint8_t reg);
uint8_t read_bytes(uint8_t address, uint8_t reg, int8_t length, uint8_t *data);


void write_register8(uint8_t address, uint8_t reg, uint8_t value)
{
    twi_packet_t packet_tx;

    packet_tx.chip = address;
    packet_tx.addr[0] = reg;
    packet_tx.addr_length = sizeof(uint8_t);
    packet_tx.buffer = &value;
    packet_tx.length = sizeof(uint8_t);

    uint32_t status = twi_master_write(IMU_TWI, &packet_tx);

    // printf("write_register status: %d\r\n", status);
    
    // if (status == TWI_SUCCESS) {
    //     puts("write_register: success\r\n");
    // }

    delay_ms(TWI_WAIT_TIME);
}


void write_register16(uint8_t address, uint8_t reg, int16_t value)
{

    twi_packet_t packet_tx;

    packet_tx.chip = address;
    packet_tx.addr[0] = reg;
    packet_tx.addr_length = sizeof(uint8_t);
    packet_tx.buffer = &value;
    packet_tx.length = sizeof(int16_t);


    uint32_t status = twi_master_write(IMU_TWI, &packet_tx);

    // printf("write_register status: %d\r\n", status);
    
    // if (status == TWI_SUCCESS) {
    //     puts("write_register: success\r\n");
    // }

    delay_ms(TWI_WAIT_TIME);
}

uint8_t read_register8(uint8_t address, uint8_t reg)
{
    uint8_t value = 0;
    twi_packet_t packet_rx;

    memset(mpu_buffer, 0, sizeof(mpu_buffer));

    packet_rx.chip = address;
    packet_rx.addr[0] = reg;
    packet_rx.addr_length = sizeof(uint8_t);
    packet_rx.buffer = &mpu_buffer;
    packet_rx.length = sizeof(uint8_t);

    uint32_t status = twi_master_read(IMU_TWI, &packet_rx);

    // printf("read_register status: %d\r\n", status);
    
    if (status == TWI_SUCCESS) {
        // puts("read_register: success\r\n");
        value = mpu_buffer[0];
    }

    delay_ms(TWI_WAIT_TIME);

    return value;
}

int16_t read_register16(uint8_t address, uint8_t reg)
{
    int16_t value = 0;
    twi_packet_t packet_rx;

    memset(mpu_buffer, 0, sizeof(mpu_buffer));

    packet_rx.chip = address;
    packet_rx.addr[0] = reg;
    packet_rx.addr_length = sizeof(uint8_t);
    packet_rx.buffer = &mpu_buffer;

    // packet_rx.length = 1;
    packet_rx.length = sizeof(uint16_t);

    uint32_t status = twi_master_read(IMU_TWI, &packet_rx);

    // printf("read_register status: %d\r\n", status);
    
    if (status == TWI_SUCCESS) {
        // puts("read_register: success\r\n");
        uint8_t value_high = mpu_buffer[0];
        uint8_t value_low = mpu_buffer[1];

        value = value_high << 8 | value_low;
    }

    delay_ms(TWI_WAIT_TIME);

    return value;
}

uint8_t read_bytes(uint8_t address, uint8_t reg, int8_t length, uint8_t *data)
{
    // uint8_t value = 0;
    twi_packet_t packet_rx;

    memset(mpu_buffer, 0, sizeof(mpu_buffer));

    packet_rx.chip = address;
    packet_rx.addr[0] = reg;
    packet_rx.addr_length = sizeof(uint8_t);
    packet_rx.buffer = data;
    packet_rx.length = length;

    uint32_t status = twi_master_read(IMU_TWI, &packet_rx);

    // printf("read_register status: %d\r\n", status);
    
    if (status == TWI_SUCCESS) {
        // puts("read_register: success\r\n");
        // value = mpu_buffer[0];
    }

    delay_ms(TWI_WAIT_TIME);

    return length;
}


void write_register_bit(uint8_t address, uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = read_register8(address, reg);

    if (state) {
        value |= (1 << pos);
    } else {
        value &= ~(1 << pos);
    }

    write_register8(address, reg, value);
}

bool read_register_bit(uint8_t address, uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = read_register8(address, reg);
    return ((value >> pos) & 1);
}

bool mpu_init(void)
{
    twi_options_t opt;

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
    opt.speed      = TWI_CLK;

    if (twi_master_init(IMU_TWI, &opt) != TWI_SUCCESS) {
        
        puts("twi_master_init: failed\r\n");
        // Give I2C time to settle
        delay_ms(TWI_WAIT_TIME);
        return false;
        
    } else {
        puts("twi_master_init: success\r\n");
        // Give I2C time to settle
        delay_ms(TWI_WAIT_TIME);
        return true;
    }
    
}

bool mpu_probe(void)
{
    if (twi_probe(IMU_TWI, IMU_ADDRESS) != TWI_SUCCESS) {
        puts("twi_probe: failed\r\n");
        // Give I2C time to settle
        delay_ms(TWI_WAIT_TIME);
        return false;
        
    } else {
        puts("twi_probe: success\r\n");
        // Give I2C time to settle
        delay_ms(TWI_WAIT_TIME);
        return true;
    }
}

bool mpu_begin(uint8_t scale, uint8_t range)
{
    // Reset calibration values
    delta_gyro.x_axis = 0;
    delta_gyro.y_axis = 0;
    delta_gyro.z_axis = 0;
    use_calibrate = false;

    // Reset threshold values
    threshold_gyro.x_axis = 0;
    threshold_gyro.y_axis = 0;
    threshold_gyro.z_axis = 0;
    actual_threshold = 0;

    if (mpu_who_am_i() != 0x68) {
        return false;
    }

    // Set clock source
    mpu_set_clock_source(MPU6050_CLOCK_PLL_XGYRO);

    // Set scale and range
    mpu_set_gyro_scale(scale);
    mpu_set_accel_range(range);

    // Disable sleep mode
    mpu_set_sleep_enabled(false);

    return true;
}

uint8_t mpu_who_am_i(void)
{
    uint8_t value = read_register8(IMU_ADDRESS, MPU6050_RA_WHO_AM_I);

    // printf("who_am_i: 0x%1x\r\n", value);

    if (value == 0x68) {
        // puts("who_am_i: success\r\n");
    }

    return value;
}

int16_t mpu_get_temperature(void)
{
    int16_t T;
    T = read_register16(IMU_ADDRESS, MPU6050_RA_TEMP_OUT_H);
    return T;
}

void mpu_set_clock_source(uint8_t source)
{
    uint8_t value;

    value = read_register8(IMU_ADDRESS, MPU6050_RA_PWR_MGMT_1);
    value &= 0b11111000; // mask
    value |= source;

    write_register8(IMU_ADDRESS, MPU6050_RA_PWR_MGMT_1, value);
}

uint8_t mpu_get_clock_source(void)
{
    uint8_t value;

    value = read_register8(IMU_ADDRESS, MPU6050_RA_PWR_MGMT_1);
    value &= 0b00000111; // mask

    return (uint8_t)value;
}

void mpu_set_gyro_scale(uint8_t scale)
{
    uint8_t value;

    switch (scale) {
        case MPU6050_GYRO_FS_250:
            dps_per_digit = .007633f;
            break;
        case MPU6050_GYRO_FS_500:
            dps_per_digit = .015267f;
            break;
        case MPU6050_GYRO_FS_1000:
            dps_per_digit = .030487f;
            break;
        case MPU6050_GYRO_FS_2000:
            dps_per_digit = .060975f;
            break;
    }

    value = read_register8(IMU_ADDRESS, MPU6050_RA_GYRO_CONFIG);
    value &= 0b11100111; // mask
    value |= (scale << 3);

    write_register8(IMU_ADDRESS, MPU6050_RA_GYRO_CONFIG, value);
}

uint8_t mpu_get_gyro_scale(void)
{
    uint8_t value;

    value = read_register8(IMU_ADDRESS, MPU6050_RA_GYRO_CONFIG);
    value &= 0b00011000; // mask
    value >>= 3;

    return (uint8_t)value;
}

void mpu_set_accel_range(uint8_t range)
{
    uint8_t value;

    switch (range) {
        case MPU6050_ACCEL_FS_2:
        	range_per_digit = .000061f;
            break;
        case MPU6050_ACCEL_FS_4:
        	range_per_digit = .000122f;
            break;   
        case MPU6050_ACCEL_FS_8:
        	range_per_digit = .000244f;
            break;   
        case MPU6050_ACCEL_FS_16:
        	range_per_digit = .0004882f;
	        break;                                                           
    }

    value = read_register8(IMU_ADDRESS, MPU6050_RA_ACCEL_CONFIG);
    value &= 0b11100111; // mask
    value |= (range << 3);

    write_register8(IMU_ADDRESS, MPU6050_RA_ACCEL_CONFIG, value);
}

uint8_t mpu_get_accel_range(void)
{
    uint8_t value;

    value = read_register8(IMU_ADDRESS, MPU6050_RA_ACCEL_CONFIG);
    value &= 0b00011000; // mask
    value >>= 3;

    return (uint8_t)value;
}

void mpu_set_dlpf_mode(uint8_t mode)
{
    uint8_t value;

    value = read_register8(IMU_ADDRESS, MPU6050_RA_ACCEL_CONFIG);
    value &= 0b11111000; // mask
    value |= (mode << 3);

    write_register8(IMU_ADDRESS, MPU6050_RA_ACCEL_CONFIG, value);
}

uint8_t mpu_get_dlpf_mode(void)
{
    uint8_t value;

    value = read_register8(IMU_ADDRESS, MPU6050_RA_ACCEL_CONFIG);
    value &= 0b00000111; // mask
    value >>= 3;

    return (uint8_t)value;
}

void mpu_set_sleep_enabled(bool state)
{
    write_register_bit(IMU_ADDRESS, MPU6050_RA_PWR_MGMT_1, 6, state);
}

bool mpu_get_sleep_enabled(void)
{
    return read_register_bit(IMU_ADDRESS, MPU6050_RA_PWR_MGMT_1, 6);
}

void mpu_set_int_zero_motion_enabled(bool state)
{
    write_register_bit(IMU_ADDRESS, MPU6050_RA_INT_ENABLE, 5, state);
}

bool mpu_get_int_zero_motion_enabled(void)
{
    return read_register_bit(IMU_ADDRESS, MPU6050_RA_INT_ENABLE, 5);
}

void mpu_set_int_motion_enabled(bool state)
{
    write_register_bit(IMU_ADDRESS, MPU6050_RA_INT_ENABLE, 6, state);
}

bool mpu_get_int_motion_enabled(void)
{
    return read_register_bit(IMU_ADDRESS, MPU6050_RA_INT_ENABLE, 6);
}

void mpu_set_int_freefall_enabled(bool state)
{
    write_register_bit(IMU_ADDRESS, MPU6050_RA_INT_ENABLE, 7, state);
}

bool mpu_get_int_freefall_enabled(void)
{
    return read_register_bit(IMU_ADDRESS, MPU6050_RA_INT_ENABLE, 7);
}

void mpu_set_motion_detection_threshold(uint8_t threshold)
{
    write_register8(IMU_ADDRESS, MPU6050_RA_MOT_THR, threshold);
}

uint8_t mpu_get_motion_detection_threshold(void)
{
    return read_register8(IMU_ADDRESS, MPU6050_RA_MOT_THR);
}

void mpu_set_motion_detection_duration(uint8_t duration)
{
    write_register8(IMU_ADDRESS, MPU6050_RA_MOT_DUR, duration);
}

uint8_t mpu_get_motion_detection_duration(void)
{
    return read_register8(IMU_ADDRESS, MPU6050_RA_MOT_DUR);
}

void mpu_set_zero_motion_detection_threshold(uint8_t threshold)
{
    write_register8(IMU_ADDRESS, MPU6050_RA_ZRMOT_THR, threshold);
}

uint8_t mpu_get_zero_motion_detection_threshold(void)
{
    return read_register8(IMU_ADDRESS, MPU6050_RA_ZRMOT_THR);
}

void mpu_set_zero_motion_detection_duration(uint8_t duration)
{
    write_register8(IMU_ADDRESS, MPU6050_RA_ZRMOT_DUR, duration);
}

uint8_t mpu_get_zero_motion_detection_duration(void)
{
    return read_register8(IMU_ADDRESS, MPU6050_RA_ZRMOT_DUR);
}

void mpu_set_freefall_detection_threshold(uint8_t threshold)
{
    write_register8(IMU_ADDRESS, MPU6050_RA_FF_THR, threshold);
}

uint8_t mpu_get_freefall_detection_threshold(void)
{
    return read_register8(IMU_ADDRESS, MPU6050_RA_FF_THR);
}

void mpu_set_freefall_detection_duration(uint8_t duration)
{
    write_register8(IMU_ADDRESS, MPU6050_RA_FF_DUR, duration);
}

uint8_t mpu_get_freefall_detection_duration(void)
{
    return read_register8(IMU_ADDRESS, MPU6050_RA_FF_DUR);
}

void mpu_set_i2c_master_mode_enabled(bool state)
{
    write_register_bit(IMU_ADDRESS, MPU6050_RA_USER_CTRL, 5, state);
}

bool mpu_get_i2c_master_mode_enabled(void)
{
    return read_register_bit(IMU_ADDRESS, MPU6050_RA_USER_CTRL, 5);
}

void mpu_set_i2c_bypass_enabled(bool state)
{
    write_register_bit(IMU_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, state);
}

bool mpu_get_i2c_bypass_enabled(void)
{
    return read_register_bit(IMU_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1);
}

uint8_t mpu_get_accel_power_on_delay(void)
{
    uint8_t value;

    value = read_register8(IMU_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL);
    value &= 0b00110000; // mask
    value >>= 4;

    return (uint8_t)value;
}

void mpu_set_accel_power_on_delay(uint8_t delay)
{
    uint8_t value;

    value = read_register8(IMU_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL);
    value &= 0b11001111; // mask
    value |= (delay << 4);

    write_register8(IMU_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, value);
}

uint8_t mpu_get_int_status(void)
{
    return read_register8(IMU_ADDRESS, MPU6050_RA_INT_STATUS);
}

void mpu_read_activities(t_bool_activity *a)
{
    uint8_t data = read_register8(IMU_ADDRESS, MPU6050_RA_INT_STATUS);

    a->is_overflow = ((data >> 4) & 1);
    a->is_freefall = ((data >> 7) & 1);
    a->is_inactivity = ((data >> 5) & 1);
    a->is_activity = ((data >> 6) & 1);
    a->is_data_ready = ((data >> 0) & 1);

    data = read_register8(IMU_ADDRESS, MPU6050_RA_MOT_DETECT_STATUS);
    
    a->is_neg_activity_on_x = ((data >> 7) & 1);
    a->is_neg_activity_on_y = ((data >> 6) & 1);
    a->is_neg_activity_on_z = ((data >> 5) & 1);

    a->is_pos_activity_on_x = ((data >> 4) & 1);
    a->is_pos_activity_on_y = ((data >> 3) & 1);
    a->is_pos_activity_on_z = ((data >> 2) & 1);

}

void mpu_read_gyro(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t i2c_buffer[6];
    memset(i2c_buffer, 0, sizeof(i2c_buffer));
    
    read_bytes(IMU_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 6, i2c_buffer);

    *x = (((int16_t)i2c_buffer[0]) << 8) | i2c_buffer[1];
    *y = (((int16_t)i2c_buffer[2]) << 8) | i2c_buffer[3];
    *z = (((int16_t)i2c_buffer[4]) << 8) | i2c_buffer[5];

}

void mpu_read_raw_gyro(void)
{
    mpu_read_gyro(&gx, &gy, &gz);
    raw_gyro.x_axis = (float)gx;
    raw_gyro.y_axis = (float)gy;
    raw_gyro.z_axis = (float)gz;

}

t_fp_vector mpu_read_normalized_gyro(void)
{
    mpu_read_raw_gyro();

    if (use_calibrate) {
        norm_gyro.x_axis = (raw_gyro.x_axis - delta_gyro.x_axis) * dps_per_digit;
        norm_gyro.y_axis = (raw_gyro.y_axis - delta_gyro.y_axis) * dps_per_digit;
        norm_gyro.z_axis = (raw_gyro.z_axis - delta_gyro.z_axis) * dps_per_digit;
    } else {
        norm_gyro.x_axis = raw_gyro.x_axis  * dps_per_digit;
        norm_gyro.y_axis = raw_gyro.y_axis  * dps_per_digit;
        norm_gyro.z_axis = raw_gyro.z_axis  * dps_per_digit;
    }

    if (actual_threshold) {
        if (abs(norm_gyro.x_axis) < threshold_gyro.x_axis) norm_gyro.x_axis = 0;
        if (abs(norm_gyro.y_axis) < threshold_gyro.y_axis) norm_gyro.y_axis = 0;
        if (abs(norm_gyro.z_axis) < threshold_gyro.z_axis) norm_gyro.z_axis = 0;
    }

    return norm_gyro;
}



void mpu_read_acceleration(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t i2c_buffer[6];
    memset(i2c_buffer, 0, sizeof(i2c_buffer));
    
    read_bytes(IMU_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 6, i2c_buffer);

    *x = (((int16_t)i2c_buffer[0]) << 8) | i2c_buffer[1];
    *y = (((int16_t)i2c_buffer[2]) << 8) | i2c_buffer[3];
    *z = (((int16_t)i2c_buffer[4]) << 8) | i2c_buffer[5];
}

void mpu_read_raw_acceleration(void)
{
    mpu_read_acceleration(&ax, &ay, &az);
    raw_accel.x_axis = (float)ax;
    raw_accel.y_axis = (float)ay;
    raw_accel.z_axis = (float)az;
}

void mpu_read_normalized_acceleration(void)
{
    mpu_read_raw_acceleration();

    norm_accel.x_axis = raw_accel.x_axis * range_per_digit * 9.80665f;
    norm_accel.y_axis = raw_accel.y_axis * range_per_digit * 9.80665f;
    norm_accel.z_axis = raw_accel.z_axis * range_per_digit * 9.80665f;
}

t_fp_vector mpu_read_scaled_acceleration(void)
{
    mpu_read_raw_acceleration();
    norm_accel.x_axis = raw_accel.x_axis * range_per_digit;
    norm_accel.y_axis = raw_accel.y_axis * range_per_digit;
    norm_accel.z_axis = raw_accel.z_axis * range_per_digit;

    return norm_accel;
}

void mpu_read_mag(int16_t *x, int16_t *y, int16_t *z)
{
    // Set bypass enable
    mpu_set_i2c_bypass_enabled(true);
    delay_ms(TWI_WAIT_TIME);

    // Enable magnetometer
    write_register8(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_ZA_OFFS_H, 0x01);
    delay_ms(TWI_WAIT_TIME);

    uint8_t i2c_buffer[6];
    memset(i2c_buffer, 0, sizeof(i2c_buffer));
    
    read_bytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, i2c_buffer);

    *x = (((int16_t)i2c_buffer[0]) << 8) | i2c_buffer[1];
    *y = (((int16_t)i2c_buffer[2]) << 8) | i2c_buffer[3];
    *z = (((int16_t)i2c_buffer[4]) << 8) | i2c_buffer[5];
}

void mpu_read_raw_mag(void)
{
    mpu_read_mag(&mx, &my, &mz);
    raw_mag.x_axis = (float) mx;
    raw_mag.y_axis = (float) my;
    raw_mag.z_axis = (float) mz;
}

void mpu_read_normalized_mag(void)
{
    mpu_read_raw_mag();

    /*
     * milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
     * apply calibration offsets in mG that correspond to your environment and magnetometer
     */

    norm_mag.x_axis = raw_mag.x_axis * 10/1229/4096 +18;
    norm_mag.y_axis = raw_mag.y_axis * 10/1229/4096 +70;
    norm_mag.z_axis = raw_mag.z_axis * 10/1229/4096 +270;    

}

void mpu_log_settings(void)
{
    printf("Device: 0x%1x\r\n", mpu_who_am_i());
    printf("Sleep Mode: %s\r\n", mpu_get_sleep_enabled() ? "Enabled" : "Disabled");
    printf("Clock Source: ");
    switch (mpu_get_clock_source()) {
        case MPU6050_CLOCK_INTERNAL:
        printf("Internal 8MHz oscillator\r\n");
        break;
        case MPU6050_CLOCK_PLL_XGYRO:
        printf("PLL with X axis gyroscope reference\r\n");
        break;
    }
    printf("Gyroscope: ");
        switch (mpu_get_gyro_scale()) {
        case MPU6050_GYRO_FS_250:
        printf("250 dps\r\n");
        break;
        case MPU6050_GYRO_FS_500:
        printf("500 dps\r\n");
        break;
        case MPU6050_GYRO_FS_1000:
        printf("1000 dps\r\n");
        break;
        case MPU6050_GYRO_FS_2000:
        printf("2000 dps\r\n");
        break;
    }
    printf("use_calibrate: %d\r\n", use_calibrate);
    printf("actual_threshold: %f\r\n", actual_threshold);
    printf("dps_per_digit: %f\r\n", dps_per_digit);
    printf("range_per_digit: %f\r\n", range_per_digit);

}


void mpu_calibrate_gyro(uint8_t samples)
{
	use_calibrate = true;

	float sum_x = 0;
	float sum_y = 0;
	float sum_z = 0;
	float sigma_x = 0;
	float sigma_y = 0;
	float sigma_z = 0;

	for (uint8_t i=0; i<samples; i++) {

        mpu_read_raw_gyro();

		sum_x += raw_gyro.x_axis;
		sum_y += raw_gyro.y_axis;
		sum_z += raw_gyro.z_axis;

        // Sum
		sigma_x += raw_gyro.x_axis * raw_gyro.x_axis;
		sigma_y += raw_gyro.y_axis * raw_gyro.y_axis;
		sigma_z += raw_gyro.z_axis * raw_gyro.z_axis;

		delay_ms(5);
	}

	// Calculate delta vectors
	delta_gyro.x_axis = sum_x/samples;
	delta_gyro.y_axis = sum_y/samples;
	delta_gyro.z_axis = sum_z/samples;

	// Calculate threshold vectors
	threshold.x_axis = sqrt((sigma_x/50) - (delta_gyro.x_axis * delta_gyro.x_axis));
	threshold.y_axis = sqrt((sigma_y/50) - (delta_gyro.y_axis * delta_gyro.y_axis));
	threshold.z_axis = sqrt((sigma_z/50) - (delta_gyro.z_axis * delta_gyro.z_axis));

	if (actual_threshold > 0) {
		mpu_set_threshold(actual_threshold);
	}

}

uint8_t mpu_get_threshold(void)
{
	return actual_threshold;
}

void mpu_set_threshold(uint8_t multiple)
{
	if (multiple > 0) {
		if (!use_calibrate) {
			mpu_calibrate_gyro(GYRO_SAMPLES);
		}

		threshold_gyro.x_axis = threshold.x_axis * multiple;
		threshold_gyro.y_axis = threshold.y_axis * multiple;
		threshold_gyro.z_axis = threshold.z_axis * multiple;
	} else {
		threshold_gyro.x_axis = 0;
		threshold_gyro.y_axis = 0;
		threshold_gyro.z_axis = 0;
	}

	actual_threshold = multiple;
}