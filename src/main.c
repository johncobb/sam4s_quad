#include <asf.h>

#include <stdio.h>
#include <string.h>
#include "cph_config.h"
#include "cph_millis.h"
#include "cph_util.h"
#include "cph_console.h"
#include "cph_cli.h"
#include "cph_uart.h"
#include "imu.h"
#include "servo.h"
#include "motor.h"
#include "pid.h"
#include "ap.h"


clock_time_t f_log_timeout = 0;


static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = 115200,
		.charlength = US_MR_CHRL_8_BIT,
		.paritytype = US_MR_PAR_NO,
		.stopbits = US_MR_NBSTOP_1_BIT,
	};

	/* Configure console UART. */
    sysclk_enable_peripheral_clock(UART1_SERIAL_BAUDRATE);
	stdio_serial_init(UART1, &uart_serial_options);
}

int main(void)
{
    sysclk_init();
    board_init();
    delay_init();
    pmc_enable_periph_clk(IMU_TWI_ID);
    pmc_enable_periph_clk(ID_PWM);
    config_init();
    cph_millis_init();

    cli_init();
    configure_console();
    ap_init();

    puts("\r\n\r\nsam4d32c imu demo...\r\n");

    for (int i=0; i<10; i++) {
        printf(".");
        delay_ms(100);
    }
    printf("\r\n");

    
    if (imu_init()) {

        motor_init();

        // Calibrate the imu
        imu_calibrate();
        config.imu_calibrate = false;
 
        

        printf("please press button to arm motors\r\n");
        while(true) {
            if(ioport_get_pin_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
                ioport_toggle_pin_level(LED0_GPIO);
                config.motor_armed = true;
                printf("motor_armed: %d\r\n", config.motor_armed);
                break;
                delay_ms(100); 
            }
        }

        while(true) {


            if (config.imu_calibrate) {
                imu_calibrate();
                pid_init();
                config.imu_calibrate = false;
            }
            // printf("imu calibrated...\r\n");
            cli_tick();
            imu_tick();
            motor_tick();

            // long y = (long) ap.imu.x_axis;

            // long x = (long) pid_tick();

            long x = ap.imu.x_axis;

            long power_left = map(x, ANGLE_MID, ANGLE_MAX, PWM_MIN, PWM_MAX);
            long power_right = map(x, ANGLE_MID, ANGLE_MIN, PWM_MIN, PWM_MAX);

            if (config.motor_armed) {
                motor_set_power(motors[1], power_left + config.motor_offset);
                motor_set_power(motors[0], power_right + config.motor_offset);
            }


            if (config.log_imu) {
                if (cph_get_millis() >= f_log_timeout) {
                    f_log_timeout = cph_get_millis() + 100;
                    // printf("x_axis:power_right\t%f\t%ld\t%ld\t%ld\r\n", ap.imu.x_axis, power_left, power_right, x);
                    printf("x_axis:\t%f\t", ap.imu.x_axis);
                    printf("y_axis:\t%f\t", ap.imu.y_axis);
                    printf("pwr_left:\t%ld\t", power_left);
                    printf("pwr_right:\t%ld\t", power_right);
                    printf("\r\n");
                    // printf("roll/pitch/yaw/mag %f %f %f %f \r\n", ap.imu.x_axis, ap.imu.y_axis, ap.imu.z_axis, ap.mag.z_axis);
                }
            }
        }

    }
}
