#include <asf.h>

#include <stdio.h>
#include <string.h>
#include "cph_config.h"
#include "cph_millis.h"
#include "cph_util.h"
#include "cph_console.h"
#include "cph_cli.h"
#include "imu.h"
#include "servo.h"
#include "motor.h"
#include "pid.h"
#include "ap.h"


clock_time_t f_log_timeout = 0;

static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
    sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
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

        // while (true) {
        //     cli_tick();
        //     delay_ms(100);
        // }

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
            cli_tick();
            imu_tick();
            motor_tick();

            // long y = (long) ap.imu.x_axis;

            long x = (long) pid_tick();

            long power_left = map(x, ANGLE_MID, ANGLE_MAX, PWM_MIN, PWM_MAX);
            long power_right = map(x, ANGLE_MID, ANGLE_MIN, PWM_MIN, PWM_MAX);

            if (config.motor_armed) {
                motor_set_power(motors[1], power_left + config.motor_offset);
                motor_set_power(motors[0], power_right + config.motor_offset);
            }


            if (config.log_imu) {
                if (cph_get_millis() >= f_log_timeout) {
                    f_log_timeout = cph_get_millis() + 50;
                    printf("roll/pitch/yaw/mag error/pid: %f %f %f %f %f %f\r\n", ap.imu.x_axis, ap.imu.y_axis, ap.imu.z_axis, ap.mag.z_axis, pid);
                }
            }
            
        }

    }


    
    // while(1) {

    //     if(ioport_get_pin_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
    //         ioport_toggle_pin_level(LED0_GPIO);
    //         delay_ms(500); 
    //     }


    // }
}




// void test_twi(void)
// {
//     uint32_t i;
//     twi_options_t opt;
//     twi_packet_t packet_tx;
//     twi_packet_t packet_rx;

// 	/* Configure the options of TWI driver */
// 	opt.master_clk = sysclk_get_peripheral_hz();
//     opt.speed      = TWI_CLK;
    

//     	/* Configure the data packet to be transmitted */
// 	packet_tx.chip        = IMU_ADDRESS;
// 	// packet_tx.addr[0]     = EEPROM_MEM_ADDR >> 8;
//     // packet_tx.addr[1]     = EEPROM_MEM_ADDR;
//     packet_tx.addr[0]     = MPU6050_RA_WHO_AM_I;
// 	packet_tx.addr_length = 1;
// 	packet_tx.buffer      = (uint8_t *) 1;
// 	packet_tx.length      = 1;

// 	/* Configure the data packet to be received */
// 	// packet_rx.chip        = packet_tx.chip;
// 	// packet_rx.addr[0]     = packet_tx.addr[0];
// 	// packet_rx.addr[1]     = packet_tx.addr[1];
// 	// packet_rx.addr_length = packet_tx.addr_length;
// 	// packet_rx.buffer      = gs_uc_test_data_rx;
//     // packet_rx.length      = packet_tx.length;
    

//     if (twi_master_init(IMU_TWI, &opt) != TWI_SUCCESS) {

//         ioport_toggle_pin_level(LED0_GPIO);
            
//         while (1) {
//             /* Capture error */
//             delay_ms(100); 
//         }
//     }
// }