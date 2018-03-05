#include "cph_cli.h"
#include "cph_console.h"


uint8_t uc_char;
uint8_t uc_flag;

// uint8_t cph_console_tick(void)
// {
//     uc_flag = uart_read(CONSOLE_UART, &uc_char);
//     if (!uc_flag) {
//         switch (uc_char) {
//             case '\r':
//             return CS_INVALID;
//             case 'l':
//             return CS_PWMMIN;
//             case 'm':
//             return CS_PWMMID;
//             case 'h':
//             return CS_PWMMAX;
//             case '-':
//             return CS_PWMSTEPDEC;
//             case '+':
//             return CS_PWMSTEPINC;
//             default:
//             return CS_NOINPUT;              
//         }
//     } else {
//         return CS_NOINPUT;
//     }
// }

uint8_t cph_console_tick(void)
{
    uint8_t uc_char;
	uint8_t uc_flag;

	if (uart_is_rx_ready(CONSOLE_UART)) {
		uc_flag = uart_read(CONSOLE_UART, &uc_char);
		if (!uc_flag) {
			cli_put_char(uc_char);
		}
	}
}