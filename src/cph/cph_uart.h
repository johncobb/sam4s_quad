

#ifndef CPH_UART_H
#define CPH_UART_H

#include <asf.h>

#define UART1_SERIAL_BAUDRATE       115200
#define UART1_SERIAL_CHANNEL_MODE   UART_MR_CHMODE_NORMAL
#define UART1_SERIAL_MODE           UART_MR_PAR_NO
#define PINS_UART1                  (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1)
#define PINS_UART1_FLAGS            (PIO_PERIPH_A | PIO_DEFAULT)
#define PINS_UART1_MASK             (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1)
#define PINS_UART1_PIO              PIOB
#define PINS_UART1_ID               ID_PIOB
#define PINS_UART1_TYPE             PIO_PERIPH_A
#define PINS_UART1_ATTR             PIO_DEFAULT

/*
 * Exploratory code to make uart handling a bit more object oriented
 */
typedef void (*uart_init_func_t)(void);
typedef void (*uart_tick_func_t)(void);

typedef void (*uart_ondatareceive_func_t)(uint8_t buffer, uint32_t len);

typedef struct{
    uart_ondatareceive_func_t on_datareceive;
} uart_event_handler_t;


typedef struct {
    uart_init_func_t uart_init;
    uart_tick_func_t uart_tick;
} uart_handler_t;

typedef struct uart_cfg {
    sam_uart_opt_t sam_uart_opt;
    Uart *p_uart;
    uint32_t uart_id;
    uint32_t *p_pio;
    pio_type_t pio_type;
    uint32_t ul_mask;
    uint32_t ul_attribute;
    uint32_t ul_irq;
    uint32_t ul_source;
    uart_ondatareceive_func_t on_datareceive;
} uart_cfg_t;

// typedef struct{
//     uart_cfg_t uart_cfg;
// 	char *tx_buff;
// 	uint32_t tx_len;
// 	char *rx_buff;
// 	uint32_t rx_len;
// 	uint32_t baudrate;
// } uart_conf_t;
/* end */


void lib_uart_cfg(uart_cfg_t *);
uint32_t lib_uart_rxready(uart_cfg_t *);
uint8_t lib_uart_read(uart_cfg_t *);
void lib_uart_write(uart_cfg_t *, uint8_t);
void lib_uart_writebytes(uart_cfg_t *, char*, int);
// void lib_uart_tick(uart_cfg_t *);
uint32_t lib_uart_tick(uart_cfg_t *);
void uart_uart1_init(void);
void uart_uart1_tick(void);
uint32_t uart_uart1_rx_ready(void);
uint8_t uart_uart1_read(void);
void uart_uart1_write(uint8_t data);
void uart_uart1_write_bytes(char* data, int size);

void uart_set_ondatareceive_func(uart_ondatareceive_func_t fnc);

#endif
