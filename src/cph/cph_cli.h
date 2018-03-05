#ifndef CPH_CLI_H_
#define CPH_CLI_H_

#include <asf.h>

// command line interface parsing results
enum cli_parse_result {
 CLI_TKNOTFOUND = 0,
 CLI_TKFOUND,
 CLI_TKERROR
};


// token start and end
#define	CLI_DELIM			" "
#define CLI_TKSTART			'*'
#define CLI_TKEND			'\r'
#define CLI_RX_BUFFER_SIZE 	512


typedef struct
{
	unsigned char buffer[CLI_RX_BUFFER_SIZE];
	int head;
	int tail;
} CLI_BUFFER;

typedef struct
{
	char *cmd;
	char *parm;
} cli_result_t;


void cli_put_char(unsigned char c);
void cli_init();
void cli_tick(void);

#endif