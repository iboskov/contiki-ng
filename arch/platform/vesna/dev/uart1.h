#ifndef USART_H_
#define USART_H_

#include "vsnusart.h"
#include "sys/process.h"

#define BAUD2UBR(baud) baud

#ifdef UART1_CONF_BAUDRATE
#define UART1_BAUDRATE		UART1_CONF_BAUDRATE
#else
#define UART1_BAUDRATE		((uint32_t)115200)
#endif


PROCESS_NAME(uart1_rx_process);

void uart1_init(unsigned long ubr);
void uart1_writeb(unsigned char chr);
void uart1_set_input(int (*input)(unsigned char chr));

#endif
