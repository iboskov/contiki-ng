#include "contiki-conf.h"
#include "dev/uart1.h"
#include "sys/process.h"

#include "vsn.h"
#include "vsnusart.h"
#include "vsnledind.h"

#include <stdio.h>

#ifndef UART1_RX_BUF_SIZE
#define UART1_RX_BUF_SIZE	(128)
#endif

#ifndef UART1_BAUDRATE
#define UART1_BAUDRATE (unsigned long)(115200)
#endif


PROCESS(uart1_rx_process, "UART1 Rx process");




static int (*input_handler)(unsigned char chr) = NULL;

static char rxBuffer[UART1_RX_BUF_SIZE];


void uart1_init(unsigned long ubr) {
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = ubr;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    vsnUSART_init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	setvbuf(stdout, NULL, _IONBF, 0);

	input_handler = NULL;
}

void uart1_writeb(unsigned char chr) {
    vsnUSART_write(USART1, (const char *)&chr, 1);
}

void uart1_set_input(int (*input) (unsigned char chr)) {
    input_handler = input;
}

/*
void contiki_uart1_isr(void) {
	process_poll(&uart1_rx_process);
}*/



PROCESS_THREAD(uart1_rx_process, ev, data)
{
	PROCESS_BEGIN();

	int count, i;

	while (1) {
		if (input_handler != NULL) {
			count = vsnUSART_read(USART1, rxBuffer, UART1_RX_BUF_SIZE);
			for (i = 0; i < count; i++) {
				input_handler((unsigned char) rxBuffer[i]);
			}
		}

		//PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
		PROCESS_PAUSE();
	}

	PROCESS_END();
}
