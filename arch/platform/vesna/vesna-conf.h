#ifndef VESNA_CONF_H_
#define VESNA_CONF_H_

// Enable/disable hardware watchdog. Default: Disabled
#ifndef WATCHDOG_CONF_ENABLED
#define WATCHDOG_CONF_ENABLED   (0)
#endif

// Enable/disable UART*. Default: Enabled
#ifndef UART_CONF_ENABLED
#define UART_CONF_ENABLED       (1)
#endif

// Set default baud rate for UART1. Default: 115200
#ifndef UART1_CONF_BAUDRATE
#define UART1_CONF_BAUDRATE     (115200)
#endif


#endif