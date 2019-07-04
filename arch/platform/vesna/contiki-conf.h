#ifndef CONTIKI_CONF_H
#define CONTIKI_CONF_H

#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#define CCIF
#define CLIF

#ifdef PROJECT_CONF_PATH
#include PROJECT_CONF_PATH
#endif


#include "vesna-def.h"


#define CLOCK_CONF_SECOND	(1000)



// Platform specific timer bit-size
// STM32F103 has 16-bit counter
#define RTIMER_CONF_CLOCK_SIZE  (8)
typedef uint64_t clock_time_t;
typedef uint32_t uip_stats_t;

#define PLATFORM_SUPPORTS_BUTTON_HAL  (1)


#define STACK_CHECK_CONF_ENABLED	(0)

#ifdef AT86RF2XX

#define rf2xx_driver_max_payload_len    (125)

//#define MAC_CONF_WITH_NULLMAC	1
#define NETSTACK_CONF_RADIO 	rf2xx_driver

#define NULLRDC_802154_AUTOACK		1
#define NULLRDC_802154_AUTOACK_HW	1

// Extended mode allows automatic retransmission
#define RDC_CONF_HARDWARE_CSMA		1

// Extended mode does automatic acknowledgements
#define RDC_CONF_HARDWARE_ACK		1
#endif

#endif
