#ifndef RF2XX_HAL_H_
#define RF2XX_HAL_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"

//#include "vsntime.h"
#include "vsnspi_new.h"

#include "rf2xx_registermap.h"
#include "rf2xx_arch.h"
#include "rf2xx.h"


#define RF2XX_MAX_FRAME_SIZE	(127)
#define RF2XX_MIN_FRAME_SIZE	(3)
#define RF2XX_CRC_SIZE			(2)
#define RF2XX_LQI_SIZE			(1)
#define RF2XX_MAX_PAYLOAD_SIZE	(RF2XX_MAX_FRAME_SIZE - RF2XX_CRC_SIZE)


typedef union {
	struct {
		uint8_t SLEEP:1;	// Indicate radio sleep state
		uint8_t PLL_LOCK:1; // Indicate when TX/RX state was reached
		uint8_t RX_START:1; // Indicate when frame was detected
		uint8_t AMI:1;		// Indicate when frame address matched
		uint8_t TRX_END:1;	// Indicate TX/RX operation completed
		uint8_t TRX_UR:1;	// Indicate TX/RX buffer access violation
		uint8_t CCA:1;		// Indicate when CCA is in progress
	};
	uint8_t value;
} rf2xx_flags_t;


// Simplify decode of AT86RF2xx IRQ bits
typedef union {
	struct {
		uint8_t IRQ0_PLL_LOCK:1;
		uint8_t IRQ1_PLL_UNLOCK:1;
		uint8_t IRQ2_RX_START:1;
		uint8_t IRQ3_TRX_END:1;
		uint8_t IRQ4_AWAKE_END:1;
		uint8_t IRQ5_AMI:1;
		uint8_t IRQ6_TRX_UR:1;
		uint8_t IRQ7_BAT_LOW:1;
	};
	uint8_t value;
} rf2xx_irq_t;


uint8_t regRead(uint8_t addr);
uint8_t bitRead(uint8_t addr, uint8_t mask, uint8_t offset);

void regWrite(uint8_t addr, uint8_t value);
void bitWrite(uint8_t addr, uint8_t mask, uint8_t offset, uint8_t value);


/*
	Gather radio metrics
*/

enum {
	rxDetected,
	rxAddrMatch,
	rxSuccess,
	rxToStack,

	txCount,
	txSuccess,
	txCollision,
	txNoAck,

	spiError,

	RF2XX_STATS_COUNT
};

#if RF2XX_CONF_STATS
	extern uint32_t rf2xxStats[RF2XX_STATS_COUNT];
	#define RF2XX_STATS_ADD(x)		do { if (RF2XX_CONF_STATS) rf2xxStats[x]++; } while(0)
	#define RF2XX_STATS_GET(x)		((RF2XX_CONF_STATS) ? (rf2xxStats[x]) : 0)
	#define RF2XX_STATS_RESET()		do { if (RF2XX_CONF_STATS) memset(rf2xxStats, 0, sizeof(rf2xxStats[0]) * RF2XX_STATS_COUNT); } while(0)
#else
	#define RF2XX_STATS_ADD(x)
	#define RF2XX_STATS_GET(x)
	#define RF2XX_STATS_RESET()

#endif // RF2XX_DEBUG





#endif // RF2XX_HAL_H_
