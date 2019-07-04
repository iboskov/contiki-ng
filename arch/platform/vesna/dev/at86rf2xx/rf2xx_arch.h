#ifndef RF2XX_ARCH_H_
#define RF2XX_ARCH_H_

#include <stdint.h>
#include "rf2xx_registermap.h"
#include "vsnspi_new.h"


/*
	Radio chip specific configuration
*/

// Radio identification procedure
// Manfacturer ID is same for all radios == 0x00 0x1F
#define RF2XX_MAN_ID_0			(0x1F)
#define RF2XX_MAN_ID_1			(0x00)

// Pattern: (PART, VERSION)
// RF233B (0x0B, 0x02)
// RF233A (0x0B, 0x01)
// RF232A (0x0A, 0x02)
// RF212B (0x07, 0x03)
// RF212A (0x07, 0x01)
// RF231  (0x03, 0x02)
// RF230B (0x02, 0x02)
// RF230A (0x02, 0x01)

typedef enum {
	RF2XX_UNDEFINED = 0x00,
	RF2XX_AT86RF212 = 0x07,
	RF2XX_AT86RF231 = 0x03,
	RF2XX_AT86RF230 = 0x02,
	//RF2XX_AT86RF232 = 0x0A, // never tested
	RF2XX_AT86RF233 = 0x0B,
} rf2xx_radio_t;


typedef enum {
	TIME__VCC_TO_P_ON,			// time to cold start
	TIME__SLEEP_TO_TRX_OFF,		// time to wake-up
	TIME__TRX_OFF_TO_PLL_ON,	// time to start internal radio periphery
	TIME__TRX_OFF_TO_RX_ON,		// time to reach radio reception state
	TIME__PLL_ON_TO_BUSY_TX,	// time to reach transmittion busy state

	TIME__FORCE_TRX_OFF,		// time to perform soft-reset

	TIME__CCA,					// channel sensing

	TIME__TRX_OFF_TO_SLEEP,		// time to reach to sleep (not reliable as it depends on CLK

	TIME__TRX_OFF_TO_AACK,		// time to reach reception with auto-acknowledge state
	TIME__FRAME,				// max frame transmission time
	//TIME__P_ACK,				// (deprecate)
} rf2xx_timing_t;


// the delay between radio Tx request and SFD sent, in rtimer ticks
// This includes (FORCE_TRX_OFF, TX_ARET/PLL_ON, delay, PREAMBLE+SFD (p.39))
#define RF2XX_DELAY_BEFORE_TX		((unsigned)US_TO_RTIMERTICKS(350)) // (2+2) + 110 + 16 + 160

// the delay between radio Rx request and start listening, in rtimer ticks
// (we need time from end of Tx until it reaches Rx mode again)
// (FORCE_TRX_OFF, RX migration)
#define RF2XX_DELAY_BEFORE_RX		((unsigned)US_TO_RTIMERTICKS(950)) // (2+2) + 1 + 110

// he number of header and footer bytes of overhead at the PHY layer after SFD
#define RF2XX_PHY_OVERHEAD			(3) // 1 length byte + 2 bytes CRC

// the delay between the end of SFD reception and the radio returning 1 to receiving_packet()
// 
#define RF2XX_DELAY_BEFORE_DETECT	US_TO_RTIMERTICKS(64) // RX_START flag

// 1 / (250 kbps / 8) == 32 us/byte
#define RF2XX_BYTE_AIR_TIME			(32) // us for 1 byte (@250kbps)

// (optional) the default TSCH timeslot timing (10ms timeslots)
//#define RF2XX_CONF_DEFAULT_TIMESLOT_TIMING	(tsch_timeslot_timing_us_10000)


/*
	Radio boards specific settings
*/
#if AT86RF2XX_BOARD_SNR
	#define SPI_PORT 			(VSN_SPI2)

	// Inverted chip select
	#define CSN_PIN				(GPIO_Pin_12)
	#define CSN_PORT			(GPIOB)

	// (Optional) DIG2 -- for timing
	#define DIG2_PIN			(GPIO_Pin_7)
	#define DIG2_PORT			(GPIOB)

	// IRQ -- interrupts from radio
	#define IRQ_PIN				(GPIO_Pin_9)
	#define IRQ_PORT			(GPIOC)

	// RSTN -- inverted RESET
	#define RSTN_PIN			(GPIO_Pin_11)
	#define RSTN_PORT			(GPIOC)

	// Multi-functional pin
	#define SLP_TR_PIN			(GPIO_Pin_10)
	#define SLP_TR_PORT			(GPIOC)

	// Mapping IRQ to ISR
	#define EXTI_IRQ_LINE		(EXTI_Line9)
	#define EXTI_IRQ_PORT		(GPIO_PortSourceGPIOC)
	#define EXTI_IRQ_PIN		(GPIO_PinSource9)
	#define EXTI_IRQ_CHANNEL	(EXTI9_5_IRQn)


#elif AT86RF2XX_BOARD_ISMTV_V1_0
	#define SPI_PORT			(VSN_SPI1)

	// Inverted chip select
	#define CSN_PIN				(GPIO_Pin_0)
	#define CSN_PORT			(GPIOA)

	// (Optional) DIG2 -- for timing
	#define DIG2_PIN			(GPIO_Pin_1)
	#define DIG2_PORT			(GPIOA)

	// IRQ -- interrupts from radio
	#define IRQ_PIN				(GPIO_Pin_3)
	#define IRQ_PORT			(GPIOA)

	// RSTN -- inverted RESET
	#define RSTN_PIN			(GPIO_Pin_13)
	#define RSTN_PORT			(GPIOA)

	// Multi-functional pin
	#define SLP_TR_PIN			(GPIO_Pin_15)
	#define SLP_TR_PORT			(GPIOA)

	// Mapping IRQ to ISR
	#define EXTI_IRQ_LINE		(EXTI_Line3)
	#define EXTI_IRQ_PORT		(GPIO_PortSourceGPIOA)
	#define EXTI_IRQ_PIN		(GPIO_PinSource3)
	#define EXTI_IRQ_CHANNEL	(EXTI3_IRQn)


#elif AT86RF2XX_BOARD_ISMTV_V1_1
	#define SPI_PORT			(VSN_SPI1)

	// Inverted chip select
	#define CSN_PIN				(GPIO_Pin_0)
	#define CSN_PORT			(GPIOA)

	// (Optional) DIG2 -- for timing
	//#define DIG2_PIN			(GPIO_Pin_1)
	//#define DIG2_PORT			(GPIOA)

	// IRQ -- interrupts from radio
	#define IRQ_PIN				(GPIO_Pin_3)
	#define IRQ_PORT			(GPIOA)

	// RSTN -- inverted RESET
	#define RSTN_PIN			(GPIO_Pin_1)
	#define RSTN_PORT			(GPIOA)

	// Multi-functional pin
	#define SLP_TR_PIN			(GPIO_Pin_8)
	#define SLP_TR_PORT			(GPIOB)

	// Mapping IRQ to ISR
	#define EXTI_IRQ_LINE		(EXTI_Line3)
	#define EXTI_IRQ_PORT		(GPIO_PortSourceGPIOA)
	#define EXTI_IRQ_PIN		(GPIO_PinSource3)
	#define EXTI_IRQ_CHANNEL	(EXTI3_IRQn)


#else
	#warning "No predefined board was selected!"
	#warning "No pins have been defined for AT86RF2xx radio!"
#endif

// Maximum supported speed is 8MHz
#define AT86RF2XX_SPI_SPEED			((uint32_t)8000000)

#endif
