#ifndef RF2XX_HAL_H_
#define RF2XX_HAL_H_

#include <stdint.h>
#include "rf2xx_registermap.h"
#include "vsnspi_new.h"

#define RF2XX_CONF_CALIBRATION_PERIOD   (240) // seconds (~4min)

// Radio identification procedure
// Manfacturer ID is same for all radios == 0x00 0x1F
#define RF2XX_MAN_ID_0			(0x1F)
#define RF2XX_MAN_ID_1			(0x00)



#define RF2XX_UNDEFINED     (0x00)
#define RF2XX_AT86RF212     (0x07)
#define RF2XX_AT86RF231     (0x03)
#define RF2XX_AT86RF230     (0x02)
//#define RF2XX_AT86RF232     (0x0A) // was never tested
#define RF2XX_AT86RF233     (0x0B)






// Maximum supported speed is 8MHz
#define RF2XX_SPI_SPEED			((uint32_t)8000000)


// Board specific configurations
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

	// Chip select pins for CC101 radio
	#define CC1101_CSN_PIN      (GPIO_Pin_9)
	#define CC1101_CSN_PORT     (GPIOB)


#else
	#warning "No predefined board was selected!"
	#error "No pins have been defined for AT86RF2xx radio!"
#endif

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


void regWrite(uint8_t addr, uint8_t value);
uint8_t regRead(uint8_t addr);

void bitWrite(uint8_t addr, uint8_t mask, uint8_t offset, uint8_t value);
uint8_t bitRead(uint8_t addr, uint8_t mask, uint8_t offset);

void CC1101_regWrite(uint8_t addr, uint8_t value);
void CC1101_reset(void);
void CC1101_set_clock_output(void);

#endif