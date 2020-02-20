#include <stdio.h>
#include <string.h>

#include "sys/log.h"

#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "vsnspi_new.h"

#include "cc1101_hal.h"


#define LOG_LEVEL   LOG_LEVEL_INFO
#define LOG_MODULE  "CC1101"


#define SPI_PORT            (VSN_SPI1)
#define CC1101_SPI_SPEED    ((uint32_t)8000000)

#define CC1101_SCLK_PIN     (GPIO_Pin_5)
#define CC1101_SCLK_PORT    (GPIOA)

#define CC1101_MISO_PIN     (GPIO_Pin_6)
#define CC1101_MISO_PORT    (GPIOA)

#define CC1101_MOSI_PIN     (GPIO_Pin_7)
#define CC1101_MOSI_PORT    (GPIOA)

#define CC1101_CSN_PIN      (GPIO_Pin_9)
#define CC1101_CSN_PORT     (GPIOB)


// SPI struct (from VESNA drivers) for CC1101 radio
static vsnSPI_CommonStructure CC1101_SPI_Structure;
vsnSPI_CommonStructure * const CC1101_SPI = &CC1101_SPI_Structure;

static void
CC1101_spiErrorCallback(void *cbDevStruct)
{
	vsnSPI_CommonStructure *spi = cbDevStruct;
	vsnSPI_chipSelect(spi, SPI_CS_HIGH);
	LOG_ERR("SPI error callback triggered for CC radio\n");
}


// Set clock output on pin GDO0 of radio CC1101 to be 13.5 MHz
// For other possible freq see radio datasheet
void
CC1101_init(void)
{
    // These settings are passed as pointer and they have to exist for the runtime
    // and does not change. That is why struct is `static`.
	static SPI_InitTypeDef spiConfig = {
		//.SPI_BaudRatePrescaler is overwritten later
		.SPI_Direction = SPI_Direction_2Lines_FullDuplex,
		.SPI_Mode = SPI_Mode_Master,
		.SPI_DataSize = SPI_DataSize_8b,
		.SPI_CPOL = SPI_CPOL_Low,
		.SPI_CPHA = SPI_CPHA_1Edge,
		.SPI_NSS = SPI_NSS_Soft,
		.SPI_FirstBit = SPI_FirstBit_MSB,
		.SPI_CRCPolynomial = 7,
	};

    vsnSPI_initCommonStructure(
        CC1101_SPI,
        SPI_PORT,
        &spiConfig,
        CC1101_CSN_PIN,
        CC1101_CSN_PORT,
        CC1101_SPI_SPEED     //speed can be the same as rf2xx
    );

    // Init SPI for CC radio
    vsnSPI_Init(CC1101_SPI, CC1101_spiErrorCallback);

    // Set GDO0 output pin to desire freq (0x32 = 13.5MHz)
        CC1101_setCS();

        // Radio must be in IDLE state
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 0) {
            CC1101_regWrite(0x02, 0x32);
        }
        else {
            // First reset the radio and then set the freq
            CC1101_reset();
            CC1101_regWrite(0x02, 0x32);
        }
        CC1101_clearCS();

        LOG_INFO("GDO0 output freq set to 13.5 MHz\n");

    // Give SPI control back to rf2xx
    vsnSPI_deInit(CC1101_SPI);
}

void
CC1101_regWrite(uint8_t addr, uint8_t value)
{
    vsnSPI_ErrorStatus CC_status;
    uint8_t dummy __attribute__((unused));
    uint8_t state;

    //CC1101_clear_CS();
    CC1101_setCS();

        int count = 0;			
			while((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)) && (count < 200000))	
					count++;										
			if(count >= 200000){ 							
				LOG_ERR("CC1101_regWrite: MISO is not low! \n");											
			}	

    CC_status = vsnSPI_pullByteTXRX(CC1101_SPI, addr , &state);
        if (VSN_SPI_SUCCESS != CC_status) LOG_WARN("ERR while sending\n");
        LOG_DBG("regWrite address state is 0x%02x  \n",state);

    CC_status = vsnSPI_pullByteTXRX(CC1101_SPI, value, &state);
        if (VSN_SPI_SUCCESS != CC_status) LOG_WARN("ERR while receiving\n");
        LOG_DBG("regWrite data state is 0x%02x  \n",state);

    CC1101_clearCS();        
}

void
CC1101_reset(void)
{
    uint8_t dummy __attribute__((unused));

    //SCLK = 1 and MOSI = 0
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	GPIO_ResetBits(GPIOA, GPIO_Pin_7);

    //Strobe CS low/high
	GPIO_SetBits(GPIOB, GPIO_Pin_9);
	vsnTime_delayUS(10);
	GPIO_ResetBits(GPIOB, GPIO_Pin_9);
	vsnTime_delayUS(10); 

    //Hold CS low for at least 40us
	GPIO_SetBits(GPIOB, GPIO_Pin_9);
	vsnTime_delayUS(100); 

    //Pull CS low and wait for MISO to go low
	GPIO_ResetBits(GPIOB, GPIO_Pin_9);

	int count = 0;
			while((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)) && (count < 200000))	
					count++;										
			if(count >= 200000){ 							
				LOG_ERR("CC1101_reset: MISO is not low!\n");												
			}		

	vsnTime_delayUS(300);

    CC1101_setCS();
       
        count = 0;			
			while((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)) && (count < 200000))	
					count++;										
			if(count >= 200000){ 							
				LOG_ERR("CC1101_reset: MISO is not low! \n");											
			}

    //Isue SRES (0x30) strobe on MOSI line
    vsnSPI_pullByteTXRX(CC1101_SPI, 0x30 , &dummy);

    vsnTime_delayUS(50);
    
    CC1101_clearCS();
	
    //When MISO goes low again, reset is complete
        count = 0;    
			while((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)) && (count < 500000))	
					count++;										
			if(count >= 500000){ 							
				LOG_ERR("CC1101_reset: MISO is not low!\n");												
			}	

	vsnTime_delayUS(300);

    LOG_INFO("CC1101 Radio reset complete!\n");
}

void
CC1101_clearCS(void)
{
    vsnSPI_chipSelect(CC1101_SPI, SPI_CS_HIGH);
}

void
CC1101_setCS(void)
{
    vsnSPI_chipSelect(CC1101_SPI, SPI_CS_LOW);
}