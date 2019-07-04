#include <stdio.h>
#include <string.h>

#if !RF2XX_CONF_CHECKSUM
#include <lib/crc16.h>
#endif

#include "contiki.h"
#include "sys/log.h"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include "sys/energest.h"

#include "at86rf2xx/rf2xx_registermap.h"
#include "at86rf2xx/rf2xx_hal.h"
#include "at86rf2xx/rf2xx.h"

#define LOG_MODULE "rf2xx"
#define LOG_LEVEL LOG_CONF_LEVEL_RF2XX


#if CONTIKI
    PROCESS(rf2xx_process, "AT86RF2xx driver");
	//PROCESS(rf2xx_calibration_process, "AT86RF2xx calibration");
#endif

#define ASSERT(condition, M, ...) do { \
    if (!(condition)) { LOG_ERR(M, ##__VA_ARGS__); } \
} while(0)

/* Macros to access I/O functions */

// SPI Chip-Select operations
#define clearCS()		(vsnSPI_chipSelect(rf2xxSPI, SPI_CS_HIGH))
#define setCS()			(vsnSPI_chipSelect(rf2xxSPI, SPI_CS_LOW))
#define getCS()			(!GPIO_ReadInputDataBit(CSN_PORT, CSN_PIN))

// Radio's RESET operations (ENABLE pin)
#define clearRST()		(GPIO_SetBits(RSTN_PORT, RSTN_PIN))
#define setRST()		(GPIO_ResetBits(RSTN_PORT, RSTN_PIN))
#define getRST()		(!GPIO_ReadInputDataBit(RSTN_PORT, RSTN_PIN))

// Radio's sleep trigger operations (radio sleep mode)
#define clearSLPTR()	(GPIO_ResetBits(SLP_TR_PORT, SLP_TR_PIN))
#define setSLPTR()		(GPIO_SetBits(SLP_TR_PORT, SLP_TR_PIN))
#define getSLPTR()		(GPIO_ReadInputDataBit(SLP_TR_PORT, SLP_TR_PIN))

// SNC interrupt line operation
#define clearEXTI()		(EXTI_ClearFlag(EXTI_IRQ_LINE))


/* SPI command bytes */

// Register access
#define CMD_REG_MASK	(0x3F)
#define CMD_REG			(0x80)

// Frame Buffer access
#define CMD_FB_MASK		(0x1F)
#define CMD_FB			(0x20)

// SRAM acess
#define CMD_SR_MASK		(0x1F)
#define CMD_SR			(0x00)

// R/W operation bit
#define CMD_READ		(0x00)
#define CMD_WRITE		(0x40)


/* Exposed structures */

// Exposed radio metrics
volatile uint8_t rf2xx_last_lqi;
volatile int8_t rf2xx_last_rssi;

// Exposed radio timings
volatile static rtimer_clock_t last_packet_timestamp;
//volatile uint16_t rf2xx_frame_start_time;
//volatile uint16_t rf2xx_frame_end_time;

// (optional) statistics of the radio
#if RF2XX_CONF_STATS
uint32_t rf2xxStats[RF2XX_STATS_COUNT];
#endif

static uint8_t txBuffer[2 + RF2XX_MAX_FRAME_SIZE];


/* Local structures */

// SPI struct (from VESNA drivers) and constant (immutable) pointer to it.
static vsnSPI_CommonStructure SPI_rf2xxStructure;
vsnSPI_CommonStructure * const rf2xxSPI = &SPI_rf2xxStructure;

// EXTI (interrupt) struct (from STM) and constant (immutable) pointer to it.
static EXTI_InitTypeDef EXTI_rf2xxStructure;
EXTI_InitTypeDef * const rf2xxEXTI = &EXTI_rf2xxStructure;

// Internal driver state machine & flags
volatile static rf2xx_flags_t flags;

// Internal timer (for timeout situations)
static vsnTime_Timeout rxTimeout;
//static uint32_t lastCalibration;

// Store radio type for later discrimitation
static rf2xx_radio_t rf2xx_radio = RF2XX_UNDEFINED;
//uint8_t rf2xx_radio_rev = 0x00;



static float rf2xx_getFrequency(void) {
	uint8_t channel, ccBand, ccNumber;

	channel = bitRead(SR_CHANNEL);

	switch (rf2xx_radio)
	{
		case RF2XX_AT86RF233:
		//case RF2XX_AT86RF232:
		case RF2XX_AT86RF231:
		case RF2XX_AT86RF230:
			// AT86RF23x radios work on 2.4GHz ISM band
			return 2405 + 5 * (channel - 11);

		case RF2XX_AT86RF212:
			// AT86RF212 is a bit more advanced.
			// It supports ISM bands as set by SR_CHANNEL bits. (CC_BAND must be 0)
			// It also support custom frequencies set by CC_BAND and CC_NUMBER bits.
			ccBand = bitRead(SR_CC_BAND);
			ccNumber = bitRead(SR_CC_NUMBER);
			switch (ccBand)
			{
				case 0: // It operates as specified in IEEE 802.15.4-2003/2006
					return channel ? (906 + 2 * (channel - 1)) : 868.3;
				case 1:
					return 769.0 + 0.1 * ccNumber;
				case 2:
					return 857.0 + 0.1 * ccNumber;
				case 3:
					return 903.0 + 0.1 * ccNumber;
				case 4:
					return 769 + ccNumber;
				case 5:
					return 833 + ccNumber;
				case 6: // reserved
				case 7: // reserved
				default:
					return 0;
			}
		default:
			return 0;
	}

	return 0;
}


uint32_t rf2xx_getTiming(rf2xx_timing_t timing) {
	/* MMohorcic suggested this way:
		Timing devices[] = {
			{0,1,2,3,4,5,6,7,8,9},
			{2,3,4,5,6,7,8,9,9,9},
			{0,1,2,3,4,5,6,7,8,9},
			{0,1,2,3,4,5,6,7,8,9},
			{0,1,2,3,4,5,6,7,8,9},
		};
		Timing* devicePtr;

		rf2xx_radio_t rf2xx_radio = RF2XX_UNDEFINED;

		uint32_t rf2xx_getTiming(rf2xx_timing_t timing) {
			uint16_t* p = (uint16_t*) (devicePtr + (((uint8_t)timing)*2));
			return *p;
		}

		OR SAFER version

		uint32_t rf2xx_getTiming(rf2xx_timing_t timing) {
			switch (timing) {
				case TIME__VCC_TO_P_ON:
					return devicePtr->TIME__VCC_TO_P_ON;
				case TIME__SLEEP_TO_TRX_OFF:
					return devicePtr->TIME__SLEEP_TO_TRX_OFF;
				case TIME__TRX_OFF_TO_PLL_ON:
					return devicePtr->TIME__TRX_OFF_TO_PLL_ON;
				case TIME__TRX_OFF_TO_RX_ON:
					return devicePtr->TIME__TRX_OFF_TO_RX_ON;
				case TIME__PLL_ON_TO_BUSY_TX:
					return devicePtr->TIME__PLL_ON_TO_BUSY_TX;
				case TIME__FORCE_TRX_OFF:
					return devicePtr->TIME__FORCE_TRX_OFF;
				case TIME__CCA:
					return devicePtr->TIME__CCA;
				case TIME__TRX_OFF_TO_SLEEP:
					return devicePtr->TIME__TRX_OFF_TO_SLEEP;
				case TIME__TRX_OFF_TO_AACK:
					return devicePtr->TIME__TRX_OFF_TO_AACK;
				case TIME__FRAME:
					return devicePtr->TIME__FRAME;
				default:
					return 4096;
			}
		}
	*/
	switch (timing) {
		case TIME__VCC_TO_P_ON:
			return 330;

		case TIME__SLEEP_TO_TRX_OFF:
			switch (rf2xx_radio) {
				case RF2XX_AT86RF233:
					return 180;
				//case RF2XX_AT86RF232:
				case RF2XX_AT86RF231:
				case RF2XX_AT86RF230:
				case RF2XX_AT86RF212:
				default:
					return 380;
			}

		case TIME__TRX_OFF_TO_PLL_ON:
		case TIME__TRX_OFF_TO_RX_ON:
			switch (rf2xx_radio) {
				case RF2XX_AT86RF233:
					return 80;
				//case RF2XX_AT86RF232:
				case RF2XX_AT86RF231:
				case RF2XX_AT86RF230:
					return 110;
				case RF2XX_AT86RF212:
				default:
					return 200;
			}

		case TIME__PLL_ON_TO_BUSY_TX:
			switch (rf2xx_radio) {
				case RF2XX_AT86RF233:
				//case RF2XX_AT86RF232:
				case RF2XX_AT86RF231:
				case RF2XX_AT86RF230:
					return 16;
				case RF2XX_AT86RF212:
				default:
					return 50; // Not sure about this
			}

		case TIME__FORCE_TRX_OFF:
			return 1;

		case TIME__CCA:
			switch (rf2xx_radio) {
				case RF2XX_AT86RF233:
				//case RF2XX_AT86RF232:
				case RF2XX_AT86RF231:
				case RF2XX_AT86RF230:
					return 140;
				case RF2XX_AT86RF212:
				default:
					return 280; // Not sure about this
			}

		case TIME__TRX_OFF_TO_SLEEP:
			// Not reliable as it depends on CLKM
			switch (rf2xx_radio) {
				case RF2XX_AT86RF233:
					return 26 * 4;
				//case RF2XX_AT86RF232:
				case RF2XX_AT86RF231:
				case RF2XX_AT86RF230:
				case RF2XX_AT86RF212:
				default:
					return 35 * 4;
			}

		case TIME__TRX_OFF_TO_AACK:
			return 110;

		case TIME__FRAME:
			return 4096; // Not sure about this for RF212

		default:
			return 4096;
	}
}


uint8_t rf2xx_getMaxChannel(void) {
	switch (rf2xx_radio) {
		case RF2XX_AT86RF233:
		case RF2XX_AT86RF231:
		case RF2XX_AT86RF230:
			return 26;
		case RF2XX_AT86RF212:
			return 10;
		default:
			return 0;
	}
}

uint8_t rf2xx_getMinChannel(void) {
	switch (rf2xx_radio) {
		case RF2XX_AT86RF233:
		case RF2XX_AT86RF231:
		case RF2XX_AT86RF230:
			return 11;
		case RF2XX_AT86RF212:
			return 0;
		default:
			return 0;
	}
}

uint8_t rf2xx_getMaxPwr(void) {
	// Get hex value to set it on radio
	switch (rf2xx_radio) {
		case RF2XX_AT86RF233:
		case RF2XX_AT86RF231:
		case RF2XX_AT86RF230:
			return 0x00;
		case RF2XX_AT86RF212:
			return 0x62; // For EU1
		default:
			return 0;
	}
}

uint8_t rf2xx_getMinPwr(void) {
	// Get hex value to set it on radio
	switch (rf2xx_radio) {
		case RF2XX_AT86RF233:
		case RF2XX_AT86RF231:
		case RF2XX_AT86RF230:
			return 0x0F;
		case RF2XX_AT86RF212:
			return 0x0D; // For EU1
		default:
			return 0;
	}
}


// Generic SPI callback function
static void
spiCallback(void *cbDevStruct)
{
	vsnSPI_ErrorStatus spiStatus;
	vsnSPI_CommonStructure *spi = cbDevStruct; // casting in separate line to simplify debugging
	spiStatus = vsnSPI_chipSelect(spi, SPI_CS_HIGH);
	if (VSN_SPI_SUCCESS != spiStatus) {
		LOG_ERR("SPI chip select error\n");
	}
}

// Generic SPI callback for errors
static void
spiErrorCallback(void *cbDevStruct)
{
	vsnSPI_CommonStructure *spi = cbDevStruct;
	vsnSPI_chipSelect(spi, SPI_CS_HIGH);
	LOG_ERR("SPI error callback triggered\n");
}


void
rf2xx_reset(void)
{
	uint8_t dummy __attribute__((unused));

	setRST(); // hold radio in reset state
	clearCS(); // clear chip select (default)
	clearSLPTR(); // prevent going to sleep
	clearEXTI(); // clear interrupt flag

	clearRST(); // release radio from RESET state

	// Radio should now go through P_ON -> TRX_OFF state migration
	vsnTime_delayUS(rf2xx_getTiming(TIME__VCC_TO_P_ON));
	ASSERT(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS), "TRX_OFF not reached\n");

	// Radio indentification process
	// This will loop as long as radio is not recognized.
	// We are not touching unknown device.
	{
		uint8_t partNum;
		//uint8_t verNum;
		do {
			LOG_INFO("Detecting AT86RF2xx radio\n");

			// Match JEDEC manufacturer ID
			if (regRead(RG_MAN_ID_0) == RF2XX_MAN_ID_0 && regRead(RG_MAN_ID_1) == RF2XX_MAN_ID_1) {
				LOG_INFO("JEDEC matches Atmel\n");

				// Match known radio (and sanitize radio type)
				partNum = regRead(RG_PART_NUM);
				switch (partNum) {
					case RF2XX_AT86RF233:
					//case RF2XX_AT86RF232:
					case RF2XX_AT86RF231:
					case RF2XX_AT86RF230:
					case RF2XX_AT86RF212:
						LOG_INFO("Radio part number 0x%02x\n", partNum);
						rf2xx_radio = partNum;
					default:
						break;
				}
			}
		} while (rf2xx_radio == RF2XX_UNDEFINED);
	}

	// CLKM clock change visible immediately
	bitWrite(SR_CLKM_SHA_SEL, 0);

	// disable CLKM
	bitWrite(SR_CLKM_CTRL, CLKM_DISABLED);

	// Enable/disable Tx autogenerating CRC16/CCITT
	bitWrite(SR_TX_AUTO_CRC_ON, RF2XX_CONF_CHECKSUM);

	// Enable RX_SAFE mode to protect buffer while reading it
	bitWrite(SR_RX_SAFE_MODE, 1);

	// Set same value for RF231 (default=0) and RF233 (default=1)
	bitWrite(SR_IRQ_MASK_MODE, 1);

	// Number of CSMA retries (part of IEEE 802.15.4)
	// Possible values [0 - 5], 6 is reserved, 7 will send immediately (no CCA)
	bitWrite(SR_MAX_CSMA_RETRIES, RF2XX_CONF_MAX_CSMA_RETRIES);

	// Number of maximum TX_ARET frame retries
	// Possible values [0 - 15], driver will disable TX_ARET for 0
	bitWrite(SR_MAX_FRAME_RETRIES, RF2XX_CONF_MAX_FRAME_RETRIES);

	// Highest allowed backoff exponent
	regWrite(RG_CSMA_BE, 0x80);

    // Randomize backoff timing
	// Upper two RSSI reg bits are random
	regWrite(RG_CSMA_SEED_0, regRead(RG_PHY_RSSI));

	// Usable only in AUTOACK mode,
	// Defined in IEEE 802.15.4-2006
	// Might be causing problems with older IEEE 802.15.4-2003 compliant devices
	bitWrite(SR_AACK_I_AM_COORD, RF2XX_CONF_I_AM_COORD);

	// Other sane defaults
	switch (rf2xx_radio) {
		case RF2XX_AT86RF212:
			regWrite(RG_TRX_CTRL_2, 0x08); // set OQPSK-SIN-RC-100
			rf2xx_setChannel(0x00); // set to channel 0 (868.3 MHz)
			regWrite(RG_PHY_TX_PWR, 0x62); // set to 4 dBm
			break;

		default:
			break;
	}

	// Set configured channel
	//rf2xx_setChannel(RF2XX_CHANNEL);
    LOG_INFO("Channel=%u Freq=%.2fMHz\n", rf2xx_getChannel(), rf2xx_getFrequency());

	//rf2xx_setTxPower(RF2XX_TX_POWER);
    // LOG_DBG("Pwr(hex): %u", rf2xx_getTxPower());
	LOG_INFO("Pwr=0x%02x\n", rf2xx_getTxPower());

	// First returned byte will be IRQ_STATUS;
	//bitWrite(SR_SPI_CMD_MODE, SPI_CMD_MODE__IRQ_STATUS);

	// Configure Promiscuous mode; Incomplete
	//bitWrite(SR_AACK_PROM_MODE, RF2XX_CONF_PROMISCUOUS);
	//bitWrite(SR_AACK_UPLD_RES_FT, 1);
	//bitWrite(SR_AACK_FLTR_RES_FT, 1);

	// Enable only specific IRQs
	//regWrite(RG_IRQ_MASK, IRQ2_RX_START | IRQ3_TRX_END | IRQ4_CCA_ED_DONE | IRQ5_AMI);
	regWrite(RG_IRQ_MASK, IRQ0_PLL_LOCK | IRQ2_RX_START | IRQ3_TRX_END | IRQ4_CCA_ED_DONE | IRQ5_AMI);

	// Read it to clear it
	dummy = regRead(RG_IRQ_STATUS);

	// Clear any interrupt pending
	clearEXTI();

    RF2XX_STATS_RESET(); // resets radio metrics
    flags.value = 0; // clear all driver internal flags

    // Initilize LQI & RSSI values
    rf2xx_last_lqi = 0;
    rf2xx_last_rssi = 0;
	last_packet_timestamp = 0;
}


int
rf2xx_init(void)
{
	// If AT86RF2xx radio is on SNR board
	#if AT86RF2XX_BOARD_SNR
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	#endif

	// If AT86RF2xx radio is on ISTMV v1.0 board
	#if AT86RF2XX_BOARD_ISMTV_V1_0
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        // Quirk: Disable JTAG since it share pin with RST (fixed in ISMTV v1.1 hardware)
        LOG_INFO("Disable JTAG in 5 seconds\n");
        vsnTime_delayS(5);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	#endif

	// If AT86RF2xx radio is on ISTMV v1.1 board
	#if AT86RF2XX_BOARD_ISMTV_V1_1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	#endif

	// Initializes only GPIO of SPI port
	vsnSPI_initHW(SPI_PORT);

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

    // Complete initialization of SPI
    vsnSPI_initCommonStructure(
        rf2xxSPI,
        SPI_PORT,
        &spiConfig,
        CSN_PIN,
        CSN_PORT,
        AT86RF2XX_SPI_SPEED
    );

	// register error callback
	vsnSPI_Init(rf2xxSPI, spiErrorCallback);

    // EXTI configuration
    rf2xxEXTI->EXTI_Line = EXTI_IRQ_LINE;
    rf2xxEXTI->EXTI_Mode = EXTI_Mode_Interrupt;
    rf2xxEXTI->EXTI_Trigger = EXTI_Trigger_Rising;
    rf2xxEXTI->EXTI_LineCmd = ENABLE;

    // GPIO init and common settings
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

	// IRQ (input)
	GPIO_InitStructure.GPIO_Pin = IRQ_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(IRQ_PORT, &GPIO_InitStructure);

	// Important. I forgot this line and ISR was triggered by UART1 output signal.
	GPIO_EXTILineConfig(EXTI_IRQ_PORT, EXTI_IRQ_PIN);

	// SLP_TR (output)
	GPIO_InitStructure.GPIO_Pin = SLP_TR_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SLP_TR_PORT, &GPIO_InitStructure);

	// RSTn (output)
	GPIO_InitStructure.GPIO_Pin = RSTN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(RSTN_PORT, &GPIO_InitStructure);

	setRST(); // hold radio in reset state
	clearCS(); // clear chip select (default)
	clearSLPTR(); // prevent going to sleep
	clearEXTI(); // clear interrupt flag

    // Initialize EXTI line, while radio is in RESET state
    EXTI_Init(rf2xxEXTI);


	rf2xx_reset();

    #if CONTIKI
	// Start Contiki process which will take care of received packets
	process_start(&rf2xx_process, NULL);
	// process_start(&rf2xx_calibration_process, NULL);
    #endif

	return 1;
}


int
rf2xx_prepare(const void *payload, unsigned short payload_len)
{
    if (payload_len > RF2XX_MAX_FRAME_SIZE - RF2XX_CRC_SIZE) {
        LOG_ERR("Payload larger than radio buffer: %u > %u\n", payload_len, RF2XX_MAX_FRAME_SIZE - RF2XX_CRC_SIZE);
        return RADIO_TX_ERR;
    }

    LOG_DBG("[rf2xxPrepare] ");

    if (flags.SLEEP) {
        rf2xx_wakeUp(); // Wake-up radio if it is asleep
        LOG_DBG_("WakeUp ");
    }

	// Force radio into TRX_OFF
    regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
    LOG_DBG_("TRX_OFF ");
    vsnTime_delayUS(rf2xx_getTiming(TIME__FORCE_TRX_OFF));

    ASSERT(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS), "TRX_OFF not reached\n");

	// Depending on FRAME_RETRIES use extended or basic radio mode
    if (RF2XX_CONF_MAX_FRAME_RETRIES > 0) {
        regWrite(RG_TRX_STATE, TRX_CMD_TX_ARET_ON); // extended mode
        LOG_DBG_("TX_ARET ");
    } else {
        regWrite(RG_TRX_STATE, TRX_CMD_PLL_ON);
        LOG_DBG_("PLL_ON ");
    }

    vsnTime_delayUS(rf2xx_getTiming(TIME__TRX_OFF_TO_PLL_ON));

    {
        vsnSPI_ErrorStatus status;
        vsnTime_Timeout timeout;
        uint8_t length = payload_len;

        uint8_t rxData[2 + RF2XX_MAX_FRAME_SIZE];
        uint8_t txData[2 + RF2XX_MAX_FRAME_SIZE];

        txData[0] = CMD_FB | CMD_WRITE;
        txData[1] = length + RF2XX_CRC_SIZE; // CRC is always part of 802.15.4 frame

        memcpy(txData+2, payload, length); // copy into appropriate position

        #if !RF2XX_CONF_CHECKSUM // if we disable radio's built-in CRC check for some reason
        {
            uint16_t crc = crc16_data((uint8_t *)payload, length, 0x00);
            memcpy(txData+2+payload_len, &crc, RF2XX_CRC_SIZE); // copy CRC after actual data
            length += RF2XX_CRC_SIZE;
            LOG_DBG_("CRC ");
        }
        #endif

        vsnTime_setTimeout(&timeout, 200);
        do {
            status = vsnSPI_checkSPIstatus(rf2xxSPI);
            if (!vsnTime_checkTimeout(&timeout)) {
                LOG_WARN("Timeout waiting SPI\n");
                return 0;
            }
        } while (status & (VSN_SPI_COM_IN_PROGRESS | VSN_SPI_BUS_BUSY));

        ASSERT(0 == getCS(), "Something wrong with SPI-CS\n");
        setCS();

        // Start sending to radio buffer
        LOG_DBG_("%ubytes ", length);
        status = vsnSPI_transferDataTXRX(rf2xxSPI, txData, rxData, 2 + length, spiCallback);


        if (status != VSN_SPI_SUCCESS) {
            if (status != VSN_SPI_COM_IN_PROGRESS) {
                return status;
            }

            vsnTime_setTimeout(&timeout, 200);

            while (status & (VSN_SPI_COM_IN_PROGRESS | VSN_SPI_BUS_BUSY)) {
                status = vsnSPI_checkSPIstatus(rf2xxSPI);
                if (!vsnTime_checkTimeout(&timeout)) {
                    return status;
                }
            }

            if (status != VSN_SPI_SUCCESS) {
                return status;
            }
        }
    }

    LOG_DBG_("\n");

    return payload_len;
}



int
rf2xx_transmit(unsigned short transmit_len)
{
    vsnTime_Timeout timeout;
    uint8_t dummy __attribute__((unused));
    uint8_t trac;

    flags.value = 0; // clear all flags
    dummy = regRead(RG_IRQ_STATUS); // clear IRQ

    ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);

    setSLPTR();
    clearSLPTR();

	vsnTime_setTimeout(&timeout, 200);
	while (!flags.TRX_END) { // will be released inside ISR
		if (!vsnTime_checkTimeout(&timeout)) {
			LOG_ERR("Timeout waiting TRX_END\n");
			flags.TRX_END = 1;
		}
	};

    ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);

    if (RF2XX_CONF_MAX_FRAME_RETRIES > 0) {
        trac = bitRead(SR_TRAC_STATUS);
    } else {
        trac = TRAC_SUCCESS;
    }

	rf2xx_on(); // Go back to Rx mode

	switch (trac) {
    case TRAC_SUCCESS:
        RF2XX_STATS_ADD(txSuccess);
        return RADIO_TX_OK;

    case TRAC_NO_ACK:
        RF2XX_STATS_ADD(txNoAck);
        return RADIO_TX_NOACK;

    case TRAC_CHANNEL_ACCESS_FAILURE:
        RF2XX_STATS_ADD(txCollision);
        return RADIO_TX_COLLISION;

    default:
        return RADIO_TX_ERR;
	}
}


int
rf2xx_send(const void *payload, unsigned short payload_len)
{
	rf2xx_prepare(payload, payload_len);
	return rf2xx_transmit(payload_len);
}

// Extend a bit this function
int
rf2xx_read(void *buf, unsigned short buf_len)
{
    vsnSPI_ErrorStatus status;
    vsnTime_Timeout timeout;
    uint8_t length, trac;
    uint8_t rxData[2 + RF2XX_MAX_FRAME_SIZE + RF2XX_LQI_SIZE];
    uint8_t txData[2 + RF2XX_MAX_FRAME_SIZE + RF2XX_LQI_SIZE];
    txData[0] = CMD_FB | CMD_READ;

    LOG_DBG("[rf2xxRead] ");

    vsnTime_setTimeout(&timeout, 200);
    do {
        status = vsnSPI_checkSPIstatus(rf2xxSPI);
        if (!vsnTime_checkTimeout(&timeout)) {
            RF2XX_STATS_ADD(spiError);
            LOG_WARN("Timeout waiting SPI\n");
            return 0;
        }
    } while (status & (VSN_SPI_COM_IN_PROGRESS | VSN_SPI_BUS_BUSY));

    LOG_DBG_("SPI ");

    setCS();
    status = vsnSPI_transferDataTXRX(rf2xxSPI, txData, rxData, 2 + RF2XX_MAX_FRAME_SIZE + RF2XX_LQI_SIZE, spiCallback);


    if (status != VSN_SPI_SUCCESS) {
        if (status != VSN_SPI_COM_IN_PROGRESS) {
            RF2XX_STATS_ADD(spiError);
            LOG_DBG_("ERROR\n");
            return 0;
        }

        vsnTime_setTimeout(&timeout, 200);

        while (status & (VSN_SPI_COM_IN_PROGRESS | VSN_SPI_BUS_BUSY)) {
            status = vsnSPI_checkSPIstatus(rf2xxSPI);
            if (!vsnTime_checkTimeout(&timeout)) {
                RF2XX_STATS_ADD(spiError);
                LOG_DBG_("ERROR\n");
                return 0;
            }
        }

        if (status != VSN_SPI_SUCCESS) {
            RF2XX_STATS_ADD(spiError);
            LOG_DBG_("ERROR\n");
            return 0;
        }
    }

    flags.TRX_END = 0; // Packet was read with all properties; await next packet;

    length = rxData[1];

    if (length < RF2XX_MIN_FRAME_SIZE || length > RF2XX_MAX_FRAME_SIZE) {
        LOG_DBG_("!%u\n", length);
        return 0;
    }

    length -= RF2XX_CRC_SIZE; // CRC is part of IEEE 802.15.4 frame

    #if !RF2XX_CONF_CHECKSUM
    {
        LOG_DBG_("CRC ");
        // received CRC
        uint16_t crc;
        memcpy(&crc, rxData + 2 + length, RF2XX_CRC_SIZE);

        // calculated CRC
        uint16_t _crc = crc16_data((const uint8_t *)(rxData + 2), length, 0x00);

        if (crc != _crc) {
            LOG_DBG_("invalid\n");
            return 0;
        }
    }
    #endif

    memcpy(buf, rxData+2, length); // Copy packet to buffer
    LOG_DBG_("%ubytes ", length);

    // Read TRAC status either if it is not mandatory (mainly for debugging)
    #if RF2XX_CONF_AUTOACK
        trac = bitRead(SR_TRAC_STATUS);
    #else
        trac = TRAC_SUCCESS;
    #endif

    switch (trac) {
    case TRAC_SUCCESS:
        // Everything is OK. CRC matches.
        LOG_DBG_("TRAC=OK ");
        break;

    case TRAC_SUCCESS_WAIT_FOR_ACK:
        // TODO: How do we inform CONTIKI about NO-ACK?
        LOG_DBG_("TRAC=NO-ACK ");
        break;

    case TRAC_INVALID:
        // Something went wrong with frame.
        // Read too fast? Buffer issues?
        // There is a change that frame is corrupted. No sense to make LQI.
        LOG_DBG_("TRAC=INVALID ");
        return 0;

    default:
        // Any other state is invalid in RX mode
        LOG_DBG_("TRAC=%u (INVALID) ", trac);
        return 0;
    }

    rf2xx_last_lqi = rxData[2 + length + RF2XX_CRC_SIZE];

    LOG_DBG_("LQI=%u; RSSI=%u;\n", rf2xx_last_lqi, rf2xx_last_rssi);

    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, rf2xx_last_lqi);
    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rf2xx_last_rssi);

    return length;
}


// TODO: Fix to operate with flags.CCA
// TODO: Fix to first check current radio state, the perform appropriate migration
int
rf2xx_cca(void)
{
    // Return 1 for IDLE, 0 for BUSY ...
    #if RF2XX_CONF_MAX_FRAME_RETRIES
        // Running in extended mode TX_ARET state will take care of CCA
        return 1;
    #else

        uint8_t cca, dummy __attribute__((unused));
        //vsnTime_Timeout timeout;

        // Wake up
        if (flags.SLEEP) {
            rf2xx_wakeUp();
        }

        // Make sure it is in TRX_OFF state
        regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
        vsnTime_delayUS(rf2xx_getTiming(TIME__FORCE_TRX_OFF));

        ASSERT(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS), "TRX_OFF not reached\n");


        bitWrite(SR_RX_PDT_DIS, 1); // disable reception; prevent interrupts; it will perform sensing;
        regWrite(RG_TRX_STATE, TRX_CMD_PLL_ON);
        vsnTime_delayUS(rf2xx_getTiming(TIME__TRX_OFF_TO_PLL_ON));

        dummy = regRead(RG_IRQ_STATUS); // clear IRQ

        bitWrite(SR_CCA_REQUEST, 1); // trigger CCA sensing
        vsnTime_delayUS(rf2xx_getTiming(TIME__CCA));

        // Read CCA value
        cca = bitRead(SR_CCA_STATUS); // 1 = IDLE, 0 = BUSY

        // Back to TRX_OFF state
        regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
        vsnTime_delayUS(rf2xx_getTiming(TIME__FORCE_TRX_OFF));

        bitWrite(SR_RX_PDT_DIS, 0); // enable reception in RX mode;

        rf2xx_on(); // enable again RX mode

        return cca;
    #endif
}


int
rf2xx_receiving_packet(void)
{
    //int receivingPacket = (flags.RX_START && vsnTime_checkTimeout(&rxTimeout)) && !flags.TRX_END;
    //LOG_DBG("receivingPacket: %u", receivingPacket);
    //return receivingPacket;
    return (flags.RX_START && vsnTime_checkTimeout(&rxTimeout)) && !flags.TRX_END;
}


int
rf2xx_pending_packet(void)
{
    //int pendingPacket = vsnTime_checkTimeout(&rxTimeout) && flags.TRX_END;
    //LOG_DBG("PendingPacket: %u", pendingPacket);
    //return pendingPacket;
    return vsnTime_checkTimeout(&rxTimeout) && flags.TRX_END;
}


int
rf2xx_on(void)
{
    uint8_t dummy __attribute__((unused));

    LOG_DBG("[rf2xxOn] ");

    if (flags.SLEEP) {
        rf2xx_wakeUp();
        LOG_DBG_("WakeUp ");
    }

    uint8_t state = bitRead(SR_TRX_STATUS);
    switch(state)
    {
        case TRX_STATUS_RX_AACK_ON:
        case TRX_STATUS_RX_ON:
        case TRX_STATUS_RX_ON_NOCLK:
        case TRX_STATUS_BUSY_RX:
        case TRX_STATUS_BUSY_RX_AACK:
        case TRX_STATUS_BUSY_RX_AACK_NOCLK:
            LOG_DBG_("noMigrate ");
            break;

        default:
            //clearSLPTR();
            regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
            vsnTime_delayUS(rf2xx_getTiming(TIME__FORCE_TRX_OFF));
            LOG_DBG_("TRX_OFF ");
            flags.value = 0; // clear at this point as it cannot be in any of the sub-states.

			#if RF2XX_CONF_AUTOACK
			regWrite(RG_TRX_STATE, TRX_CMD_RX_AACK_ON);
			LOG_DBG_("RX_AACK ");
			#else
			regWrite(RG_TRX_STATE, TRX_CMD_RX_ON);
			LOG_DBG_("RX_ON ");
			#endif

            vsnTime_delayUS(rf2xx_getTiming(TIME__TRX_OFF_TO_RX_ON));
            dummy = regRead(RG_IRQ_STATUS); // clear, because PLL_LOCK was triggered
            ASSERT(flags.PLL_LOCK, "PLL_LOCK should be ON\n");

            ENERGEST_ON(ENERGEST_TYPE_LISTEN);
            break;
    }

    LOG_DBG_("\n");

    return 1;
}


int
rf2xx_off(void)
{
    uint8_t dummy __attribute__((unused));

    if (flags.SLEEP) {
        rf2xx_wakeUp();
    }

    uint8_t state = bitRead(SR_TRX_STATUS);
    switch (state)
    {
        case TRX_STATUS_TRX_OFF:
            break;

        /*case TRX_STATUS_RX_AACK_ON:
        case TRX_STATUS_RX_ON:
        case TRX_STATUS_TX_ARET_ON:
        case TRX_STATUS_TX_ON:*/
        default:
            regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
            vsnTime_delayUS(rf2xx_getTiming(TIME__FORCE_TRX_OFF)); // TODO: Find migration time
            ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
            break;
    }

    return 1;
}



void rf2xx_isr(void) {
	ENERGEST_ON(ENERGEST_TYPE_IRQ);

	volatile rf2xx_irq_t irq;
	irq.value = regRead(RG_IRQ_STATUS);

	if (irq.IRQ0_PLL_LOCK) {
		flags.PLL_LOCK = 1;
		//lastCalibration = vsnTime_uptime();
	}

	if (irq.IRQ1_PLL_UNLOCK) {
		flags.PLL_LOCK = 0;
	}

	if (irq.IRQ2_RX_START) {
		flags.RX_START = 1;
		flags.AMI = 0;
		flags.TRX_END = 0;

		RF2XX_STATS_ADD(rxDetected);

        #if !RF2XX_CONF_AUTOACK
        // When using non-extended mode, RSSI should be read on RX_START IRQ
        rf2xx_last_rssi = 3 * bitRead(SR_RSSI);
        #endif
	}

	if (irq.IRQ5_AMI) {
		flags.AMI = 1;
		RF2XX_STATS_ADD(rxAddrMatch);
	}

	if (irq.IRQ6_TRX_UR) {
		flags.TRX_UR = 1;
	}


	if (irq.IRQ3_TRX_END) {
		flags.TRX_END = 1;

        #if RF2XX_CONF_AUTOACK
        // In extended mode IRQ3_TRX_END is best time to read RSSI
        rf2xx_last_rssi = regRead(RG_PHY_ED_LEVEL);
        #endif

		if (flags.RX_START) {
			RF2XX_STATS_ADD(rxSuccess);
            process_poll(&rf2xx_process); // inform process about packet
			// rf2xx_last_packet_timestamp = ???
		} else {
			RF2XX_STATS_ADD(txCount);
		}
	}

	if (irq.IRQ4_AWAKE_END) { // CCA_***_IRQ
		flags.SLEEP = 0;
		flags.CCA = 0;
	}

	if (irq.IRQ7_BAT_LOW) {}

	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}










int
rf2xx_sleep(void) {
    rf2xx_off(); // This will put radio into TRX_OFF mode

    setSLPTR(); // SLP_TR pin will trigger migration to SLEEP state
    vsnTime_delayUS(rf2xx_getTiming(TIME__TRX_OFF_TO_SLEEP));
    flags.SLEEP = 1;

    return 1;
}

int rf2xx_wakeUp(void)
{
	if (flags.SLEEP || getSLPTR())
	{
		uint8_t dummy __attribute__((unused));

		clearSLPTR(); // release SLP_TR pin and will go out of SLEEP state
		vsnTime_delayUS(rf2xx_getTiming(TIME__SLEEP_TO_TRX_OFF));
		dummy = regRead(RG_IRQ_STATUS); // clear, because IRQ4_AWAKE_END was triggered
		flags.SLEEP = 0;
	}

    return 1;
}


#if CONTIKI

PROCESS_THREAD(rf2xx_process, ev, data)
{
	int len;
	PROCESS_BEGIN();

	LOG_INFO("AT86RF2xx driver process started!\n");

	while(1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

        RF2XX_STATS_ADD(rxToStack);
		LOG_DBG("calling receiver callback\n");

		packetbuf_clear();
		//packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
		len = rf2xx_read(packetbuf_dataptr(), PACKETBUF_SIZE);
		packetbuf_set_datalen(len);
		NETSTACK_MAC.input();
	}

	PROCESS_END();
}

/*
PROCESS_THREAD(rf2xx_calibration_process, ev, data)
{
	PROCESS_BEGIN();
	LOG_DBG("AT86RF2xx calibration process started");
	while(1) {
		rf2xx_calibration();
		PROCESS_PAUSE();
	}
	PROCESS_END();
}
*/

static radio_result_t get_value(radio_param_t param, radio_value_t *value) {
	switch (param) {
		case RADIO_PARAM_POWER_MODE:
            *value = flags.PLL_LOCK ? RADIO_POWER_MODE_ON : RADIO_POWER_MODE_OFF;
		    return RADIO_RESULT_OK;

		case RADIO_PARAM_CHANNEL:
			*value = rf2xx_getChannel();
			return RADIO_RESULT_OK;

		case RADIO_PARAM_16BIT_ADDR:
			*value = rf2xx_getShortAddr();
			return RADIO_RESULT_OK;

		case RADIO_PARAM_RX_MODE:
			*value = 0;
			*value |= RADIO_RX_MODE_ADDRESS_FILTER; // will change if promiscuous mode
			#if RF2XX_AUTOACK
				*value |= RADIO_RX_MODE_AUTOACK;
			#endif
			//*value = RADIO_RX_MODE_ADDRESS_FILTER | RADIO_RX_MODE_AUTOACK;
			return RADIO_RESULT_OK;

		case RADIO_PARAM_TX_MODE:
			#if RF2XX_CONF_MAX_FRAME_RETRIES
				*value = 0; // will be handled by TX_ARET
			#else
				// Radio in traditional TX_ON, CCA has to be done manually
				*value = RADIO_TX_MODE_SEND_ON_CCA;
			#endif
			return RADIO_RESULT_OK;

		case RADIO_PARAM_TXPOWER:
			*value = rf2xx_getTxPower();
			return RADIO_RESULT_OK;

		case RADIO_PARAM_CCA_THRESHOLD:
			return RADIO_RESULT_NOT_SUPPORTED;

		case RADIO_PARAM_RSSI:
			*value = rf2xx_last_rssi;
			return RADIO_RESULT_OK;

		case RADIO_CONST_CHANNEL_MIN:
			*value = rf2xx_getMinChannel();
			return RADIO_RESULT_OK;

		case RADIO_CONST_CHANNEL_MAX:
			*value = rf2xx_getMaxChannel();
			return RADIO_RESULT_OK;

		case RADIO_CONST_TXPOWER_MIN:
			*value = rf2xx_getMinPwr();
			return RADIO_RESULT_OK;

		case RADIO_CONST_TXPOWER_MAX:
			*value = rf2xx_getMaxPwr();
			return RADIO_RESULT_OK;

		default:
			return RADIO_RESULT_NOT_SUPPORTED;
	}

}


static radio_result_t set_value(radio_param_t param, radio_value_t value) {
	switch (param) {
		case RADIO_PARAM_CHANNEL:
			rf2xx_setChannel(value);
			return RADIO_RESULT_OK;

		case RADIO_PARAM_PAN_ID:
			rf2xx_setPanId(value);
			return RADIO_RESULT_OK;

		case RADIO_PARAM_16BIT_ADDR:
			rf2xx_setShortAddr(value);
			return RADIO_RESULT_OK;

		case RADIO_PARAM_TXPOWER:
			rf2xx_setTxPower(value);
			return RADIO_RESULT_OK;

		default:
			return RADIO_RESULT_NOT_SUPPORTED;
	}
}


static radio_result_t get_object(radio_param_t param, void *dest, size_t size) {
	if (dest == NULL) return RADIO_RESULT_ERROR;
	switch (param) {
		case RADIO_PARAM_64BIT_ADDR:
			rf2xx_getLongAddr((uint8_t *)dest, size);
			return RADIO_RESULT_OK;

		default:
			return RADIO_RESULT_NOT_SUPPORTED;
	}
}


static radio_result_t set_object(radio_param_t param, const void *src, size_t size) {
	if (src == NULL) return RADIO_RESULT_ERROR;
	switch (param) {
		case RADIO_PARAM_64BIT_ADDR:
			rf2xx_setLongAddr((const uint8_t *)src, size);
			return RADIO_RESULT_OK;

		default:
			return RADIO_RESULT_NOT_SUPPORTED;
	}
}


const struct radio_driver rf2xx_driver = {
	rf2xx_init,
	rf2xx_prepare,
	rf2xx_transmit,
	rf2xx_send,
	rf2xx_read,
	rf2xx_cca,
	rf2xx_receiving_packet,
	rf2xx_pending_packet,
	rf2xx_on,
	rf2xx_off,
	get_value,
	set_value,
	get_object,
	set_object,
};


#endif // CONTIKI






void rf2xx_setChannel(uint8_t ch) {
	switch (rf2xx_radio) {
		case RF2XX_AT86RF233:
		//case RF2XX_AT86RF232:
		case RF2XX_AT86RF231:
		case RF2XX_AT86RF230:
			if (ch >= 11 && ch <= 26) {
				bitWrite(SR_CHANNEL, ch);
			}
			break;

		case RF2XX_AT86RF212:
			if (ch < 11) {
				bitWrite(SR_CHANNEL, ch);
			}
			break;

		default:
			break;
	}

	LOG_INFO("Channel=%u Freq=%.2fMHz\n", rf2xx_getChannel(), rf2xx_getFrequency());
}

uint8_t rf2xx_getChannel(void) {
	return bitRead(SR_CHANNEL);
}

void rf2xx_setTxPower(uint8_t pwr) {
	switch (rf2xx_radio) {
		case RF2XX_AT86RF212:
			bitWrite(SR_TX_PWR_RF21x_ALL, pwr);
			break;

		case RF2XX_AT86RF230:
		case RF2XX_AT86RF231:
		case RF2XX_AT86RF233:
		default:
			bitWrite(SR_TX_PWR, pwr);
			break;
	}
}

uint8_t rf2xx_getTxPower(void) {
	switch (rf2xx_radio) {
		case RF2XX_AT86RF212:
			return bitRead(SR_TX_PWR_RF21x_ALL);

		case RF2XX_AT86RF230:
		case RF2XX_AT86RF231:
		case RF2XX_AT86RF233:
		default:
			return bitRead(SR_TX_PWR);
	}
}

void rf2xx_setPanId(uint16_t pan) {
	regWrite(RG_PAN_ID_0, pan & 0xFF);
	regWrite(RG_PAN_ID_1, pan >> 8);
	LOG_INFO("PAN == 0x%02x\n", pan);
}


uint16_t rf2xx_getPanId(void) {
	uint16_t pan = ((uint16_t)regRead(RG_PAN_ID_1) << 8) & 0xFF00;
    pan |= (uint16_t)regRead(RG_PAN_ID_0) & 0xFF;
    return pan;
}

uint16_t rf2xx_getShortAddr(void) {
	uint16_t addr = ((uint16_t)regRead(RG_SHORT_ADDR_1) << 8) & 0xFF00;
    addr |= (uint16_t)regRead(RG_SHORT_ADDR_0) & 0xFF;
    return addr;
}

void rf2xx_setShortAddr(uint16_t addr) {
	regWrite(RG_SHORT_ADDR_0, addr & 0xFF);
	regWrite(RG_SHORT_ADDR_1, addr >> 8);
	LOG_INFO("Short addr == 0x%02x\n", addr);
}

void rf2xx_setLongAddr(const uint8_t * addr, uint8_t len) {
    if (len > 8) len = 8;

    // The usual representation of MAC address has big-endianess. However,
    // This radio uses little-endian, so the order of bytes has to be reversed.
	// When we define IEEE addr, the most important byte is ext_addr[0], while
	// on radio RG_IEEE_ADDR_0 must contain the lowest/least important byte.
    for (uint8_t i = 0; i < len; i++) {
        regWrite(RG_IEEE_ADDR_7 - i, addr[i]);
    }
}


void rf2xx_getLongAddr(uint8_t *addr, uint8_t len) {
	if (len > 8) len = 8;
    for (uint8_t i = 0; i < len; i++) {
        addr[i] = regRead(RG_IEEE_ADDR_7 - i);
    }
}





// TODO: FIX: return signed integer with proper value
int8_t rf2xx_ED_LEVEL(void) {
	// Value is refreshed every 128us.

	// value should be in range [0x00, 0x54]
	// or [0, 84] in decimal
	// Precision is 1dBm with +/- 5dBm absolute precision

	// 0xFF is a reset value

	// P_{RF} = -91 + ED_LEVEL
	//return - 91 + bitRead(SR_ED_LEVEL);
	int8_t rssi = -91 + bitRead(SR_ED_LEVEL);
	//LOG_DBG("RSSI=%i\n", rssi);
	return rssi;

}

// TODO: FIX: return signed integer with proper value
int8_t rf2xx_RSSI(void) {
	// RSSI value is refreshed every 2us.

	// value should be in range [0x00, 0x1C] or [0, 28] in decimal
	//int8_t rssi = bitRead(SR_RSSI) - 1;

	// 0 indicates signal strength < -91dBm
	//if (rssi == 0) return rssi; // This means that RSSI is below threshold

	// precision is 3dBm
	//rssi = (rssi - 1) * 3; // calculate appropriate RSSI (according to the reference manuals)

	// P_{RF} = -91 + 3 * (RSSI - 1)

	//return -91 + 3 * (rssi - 1);

	int8_t rssi = -91 + 3 * (bitRead(SR_RSSI) - 1);
	//LOG_DBG("RSSI=%i\n", rssi);
	return rssi;
}


int8_t rf2xx_rssi(void) {
	return extMode ? rf2xx_ED_LEVEL() : rf2xx_RSSI();
}

/*
// TODO: Should this be split into two functions?
void rf2xx_calibration(void)
{
	vsnTime_Timeout timeout;
	uint32_t now = vsnTime_uptime();
	if (now - lastCalibration < RF2XX_CAL_PERIOD) { // 240s == 4min
		return;
	}
	// If radio is more than 5 minutes in any Tx/Rx mode, callibration
	// loop has to be triggered manually.

	// Read radios current state to take proper action
	uint8_t curState = flags.SLEEP
		? TRX_STATUS_SLEEP
		: bitRead(SR_TRX_STATUS);

	switch (curState) {
		case TRX_STATUS_TRX_OFF:
		case TRX_STATUS_SLEEP:
			// needs no calibration, since it will be performed on TRX_OFF -> PLL_ON
			break;

		case TRX_STATUS_BUSY_RX_AACK_NOCLK:
		case TRX_STATUS_BUSY_RX_AACK:
		case TRX_STATUS_BUSY_RX:
			// If in any busy state, wait to complete
			//while (rf2xx_receiving_packet());
			// fall through
			break; // let's skip this time

		case TRX_STATUS_RX_AACK_ON:
		case TRX_STATUS_RX_ON_NOCLK:
		case TRX_STATUS_RX_AACK_ON_NOCLK:
		case TRX_STATUS_RX_ON:
		case TRX_STATUS_TX_ARET_ON:
		case TRX_STATUS_TX_ON:
			lastCalibration = now;
			LOG_DBG("Triggered calibration ...");

			// Temporary disable radio reception in any Rx state
			bitWrite(SR_RX_PDT_DIS, 1); // radio will ignore incoming packages

			bitWrite(SR_FTN_START, 1); // filter tuning network calibration (~25us)
			bitWrite(SR_PLL_CF_START, 1); // center frequency calibration (~35us)
			bitWrite(SR_PLL_DCU_START, 1); // delay cell calibration loop (~6us)

			vsnTime_setTimeout(&timeout, 200);

			while(!flags.PLL_LOCK) {
				if (!vsnTime_checkTimeout(&timeout)) {
					LOG_ERR("Callibration error. PLL_LOCK was not triggered");
					break;
				}
			}

			bitWrite(SR_RX_PDT_DIS, 0); // Enable reception back again

			LOG_DBG("Calibration complete");
			break;

		default:
			LOG_DBG("Callibration: state=0x%02x", curState);
			break;
	}
}
*/
/*
void rf2xx_calibration(void) {
	LOG_DBG("Running calibration procedure ...\n");
	// If radio is more than 5 minutes in any Tx/Rx mode, callibration
	// loop has to be triggered manually.

	// Aggresively put radio into TRX_OFF state
	regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
	vsnTime_delayUS(rf2xx_getTiming(TIME__FORCE_TRX_OFF) * 2);

	// Temporary disable radio reception in any Rx state
	bitWrite(SR_RX_PDT_DIS, 1);

	// Put radio into "basic" Rx mode
	regWrite(RG_TRX_STATE, TRX_CMD_RX_ON);
	vsnTime_delayUS(rf2xx_getTiming(TIME__TRX_OFF_TO_RX_ON));

	uint8_t curState = bitRead(SR_TRX_STATUS);

	switch (curState) {
		case TRX_STATUS_TRX_OFF:
		case TRX_STATUS_RX_AACK_ON:
		case TRX_STATUS_RX_ON:
			bitWrite(SR_FTN_START, 1); // filter tuning (~25us)
			vsnTime_delayUS(30); // TODO: Put this value into radio specific settings
			break;
		default:
			LOG_ERR("Filter tuning calibration was skipped, due to incorrect radio state\n");
			break;
	}

	switch (curState) {
		case TRX_STATUS_TX_ARET_ON:
		case TRX_STATUS_TX_ON:
		case TRX_STATUS_RX_AACK_ON:
		case TRX_STATUS_RX_ON:
			bitWrite(SR_PLL_CF_START, 1); // center frequency calibration (~35us)
			bitWrite(SR_PLL_DCU_START, 1); // delay cell calibration loop (~6us)
			vsnTime_delayUS(80); // TODO: Put timings into radio specific settings
			break;
		default:
			LOG_ERR("Center frequency and delay cell calibration was skipped, due to incorrect radio state\n");
			break;
	}

	// Aggresively put radio into TRX_OFF state
	regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
	vsnTime_delayUS(rf2xx_getTiming(TIME__FORCE_TRX_OFF) * 2);

	// Enable reception back again
	bitWrite(SR_RX_PDT_DIS, 0);

	LOG_DBG("Calibration complete. Radio is currently in TRX_OFF\n");
}
*/


uint8_t regRead(uint8_t addr)
{
    vsnSPI_ErrorStatus status;
	vsnTime_Timeout timeout;
    uint8_t value;

    addr = (addr & CMD_REG_MASK) | CMD_REG | CMD_READ;

    do {
        vsnTime_setTimeout(&timeout, 200);
        do {
            status = vsnSPI_checkSPIstatus(rf2xxSPI);
            if (!vsnTime_checkTimeout(&timeout)) {
                break;
            }
        } while (status & (VSN_SPI_COM_IN_PROGRESS | VSN_SPI_BUS_BUSY));

        setCS();
        status = vsnSPI_pullByteTXRX(rf2xxSPI, addr, &value);
        status |= vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, &value);
        clearCS();

    } while (status != VSN_SPI_SUCCESS);

    return value;
}


void regWrite(uint8_t addr, uint8_t value)
{
    vsnSPI_ErrorStatus status;
	vsnTime_Timeout timeout;
    uint8_t dummy __attribute__((unused));

    addr = (addr & CMD_REG_MASK) | CMD_REG | CMD_WRITE;

    do {
        vsnTime_setTimeout(&timeout, 200);
        do {
            status = vsnSPI_checkSPIstatus(rf2xxSPI);
            if (!vsnTime_checkTimeout(&timeout)) {
                break;
            }
        } while (status & (VSN_SPI_COM_IN_PROGRESS | VSN_SPI_BUS_BUSY));

        setCS();
        status = vsnSPI_pullByteTXRX(rf2xxSPI, addr, &dummy);
        status |= vsnSPI_pullByteTXRX(rf2xxSPI, value, &dummy);
        clearCS();

    } while (status != VSN_SPI_SUCCESS);
}




uint8_t bitRead(uint8_t addr, uint8_t mask, uint8_t offset) {
	return (regRead(addr) & mask) >> offset;
}

void bitWrite(uint8_t addr, uint8_t mask, uint8_t offset, uint8_t value) {
	uint8_t tmp = regRead(addr) & ~mask;
	tmp |= (value << offset) & mask;
	regWrite(addr, tmp);
}


void rf2xx_test(void) {
	vsnSPI_ErrorStatus status;
	vsnTime_Timeout timeout;

	//uint8_t trxState = 0;

	// 1. RESET
	setRST();
	vsnTime_delayMS(1);
	clearRST();

	// 2. Set IRQ mask register, enable IRQ_0 (PLL_LOCK)
	regWrite(RG_IRQ_MASK, 0x01);

	// 3. Disable TX_AUTO_CRC_ON
	regWrite(RG_TRX_CTRL_1, 0x00);

	// 4. Set radio transceiver state TRX_OFF
	regWrite(RG_TRX_STATE, 0x03);

	// 5. Set clock at pin 17 (CLKM)
	regWrite(RG_TRX_CTRL_0, 0x01);

	// 6. Set IEEE 802.15.4 CHANNEL, e.g. 19
	//regWrite(0x08, 0x33);

	// 7. Set TX output power, e.g. to Pmax
	//regWrite(0x05, 0x00);

	// 8. Verify TRX_OFF state
	vsnTime_setTimeout(&timeout, 200);
	while (bitRead(SR_TRX_STATUS) != TRX_STATUS_TRX_OFF) {
		if (!vsnTime_checkTimeout(&timeout)) {
			printf("TRX_OFF not reached ...");
			break;
		}
	}

	// 9. Enable Continuous Transmission Test Mode - step # 1
	regWrite(RG_TST_CTRL_DIGI, 0x0F);

	// 10. Enable High Data Rate Mode, 2 Mb/s
	//regWrite(0x0C, 0x03);

	// 11. Configure High Data Rate Mode
	//regWrite(0x0A, 0xA7);

	// 12. Write PHR and PSDU data (even for CW mode)
	uint8_t rxData[2 + RF2XX_MAX_FRAME_SIZE];
	uint8_t txData[2 + RF2XX_MAX_FRAME_SIZE];

	txData[0] = CMD_FB | CMD_WRITE;
	txData[1] = RF2XX_MAX_FRAME_SIZE; // CRC is always part of 802.15.4 frame

	memset(txData+2, 0xAA, RF2XX_MAX_FRAME_SIZE);

	setCS();

	// Start sending to radio buffer
	status = vsnSPI_transferDataTXRX(rf2xxSPI, txData, rxData, 2 + RF2XX_MAX_FRAME_SIZE, spiCallback);

	if (status != VSN_SPI_SUCCESS) {
		if (status != VSN_SPI_COM_IN_PROGRESS) {
			printf("SPI communication already in progress ...");
			//return status;
		}

		vsnTime_setTimeout(&timeout, 200);

		while (status & (VSN_SPI_COM_IN_PROGRESS | VSN_SPI_BUS_BUSY)) {
			status = vsnSPI_checkSPIstatus(rf2xxSPI);
			if (!vsnTime_checkTimeout(&timeout)) {
				printf("SPI error ...");
				//return status;
			}
		}

		if (status != VSN_SPI_SUCCESS) {
			//return status;
			printf("SPI return status 0x%02x != 0x%02x", status, VSN_SPI_SUCCESS);
		}
	}

	// 13. Enable Continuous Transmission Test Mode - step # 2
	regWrite(RG_PART_NUM, 0x54);

	// 14. Enable Continuous Transmission Test Mode - step # 3
	regWrite(RG_PART_NUM, 0x46);

	// 15. Enable PLL_ON state
	regWrite(RG_TRX_STATE, 0x09);

	// 16. Wait for IRQ_0 (PLL_LOCK)
	vsnTime_setTimeout(&timeout, 200);
	while(bitRead(SR_TRX_STATUS) == TRX_STATUS_PLL_ON) {
		if (!vsnTime_checkTimeout(&timeout)) {
			printf("PLL_ON not reached ...");
			break;
		}
	}

	// 17. Initiate Transmission, enter BUSY_TX state
	regWrite(RG_TRX_STATE, 0x02);

	printf("You should see the signal ...");

	// 18. Perform measurement
	while(1);
	//vsnTime_delayS(30);

	// 19. Disable Continuous Transmission Test Mode
	regWrite(RG_PART_NUM, 0x00);

	// 20. Reset AT86RF231
	setRST();
	vsnTime_delayMS(1);
	clearRST();

}
