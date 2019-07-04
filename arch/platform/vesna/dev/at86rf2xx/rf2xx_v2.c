#include <string.h>
#include <lib/crc16.h>

#include "sys/log.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "sys/energest.h"
#include "sys/rtimer.h"

#include "at86rf2xx/rf2xx_registermap.h"
#include "at86rf2xx/rf2xx_hal.h"
#include "at86rf2xx/rf2xx.h"

#define LOG_MODULE "rf2xx"
#define LOG_LEVEL LOG_CONF_LEVEL_RF2XX

PROCESS(rf2xx_process, "AT86RF2XX driver");

#define DEFAULT_IRQ_MASK    (IRQ0_PLL_LOCK | IRQ2_RX_START | IRQ3_TRX_END | IRQ4_CCA_ED_DONE | IRQ5_AMI)

#define CALIBRATION_PERIOD    (240) // seconds (~4min)


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


volatile uint8_t rf2xx_last_lqi;
volatile int8_t rf2xx_last_rssi;

// Exposed radio timings
volatile static rtimer_clock_t last_packet_timestamp;
volatile static rtimer_clock_t frame_start_timestamp;
volatile static rtimer_clock_t frame_end_timestamp;


#if RF2XX_CONF_STATS
// Access to radio statistics
uint32_t rf2xxStats[RF2XX_STATS_COUNT];
#endif

// Driver Tx buffer
static uint8_t txBuffer[2 + RF2XX_MAX_FRAME_SIZE];

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

// Store radio type for later discrimitation
static rf2xx_radio_t rf2xx_radio = RF2XX_UNDEFINED;


// !!! Experimantal feature/option struct struct
typedef union {
	struct {
        // Enable extended RX_AACK & TX_ARET instead of basic RX_ON & TX_ON
        uint8_t extMode:1;

        // Enable autogenerate CRC at Tx
        uint8_t autoCRC:1;

        // Use polling mode instead of interrupt mode
        uint8_t pollMode:1;
	};
	uint8_t value;
} rf2xx_options_t;

// internal feature/option flags
static rf2xx_options_t opts;


inline static int8_t getCcaThreshold(void) {
    return -91 + 2 * bitRead(SR_CCA_ED_THRES);
}

inline static void setCcaThreshold(int8_t threshold) {
    bitWrite(SR_CCA_ED_THRES, threshold / 2 + 91);
}

inline static void setSendOnCca(uint8_t enabled) {
    bitWrite(SR_MAX_CSMA_RETRIES, enabled ? RF2XX_CONF_MAX_CSMA_RETRIES : 0);
}

inline static uint8_t getSendOnCca(void) {
    return bitRead(SR_MAX_CSMA_RETRIES) > 0;
}


inline static void setPollMode(uint8_t enabled) {
    opts.pollMode = !!enabled;
}

inline static uint8_t getPollMode(void) {
    return opts.pollMode;
} 



static float rf2xx_getFrequency(void) {
	uint8_t channel = bitRead(SR_CHANNEL);

	switch (rf2xx_radio) {
    case RF2XX_AT86RF233:
    //case RF2XX_AT86RF232:
    case RF2XX_AT86RF231:
    case RF2XX_AT86RF230:
        // AT86RF23x radios work on 2.4GHz ISM band
        return 2405 + 5 * (channel - 11);

    case RF2XX_AT86RF212:
    {
        // AT86RF212 works sub-1GHz and is more complex.
        // It supports ISM bands as set by SR_CHANNEL bits. (CC_BAND must be 0)
        // It also support custom frequencies set by CC_BAND and CC_NUMBER bits.
        uint8_t ccBand = bitRead(SR_CC_BAND);
        uint8_t ccNumber = bitRead(SR_CC_NUMBER);
        switch (ccBand) {
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
    }
    default:
        return 0;
	}

	return 0;
}


uint16_t rf2xx_getTiming(rf2xx_timing_t timing) {
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
static void spiCallback(void *cbDevStruct) {
	vsnSPI_ErrorStatus spiStatus;
	vsnSPI_CommonStructure *spi = cbDevStruct; // casting in separate line to simplify debugging
	spiStatus = vsnSPI_chipSelect(spi, SPI_CS_HIGH);
	if (VSN_SPI_SUCCESS != spiStatus) {
		LOG_ERR("SPI chip select error\n");
	}
}

// Generic SPI callback for errors
static void spiErrorCallback(void *cbDevStruct) {
	vsnSPI_CommonStructure *spi = cbDevStruct;
	vsnSPI_chipSelect(spi, SPI_CS_HIGH);
	LOG_ERR("SPI error callback triggered\n");
}


void rf2xx_reset(void) {
	uint8_t dummy __attribute__((unused));

	setRST(); // hold radio in reset state
	clearCS(); // clear chip select (default)
	clearSLPTR(); // prevent going to sleep
	clearEXTI(); // clear interrupt flag

	clearRST(); // release radio from RESET state

	// Radio should now go through P_ON -> TRX_OFF state migration
	vsnTime_delayUS(rf2xx_getTiming(TIME__VCC_TO_P_ON));
	//ASSERT(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS), "TRX_OFF not reached\n");

	// Print for what pin layout was compiled.
	LOG_INFO("Compiled for " AT86RF2XX_BOARD_STRING "\n");

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
	// Possible values [0 - 15]
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
		rf2xx_setChannel(0); // set to channel 0 (868.3 MHz)
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
	regWrite(RG_IRQ_MASK, DEFAULT_IRQ_MASK);

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



int rf2xx_init(void) {
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

	// Start Contiki process which will take care of received packets
	process_start(&rf2xx_process, NULL);

	return 1;
}


int rf2xx_prepare(const void *payload, unsigned short payload_len) {
	if (payload_len > RF2XX_MAX_FRAME_SIZE - RF2XX_CRC_SIZE) {
        LOG_ERR("Payload larger than radio buffer: %u > %u\n", payload_len, RF2XX_MAX_FRAME_SIZE - RF2XX_CRC_SIZE);
        return 0;
    }

	txBuffer[0] = CMD_FB | CMD_WRITE;
	txBuffer[1] = payload_len + RF2XX_CRC_SIZE; // CRC is always part of 802.15.4 frame
	memcpy(txBuffer+2, payload, payload_len);

	// if we disabled radio's built-in CRC check for some reason
	if (!opts.autoCRC) {
		uint16_t crc = crc16_data(payload, payload_len, 0x00);
        // copy 2 bytes of CRC after payload bytes
		memcpy(txBuffer + 2 + payload_len, &crc, RF2XX_CRC_SIZE);
	}

	return payload_len;
}


int rf2xx_transmit(unsigned short transmit_len) {
	vsnSPI_ErrorStatus status;
    vsnTime_Timeout timeout;
    uint8_t dummy __attribute__((unused));
    uint8_t trac;

	LOG_DBG("Start transmit ");

	// Check if radio is asleep
	if (flags.SLEEP) {
		rf2xx_wakeUp();
		LOG_DBG_("WakeUp ");
	}

	regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
    LOG_DBG_("TRX_OFF ");
    vsnTime_delayUS(rf2xx_getTiming(TIME__FORCE_TRX_OFF));

	ASSERT(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS), "TRX_OFF not reached\n");

	// Extended mode or not.
    if (opts.extMode) {
        regWrite(RG_TRX_STATE, TRX_CMD_TX_ARET_ON); // extended mode
        LOG_DBG_("TX_ARET ");
    } else {
        regWrite(RG_TRX_STATE, TRX_CMD_PLL_ON);
        LOG_DBG_("PLL_ON ");
    }

	vsnTime_delayUS(rf2xx_getTiming(TIME__TRX_OFF_TO_PLL_ON));

	// Check SPI status
	vsnTime_setTimeout(&timeout, 200);
	do {
		status = vsnSPI_checkSPIstatus(rf2xxSPI);
		if (!vsnTime_checkTimeout(&timeout)) {
			LOG_WARN("SPI-timeout\n");
			return 0;
		}
	} while (status & (VSN_SPI_COM_IN_PROGRESS | VSN_SPI_BUS_BUSY));

	// Sanity checks
	//ASSERT(0 == getCS(), "Something wrong with SPI-CS\n");
	setCS(); // Chip select

	// If CRC is not offloaded to a radio, add precomputed CRC
	if (!opts.autoCRC) transmit_len += RF2XX_CRC_SIZE;

	LOG_DBG_("%ubytes ", transmit_len);

	// Preparations for transmit and interrupt perception
	flags.value = 0;
	dummy = regRead(RG_IRQ_STATUS);

	// Trigger transmit (see Manual p. 127, bottom)
	setSLPTR();
	clearSLPTR();

	// Contiki's energest
	ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);

	// length is command byte + payload size byte + len
	status = vsnSPI_transferDataTX(rf2xxSPI, txBuffer, 2 + transmit_len, spiCallback);

	// Observe SPI status
	if (status != VSN_SPI_SUCCESS) {
		if (status != VSN_SPI_COM_IN_PROGRESS) {
			return RADIO_TX_ERR;
		}

		vsnTime_setTimeout(&timeout, 200);

		while (status & (VSN_SPI_COM_IN_PROGRESS | VSN_SPI_BUS_BUSY)) {
			status = vsnSPI_checkSPIstatus(rf2xxSPI);
			if (!vsnTime_checkTimeout(&timeout)) {
				return RADIO_TX_ERR;
			}
		}

		if (status != VSN_SPI_SUCCESS) {
			return RADIO_TX_ERR;
		}
	}

    if (opts.pollMode) {
        rf2xx_irq_t irq;
        do {
            status = 
        }
        vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, )
    } else {
        vsnTime_setTimeout(&timeout, 200);
        while (!flags.TRX_END) { // will be released inside ISR
            if (!vsnTime_checkTimeout(&timeout)) {
                LOG_ERR("Timeout waiting TRX_END\n");
                flags.TRX_END = 1;
            }
        };
    }


    ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);

    if (extMode) {
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
		LOG_DBG_("noACK ");
        return RADIO_TX_NOACK;

    case TRAC_CHANNEL_ACCESS_FAILURE:
        RF2XX_STATS_ADD(txCollision);
		LOG_DBG_("collision");
        return RADIO_TX_COLLISION;

    default:
		LOG_DBG_("Err(trac=0x%02x)", trac);
        return RADIO_TX_ERR;
	}

	LOG_DBG_("\n");
	return transmit_len;
}

