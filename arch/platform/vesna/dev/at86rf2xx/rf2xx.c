#include <stdio.h>
#include <string.h>

#include <lib/crc16.h>

#include "contiki.h"
#include "sys/log.h"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include "sys/energest.h"
#include "sys/rtimer.h"

#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "vsnspi_new.h"

#include "rf2xx_registermap.h"
#include "rf2xx.h"
#include "rf2xx_hal.h"


#define LOG_MODULE "rf2xx"
#define LOG_LEVEL LOG_CONF_LEVEL_RF2XX


#define DEFAULT_IRQ_MASK    (IRQ0_PLL_LOCK | IRQ2_RX_START | IRQ3_TRX_END | IRQ4_CCA_ED_DONE | IRQ5_AMI)


/// Macros to access I/O functions
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


/// SPI COMMAND BYTES
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


volatile static rtimer_clock_t last_packet_timestamp;

static uint8_t txBuffer[RF2XX_MAX_FRAME_SIZE];


// SPI struct (from VESNA drivers) and constant (immutable) pointer to it.
static vsnSPI_CommonStructure SPI_rf2xxStructure;
vsnSPI_CommonStructure * const rf2xxSPI = &SPI_rf2xxStructure;

// EXTI (interrupt) struct (from STM) and constant (immutable) pointer to it.
static EXTI_InitTypeDef EXTI_rf2xxStructure = {
    .EXTI_Line = EXTI_IRQ_LINE,
    .EXTI_Mode = EXTI_Mode_Interrupt,
    .EXTI_Trigger = EXTI_Trigger_Rising,
    .EXTI_LineCmd = ENABLE,
};
EXTI_InitTypeDef * const rf2xxEXTI = &EXTI_rf2xxStructure;

// Internal driver state machine & flags
volatile static rf2xx_flags_t flags;


typedef union {
    struct {
        uint8_t sleep:1;
        uint8_t rxMode:1;
        uint8_t txMode:1;
        uint8_t pollMode:1; // 6TISCH disables
        uint8_t autoACK:1; // RX_AACK
        uint8_t autoCCA:1; // TX_ARET
        uint8_t addrFilter:1;
    };
    uint8_t value;
} rf2xx_options_t;
static rf2xx_options_t opts;


typedef struct {
    //uint16_t SLEEP_TO_TRX_OFF;
    //uint16_t TRX_OFF_TO_SLEEP;
    uint16_t TRX_OFF_TO_PLL_ON;
    uint16_t TRX_OFF_TO_RX_ON;
    uint16_t PLL_ON_TO_BUSY_TX;
    uint16_t FORCE_TRX_OFF;
    uint16_t CCA;
} rf2xx_timetable_t;

rf2xx_timetable_t time = {
    .TRX_OFF_TO_PLL_ON = 110,
    .TRX_OFF_TO_RX_ON = 110,
    .PLL_ON_TO_BUSY_TX = 16,
    .FORCE_TRX_OFF = 1,
    .CCA = 140,
};


// Store radio type for later discrimitation
static uint8_t chip = RF2XX_UNDEFINED;



static int on(void);
static int off(void);
static uint8_t getChannel(void);
static float getFrequency(void);



PROCESS(rf2xx_process, "AT86RF2xx driver");
//PROCESS(rf2xx_calibration_process, "AT86RF2xx calibration");


inline static int8_t getRSSI(void) {
    int8_t value = bitRead(SR_ED_LEVEL);
    if (value >= 0x00 && value <= 0x54) return value - 91;
    return 0;
}

inline static rf2xx_irq_t getIRQs(void) {
	rf2xx_irq_t irq;
    setCS();
    vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, &irq.value);
    clearCS();

    return irq;
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


void setChannel(uint8_t ch) {
	switch (chip) {
		case RF2XX_AT86RF233:
		//case RF2XX_AT86RF232:
		case RF2XX_AT86RF231:
		case RF2XX_AT86RF230:
		case RF2XX_AT86RF212:
			bitWrite(SR_CHANNEL, ch);
			if (flags.PLL_LOCK) {
				RTIMER_BUSYWAIT_UNTIL(
					!getIRQs().IRQ0_PLL_LOCK,
					US_TO_RTIMERTICKS(20)
				);
			}
			break;

		default:
			break;
	}

	//LOG_INFO("Channel=%u Freq=%.2fMHz\n", getChannel(), getFrequency());
}

uint8_t getChannel(void) {
	return bitRead(SR_CHANNEL);
}

void setTxPower(uint8_t pwr) {
	switch (chip) {
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

uint8_t getTxPower(void) {
	switch (chip) {
		case RF2XX_AT86RF212:
			return bitRead(SR_TX_PWR_RF21x_ALL);

		case RF2XX_AT86RF230:
		case RF2XX_AT86RF231:
		case RF2XX_AT86RF233:
		default:
			return bitRead(SR_TX_PWR);
	}
}

void setPanID(uint16_t pan) {
	regWrite(RG_PAN_ID_0, pan & 0xFF);
	regWrite(RG_PAN_ID_1, pan >> 8);
	LOG_INFO("PAN == 0x%02x\n", pan);
}


uint16_t getPanID(void) {
	uint16_t pan = ((uint16_t)regRead(RG_PAN_ID_1) << 8) & 0xFF00;
    pan |= (uint16_t)regRead(RG_PAN_ID_0) & 0xFF;
    return pan;
}

uint16_t getShortAddr(void) {
	uint16_t addr = ((uint16_t)regRead(RG_SHORT_ADDR_1) << 8) & 0xFF00;
    addr |= (uint16_t)regRead(RG_SHORT_ADDR_0) & 0xFF;
    return addr;
}

void setShortAddr(uint16_t addr) {
	regWrite(RG_SHORT_ADDR_0, addr & 0xFF);
	regWrite(RG_SHORT_ADDR_1, addr >> 8);
	LOG_INFO("Short addr == 0x%02x\n", addr);
}

void setLongAddr(const uint8_t * addr, uint8_t len) {
    if (len > 8) len = 8;

    // The usual representation of MAC address has big-endianess. However,
    // This radio uses little-endian, so the order of bytes has to be reversed.
	// When we define IEEE addr, the most important byte is ext_addr[0], while
	// on radio RG_IEEE_ADDR_0 must contain the lowest/least important byte.
    for (uint8_t i = 0; i < len; i++) {
        regWrite(RG_IEEE_ADDR_7 - i, addr[i]);
    }
}


void getLongAddr(uint8_t *addr, uint8_t len) {
	if (len > 8) len = 8;
    for (uint8_t i = 0; i < len; i++) {
        addr[i] = regRead(RG_IEEE_ADDR_7 - i);
    }
}



static float getFrequency(void) {
	uint8_t channel = bitRead(SR_CHANNEL);

	switch (chip) {
		case RF2XX_AT86RF233:
		//case RF2XX_AT86RF232:
		case RF2XX_AT86RF231:
		case RF2XX_AT86RF230:
			// AT86RF23x radios work on 2.4GHz ISM band
			return 2405 + 5 * (channel - 11);

		case RF2XX_AT86RF212:
		{
			// AT86RF212 is a bit more advanced.
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


uint8_t getMaxChannel(void) {
	switch (chip) {
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

uint8_t getMinChannel(void) {
	switch (chip) {
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

uint8_t getMaxPwr(void) {
	// Get hex value to set it on radio
	switch (chip) {
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

// TODO: Still don't know how to do it for RF212
uint8_t getMinPwr(void) {
	// Get hex value to set it on radio
	switch (chip) {
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

inline static int8_t getCcaThreshold(void) {
    return -91 + 2 * bitRead(SR_CCA_ED_THRES);
}

inline static void setCcaThreshold(int8_t threshold) {
	LOG_INFO("CCA-threshod=%u\n", threshold);
    bitWrite(SR_CCA_ED_THRES, threshold / 2 + 91);
}






void reset(void) {
    uint8_t dummy __attribute__((unused));
    uint8_t partNum;

	setRST(); // hold radio in reset state

	clearCS(); // clear chip select (default)
	clearSLPTR(); // prevent going to sleep
	clearEXTI(); // clear interrupt flag
    clearRST(); // release radio from RESET state

    RTIMER_BUSYWAIT(US_TO_RTIMERTICKS(330));

	// Print for what pin layout was compiled.
	LOG_INFO("Compiled for " AT86RF2XX_BOARD_STRING "\n");

    // Radio indentification procedure
    do {
        LOG_INFO("Detecting AT86RF2xx radio\n");

        // Match JEDEC manufacturer ID
        if (regRead(RG_MAN_ID_0) == RF2XX_MAN_ID_0 && regRead(RG_MAN_ID_1) == RF2XX_MAN_ID_1) {
            LOG_INFO("JEDEC ID matches Atmel\n");

            // Match known radio (and sanitize radio type)
            partNum = regRead(RG_PART_NUM);
            switch (partNum) {
            case RF2XX_AT86RF233:
                time = (rf2xx_timetable_t){
                    .TRX_OFF_TO_PLL_ON = 80,
                    .TRX_OFF_TO_RX_ON = 80,
                    .PLL_ON_TO_BUSY_TX = 16,
                    .FORCE_TRX_OFF = 1,
                    .CCA = 140,
                };
            //case RF2XX_AT86RF232:
            case RF2XX_AT86RF231:
            case RF2XX_AT86RF230:
                time = (rf2xx_timetable_t){
                    .TRX_OFF_TO_PLL_ON = 110,
                    .TRX_OFF_TO_RX_ON = 110,
                    .PLL_ON_TO_BUSY_TX = 16,
                    .FORCE_TRX_OFF = 1,
                    .CCA = 140,
                };
            case RF2XX_AT86RF212:
                // Not sure about this timetable ...
                time = (rf2xx_timetable_t){
                    .TRX_OFF_TO_PLL_ON = 200,
                    .TRX_OFF_TO_RX_ON = 200,
                    .PLL_ON_TO_BUSY_TX = 50,
                    .FORCE_TRX_OFF = 1,
                    .CCA = 280,
                };
                LOG_INFO("Radio part number 0x%02x\n", partNum);
                chip = partNum;
            default:
                break;
            }
        }
    } while (RF2XX_UNDEFINED == chip);


	// CLKM clock change visible immediately
	bitWrite(SR_CLKM_SHA_SEL, 0);

	// disable CLKM (as output)
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
	//bitWrite(SR_AACK_I_AM_COORD, RF2XX_CONF_I_AM_COORD);

	// Set configured channel
	//rf2xx_setChannel(RF2XX_CHANNEL);
    LOG_INFO("Channel=%u Freq=%.2fMHz\n", getChannel(), getFrequency());

	//rf2xx_setTxPower(RF2XX_TX_POWER);
    // LOG_DBG("Pwr(hex): %u", rf2xx_getTxPower());
	LOG_INFO("Pwr=0x%02x\n", getTxPower());

	// First returned byte will be IRQ_STATUS;
	bitWrite(SR_SPI_CMD_MODE, SPI_CMD_MODE__IRQ_STATUS);

	// Configure Promiscuous mode; Incomplete
	//bitWrite(SR_AACK_PROM_MODE, RF2XX_CONF_PROMISCUOUS);
	//bitWrite(SR_AACK_UPLD_RES_FT, 1);
	//bitWrite(SR_AACK_FLTR_RES_FT, 1);

	// Enable only specific IRQs
	regWrite(RG_IRQ_MASK, DEFAULT_IRQ_MASK);

	// Read IRQ register to clear it
	dummy = regRead(RG_IRQ_STATUS);

	// Clear any interrupt pending
	clearEXTI();

    //RF2XX_STATS_RESET(); // resets radio metrics
    flags.value = 0; // clear all driver internal flags
    opts.autoACK = 1;
    opts.autoCCA = 1;

    // Initilize LQI & RSSI values
    rf2xx_last_lqi = 0;
    rf2xx_last_rssi = 0;
	last_packet_timestamp = 0;
}


int
init(void)
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
        RF2XX_SPI_SPEED
    );

	// register error callback
	vsnSPI_Init(rf2xxSPI, spiErrorCallback);

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

    // Reset internal and hardware states
	reset();

	// Start Contiki process which will take care of received packets
	process_start(&rf2xx_process, NULL);
	// process_start(&rf2xx_calibration_process, NULL);

	return 1;
}


static int prepare(const void *payload, unsigned short payload_len) {
    
    // Verify that payload is not bigger than radio's buffer
    if (payload_len > RF2XX_MAX_PAYLOAD_SIZE) {
        return RADIO_TX_ERR;
    }

	memcpy(txBuffer, payload, payload_len);

    #if !RF2XX_CONF_CHECKSUM
    {
        uint16_t crc = crc16_data(payload, payload_len, 0x00);
        memcpy(txBuffer+2+payload_len, &crc, RF2XX_CRC_SIZE); // copy CRC after actual data
    }
    #endif

    return RADIO_TX_OK;
}


static int transmit(unsigned short transmit_len) {
	vsnSPI_ErrorStatus status;
    vsnTime_Timeout timeout;
    uint8_t dummy __attribute__((unused));
    uint8_t trac;

    // Force radio into TRX_OFF mode
    regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
    RTIMER_BUSYWAIT(US_TO_RTIMERTICKS(time.FORCE_TRX_OFF));

	flags.value = 0;

    // Migrate to Tx mode
    regWrite(RG_TRX_STATE, opts.pollMode ? TRX_CMD_PLL_ON : TRX_CMD_TX_ARET_ON);

    // Wait Tx mode to be reached
    RTIMER_BUSYWAIT_UNTIL(
        !flags.PLL_LOCK,
        US_TO_RTIMERTICKS(time.TRX_OFF_TO_PLL_ON)
    );

	flags.PLL_LOCK = 1;

    setCS();

    ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);

	// Trigger transmit (see Manual p. 127, bottom)
    // The radio will start Tx, first byte needs to be written within 16us
    // (at default PA_BUF_LT and PA_LT)
	setSLPTR();
	clearSLPTR();

	// Send framebuffer write command
	status |= vsnSPI_pullByteTXRX(rf2xxSPI, (CMD_FB | CMD_WRITE), &dummy);

	// inform radio how many bytes will frame contain
	status |= vsnSPI_pullByteTXRX(rf2xxSPI, transmit_len + RF2XX_CRC_SIZE, &dummy);


	// When CRC is offloaded to the radio, we don't need to copy CRC bytes
	#if !RF2XX_CONF_CHECKSUM
	transmit_len += RF2XX_CRC_SIZE;
	#endif

	// Copy frame to the radio buffer. Radio is already transmitting data
	for (uint8_t i = 0; i < transmit_len; i++) {
		status |= vsnSPI_pullByteTXRX(rf2xxSPI, txBuffer[i], &dummy);
	}

	clearCS();

    // We expect SPI to work in polling mode (since interrupts are disabled, because of 6TISCH)
    if (VSN_SPI_SUCCESS != status) {
        return RADIO_TX_ERR;
    }

    // wait for transmit to complete
    RTIMER_BUSYWAIT_UNTIL(
        opts.pollMode ? !getIRQs().IRQ3_TRX_END : !flags.TRX_END,
        US_TO_RTIMERTICKS(RADIO_BYTE_AIR_TIME * 127) // Max transmit time
    );

	// Manually set flag, when timeout reached or pollMode
	flags.TRX_END = 1;

    ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);

    trac = opts.autoCCA ? bitRead(SR_TRAC_STATUS) : TRAC_SUCCESS;

    //rf2xx_on(); // Go back to Rx mode

	switch (trac) {
        case TRAC_SUCCESS:
            //RF2XX_STATS_ADD(txSuccess);
            return RADIO_TX_OK;

        case TRAC_NO_ACK:
            //RF2XX_STATS_ADD(txNoAck);
            //LOG_DBG_("noACK ");
            return RADIO_TX_NOACK;

        case TRAC_CHANNEL_ACCESS_FAILURE:
            //RF2XX_STATS_ADD(txCollision);
            //LOG_DBG_("collision");
            return RADIO_TX_COLLISION;

        default:
            //LOG_DBG_("Err(trac=0x%02x)", trac);
            return RADIO_TX_ERR;
	}
}


static int send(const void *payload, unsigned short payload_len) {
	prepare(payload, payload_len);
	return transmit(payload_len);
}


static int read(void *buf, unsigned short buf_len) {
    vsnSPI_ErrorStatus status;
    uint8_t payload_len;
    uint8_t trac;
    uint8_t dummy __attribute__((unused));
    uint16_t crc;

    setCS();
    status = vsnSPI_pullByteTXRX(rf2xxSPI, (CMD_FB | CMD_READ), &dummy); // command byte
    status = vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, &payload_len); // length byte

    if (payload_len < 3 && payload_len > 127) {
        // payload_len is not within valid range
        clearCS();
        return 0;
    }

    // CRC is always part of IEEE 802.15.4 frame
    payload_len -= RF2XX_CRC_SIZE;

    // Check if content will fit into buffer
    if (buf_len < payload_len) {
        clearCS();
        return 0;
    }

    // Transfer payload
    for (uint8_t i = 0; i < payload_len; i++) {
        status = vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, buf + i);
    }

    // transfer CRC (2 bytes)
    status = vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, (uint8_t *)&crc);
    status = vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, (uint8_t *)&crc + 1);

    // transfer LQI (1 byte)
    status = vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, &rf2xx_last_lqi);

    // Done!
    clearCS();

    #if !RF2XX_CONF_CHECKSUM
    {
        uint16_t _crc = crc16_data(buf, payload_len, 0x00);
        if (crc != _crc) {
            clearCS();
            return 0;
        }
    }
    #endif

    if (!opts.pollMode) {
        // measured on 8 symbols (128us) @ 250kbps
        rf2xx_last_rssi = getRSSI();

        packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, rf2xx_last_lqi);
    	packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rf2xx_last_rssi);
    }

    /*
    // Read TRAC status either if it is not mandatory (mainly for debugging)
	trac = extMode ? bitRead(SR_TRAC_STATUS) : TRAC_SUCCESS;

    switch (trac) {
    case TRAC_SUCCESS:
        // Everything is OK. CRC matches.
        //LOG_DBG_("TRAC=OK ");
        break;

    case TRAC_SUCCESS_WAIT_FOR_ACK:
        // TODO: How do we inform CONTIKI about NO-ACK?
        //LOG_DBG_("TRAC=NO-ACK ");
        break;

    case TRAC_INVALID:
        // Something went wrong with frame.
        // Read too fast? Buffer issues?
        // There is a change that frame is corrupted. No sense to make LQI.
        //LOG_DBG_("TRAC=INVALID ");
        return 0;

    default:
        // Any other state is invalid in RX mode
        //LOG_DBG_("TRAC=%u (INVALID) ", trac);
        return 0;
    }
    */

    return payload_len;
}





static int cca(void) {
    // return 1 for IDLE, 0 for BUSY channel

    if (opts.autoCCA) return 1; // extended mode does CCA on its own.


    regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
    RTIMER_BUSYWAIT(US_TO_RTIMERTICKS(time.FORCE_TRX_OFF));

	bitWrite(SR_RX_PDT_DIS, 1); // disable reception
	regWrite(RG_TRX_STATE, TRX_CMD_PLL_ON);

    bitWrite(SR_CCA_REQUEST, 1); // trigger CCA sensing
    RTIMER_BUSYWAIT(US_TO_RTIMERTICKS(time.CCA));

    // Read CCA value
	uint8_t cca = bitRead(SR_CCA_STATUS); // 1 = IDLE, 0 = BUSY

    // Back to TRX_OFF
    regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
	RTIMER_BUSYWAIT(US_TO_RTIMERTICKS(time.FORCE_TRX_OFF));

    bitWrite(SR_RX_PDT_DIS, 0); // enable reception in RX mode;

    on();

    return cca;
}


static int receiving_packet(void) {
    if (opts.pollMode) return getIRQs().IRQ2_RX_START;

    return flags.RX_START && !flags.TRX_END;
}

static int pending_packet(void) {
    if (opts.pollMode) return getIRQs().IRQ3_TRX_END;

    return flags.TRX_END; // this was set in interrupt
}

static int on(void) {
    uint8_t dummy __attribute__((unused));

	regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
	RTIMER_BUSYWAIT(US_TO_RTIMERTICKS(time.FORCE_TRX_OFF));

    flags.value = 0;

    regWrite(RG_TRX_STATE, opts.autoACK ? TRX_CMD_RX_AACK_ON : TRX_CMD_RX_ON);

	RTIMER_BUSYWAIT_UNTIL(
		!flags.PLL_LOCK, //getIRQs().IRQ0_PLL_LOCK,
		US_TO_RTIMERTICKS(time.TRX_OFF_TO_RX_ON)
	);

    flags.PLL_LOCK = 1;

    ENERGEST_ON(ENERGEST_TYPE_LISTEN);

    return 1;
}


static int off(void) {

    regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
    RTIMER_BUSYWAIT(US_TO_RTIMERTICKS(time.FORCE_TRX_OFF));

    ENERGEST_OFF(ENERGEST_TYPE_LISTEN);

    return 1;
}


void rf2xx_isr(void) {
	ENERGEST_ON(ENERGEST_TYPE_IRQ);

	volatile rf2xx_irq_t irq;
	irq.value = regRead(RG_IRQ_STATUS);

	if (irq.IRQ1_PLL_UNLOCK) {
		flags.PLL_LOCK = 0;
	}

	if (irq.IRQ0_PLL_LOCK) {
		flags.PLL_LOCK = 1;
		//lastCalibration = vsnTime_uptime();
	}

	if (irq.IRQ2_RX_START) {
		flags.RX_START = 1;
		flags.AMI = 0;
		flags.TRX_END = 0;

		//RF2XX_STATS_ADD(rxDetected);

		// When using non-extended mode, RSSI should be read on RX_START IRQ
		//if (!extMode) rf2xx_last_rssi = 3 * bitRead(SR_RSSI);
		if (!opts.pollMode) rf2xx_last_rssi = getRSSI();
	}

	if (irq.IRQ5_AMI) {
		flags.AMI = 1;
		//RF2XX_STATS_ADD(rxAddrMatch);
	}

	if (irq.IRQ6_TRX_UR) {
		flags.TRX_UR = 1;
	}


	if (irq.IRQ3_TRX_END) {
		flags.TRX_END = 1;

		// In extended mode IRQ3_TRX_END is best time to read RSSI
		//if (extMode) rf2xx_last_rssi = regRead(RG_PHY_ED_LEVEL);
		//if (extMode) rf2xx_last_rssi = rf2xx_ED_LEVEL();


		if (flags.RX_START) {
			last_packet_timestamp = RTIMER_NOW();
			//RF2XX_STATS_ADD(rxSuccess);

			// inform process about packet
			if (!opts.pollMode) process_poll(&rf2xx_process);
		} else {
			//RF2XX_STATS_ADD(txCount);
		}
	}

	if (irq.IRQ4_AWAKE_END) { // CCA_***_IRQ
		flags.SLEEP = 0;
		flags.CCA = 0;
	}

	if (irq.IRQ7_BAT_LOW) {}

	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}


PROCESS_THREAD(rf2xx_process, ev, data)
{
	int len;
	PROCESS_BEGIN();

	LOG_INFO("AT86RF2xx driver process started!\n");

	while(1) {
		PROCESS_YIELD_UNTIL(!opts.pollMode && ev == PROCESS_EVENT_POLL);

        //RF2XX_STATS_ADD(rxToStack);
		//LOG_DBG("calling receiver callback\n");

		packetbuf_clear();
		packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
		len = read(packetbuf_dataptr(), PACKETBUF_SIZE);

		packetbuf_set_datalen(len);

		NETSTACK_MAC.input();
	}

	PROCESS_END();
}




static radio_result_t
get_value(radio_param_t param, radio_value_t *value) {
	if (!value) return RADIO_RESULT_INVALID_VALUE;

	switch (param) {
		case RADIO_PARAM_POWER_MODE:
			// TODO: return RADIO_POWER_MODE_CARRIER_ON when in Tx mode
			*value = flags.PLL_LOCK ? RADIO_POWER_MODE_ON : RADIO_POWER_MODE_OFF;
			return RADIO_RESULT_OK;

		case RADIO_PARAM_CHANNEL:
			*value = (radio_value_t)getChannel();
			return RADIO_RESULT_OK;

		case RADIO_PARAM_PAN_ID:
			*value = (radio_value_t)getPanID();
			return RADIO_RESULT_OK;

		case RADIO_PARAM_16BIT_ADDR:
			*value = (radio_value_t)getShortAddr();
			return RADIO_RESULT_OK;

		case RADIO_PARAM_RX_MODE:
			*value = 0;
			if (opts.addrFilter) *value |= RADIO_RX_MODE_ADDRESS_FILTER;
			if (opts.autoACK) *value |= RADIO_RX_MODE_AUTOACK;
			if (opts.pollMode) *value |= RADIO_RX_MODE_POLL_MODE;
			return RADIO_RESULT_OK;

		case RADIO_PARAM_TX_MODE:
			if (opts.autoCCA) *value |= RADIO_TX_MODE_SEND_ON_CCA;
			return RADIO_RESULT_OK;

		case RADIO_PARAM_TXPOWER:
			*value = getTxPower();
			return RADIO_RESULT_OK;

		case RADIO_PARAM_CCA_THRESHOLD:
			return getCcaThreshold();

		case RADIO_PARAM_RSSI:
			*value = rf2xx_last_rssi;
			return RADIO_RESULT_OK;

		case RADIO_PARAM_LAST_LINK_QUALITY:
			*value = rf2xx_last_lqi;
			return RADIO_RESULT_OK;

		case RADIO_CONST_CHANNEL_MIN:
			*value = getMinChannel();
			return RADIO_RESULT_OK;

		case RADIO_CONST_CHANNEL_MAX:
			*value = getMaxChannel();
			return RADIO_RESULT_OK;

		case RADIO_CONST_TXPOWER_MIN:
			*value = getMinPwr();
			return RADIO_RESULT_OK;

		case RADIO_CONST_TXPOWER_MAX:
			*value = getMaxPwr();
			return RADIO_RESULT_OK;

		case RADIO_CONST_PHY_OVERHEAD:
			*value = (radio_value_t)RF2XX_PHY_OVERHEAD;
			return RADIO_RESULT_OK;

		case RADIO_CONST_BYTE_AIR_TIME:
			*value = (radio_value_t)RF2XX_BYTE_AIR_TIME;
			return RADIO_RESULT_OK;

		case RADIO_CONST_DELAY_BEFORE_TX:
			*value = (radio_value_t)RF2XX_DELAY_BEFORE_TX;
			return RADIO_RESULT_OK;

		case RADIO_CONST_DELAY_BEFORE_RX:
			*value = (radio_value_t)RF2XX_DELAY_BEFORE_RX;
			return RADIO_RESULT_OK;

		case RADIO_CONST_DELAY_BEFORE_DETECT:
			*value = (radio_value_t)RF2XX_DELAY_BEFORE_DETECT;
			return RADIO_RESULT_OK;

		default:
			return RADIO_RESULT_NOT_SUPPORTED;
	}

}


static radio_result_t
set_value(radio_param_t param, radio_value_t value) {
	switch (param) {
	case RADIO_PARAM_POWER_MODE:
		switch (value) {
			case RADIO_POWER_MODE_ON:
				on();
				return RADIO_RESULT_OK;
			case RADIO_POWER_MODE_OFF:
				off();
				return RADIO_RESULT_OK;
			default:
				return RADIO_RESULT_INVALID_VALUE;
		}

	case RADIO_PARAM_CHANNEL:
		if (value >= getMinChannel() && value <= getMaxChannel()) {
			setChannel(value);
			return RADIO_RESULT_OK;
		}
		return RADIO_RESULT_INVALID_VALUE;
		

	case RADIO_PARAM_PAN_ID:
		setPanID(value);
		return RADIO_RESULT_OK;

	case RADIO_PARAM_16BIT_ADDR:
		setShortAddr(value);
		return RADIO_RESULT_OK;

	case RADIO_PARAM_RX_MODE:
		LOG_DBG("RADIO_PARAM_RX_MODE 0x%02x\n", value);
        opts.addrFilter = !!(value & RADIO_RX_MODE_ADDRESS_FILTER);
        opts.autoACK = !!(value & RADIO_RX_MODE_AUTOACK);
        opts.pollMode = !!(value & RADIO_RX_MODE_POLL_MODE);

		regWrite(RG_IRQ_MASK, opts.pollMode ? 0x00 : DEFAULT_IRQ_MASK);


		//setAddrFilter(value & RADIO_RX_MODE_ADDRESS_FILTER);
		//setAutoAck(value & RADIO_RX_MODE_AUTOACK);
		//setPollMode(value & RADIO_RX_MODE_POLL_MODE);
		return RADIO_RESULT_OK;

	case RADIO_PARAM_TX_MODE:
		opts.autoCCA = !!(value & RADIO_TX_MODE_SEND_ON_CCA);
		// Disable automatic CCA when MAX_CSMA_RETRIES = 7
		bitWrite(SR_MAX_CSMA_RETRIES, opts.autoCCA ? 7 : RF2XX_CONF_MAX_CSMA_RETRIES);
		return RADIO_RESULT_OK;

	case RADIO_PARAM_TXPOWER:
		if (value >= getMinPwr() && value <= getMaxPwr()) {
			setTxPower(value);
			return RADIO_RESULT_OK;
		}
		return RADIO_RESULT_INVALID_VALUE;

	case RADIO_PARAM_CCA_THRESHOLD:
		setCcaThreshold(value);
		return RADIO_RESULT_OK;

	case RADIO_PARAM_SHR_SEARCH:
		//rf2xx_setShrSearch(value);
		//return RADIO_RESULT_OK;
		return RADIO_RESULT_NOT_SUPPORTED;

	default:
		return RADIO_RESULT_NOT_SUPPORTED;
	}
}


static radio_result_t get_object(radio_param_t param, void *dest, size_t size) {
	if (dest == NULL) return RADIO_RESULT_ERROR;

	switch (param) {
		case RADIO_PARAM_64BIT_ADDR:
			getLongAddr((uint8_t *)dest, size);
			return RADIO_RESULT_OK;

		case RADIO_PARAM_LAST_PACKET_TIMESTAMP:
			// TODO: Some extra chech of size???
			*(rtimer_clock_t *)dest = last_packet_timestamp;
			return RADIO_RESULT_OK;

		//#if MAC_CONF_WITH_TSCH
		//case RADIO_CONST_TSCH_TIMING:
		//	*(const uint16_t **)dest = TSCH_CONF_DEFAULT_TIMESLOT_TIMING;
		//#endif

		default:
			return RADIO_RESULT_NOT_SUPPORTED;
	}
}


static radio_result_t set_object(radio_param_t param, const void *src, size_t size) {
	if (src == NULL) return RADIO_RESULT_ERROR;
	switch (param) {
		case RADIO_PARAM_64BIT_ADDR:
			setLongAddr((const uint8_t *)src, size);
			return RADIO_RESULT_OK;

		default:
			return RADIO_RESULT_NOT_SUPPORTED;
	}
}


const struct radio_driver rf2xx_driver = {
	init,
	prepare,
	transmit,
	send,
	read,
	cca,
	receiving_packet,
	pending_packet,
	on,
	off,
	get_value,
	set_value,
	get_object,
	set_object,
};

// CONTIKI











// TODO: FIX: return signed integer with proper value
int8_t get_ED_LEVEL(void) {
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

// This one is for SNR measurements
/*int8_t rf2xx_RSSI(void) {
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
}*/


/*int8_t rf2xx_rssi(void) {
	return opts.pollMode ? rf2xx_ED_LEVEL() : rf2xx_RSSI();
}*/

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

    /* do {
        vsnTime_setTimeout(&timeout, 200);
        do {
            status = vsnSPI_checkSPIstatus(rf2xxSPI);
            if (!vsnTime_checkTimeout(&timeout)) {
				LOG_ERR("SPI timeout\n");
                break;
            }
        } while (status & (VSN_SPI_COM_IN_PROGRESS | VSN_SPI_BUS_BUSY));*/

        setCS();
        status = vsnSPI_pullByteTXRX(rf2xxSPI, addr, &value);
        status |= vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, &value);
        clearCS();

    //} while (status != VSN_SPI_SUCCESS);

    return value;
}


void regWrite(uint8_t addr, uint8_t value) {
    vsnSPI_ErrorStatus status;
	vsnTime_Timeout timeout;
    uint8_t dummy __attribute__((unused));

    addr = (addr & CMD_REG_MASK) | CMD_REG | CMD_WRITE;

    /*do {
        vsnTime_setTimeout(&timeout, 200);
        do {
            status = vsnSPI_checkSPIstatus(rf2xxSPI);
            if (!vsnTime_checkTimeout(&timeout)) {
				LOG_ERR("SPI timeout\n");
                break;
            }
        } while (status & (VSN_SPI_COM_IN_PROGRESS | VSN_SPI_BUS_BUSY));*/

        setCS();
        status = vsnSPI_pullByteTXRX(rf2xxSPI, addr, &dummy);
        status |= vsnSPI_pullByteTXRX(rf2xxSPI, value, &dummy);
        clearCS();

    //} while (status != VSN_SPI_SUCCESS);
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
