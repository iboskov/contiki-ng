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
#include "rf2xx_hal.h"
#include "rf2xx.h"
#include "rf2xx_arch.h"


#define LOG_MODULE "rf2xx"
#define LOG_LEVEL LOG_CONF_LEVEL_RF2XX

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

#define RSSI_BASE_VAL   (-91)

#define TX_ARET_MODE    (1)
#define RX_AACK_MODE    (1)
#define POLLING_MODE    (0)
#define SEND_ON_CCA     (0) // Do we manually perform CCA?


PROCESS(rf2xx_process, "AT86RF2xx driver");


#define DEFAULT_IRQ_MASK    (IRQ2_RX_START | IRQ3_TRX_END | IRQ4_CCA_ED_DONE | IRQ5_AMI)


#if LOG_DBG_ENABLED
// SRC: https://barrgroup.com/Embedded-Systems/How-To/Define-Assert-Macro
#define ASSERT(expr) if (!(expr)) LOG_ERR("Err: " #expr "\n")
#endif

#define BUSYWAIT_UNTIL(expr) while(!(expr))

static uint8_t rf2xxChip = RF2XX_UNDEFINED;

volatile uint8_t rf2xx_last_lqi;
volatile int8_t rf2xx_last_rssi;


static uint8_t txBuffer[RF2XX_MAX_FRAME_SIZE];


volatile static rf2xx_flags_t flags;


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


static void
spiErrorCallback(void *cbDevStruct)
{
	vsnSPI_CommonStructure *spi = cbDevStruct;
	vsnSPI_chipSelect(spi, SPI_CS_HIGH);
	LOG_ERR("SPI error callback triggered\n");
}



void
setPanID(uint16_t pan)
{
	regWrite(RG_PAN_ID_0, pan & 0xFF);
	regWrite(RG_PAN_ID_1, pan >> 8);
	LOG_INFO("PAN == 0x%02x\n", pan);
}


uint16_t
getPanID(void)
{
	uint16_t pan = ((uint16_t)regRead(RG_PAN_ID_1) << 8) & 0xFF00;
    pan |= (uint16_t)regRead(RG_PAN_ID_0) & 0xFF;
    return pan;
}

uint16_t
getShortAddr(void)
{
	uint16_t addr = ((uint16_t)regRead(RG_SHORT_ADDR_1) << 8) & 0xFF00;
    addr |= (uint16_t)regRead(RG_SHORT_ADDR_0) & 0xFF;
    return addr;
}

void
setShortAddr(uint16_t addr)
{
	regWrite(RG_SHORT_ADDR_0, addr & 0xFF);
	regWrite(RG_SHORT_ADDR_1, addr >> 8);
	LOG_INFO("Short addr == 0x%02x\n", addr);
}

void
setLongAddr(const uint8_t * addr, uint8_t len)
{
    if (len > 8) len = 8;

    // The usual representation of MAC address has big-endianess. However,
    // This radio uses little-endian, so the order of bytes has to be reversed.
	// When we define IEEE addr, the most important byte is ext_addr[0], while
	// on radio RG_IEEE_ADDR_0 must contain the lowest/least important byte.
    for (uint8_t i = 0; i < len; i++) {
        regWrite(RG_IEEE_ADDR_7 - i, addr[i]);
    }
}


void
getLongAddr(uint8_t *addr, uint8_t len)
{
	if (len > 8) len = 8;
    for (uint8_t i = 0; i < len; i++) {
        addr[i] = regRead(RG_IEEE_ADDR_7 - i);
    }
}


inline static uint8_t
getIRQs(void)
{
    vsnSPI_ErrorStatus status = VSN_SPI_SUCCESS;
    uint8_t value;

	setCS();
	status = vsnSPI_pullByteTXRX(rf2xxSPI, (CMD_REG | CMD_READ), &value);
	clearCS();

    ASSERT(VSN_SPI_SUCCESS == status);

    return value;
}




int
rf2xx_reset(void)
{
    uint8_t dummy __attribute__((unused));
    uint8_t partNum;

    LOG_DBG("%s\n", __func__);

	setRST(); // hold radio in reset state

	clearCS(); // clear chip select (default)
	clearSLPTR(); // prevent going to sleep
	clearEXTI(); // clear interrupt flag
    clearRST(); // release radio from RESET state

    //regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
    //BUSYWAIT_UNTIL(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS));
    //ASSERT(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS));

    //flags.value = 0;

    rf2xx_off();

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
                case RF2XX_AT86RF231:
                case RF2XX_AT86RF230:
                //case RF2XX_AT86RF212:
                    rf2xxChip = partNum;
                default:
                    break;
            }
        }
    } while (RF2XX_UNDEFINED == rf2xxChip);

    ASSERT(RF2XX_UNDEFINED != rf2xxChip);

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

#if !SEND_ON_CCA
	// Number of CSMA retries (part of IEEE 802.15.4)
	// Possible values [0 - 5], 6 is reserved, 7 will send immediately (no CCA)
	bitWrite(SR_MAX_CSMA_RETRIES, RF2XX_CONF_MAX_CSMA_RETRIES);
#else
    bitWrite(SR_MAX_CSMA_RETRIES, 7);
#endif

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
    //LOG_INFO("Channel=%u Freq=%.2fMHz\n", getChannel(), getFrequency());

	//rf2xx_setTxPower(RF2XX_TX_POWER);
    // LOG_DBG("Pwr(hex): %u", rf2xx_getTxPower());
	//LOG_INFO("Pwr=0x%02x\n", getTxPower());

	// First returned byte will be IRQ_STATUS;
	//bitWrite(SR_SPI_CMD_MODE, SPI_CMD_MODE__IRQ_STATUS);

	// Configure Promiscuous mode; Incomplete
	bitWrite(SR_AACK_PROM_MODE, 1);
	bitWrite(SR_AACK_UPLD_RES_FT, 1);
	bitWrite(SR_AACK_FLTR_RES_FT, 1);

	// Enable only specific IRQs
	regWrite(RG_IRQ_MASK, DEFAULT_IRQ_MASK);

	// Read IRQ register to clear it
	dummy = regRead(RG_IRQ_STATUS);

	// Clear any interrupt pending
	clearEXTI();

	return 1;
}


int
rf2xx_init(void)
{
    LOG_DBG("%s\n", __func__);

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
	rf2xx_reset();

	// Start Contiki process which will take care of received packets
	process_start(&rf2xx_process, NULL);
	// process_start(&rf2xx_calibration_process, NULL);

	return 1;
}


int
rf2xx_prepare(const void *payload, unsigned short payload_len)
{
    LOG_DBG("%s\n", __func__);

    if (payload_len > RF2XX_MAX_PAYLOAD_SIZE) {
        LOG_ERR("Payload larger than radio buffer: %u > %u\n", payload_len, RF2XX_MAX_PAYLOAD_SIZE);
        return RADIO_TX_ERR;
    }

    memcpy(txBuffer, payload, payload_len);
    LOG_DBG("Prepared %u bytes\n", payload_len);

#if !RF2XX_CONF_CHECKSUM
    {
        // Copy CRC
        uint8_t crc = crc16_data(payload, payload_len, 0x00);
        memcpy(txBuffer + payload_len, (uint8_t *)&crc, 2);
        LOG_DBG("CRC == 0x%04x\n", crc);
    }
#endif

    return RADIO_TX_OK;
}


int
rf2xx_transmit(unsigned short transmit_len)
{
    uint8_t dummy __attribute__((unused));
    uint8_t trac = TRAC_SUCCESS;
    vsnSPI_ErrorStatus status = VSN_SPI_SUCCESS;

    //uint8_t buffer[2 + sizeof(txBuffer)];

    LOG_DBG("%s\n", __func__);

    // Force radio into TRX_OFF mode
    regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
    BUSYWAIT_UNTIL(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS));
    ASSERT(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS));

    flags.value = 0;

#if TX_ARET_MODE
    regWrite(RG_TRX_STATE, TRX_CMD_TX_ARET_ON);
    BUSYWAIT_UNTIL(TRX_STATUS_TX_ARET_ON == bitRead(SR_TRX_STATUS));
    ASSERT(TRX_STATUS_TX_ARET_ON == bitRead(SR_TRX_STATUS));
#else
    regWrite(RG_TRX_STATE, TRX_CMD_TX_ON);
    BUSYWAIT_UNTIL(TRX_STATUS_TX_ON == bitRead(SR_TRX_STATUS));
    ASSERT(TRX_STATUS_TX_ON == bitRead(SR_TRX_STATUS));
#endif

    flags.PLL_LOCK = 1;

    // Trigger transmit
    setSLPTR();
    clearSLPTR();

    setCS();

    status |= vsnSPI_pullByteTXRX(rf2xxSPI, (CMD_FB | CMD_WRITE), &dummy); // write to FB
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);

    status |= vsnSPI_pullByteTXRX(rf2xxSPI, transmit_len + 2, &dummy); // payload + CRC
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);

#if !RF2XX_CONF_CHECKSUM
    // Copy CRC bytes
    transmit_len += RF2XX_CRC_SIZE;
#endif

    for (uint8_t i = 0; i < transmit_len; i++) {
        status |= vsnSPI_pullByteTXRX(rf2xxSPI, txBuffer[i], &dummy);
        BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);
    }

    clearCS();

    ASSERT(VSN_SPI_SUCCESS == status);

    LOG_DBG("Sent %u bytes\n", transmit_len);

    // Wait to complete BUSY STATE
    BUSYWAIT_UNTIL(flags.TRX_END);

    flags.TRX_END = 0;

#if TX_ARET_MODE
    ASSERT(TRX_STATUS_TX_ARET_ON == bitRead(SR_TRX_STATUS));

    // Read TRAC status 
    trac = bitRead(SR_TRAC_STATUS);

    // TODO: Check TRAC status
#else
    ASSERT(TRX_STATUS_TX_ON == bitRead(SR_TRX_STATUS));
#endif

    // Migrate to RX mode
#if RX_AACK_MODE
    regWrite(RG_TRX_STATE, TRX_CMD_RX_AACK_ON);
    BUSYWAIT_UNTIL(TRX_STATUS_RX_AACK_ON == bitRead(SR_TRX_STATUS));
    ASSERT(TRX_STATUS_RX_AACK_ON == bitRead(SR_TRX_STATUS));
#else
    regWrite(RG_TRX_STATE, TRX_CMD_RX_ON);
    BUSYWAIT_UNTIL(TRX_STATUS_RX_AACK_ON == bitRead(SR_TRX_STATUS));
    ASSERT(TRX_STATUS_RX_ON == bitRead(SR_TRX_STATUS));
#endif
    flags.PLL_LOCK = 1;

    //rf2xx_on();

    return RADIO_TX_OK;
}

int
rf2xx_send(const void *payload, unsigned short payload_len)
{
    LOG_DBG("%s\n", __func__);
	rf2xx_prepare(payload, payload_len);
	return rf2xx_transmit(payload_len);
}

int rf2xx_read(void *buf, unsigned short buf_len)
{
    uint8_t trac;
    uint8_t dummy __attribute__((unused));
    uint8_t payload_len = 0;
    uint16_t crc;
    vsnSPI_ErrorStatus status = VSN_SPI_SUCCESS;

    //uint8_t txBuf[2 + 127 + 1];
    //uint8_t rxBuf[2 + 127 + 1];

    //txBuf[0] = (CMD_FB | CMD_READ);

    LOG_DBG("%s\n", __func__);

    //dummy = regRead(RG_IRQ_STATUS);

    flags.TRX_END = 0;
    flags.RX_START = 0;
    flags.AMI = 0;

    setCS();
    //status = vsnSPI_transferDataTXRX(rf2xxSPI, txBuf, rxBuf, 2 + 127 + 1, spiCallback);

    //while (getCS());

    //payload_len = rxBuf[1];
    status |= vsnSPI_pullByteTXRX(rf2xxSPI, (CMD_FB | CMD_READ), &dummy);
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);

    status |= vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, &payload_len);
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);

    payload_len -= 2;

    if (payload_len > buf_len) {
        clearCS();
        LOG_ERR("Frame too big for buffer (%u > %u)\n", payload_len, buf_len);
        return 0;
    }

    for (int i = 0; i < payload_len; i++) {
        vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, buf + i);
        BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);
    }

#if !RF2XX_CONF_CHECKSUM
    vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, (uint8_t *)&crc);
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);

    vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, (uint8_t *)&crc + 1);
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);

    ASSERT(crc16_data(buf, payload_len, 0x00) == crc);
#else
    vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, &dummy);
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);

    vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, &dummy);
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);
#endif

    vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, (uint8_t *)&rf2xx_last_lqi);
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);

    clearCS();

    ASSERT(VSN_SPI_SUCCESS == status);

    // RSSI of packet
    rf2xx_last_rssi = bitRead(SR_ED_LEVEL) - 91;

    LOG_DBG("RSSI=%idBm LQI=%u\n", rf2xx_last_rssi, rf2xx_last_lqi);

#if !POLLING_MODE
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, rf2xx_last_lqi);
    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rf2xx_last_rssi);
#endif

    return payload_len;
}


int
rf2xx_channel_clear(void)
{
    LOG_DBG("%s\n", __func__);

#if TX_ARET_MODE || !SEND_ON_CCA
    return 1;
#else
    regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
    BUSYWAIT_UNTIL(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS));
    ASSERT(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS));

    flags.value = 0;

	bitWrite(SR_RX_PDT_DIS, 1); // disable reception
	regWrite(RG_TRX_STATE, TRX_CMD_RX_ON);
    BUSYWAIT_UNTIL(TRX_STATUS_RX_ON == bitRead(SR_TRX_STATUS));
    ASSERT(TRX_STATUS_RX_ON == bitRead(SR_TRX_STATUS));

    bitWrite(SR_CCA_REQUEST, 1); // trigger CCA sensing
    BUSYWAIT_UNTIL(1 == bitRead(SR_CCA_DONE));

    uint8_t cca = bitRead(SR_CCA_STATUS); // 1 = IDLE, 0 = BUSY

    regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
    BUSYWAIT_UNTIL(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS));
    ASSERT(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS));

    flags.value = 0;

	bitWrite(SR_RX_PDT_DIS, 0); // Enable reception

    return cca;
#endif
}


int
rf2xx_receiving_packet(void)
{
    LOG_DBG("%s\n", __func__);
    //return flags.RX_START && !flags.TRX_END;
    uint8_t trxState = bitRead(SR_TRX_STATUS);
    switch (trxState) {
        case TRX_STATUS_BUSY_RX:
        case TRX_STATUS_BUSY_RX_AACK:
        case TRX_STATUS_BUSY_RX_AACK_NOCLK:
            return 1;

        default:
            return 0;
    }
}

int
rf2xx_pending_packet(void)
{
    return flags.RX_START && flags.TRX_END;
}


int
rf2xx_on(void)
{
    LOG_DBG("%s\n", __func__);

    regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
    BUSYWAIT_UNTIL(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS));
    ASSERT(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS));

    flags.value = 0;

#if RX_AACK_MODE
	regWrite(RG_TRX_STATE, TRX_CMD_RX_AACK_ON);
    BUSYWAIT_UNTIL(TRX_STATUS_RX_AACK_ON == bitRead(SR_TRX_STATUS));
    ASSERT(TRX_STATUS_RX_AACK_ON == bitRead(SR_TRX_STATUS));
#else
	regWrite(RG_TRX_STATE, TRX_CMD_RX_ON);
    BUSYWAIT_UNTIL(TRX_STATUS_RX_ON == bitRead(SR_TRX_STATUS));
    ASSERT(TRX_STATUS_RX_ON == bitRead(SR_TRX_STATUS));
#endif
    flags.PLL_LOCK = 1;

    return 1;
}


int
rf2xx_off(void)
{
    LOG_DBG("%s\n", __func__);

    if (!rf2xx_receiving_packet()) {
        regWrite(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
        BUSYWAIT_UNTIL(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS));
        ASSERT(TRX_STATUS_TRX_OFF == bitRead(SR_TRX_STATUS));

        flags.value = 0;

        return 1;
    }

    return 0;
}

void
rf2xx_isr(void)
{
    rf2xx_irq_t irq;
    irq.value = bitRead(SR_IRQ_STATUS);

    if (irq.IRQ1_PLL_UNLOCK) {
        flags.PLL_LOCK = 0;
    }

    if (irq.IRQ0_PLL_LOCK) {
        flags.PLL_LOCK = 1;
    }

    if (irq.IRQ2_RX_START) {
        flags.RX_START = 1;
        flags.AMI = 0;
        flags.TRX_END = 0;

        printf("RX_START\n");
    }

    if (irq.IRQ5_AMI) {
        flags.AMI = 1;
    }

    if (irq.IRQ6_TRX_UR) {
        flags.TRX_UR = 1;
    }

    if (irq.IRQ3_TRX_END) {
        flags.TRX_END = 1;

        

        // RSSI ?

        if (flags.RX_START) {
            // last_packet_timestamp

            process_poll(&rf2xx_process);
            printf("RX_END\n");
        } else {
            // TX mode
            printf("TX_END\n");
        }
    }

	if (irq.IRQ4_AWAKE_END) { // CCA_***_IRQ
		flags.SLEEP = 0;
		flags.CCA = 0;
	}

	if (irq.IRQ7_BAT_LOW) {}

}


PROCESS_THREAD(rf2xx_process, ev, data)
{
	int len;
	PROCESS_BEGIN();

	LOG_INFO("AT86RF2xx driver process started!\n");

	while(1) {
		PROCESS_YIELD_UNTIL(!POLLING_MODE && ev == PROCESS_EVENT_POLL);
        //PROCESS_YIELD_UNTIL(rf2xx_pending_packet());
        //RF2XX_STATS_ADD(rxToStack);
        LOG_DBG("calling receiver callback\n");

        packetbuf_clear();
        //packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
        len = rf2xx_read(packetbuf_dataptr(), PACKETBUF_SIZE);

        packetbuf_set_datalen(len);

        NETSTACK_MAC.input();
        
	}
	PROCESS_END();
}




static radio_result_t
get_value(radio_param_t param, radio_value_t *value)
{
	if (!value) return RADIO_RESULT_INVALID_VALUE;

	switch (param) {
		case RADIO_PARAM_POWER_MODE:
        {
            uint8_t trxState = bitRead(SR_TRX_STATUS);
            switch (trxState) {
                case TRX_STATUS_RX_ON:
                case TRX_STATUS_RX_ON_NOCLK:
                case TRX_STATUS_RX_AACK_ON:
                case TRX_STATUS_RX_AACK_ON_NOCLK:
                case TRX_STATUS_BUSY_RX:
                case TRX_STATUS_BUSY_RX_AACK:
                case TRX_STATUS_BUSY_RX_AACK_NOCLK:
                    *value = RADIO_POWER_MODE_ON;
                    return RADIO_RESULT_OK;

                case TRX_STATUS_TX_ON:
                case TRX_STATUS_TX_ARET_ON:
                case TRX_STATUS_BUSY_TX:
                case TRX_STATUS_BUSY_TX_ARET:
                    *value = RADIO_POWER_MODE_CARRIER_ON;
                    return RADIO_RESULT_OK;

                case TRX_STATUS_TRX_OFF:
                default:
                    *value = RADIO_POWER_MODE_OFF;
                    return RADIO_RESULT_OK;
            }
        }

		case RADIO_PARAM_CHANNEL:
			*value = (radio_value_t)bitRead(SR_CHANNEL);
			return RADIO_RESULT_OK;

		case RADIO_PARAM_PAN_ID:
			*value = (radio_value_t)getPanID();
			return RADIO_RESULT_OK;

		case RADIO_PARAM_16BIT_ADDR:
			*value = (radio_value_t)getShortAddr();
			return RADIO_RESULT_OK;

		case RADIO_PARAM_RX_MODE:
            *value = 0;
			if (RX_AACK_MODE) *value |= RADIO_RX_MODE_ADDRESS_FILTER;
			if (RX_AACK_MODE) *value |= RADIO_RX_MODE_AUTOACK;
			if (POLLING_MODE) *value |= RADIO_RX_MODE_POLL_MODE;
            return RADIO_RESULT_OK;

		case RADIO_PARAM_TX_MODE:
            *value = 0;
			if (SEND_ON_CCA) *value |= RADIO_TX_MODE_SEND_ON_CCA;
			return RADIO_RESULT_OK;

		case RADIO_PARAM_TXPOWER:
			*value = (radio_value_t)bitRead(SR_TX_PWR);
			return RADIO_RESULT_OK;

		case RADIO_PARAM_CCA_THRESHOLD:
			return RSSI_BASE_VAL + 2 * (radio_value_t)bitRead(SR_CCA_ED_THRES);

		case RADIO_PARAM_RSSI:
            LOG_DBG("Request current RSSI\n");
			*value = (radio_value_t)(3 * (bitRead(SR_RSSI) - 1) + RSSI_BASE_VAL);
			return RADIO_RESULT_OK;

		case RADIO_PARAM_LAST_LINK_QUALITY:
			*value = (radio_value_t)rf2xx_last_lqi;
			return RADIO_RESULT_OK;

		case RADIO_CONST_CHANNEL_MIN:
			*value = 11;
			return RADIO_RESULT_OK;

		case RADIO_CONST_CHANNEL_MAX:
			*value = 26;
			return RADIO_RESULT_OK;

		case RADIO_CONST_TXPOWER_MIN:
			*value = 0xF;
			return RADIO_RESULT_OK;

		case RADIO_CONST_TXPOWER_MAX:
			*value = 0x0;
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
set_value(radio_param_t param, radio_value_t value)
{
	switch (param) {
        case RADIO_PARAM_POWER_MODE:
            switch (value) {
                case RADIO_POWER_MODE_ON:
                    rf2xx_on();
                    return RADIO_RESULT_OK;

                case RADIO_POWER_MODE_OFF:
                    rf2xx_off();
                    return RADIO_RESULT_OK;

                default:
                    return RADIO_RESULT_INVALID_VALUE;
            }

        case RADIO_PARAM_CHANNEL:
            if (value < 11 || value > 26) {
                return RADIO_RESULT_INVALID_VALUE;
            }
            bitWrite(SR_CHANNEL, value);
            return RADIO_RESULT_OK;

        case RADIO_PARAM_PAN_ID:
            setPanID(value);
            return RADIO_RESULT_OK;

        case RADIO_PARAM_16BIT_ADDR:
            setShortAddr(value);
            return RADIO_RESULT_OK;

        //case RADIO_PARAM_RX_MODE:
            //LOG_DBG("RADIO_PARAM_RX_MODE 0x%02x\n", value);
            //opts.addrFilter = !!(value & RADIO_RX_MODE_ADDRESS_FILTER);
            //opts.autoACK = !!(value & RADIO_RX_MODE_AUTOACK);
            //opts.pollMode = !!(value & RADIO_RX_MODE_POLL_MODE);

            //regWrite(RG_IRQ_MASK, opts.pollMode ? 0x00 : DEFAULT_IRQ_MASK);


            //setAddrFilter(value & RADIO_RX_MODE_ADDRESS_FILTER);
            //setAutoAck(value & RADIO_RX_MODE_AUTOACK);
            //setPollMode(value & RADIO_RX_MODE_POLL_MODE);
            //return RADIO_RESULT_OK;

        //case RADIO_PARAM_TX_MODE:
        //	opts.autoCCA = !!(value & RADIO_TX_MODE_SEND_ON_CCA);
        //	// Disable automatic CCA when MAX_CSMA_RETRIES = 7
        //	bitWrite(SR_MAX_CSMA_RETRIES, opts.autoCCA ? RF2XX_CONF_MAX_CSMA_RETRIES : 7);
        //	return RADIO_RESULT_OK;

        case RADIO_PARAM_TXPOWER:
            if (value < 0 || value > 0xF) {
                return RADIO_RESULT_INVALID_VALUE;
            }
            bitWrite(SR_TX_PWR, value);
            return RADIO_RESULT_OK;


        case RADIO_PARAM_CCA_THRESHOLD:
            bitWrite(SR_CCA_ED_THRES, value / 2 + 91);
            return RADIO_RESULT_OK;

        case RADIO_PARAM_SHR_SEARCH:
        case RADIO_PARAM_RX_MODE:
        case RADIO_PARAM_TX_MODE:
        default:
            return RADIO_RESULT_NOT_SUPPORTED;
	}
}


static radio_result_t
get_object(radio_param_t param, void *dest, size_t size)
{
	if (dest == NULL) return RADIO_RESULT_ERROR;

	switch (param) {
		case RADIO_PARAM_64BIT_ADDR:
			getLongAddr((uint8_t *)dest, size);
			return RADIO_RESULT_OK;

		case RADIO_PARAM_LAST_PACKET_TIMESTAMP:
		//	// TODO: Some extra chech of size???
		//	*(rtimer_clock_t *)dest = last_packet_timestamp;
		//	return RADIO_RESULT_OK;

		//#if MAC_CONF_WITH_TSCH
		//case RADIO_CONST_TSCH_TIMING:
		//	*(const uint16_t **)dest = TSCH_CONF_DEFAULT_TIMESLOT_TIMING;
		//#endif

		default:
			return RADIO_RESULT_NOT_SUPPORTED;
	}
}


static radio_result_t
set_object(radio_param_t param, const void *src, size_t size)
{
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
	.init = rf2xx_init,
	.prepare = rf2xx_prepare,
	.transmit = rf2xx_transmit,
	.send = rf2xx_send,
	.read = rf2xx_read,
	.channel_clear = rf2xx_channel_clear,
	.receiving_packet = rf2xx_receiving_packet,
	.pending_packet = rf2xx_pending_packet,
	.on = rf2xx_on,
	.off = rf2xx_off,
	.get_value = get_value,
	.set_value = set_value,
	.get_object = get_object,
	.set_object = set_object,
};




uint8_t
regRead(uint8_t addr)
{
    vsnSPI_ErrorStatus status = VSN_SPI_SUCCESS;
    uint8_t value;

	setCS();
	status = vsnSPI_pullByteTXRX(rf2xxSPI, ((addr & CMD_REG_MASK) | CMD_REG | CMD_READ), &value);
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);
	status |= vsnSPI_pullByteTXRX(rf2xxSPI, 0x00, &value);
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);
	clearCS();

    ASSERT(VSN_SPI_SUCCESS == status);

    return value;
}

void
regWrite(uint8_t addr, uint8_t value)
{
    vsnSPI_ErrorStatus status = VSN_SPI_SUCCESS;
    uint8_t dummy __attribute__((unused));

	setCS();
	status = vsnSPI_pullByteTXRX(rf2xxSPI, ((addr & CMD_REG_MASK) | CMD_REG | CMD_WRITE), &dummy);
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);
	status |= vsnSPI_pullByteTXRX(rf2xxSPI, value, &dummy);
    BUSYWAIT_UNTIL(vsnSPI_checkSPIstatus(rf2xxSPI) == VSN_SPI_SUCCESS);
	clearCS();

    ASSERT(VSN_SPI_SUCCESS == status);
}

uint8_t
bitRead(uint8_t addr, uint8_t mask, uint8_t offset)
{
	return (regRead(addr) & mask) >> offset;
}

void
bitWrite(uint8_t addr, uint8_t mask, uint8_t offset, uint8_t value)
{
	uint8_t tmp = regRead(addr) & ~mask;
	tmp |= (value << offset) & mask;
	regWrite(addr, tmp);
}