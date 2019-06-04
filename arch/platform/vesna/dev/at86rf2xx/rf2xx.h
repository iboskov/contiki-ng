#ifndef RF2XX_H_
#define RF2XX_H_

#include "contiki-net.h"


#ifndef RF2XX_DEBUG
#define RF2XX_DEBUG				(3)
#endif

#ifndef RF2XX_STATS
#define RF2XX_STATS				(1)
#endif

#ifndef RF2XX_CSMA_RETRIES
#define RF2XX_CSMA_RETRIES		(4)
#endif

#ifndef RF2XX_FRAME_RETRIES
#define RF2XX_FRAME_RETRIES		(15)
#endif

#ifndef RF2XX_AUTOACK
#define RF2XX_AUTOACK			(1)
#endif

#ifndef RF2XX_CHECKSUM
#define RF2XX_CHECKSUM			(1)
#endif

#ifndef RF2XX_PROMISCUOUS
#define RF2XX_PROMISCUOUS		(0)
#endif

#ifndef RF2XX_I_AM_COORD
#define RF2XX_I_AM_COORD		(0)
#endif


extern const struct radio_driver rf2xx_driver;

void rf2xx_isr(void);

int rf2xx_init(void);
int rf2xx_prepare(const void *payload, unsigned short payload_len);
int rf2xx_transmit(unsigned short transmit_len);
int rf2xx_send(const void *payload, unsigned short payload_len);
int rf2xx_read(void *buf, unsigned short buf_len);
int rf2xx_cca(void);
int rf2xx_receiving_packet(void);
int rf2xx_pending_packet(void);
int rf2xx_on(void);
int rf2xx_off(void);

void rf2xx_reset(void);

// Raw values
extern volatile uint8_t rf2xx_last_rssi;
extern volatile uint8_t rf2xx_last_lqi;

uint8_t rf2xx_getTxPower(void);
void rf2xx_setTxPower(uint8_t pwr);

uint8_t rf2xx_getChannel(void);
void rf2xx_setChannel(uint8_t ch);

uint16_t rf2xx_getPanId(void);
void rf2xx_setPanId(uint16_t pan);

uint16_t rf2xx_getShortAddr(void);
void rf2xx_setShortAddr(uint16_t addr);

void rf2xx_getLongAddr(uint8_t *addr, uint8_t len);
void rf2xx_setLongAddr(const uint8_t *addr, uint8_t len);

void rf2xx_calibration(void);


void rf2xx_test(void);


int rf2xx_sleep(void);
int rf2xx_wakeUp(void);


// Quick access functions
uint8_t rf2xx_RSSI(void);
uint8_t rf2xx_ED_LEVEL(void);

#endif
