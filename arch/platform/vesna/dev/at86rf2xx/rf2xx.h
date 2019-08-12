#ifndef RF2XX_H_
#define RF2XX_H_

#include "contiki-net.h"


// Default log level
#ifndef LOG_CONF_LEVEL_RF2XX
#define LOG_CONF_LEVEL_RF2XX            LOG_LEVEL_INFO
#endif


#ifndef RF2XX_CONF_STATS
#define RF2XX_CONF_STATS            (0)
#endif

/* Number of CSMA retries 0-5, 6 = reserved, 7 = immediately without CSMA/CA */
#ifndef RF2XX_CONF_MAX_CSMA_RETRIES
#define RF2XX_CONF_MAX_CSMA_RETRIES	    (5)
#endif /* RF2XX_CONF_CSMA_RETRIES */

/* Number of frame retries, if no ACK, 0-15 */
#ifndef RF2XX_CONF_MAX_FRAME_RETRIES
#define RF2XX_CONF_MAX_FRAME_RETRIES    (15)
#endif /* RF2XX_CONF_FRAME_RETRIES */

// Shall CRC16-CITT checksum be offload to the radio?
#ifndef RF2XX_CONF_CHECKSUM
#define RF2XX_CONF_CHECKSUM		        (1)
#endif /* RF2XX_CONF_CHECKSUM */



// AT86RF2xx driver for Contiki(-NG)
extern const struct radio_driver rf2xx_driver;

// Radio driver API
int rf2xx_init(void);

int rf2xx_reset(void);

int rf2xx_prepare(const void *payload, unsigned short payload_len);
int rf2xx_transmit(unsigned short payload_len);
int rf2xx_send(const void *payload, unsigned short payload_len);
int rf2xx_read(void *buf, unsigned short buf_len);
int rf2xx_cca(void);
int rf2xx_receiving_packet(void);
int rf2xx_pending_packet(void);
int rf2xx_on(void);
int rf2xx_off(void);

// Interrupt routine function
void rf2xx_isr(void);




enum {
	rxDetected,
	rxAddrMatch,
	rxSuccess,
	rxToStack,

	txCount,
	txSuccess,
	txCollision,
	txNoAck,

	RF2XX_STATS_COUNT
};


#if RF2XX_CONF_STATS
extern volatile uint32_t rf2xxStats[RF2XX_STATS_COUNT];
#define RF2XX_STATS_GET(event)		rf2xxStats[event]
#define RF2XX_STATS_COUNT(event)	rf2xxStats[event]++
#define RF2XX_STATS_RESET()    		memset(rf2xxStats, 0, sizeof(rf2xxStats[0]) * RF2XX_STATS_COUNT)
#endif

#define RF2XX_STATS_GET(event)		(0)
#define RF2XX_STATS_COUNT(event)
#define RF2XX_STATS_RESET()

#endif