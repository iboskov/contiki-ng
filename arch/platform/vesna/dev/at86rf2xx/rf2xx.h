#ifndef RF2XX_H_
#define RF2XX_H_

#include "contiki-net.h"


// Default log level
#ifndef LOG_CONF_LEVEL_RF2XX
#define LOG_CONF_LEVEL_RF2XX            LOG_LEVEL_DBG
#endif


#ifdef RF2XX_CONF_STATS
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

/* Enable/disable radio's auto-acknowledge capability */
#ifndef RF2XX_CONF_AUTOACK
#define RF2XX_CONF_AUTOACK			    (1)
#endif /* RF2XX_CONF_AUTOACK */

// Shall CRC16-CITT checksum be offload to the radio?
#ifndef RF2XX_CONF_CHECKSUM
#define RF2XX_CONF_CHECKSUM		        (1)
#endif /* RF2XX_CONF_CHECKSUM */



// AT86RF2xx driver for Contiki(-NG)
extern const struct radio_driver rf2xx_driver;

// Interrupt routine function
void rf2xx_isr(void);




#endif