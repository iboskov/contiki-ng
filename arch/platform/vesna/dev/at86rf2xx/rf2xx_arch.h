#ifndef RF2XX_ARCH_H_
#define RF2XX_ARCH_H_


//#include "rtimer-arch.h"


#define RF2XX_MAX_FRAME_SIZE	(127)
#define RF2XX_MIN_FRAME_SIZE	(3)
#define RF2XX_CRC_SIZE			(2)
#define RF2XX_LQI_SIZE			(1)
#define RF2XX_MAX_PAYLOAD_SIZE	(RF2XX_MAX_FRAME_SIZE - RF2XX_CRC_SIZE)


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


#endif