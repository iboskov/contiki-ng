#ifndef RF2XX_ARCH_H_
#define RF2XX_ARCH_H_

#define RF2XX_MAX_FRAME_SIZE	(127)
#define RF2XX_MIN_FRAME_SIZE	(3)
#define RF2XX_CRC_SIZE			(2)
#define RF2XX_LQI_SIZE			(1)
#define RF2XX_MAX_PAYLOAD_SIZE	(RF2XX_MAX_FRAME_SIZE - RF2XX_CRC_SIZE)


// the delay between radio Tx request and SFD sent, in rtimer ticks
// This includes (FORCE_TRX_OFF, TX_ARET/PLL_ON, delay, PREAMBLE+SFD (p.39))
#define RF2XX_DELAY_BEFORE_TX		((unsigned)US_TO_RTIMERTICKS(200))
// * FORCE_TRX_OFF        - 1us
// * TRX_OFF -> PLL_ON    - 110us
// * PLL_ON -> BUSY_RX    - 16us
// * PREAMBLE + SFD       - 160us (96us)
//                        = 223us

// the delay between radio Rx request and start listening, in rtimer ticks
// (we need time from end of Tx until it reaches Rx mode again)
#define RF2XX_DELAY_BEFORE_RX		((unsigned)US_TO_RTIMERTICKS(120))
// * PLL_ON -> RX_ON      - 1us
// * Time until PLL_LOCK                  // Even if the registerbits TRX_STATUS indicates RX_ON
//   should occur         - 32us          // actual frame reception canonly start once the PLL has locked.
// * PLL settling time                    
//   on chanel switch     - (11us)          
// * TRX_OFF -> RX_ON     - 110us         // when rf2xx_on is called
//                        = 114us

// The number of header and footer bytes of overhead at the PHY layer after SFD
#define RF2XX_PHY_OVERHEAD			(3) 
// * 1 length byte + 2 bytes CRC

// the delay between the end of SFD reception and the radio returning 1 to receiving_packet()
#define RF2XX_DELAY_BEFORE_DETECT	((unsigned)US_TO_RTIMERTICKS(40))
// * RX_START flag
//   interrupt latency    - 9us
// * PHR reception        - 32us


#define RF2XX_BYTE_AIR_TIME			(32) // us for 1 byte (@250kbps)
// 1 / (250 kbps / 8) == 32 us/byte


// The drift compared to "true" 10ms slots. --> Enable adaptive sync to enable compensation for this (enabled by default).
#define RF2XX_BASE_DRIFT_PPM        (RTIMER_ARCH_PPM)
// Slot length 10000 usec, 
// which gives us 655 ticks (from macro US_TO_RTIMERTICKS)
// Tick duration 15.2587890625 usec
//             
// Real slot duration is then : 9994.506835937 usec
// Target - Real duration = 5.49316 usec
// TSCH_CONF_BASE_DRIFT_PPM 549
 

// TSCH timeslot timing (default is: 10ms tsch_timeslot_timing_us_10000)
#define RF2XX_CONF_DEFAULT_TIMESLOT_TIMING	(tsch_timeslot_timing_us_20000)

// TODO: migrate "tsch_timeslot_timing_us_20000" into this file
#endif