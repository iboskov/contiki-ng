#ifndef RF2XX_STATS_H_
#define RF2XX_STATS_H_

#include "contiki.h"
#include "sys/log.h"
#include "rf2xx.h"
#include "rf2xx_hal.h"  // for SR_CHANNEL

#if RF2XX_STATS
	/* =========================
	*    PACKETS STATISTICS
	* ========================= 
	*  Whenever packed is sent or received its statistics are stored in a circular buffer.
	*  Each buffer (for RX packets and TX packets) must be first initialized (ex. in rf2xx_reset())
	*  Later the statistics are displayed in app with functions display (human readable) and print.
	*/

	typedef struct{
		void *buffer_start;
		void *buffer_end;
		int  *head;
		int  *tail;
		uint16_t capacity;	// Max is 65536
		uint8_t size;
		uint16_t count;
	}buffer_t; 

	typedef struct{
		uint32_t timestamp_s;
		uint32_t timestamp_us;
		uint8_t  type;
		uint8_t  channel;
		uint8_t  sqn;
		uint8_t  len;
		uint16_t source_addr;
		int8_t   rssi;
		uint8_t  lqi;
		uint16_t count;
	}rxPacket_t;

	typedef struct{
		uint32_t timestamp_s;
		uint32_t timestamp_us;
		uint8_t  type;
		uint8_t  channel;
		uint8_t  sqn;
		uint8_t  len;
		uint16_t dest_addr;
		uint8_t  unicast;
		uint8_t  power;
		uint16_t count;
	} txPacket_t;

	void    STATS_init_packet_buffer(uint8_t capacity);
	//void  STATS_free_packet_buffer(void);
	uint8_t STATS_put_rx_packet(rxFrame_t *frame);
	uint8_t STATS_put_tx_packet(txFrame_t *frame);
	uint8_t STATS_get_rx_packet(rxPacket_t *packet);
	uint8_t STATS_get_tx_packet(txPacket_t *packet);

	void    STATS_display_packet_stats(void);
	void    STATS_print_packet_stats(void);

	/* =========================
	*    BACKGROUND NOISE
	* ========================= 
	* On each channel store its RSSI in a buffer and later display it in app. Each channel has its buffer for 
	* storing values. They are initialized when need - if we get on new channel, that has not beenyet init, 
	* it gets alocated then. List of allready init channels is stored in channel_stats_index - 16bit value,
	* each bit representing its channel. 
	*/

	typedef struct{
		uint32_t timestamp_s;
		uint32_t timestamp_us;
		int rssi;
	} bgNoise_t;

	uint8_t STATS_init_channel_buffer(buffer_t *b, uint8_t ch, uint16_t capacity);
	void    STATS_free_channel_buffer(buffer_t *b, uint8_t ch);
	uint8_t STATS_get_channel_rssi(buffer_t *b, uint8_t ch, bgNoise_t *bgn);
	uint8_t STATS_put_channel_rssi(buffer_t *b, uint8_t ch, bgNoise_t *bgn);

	void STATS_update_background_noise(uint16_t capacity);
	void STATS_print_background_noise(void);

	uint8_t STATS_is_channel_init(uint8_t channel);

#endif

/* =========================
 *     DRIVER STATISTICS
 * ========================= 
 * Statistics of driver are stored in rf2xxStats[]  Every time a specific event occurs,
 * its counter adds 1. 
*/
enum {
	rxDetected,		// Detected packets
	rxSuccess,		// Successfully received packets
	rxToStack,		// Not used in TSCH
	rxAddrMatch,	// Not used in TSCH

	rxData,			// Received Data packet
	rxBeacon,		// Received Beacon packet
	rxAck,			// Received Acknowledge

	txCollision,	// Not used in TSCH
	txNoAck,		// Not used in TSCH
	txSuccess,		// Not used in TSCH (it's same as txCount)
	txCount,		// Num of sent packets
	txError,		// Num of errors
	txTry,			// Num of TX tries

	txAck,			// Transmited Ack
	txBeacon,		// Transmited Beacon
	txData,			// Transmited Data
	txReqAck,		// Request ACK (unicast packet)

	RF2XX_STATS_COUNT
};

	void STATS_display_driver_stats_inline(void);
	void STATS_display_driver_stats(void);
	void STATS_print_driver_stats(void);

#if RF2XX_STATS
	extern volatile uint32_t rf2xxStats[RF2XX_STATS_COUNT];
	#define RF2XX_STATS_GET(event)		rf2xxStats[event]
	#define RF2XX_STATS_ADD(event)		rf2xxStats[event]++
	#define RF2XX_STATS_RESET()    		memset(rf2xxStats, 0, sizeof(rf2xxStats[0]) * RF2XX_STATS_COUNT)
#else
	#define RF2XX_STATS_GET(event)		(0)
	#define RF2XX_STATS_ADD(event)
	#define RF2XX_STATS_RESET()
#endif



#endif