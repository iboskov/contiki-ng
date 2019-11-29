/* Project configuration */

// All logs to LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_IPV6                        LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_RPL                         LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_6LOWPAN                     LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_TCPIP                       LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_MAC                         LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_FRAMER                      LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_RF2XX                       LOG_LEVEL_NONE
#define TSCH_LOG_CONF_PER_SLOT                     (0)

// Defines for app
#define RF2XX_CONF_STATS                          (1)

#define STATS_CONF_PACKET_BUFF_CAPACITY           (20)  

#define STATS_BGN_MEASURMENT_EVERY_5MS            (1)
#define STATS_BGN_3_CHANNELS                      (1)
#define STATS_BGN_6_CHANNELS                      (0)
#define STATS_BGN_16_CHANNELS                     (0)

/* Configuration table for measuring and displaying back ground noise:
   -------------------------|-------|-------|
   Measure every:           |  1ms  |  5ms  |
   -------------------------|-------|-------|
                      1-CH  | (3s)  |  15s  |
                      3-CH  |  1s   |   5s  |
   Display every:     6-CH  | 0.5s  |  2.5s |
                     16-CH  | 0.2s  |   1s  |
   -------------------------|-------|-------|

If no defines are set, app will go with default: measure BGN on one channel every 1 ms 
*/
#if STATS_BGN_3_CHANNELS
    #define STATS_BGN_BUFFER_CAPACITY 1001
    #define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 15, 20, 26 }
#elif STATS_BGN_6_CHANNELS
    #define STATS_BGN_BUFFER_CAPACITY 501
    #define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 16, 18, 20, 22, 24, 26 }
#elif STATS_BGN_16_CHANNELS
    #define STATS_BGN_BUFFER_CAPACITY 201
    #define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26 }
#else
    #define STATS_BGN_BUFFER_CAPACITY 3001
    #define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 26 }
#endif
// for more channels use smaller buffer size ... 1 measurment of bgn = 12B
// buffer of 1001 = 12012 B --> 3  channels would then use 36 036 B
// buffer of  501 = 6012 B  --> 6  channels would then use 36 072 B
// buffer of  201 = 2412 B  --> 16 channels would then use 38 592 B

// Library heapmem.h --> https://github.com/contiki-ng/contiki-ng/wiki/Documentation:-Memory-management
#define HEAPMEM_CONF_ARENA_SIZE             (39000)     // Currently avaliable 44708 B
