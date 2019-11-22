
// rf2xx statistics
#define RF2XX_CONF_STATS                  (1)
#define STATS_CONF_BGN_EVERY_x_MS         (2)  // possible 1, 2, 10
#define STATS_CONF_PACKET_BUFF_CAPACITY   (20)

#define LOG_CONF_LEVEL_IPV6                        LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_RPL                         LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_6LOWPAN                     LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_TCPIP                       LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_MAC                         LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_FRAMER                      LOG_LEVEL_NONE
#define TSCH_LOG_CONF_PER_SLOT                     0
#define LOG_CONF_LEVEL_RF2XX                       LOG_LEVEL_NONE

// Channels on which nodes are communicating
#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 26 }
//#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 15, 17, 19, 21, 23, 25 }