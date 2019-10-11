

// rf2xx statistics
#define RF2XX_CONF_STATS 1
#define STATS_CONF_PACKET_BUFF_CAPACITY   (20)

#define LOG_CONF_LEVEL_IPV6                        LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_RPL                         LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_6LOWPAN                     LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_TCPIP                       LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_MAC                         LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_FRAMER                      LOG_LEVEL_NONE
#define TSCH_LOG_CONF_PER_SLOT                     0
#define LOG_CONF_LEVEL_RF2XX                       LOG_LEVEL_NONE



// Contiki statistics
//#define TSCH_STATS_CONF_ON 1
// Enable periodic RSSI sampling for TSCH statistics 
//#define TSCH_STATS_CONF_SAMPLE_NOISE_RSSI 1
//Reduce the TSCH stat "decay to normal" period to get printouts more often 
//#define TSCH_STATS_CONF_DECAY_INTERVAL (60 * CLOCK_SECOND)

// Reduce the EB period in order to update the network nodes with more agility 
//#define TSCH_CONF_EB_PERIOD     (4 * CLOCK_SECOND)
//#define TSCH_CONF_MAX_EB_PERIOD (4 * CLOCK_SECOND)


// Optional:
//#define WITH_ORCHESTRA 1

// Channels on which nodes are communicating
//#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 26 }

// In case of a big drift, nodes have to sync more often (KA interval)
//#define TSCH_CONF_KEEPALIVE_TIMEOUT (5 * CLOCK_SECOND)
//#define TSCH_CONF_MAX_KEEPALIVE_TIMEOUT (20 * CLOCK_SECOND)


