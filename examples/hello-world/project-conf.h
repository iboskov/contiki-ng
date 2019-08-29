//#define LOG_CONF_LEVEL_IPV6                        LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_RPL                         LOG_LEVEL_INFO
//#define LOG_CONF_LEVEL_6LOWPAN                     LOG_LEVEL_DBG
//#define LOG_CONF_LEVEL_TCPIP                       LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_MAC                         LOG_LEVEL_DBG
//#define LOG_CONF_LEVEL_FRAMER                      LOG_LEVEL_DBG

// Optional:
//#define WITH_ORCHESTRA 1

// Channels on which nodes are communicating
#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 26 }

// Is radio on between TX and RX
//#define TSCH_CONF_RADIO_ON_DURING_TIMESLOT 1

// In case of a big drift, nodes have to sync more often (KA interval)
#define TSCH_CONF_KEEPALIVE_TIMEOUT (5 * CLOCK_SECOND)
#define TSCH_CONF_MAX_KEEPALIVE_TIMEOUT (20 * CLOCK_SECOND)



