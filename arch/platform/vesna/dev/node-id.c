#include <stdio.h>

#include "sys/node-id.h"
#include "lib/crc16.h"

#include "contiki-conf.h"
#include "vesna-def.h"

// required by Contiki
uint16_t node_id = 0;

void
node_id_init(void)
{
	#if NODEID
		node_id = NODEID; // can be manually configured
	#else
		/*
		node_id = 0;

		node_id = crc16_add(STM32F1_UUID[0] & 0xFFFF, node_id);
		node_id = crc16_add(STM32F1_UUID[0] >> 16, node_id);

		node_id = crc16_add(STM32F1_UUID[1] & 0xFFFF, node_id);
		node_id = crc16_add(STM32F1_UUID[1] >> 16, node_id);

		node_id = crc16_add(STM32F1_UUID[2] & 0xFFFF, node_id);
		node_id = crc16_add(STM32F1_UUID[2] >> 16, node_id);
		*/
		node_id = crc16_data(STM32F1_UUID.u8, sizeof(STM32F1_UUID.u8), 0);
	#endif
}

