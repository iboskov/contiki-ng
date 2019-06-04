#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"

#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/ipv6/uip-nd6.h"
#include "net/routing/routing.h"

#include "sys/node-id.h"

#include "vsn.h"
#include "vsnledind.h"
#include "net/netstack.h"

#define DEBUG DEBUG_PRINT
#include "net/ipv6/uip-debug.h"

PROCESS(display_routing_tables, "Show IPv6 routes");
AUTOSTART_PROCESSES(&display_routing_tables);

static void print_local_addresses(void)
{
	uint8_t state;

	printf("Local addresses:\n");
	for (uint8_t i = 0; i < UIP_DS6_ADDR_NB; i++)
	{
		state = uip_ds6_if.addr_list[i].state;
		if (uip_ds6_if.addr_list[i].isused && (state == ADDR_TENTATIVE || state == ADDR_PREFERRED))
		{
			printf("  [");
			uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
			printf("]");
			if (state == ADDR_TENTATIVE)
				printf(" [tentative]");
			if (state == ADDR_PREFERRED)
				printf(" [preferred]");
			printf("\n");
			if (state == ADDR_TENTATIVE)
			{
				uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
			}
		}
	}
}

static void
print_neighbor_table(void)
{
	printf("Neighbors:\n");
	for (uip_ds6_nbr_t *r = uip_ds6_nbr_head(); r != NULL; r = uip_ds6_nbr_next(r))
	{
		printf("  [");
		uip_debug_ipaddr_print(&r->ipaddr);
		printf("]\n");
	}
}

static void
print_routing_table(void)
{
	printf("Routing table:\n");
	for (uip_ds6_route_t *r = uip_ds6_route_head(); r != NULL; r = uip_ds6_route_next(r))
	{
		printf("  ");
		uip_debug_ipaddr_print(&r->ipaddr);
		printf("/%u (via ", r->length);
		uip_debug_ipaddr_print(uip_ds6_route_nexthop(r));
		printf(")\n");
	}
}

PROCESS_THREAD(display_routing_tables, ev, data)
{
	static struct etimer et;

	PROCESS_BEGIN();
	PROCESS_PAUSE();

	etimer_set(&et, CLOCK_SECOND * 8);

	while (1)
	{
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		print_local_addresses();
		print_neighbor_table();
		print_routing_table();

		etimer_reset(&et);
	}

	PROCESS_END();
}
