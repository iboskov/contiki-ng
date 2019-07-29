#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"
#include "dev/slip.h"
#include "dev/uart1.h"

#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/ipv6/uip-nd6.h"
#include "net/routing/routing.h"

#include "sys/node-id.h"


#include "vsn.h"
#include "vsnledind.h"
#include "net/netstack.h"

#include "at86rf2xx/rf2xx.h"
#include "at86rf2xx/rf2xx_hal.h"


#define DEBUG DEBUG_PRINT
#include "net/ipv6/uip-debug.h"


static uip_ipaddr_t prefix;
static uint8_t prefix_set;


PROCESS(display_routing_tables, "Show IPv6 routes");
PROCESS(border_router_process, "Border router process");

AUTOSTART_PROCESSES(&display_routing_tables, &border_router_process);


void
set_prefix_64(uip_ipaddr_t *prefix_64)
{
	
  prefix_set = 1;
  NETSTACK_ROUTING.root_set_prefix(prefix_64, NULL);
  NETSTACK_ROUTING.root_start();
}


static void
print_local_addresses(void)
{
	uint8_t state;

	printf("Local addresses:\n");
    for (uint8_t i = 0; i < UIP_DS6_ADDR_NB; i++) {
        state = uip_ds6_if.addr_list[i].state;
        if (uip_ds6_if.addr_list[i].isused && (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
            printf("  [");
            uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
			printf("]");
            if (state == ADDR_TENTATIVE) printf(" [tentative]");
            if (state == ADDR_PREFERRED) printf(" [preferred]");
            printf("\n");
            if (state == ADDR_TENTATIVE) {
                uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
            }
        }
    }
}

static void
print_neighbor_table(void)
{
	printf("Neighbors (%u):\n", uip_ds6_nbr_num());
	for (uip_ds6_nbr_t *r = uip_ds6_nbr_head(); r != NULL; r = uip_ds6_nbr_next(r))
	{
		printf("  [");
		uip_debug_ipaddr_print(&r->ipaddr);
		printf("]");
		printf(", router: %s", r->isrouter ? "Y" : "N");
		printf(", state: ");
		switch (r->state) {
			case NBR_INCOMPLETE:
				printf("INCOMPLETE");
				break;

			case NBR_REACHABLE:
				printf("REACHABLE");
				break;

			case NBR_STALE:
				printf("STALE");
				break;

			case NBR_DELAY:
				printf("DELAY");
				break;

			case NBR_PROBE:
				printf("PROBE");
				break;

			default:
				printf("???");
		}

		printf("\n");
	}
}

static void
print_routing_table(void)
{
	printf("Routing table (%u):\n", uip_ds6_route_num_routes());
	for (uip_ds6_route_t *r =  uip_ds6_route_head(); r != NULL; r = uip_ds6_route_next(r))
	{
		printf("  ");
		uip_debug_ipaddr_print(&r->ipaddr);
		printf("/%u (via ", r->length);
		uip_debug_ipaddr_print(uip_ds6_route_nexthop(r));
		printf(") %lus\n", (unsigned long)r->state.lifetime);
	}
}


PROCESS_THREAD(display_routing_tables, ev, data)
{
	static struct etimer et;

	PROCESS_BEGIN();
	PROCESS_PAUSE();

	etimer_set(&et, CLOCK_SECOND*4);

	while (1) {
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		print_local_addresses();
		print_neighbor_table();
		print_routing_table();


		printf(
			"Radio stats: RxDetect: %lu; RxAddrMatch: %lu; rxSuccess: %lu; rxToStack: %lu;\n",
			RF2XX_STATS_GET(rxDetected),
			RF2XX_STATS_GET(rxAddrMatch),
			RF2XX_STATS_GET(rxSuccess),
			RF2XX_STATS_GET(rxToStack)
		);

		printf(
			"Radio stats: txCount: %lu; txSuccess: %lu; txCollision: %lu; txNoAck: %lu;\n",
			RF2XX_STATS_GET(txCount),
			RF2XX_STATS_GET(txSuccess),
			RF2XX_STATS_GET(txCollision),
			RF2XX_STATS_GET(txNoAck)
		);

		printf("Radio SPI errors: %lu;\n", RF2XX_STATS_GET(spiError));

		etimer_reset(&et);
	}

	PROCESS_END();
}



void request_prefix(void);

/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(border_router_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();

	/* While waiting for the prefix to be sent through the SLIP connection, the future
	* border router can join an existing DAG as a parent or child, or acquire a default
	* router that will later take precedence over the SLIP fallback interface.
	* Prevent that by turning the radio off until we are initialized as a DAG root.
	*/


  prefix_set = 0;
  NETSTACK_MAC.off();

  PROCESS_PAUSE();

  printf("RPL-Border router started\n");

  /*while (!prefix_set)
  {
    etimer_set(&et, CLOCK_SECOND);
    request_prefix();
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    printf("Waiting for prefix\n");
  }*/

  uip_ip6addr(&prefix, 0xAAAA,0,0,0,0,0,0,0);
	NETSTACK_ROUTING.root_set_prefix(&prefix, NULL);
  	NETSTACK_ROUTING.root_start();

  NETSTACK_MAC.on();

  print_local_addresses();

  etimer_set(&et, CLOCK_SECOND * 120);

	while (1) {
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		etimer_reset(&et);

		printf("Initiating global repair\n");
		NETSTACK_ROUTING.global_repair("Timeout");
	}

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
