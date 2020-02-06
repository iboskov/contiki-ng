/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include <stdio.h>
#include "dev/serial-line.h"
#include "arch/platform/vesna/dev/at86rf2xx/rf2xx.h"
#include "arch/platform/vesna/dev/at86rf2xx/rf2xx_stats.h"
#include "net/ipv6/uip.h"

/*---------------------------------------------------------------------------*/
#define SECOND 		  (1000)
#define MAX_APP_TIME  (SECOND * 180) // 3 min

uint32_t counter = 0;
extern uint8_t appIsRunning;

/*---------------------------------------------------------------------------*/
void STATS_print_help(void);
void STATS_input_command(char *data);
void STATS_set_device_as_root(void);
void STATS_close_app(void);

/*---------------------------------------------------------------------------*/
PROCESS(stats_process, "Stats app process");
PROCESS(serial_input_process, "Serial input command");
//PROCESS(ping_process, "Pinging process");

AUTOSTART_PROCESSES(&serial_input_process);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_input_process, ev, data)
{
    PROCESS_BEGIN();
    while(1){
      PROCESS_WAIT_EVENT_UNTIL(
        (ev == serial_line_event_message) && (data != NULL));
      STATS_input_command(data);
    }
    PROCESS_END();
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(stats_process, ev, data)
{
	static struct etimer timer;

	PROCESS_BEGIN();

	printf(">Starting app! \n");
	counter = 0;  
	appIsRunning = 1;

	// Empty buffers if they have some values from before
	RF2XX_STATS_RESET();
	STATS_clear_packet_stats();
	STATS_clear_background_noise();

	printf("AD %d\n", (MAX_APP_TIME/1000));

	// Optional: print help into log file
	STATS_print_help();

	etimer_set(&timer, 1);  //ms = 1, sec = 1000

	while(1) {
		counter++;

	// Measure and display background noise
	#if STATS_BGN_MEASURMENT_EVERY_10MS
		if((counter%10) == 0) {
			STATS_update_background_noise(STATS_BGN_BUFFER_CAPACITY);
		}

		#if STATS_BGN_3_CHANNELS
			if((counter%(10 * SECOND)) == 0) {
				STATS_print_background_noise();
			}
		#elif STATS_BGN_6_CHANNELS	
			if((counter%(5 * SECOND)) == 0) {
				STATS_print_background_noise();
			}
		#elif STATS_BGN_16_CHANNELS
			if((counter%(2 * SECOND)) == 0) {
				STATS_print_background_noise();
			}
		#else
			if((counter%(30 * SECOND)) == 0) {
				STATS_print_background_noise();
			}
		#endif
	#else

		STATS_update_background_noise(STATS_BGN_BUFFER_CAPACITY);

		#if STATS_BGN_3_CHANNELS
			if((counter%(SECOND)) == 0) {
				STATS_print_background_noise();
			}
		#elif STATS_BGN_6_CHANNELS	
			if((counter%(SECOND/2)) == 0) {
				STATS_print_background_noise();
			}
		#elif STATS_BGN_16_CHANNELS
			if((counter%(SECOND/5)) == 0) {
				STATS_print_background_noise();
			}
		#else
			if((counter%(3 * SECOND)) == 0) {
				STATS_print_background_noise();
			}
		#endif
	#endif

		// Every 10 seconds print packet statistics and clear the buffer
		if((counter%(SECOND * 10)) == 0){
			STATS_print_packet_stats();
		}

		// After max time send stop command ('=') and print driver statistics
		if(counter == (MAX_APP_TIME)){
			STATS_close_app();
			PROCESS_EXIT();
		}

		// Wait for the periodic timer to expire and then restart the timer.
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
		etimer_reset(&timer);
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*
// TODODODO
PROCESS_THREAD(ping_process, ev, data)
{
	static uip_ipaddr_t remote_addr;
	static struct etimer timeout_timer;
	char *next_args;

	PROCESS_BEGIN();

	// Dobi argumente - koga pingat

	if(data == NULL) {
		printf("Destination IPv6 address is not specified\n");
		PROCESS_EXIT();
	} else if(uiplib_ipaddrconv(data, &remote_addr) == 0) {
		printf("Invalid IPv6 address: %s\n", data);
		PROCESS_EXIT();
	}

	printf("Pinging ");
	printf( &remote_addr);
	printf("\n");

	// Send ping request 
	etimer_set(&timeout_timer, (5 * CLOCK_SECOND));
	uip_icmp6_send(&remote_addr, ICMP6_ECHO_REQUEST, 0, 4);
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timeout_timer));


	if(curr_ping_output_func != NULL) {
		SHELL_OUTPUT(output, "Timeout\n");
		curr_ping_output_func = NULL;
	} else {
		SHELL_OUTPUT(output, "Received ping reply from ");
		shell_output_6addr(output, &remote_addr);
		SHELL_OUTPUT(output, ", len %u, ttl %u, delay %lu ms\n",
		curr_ping_datalen, curr_ping_ttl, (1000*(clock_time() - timeout_timer.timer.start))/CLOCK_SECOND);
	}

	PROCESS_END();
}
*/


/*---------------------------------------------------------------------------*/
void
STATS_input_command(char *data){
    char cmd = data[0];
    switch(cmd){
      case '>':
        process_start(&stats_process, NULL);
        break;
      
      case '*':
        STATS_set_device_as_root();
        break;
      
      case '=':
        process_exit(&stats_process);
        STATS_close_app();
        break;

	case '?':
		printf("solata: %d \n", sizeof(txPacket_t));
		break;

      case '!':
        //process_start(&ping_process, NULL);
        //STATS_ping_neighbour..
        break;

      //case 'reboot':
        //watchdog_reboot();
        //break;
    }
}

/*---------------------------------------------------------------------------*/
void
STATS_set_device_as_root(void){
	static uip_ipaddr_t prefix;
	const uip_ipaddr_t *default_prefix = uip_ds6_default_prefix();
	uip_ip6addr_copy(&prefix, default_prefix);

  	if(!NETSTACK_ROUTING.node_is_root()) {
     	NETSTACK_ROUTING.root_set_prefix(&prefix, NULL);
     	NETSTACK_ROUTING.root_start();
	} else {
      	printf("Node is already a DAG root\n");
    }
}

/*---------------------------------------------------------------------------*/
void
STATS_close_app(void){
	appIsRunning = 0;

	STATS_print_driver_stats();
	// Send '=' cmd to stop the monitor
	printf("=End monitoring serial port\n");

	// Empty buffers
	RF2XX_STATS_RESET();
	STATS_clear_background_noise();
	STATS_clear_packet_stats();

	// Reset the network
	if(NETSTACK_ROUTING.node_is_root()){
		NETSTACK_ROUTING.leave_network();
	}
}

/*---------------------------------------------------------------------------*/
void
STATS_print_help(void){
	uint8_t addr[8];

	rf2xx_driver.get_object(RADIO_PARAM_64BIT_ADDR, &addr, 8);
	printf("Device ID: ");
	for(int j=0; j<8; j++){
		printf("%X",addr[j]);
	}

	printf("\n"); 
	printf("----------------------------------------------------------------------------\n");
	printf("\n");
	printf("       DESCRIPTION\n");
	printf("----------------------------------------------------------------------------\n");
	printf("CH :(buff-count)[time-stamp]: [delay]RSSI [delay]RSSI [delay]RSSI ...\n");
	printf("----------------------------------------------------------------------------\n");
	printf("Tx [time-stamp] packet-type  dest-addr (chn len sqn | pow) BC/UC \n");
	printf("----------------------------------------------------------------------------\n");
	printf("Rx [time-stamp] packet-type  sour-addr (chn len sqn | rssi lqi) \n");
	printf("----------------------------------------------------------------------------\n");
	printf("\n");
	printf("On the end of file, there is a count of all received and transmited packets. \n");
	printf("----------------------------------------------------------------------------\n");
}


/*
void
STATS_print_help(void){
	uint8_t addr[8];

	rf2xx_driver.get_object(RADIO_PARAM_64BIT_ADDR, &addr, 8);
	printf("Device ID: ");
	for(int j=0; j<8; j++){
		printf("%X",addr[j]);
	}

	printf("\n"); 
	printf("----------------------------------------------------------------------------\n");
	printf("\n");
	printf("       DESCRIPTION\n");
	printf("----------------------------------------------------------------------------\n");
	printf("CH[cc][xxx]([ss:us]): (tt)[v] (tt)[v] (tt)[v] ...\n");

	printf("cc   - Channel number, where RSSI is measured \n");
	printf("xxx  - Count of measured values \n");
	printf("ss   - Timestamp in seconds \n");
	printf("us   - Timestamp in micro seconds \n");
	printf("tt   - Deviation from first measurment in micro seconds\n");
	printf("v    - Measured RSSI values at theirs timestamp \n");
	printf("----------------------------------------------------------------------------\n");
	printf("T[n] [u] [t] [0xaaaa] \n");
	printf("[s]:[us]\n");
	printf("C[cc] L[ll] S[ss] | P[pp] \n");

	printf("n    - Transmited packt count \n");
	printf("u    - (1/0 = unicast/broadcast) \n");
	printf("t    - Type of packet (D/B/A = data/beacon/ACK) \n");
	printf("aaaa - Destination address \n");
	printf("s    - Timestamp in seconds \n");
	printf("us   - Timestamp in micro seconds \n");
	printf("cc   - Channel number \n");
	printf("ll   - Packet length in bytes \n");
	printf("ss   - Sequence number \n");
	printf("pp   - Transmision power (0 = 3dBm)\n");
	printf("----------------------------------------------------------------------------\n");
	printf("R[n] [t] [0xaaaa] \n");
	printf("[s]:[us]\n");
	printf("C[cc] L[ll] S[ss] | R[rr] Q[qq] \n");

	printf("n    - Received packt count \n");
	printf("t    - Type of packet (D/B/A = data/beacon/ACK) \n");
	printf("aaaa - Source address \n");
	printf("s    - Timestamp in seconds \n");
	printf("us   - Timestamp in micro seconds \n");
	printf("cc   - Channel number \n");
	printf("ll   - Packet length in bytes \n");
	printf("ss   - Sequence number \n");
	printf("rr   - RSSI when packet was received\n");
	printf("qq   - LQI when packet was received \n");
	printf("----------------------------------------------------------------------------\n");
	printf("\n");
	printf("On the end of file, there is a count of all received and transmited packets. \n");
	printf("----------------------------------------------------------------------------\n");
}
*/