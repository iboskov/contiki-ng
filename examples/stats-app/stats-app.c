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

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "arch/platform/vesna/dev/at86rf2xx/rf2xx.h"
#include "arch/platform/vesna/dev/at86rf2xx/rf2xx_stats.h"

#define SECOND 1000
#define STATS_BG_NOISE_BUFF_CAPACITY 1100
uint32_t i = 0;

void STATS_print_help(void);

/*---------------------------------------------------------------------------*/
PROCESS(stats_process, "Stats app process");
AUTOSTART_PROCESSES(&stats_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(stats_process, ev, data)
{
  static struct etimer timer;

  PROCESS_BEGIN();

  etimer_set(&timer, 1);  //ms = 1, sec = 1000

  printf("***** Start monitoring serial port *****\n");

  STATS_print_help(); 

  while(1) {
    i++;

  #if RF2XX_CONF_STATS
    // Every 1ms easure rssi and store it into buffer
    STATS_update_background_noise(STATS_BG_NOISE_BUFF_CAPACITY);

    // Every 1s
    if(i%SECOND == 0) {
      STATS_print_background_noise();
    }

    // Every 10s
    if((i%(SECOND * 10)) == 0){
      STATS_print_packet_stats();
    }

    // After 120 seconds 
    if((i%(SECOND * 600)) == 0){
      STATS_print_driver_stats();
      printf("===== Stop monitoring serial port =====\n");
    }
  #endif
    /* Wait for the periodic timer to expire and then restart the timer. */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    etimer_reset(&timer);
  }

  PROCESS_END();
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
  printf("-------------------------------------------------------------------------\n");
  printf("\n");
  printf("       DESCRIPTION of this mumbojumbo\n");
  printf("-------------------------------------------------------------------------\n");
  printf("CH[cc]([xxx]): [v] [v] [v]\n");

  printf("cc   - Channel number, where RSSI is measured \n");
  printf("xxx  - Count of measured values \n");
  printf("v    - Actual RSSI values \n");
  printf("-------------------------------------------------------------------------\n");
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
  printf("-------------------------------------------------------------------------\n");
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
  printf("-------------------------------------------------------------------------\n");
  printf("\n");
  printf("On the end of file, there is a count of all received and transmited packets. \n");
  printf("-------------------------------------------------------------------------\n");
}


