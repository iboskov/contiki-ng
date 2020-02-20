#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "rf2xx_stats.h"
#include "rf2xx.h"
#include "heapmem.h"
#include "sys/log.h"

#define LOG_MODULE  "STATS"
#define LOG_LEVEL   LOG_LEVEL_INFO


#if RF2XX_STATS


/**********************************************************************************************************
 * PACKETS STATISTICS
 **********************************************************************************************************/
static buffer_t rxBuffer;
static buffer_t txBuffer;

static uint16_t txPacketCount = 1;
static uint16_t rxPacketCount = 1;

uint8_t rx_buffer[(20 * STATS_CONF_PACKET_BUFF_CAPACITY)];  //20 is the length of rxPacket_t struct
uint8_t tx_buffer[(20 * STATS_CONF_PACKET_BUFF_CAPACITY)];

/** 
 * @brief Prepare 2 buffers for storing TX and RX packets statistics.
 */
void
STATS_init_packet_buffer(){
    LOG_DBG("Init buffer!\n");

    // Size of packet struct - bigger than sum of elements (compiler aligments)
    rxBuffer.size = sizeof(rxPacket_t);

    // Reset buffer
    memset(rx_buffer, 0, (20 * STATS_CONF_PACKET_BUFF_CAPACITY));

    // Save a pointer of the begining of memory
    rxBuffer.buffer_start = rx_buffer;

    // Calculate pointer to the end of memory
    rxBuffer.buffer_end = (char *)rxBuffer.buffer_start + STATS_CONF_PACKET_BUFF_CAPACITY * rxBuffer.size;

    // Reset buffer variables
    rxBuffer.head = rxBuffer.buffer_start;
    rxBuffer.tail = rxBuffer.buffer_start;
    rxBuffer.capacity = STATS_CONF_PACKET_BUFF_CAPACITY;
    rxBuffer.count = 0;

    // Repeat proces for TX buffer
    txBuffer.size = sizeof(txPacket_t);
    memset(tx_buffer, 0, (20 * STATS_CONF_PACKET_BUFF_CAPACITY));
    txBuffer.buffer_start = tx_buffer;

    txBuffer.buffer_end = (char *)txBuffer.buffer_start + STATS_CONF_PACKET_BUFF_CAPACITY * txBuffer.size;
    txBuffer.capacity = STATS_CONF_PACKET_BUFF_CAPACITY;
    txBuffer.head = txBuffer.buffer_start;
    txBuffer.tail = txBuffer.buffer_start;
    txBuffer.count = 0;
}


/** 
 * @brief Put statistics of received packet into RX buffer.
 * @param  frame pointer to the struct variable where received payload is stored.
 * @return  1 if success, else  (if buffer is full)  
 */
uint8_t
STATS_put_rx_packet(rxFrame_t *frame){

// ----- Check if buffer has space -----
    if((rxBuffer.count == rxBuffer.capacity) || (rxBuffer.head == rxBuffer.tail-(rxBuffer.size/4))){
        LOG_WARN("RX Buffer is full! \n");
        // TODO: (If needed) Add count of packets that didn't fit into buffer
        return 0;
    }
// ----- Get statistics -----
    rxPacket_t rxPacket;
    uint8_t temp;
    uint8_t dest, sour;

    // First get timestamp
    vsnTime_preciseUptime(&rxPacket.timestamp_s, &rxPacket.timestamp_us);

    // Get a packet type
    temp = frame->content[0];
    temp = temp & 0x07;  
    if(temp == 0){      // Beacon
        RF2XX_STATS_ADD(rxBeacon);
    }
    else if(temp == 1){ // Data
        RF2XX_STATS_ADD(rxData);
    }
    else if(temp == 2){  // ACK
        RF2XX_STATS_ADD(rxAck);
    }
    rxPacket.type = temp;   

    // Source addres info is stored in FCF byte 2
    // 00 : no address present
    // 10 : short address (16bit)
    // 11 : long address  (64bit)
    sour = dest = frame->content[1];
    sour &= 0xC0;
    sour = sour >> 6;

    // Destination addressing mode
    dest &= 0x0C;
    dest = dest >> 2;
    if(dest == 2){
        if(sour == 3){
            rxPacket.source_addr = (( (uint16_t) frame->content[7]) << 8 | frame->content[6]);
            //printf("%02X %02X \n", frame->content[7], frame->content[6]);
        } else{ 
            LOG_ERR("Invalid address mode in TX\n"); // ...not tested yet
        }
        // TODO: When dest == 2, SQN is 205...dkw
    }
    if(dest == 3){
        if(sour == 3){
            rxPacket.source_addr = (( (uint16_t) frame->content[14]) << 8 | frame->content[13]);
            //printf("%02X %02X \n", frame->content[14], frame->content[13]);
        } else if(sour == 0){
            rxPacket.source_addr = 0;
        } else{
            LOG_ERR("Invalid address mode in TX\n"); // ...not tested yet
        }
    }

    // SQN is stored in Byte 3 of payload 
    rxPacket.sqn = frame->content[2]; 

    // Get other values
    rxPacket.rssi = frame->rssi;
    rxPacket.lqi  = frame->lqi;
    rxPacket.len  = frame->len;
    rxPacket.channel  = bitRead(SR_CHANNEL);

    rxPacket.count = rxPacketCount;
    rxPacketCount++;

// ----- Copy statistics into buffer -----
    memcpy(rxBuffer.head, &rxPacket, rxBuffer.size);

    // Increase head pointer (must divide with 4, otherwise data gets overwritten)
    rxBuffer.head = rxBuffer.head + (rxBuffer.size/4);    
    if (rxBuffer.head == rxBuffer.buffer_end){
        rxBuffer.head = rxBuffer.buffer_start;
    }
    rxBuffer.count++;
    return 1;
}

/** 
 * @brief Get statistics of received packet from RX buffer.
 * @param  packet pointer to the struct variable where statistics will be stored.
 * @return  1 if success, else 0 (if buffer is empty)   
 */
uint8_t
STATS_get_rx_packet(rxPacket_t *packet){

    if(rxBuffer.count == 0){
        LOG_WARN("RX Buffer is empty.. \n");
        return 0;
    }
    memcpy(packet, rxBuffer.tail, rxBuffer.size);

    rxBuffer.tail = rxBuffer.tail + (rxBuffer.size/4);
    if (rxBuffer.tail == rxBuffer.buffer_end){
        rxBuffer.tail = rxBuffer.buffer_start;
    }
    rxBuffer.count--;
    return 1;
}

/** 
 * @brief Put statistics of transmited packet into buffer.
 * @param  frame pointer to the struct variable where prepared payload is stored.
 * @return  1 if success, else 0 (if buffer is full)  
 */
uint8_t
STATS_put_tx_packet(txFrame_t *frame){

    // First check if buffer has space left
    if((txBuffer.count == txBuffer.capacity) || (txBuffer.head == txBuffer.tail-(txBuffer.size/4))){
        LOG_WARN("TX Buffer is full! \n");
        return 0;
    }

// ----- Get statistics -----
    txPacket_t txPacket;
    uint8_t tmp;

    // First get timestamp
    vsnTime_preciseUptime(&txPacket.timestamp_s, &txPacket.timestamp_us);

    // Get a packet type
    tmp = frame->content[0];
    tmp = tmp & 0x07;
    if(tmp == 0){
        RF2XX_STATS_ADD(txBeacon);
    }
    else if(tmp == 1){
        RF2XX_STATS_ADD(txData);
    }
    else if(tmp == 2 ){
        RF2XX_STATS_ADD(txAck);
    }
    txPacket.type = tmp;

    // Broadcast or Unicast 
    tmp = frame->content[0];
    tmp &= 0x20;
    tmp = tmp >> 5;

    txPacket.unicast = tmp;
    if(tmp == 1) RF2XX_STATS_ADD(txReqAck);  

    // Dest addres info is stored in byte 2 (FCF)
    tmp = frame->content[1];
    tmp &= 0x0C;
    tmp = tmp >> 2;
    if(tmp == 0){           // No address present
        txPacket.dest_addr = 0;
    }
    else if(tmp == 2){      // Short address                                                             
        txPacket.dest_addr = (( (uint16_t) frame->content[5]) << 8 | frame->content[4]);
        //printf("%02X %02X\n", frame->content[5], frame->content[4]);
        // TODO mislm da tuki ni prau seqn :)
    }
    else if(tmp == 3){      // Long address
        txPacket.dest_addr = (( (uint16_t)frame->content[6]) << 8 | frame->content[5]);
        //printf("%02X %02X \n", frame->content[6], frame->content[5]);
    }
    else{
        LOG_ERR("Invalid address mode in TX\n");
    }

    // SQN is stored in Byte 3 of payload
    txPacket.sqn = frame->content[2];

    // Get other values
    txPacket.channel = bitRead(SR_CHANNEL);
    txPacket.power   = bitRead(SR_TX_PWR);
    txPacket.len = frame->len;

    txPacket.count = txPacketCount;
    txPacketCount++;

// ----- Copy statistics into buffer -----
    // Add colected statistics into buffer
    memcpy(txBuffer.head, &txPacket, txBuffer.size);

    // Increase head pointer
    txBuffer.head = txBuffer.head + (txBuffer.size/4);
    if (txBuffer.head == txBuffer.buffer_end){
        txBuffer.head = txBuffer.buffer_start;
    }
    txBuffer.count++;
    return 1;
}

/** 
 * @brief Get statistics of transmited packet from TX buffer.
 * @param  packet pointer to the struct variable where statistics will be stored.
 * @return  1 if success, else 0 (if buffer is empty)   
 */
uint8_t
STATS_get_tx_packet(txPacket_t *packet){

    if(txBuffer.count == 0){
        LOG_WARN("TX Buffer is empty.. \n");
        return 0;
    }
    memcpy(packet, txBuffer.tail, txBuffer.size);

    txBuffer.tail = txBuffer.tail + (txBuffer.size/4);    
    if (txBuffer.tail == txBuffer.buffer_end){
        txBuffer.tail = txBuffer.buffer_start;
    }
    txBuffer.count--;
    return 1;
}



/*------------------------------------------------------------ 
    --- TX PACKET STATISTICS ---
    Packet 11 was: unicast DATA for node 0x4CF8
    * CHN: 26 | LEN: 21 | SQN: 51
    * Timestamp: 60705

    --- RX PACKET STATISTICS ---
    Packet 17 was: BCN from node 0x4CF8
    * CHN: 26 | LEN: 35 | SQN: 53
    * RSSI: -82 | LQI: 255
    * Timestamp: 64211
 ------------------------------------------------------------*/
/** 
 * @brief Costum function for stats-app.c.
 */
void
STATS_display_packet_stats(void){
    rxPacket_t rxPacket;
    txPacket_t txPacket;

    if(txBuffer.count != 0){
        LOG_INFO("--- TX PACKET STATISTICS ---\n");   
        while(txBuffer.count != 0){
            STATS_get_tx_packet(&txPacket);

            LOG_INFO("Packet %d was: ",(txPacket.count));
            if(txPacket.unicast == 1)LOG_INFO_("unicast ");
            switch(txPacket.type){
                case 0:
                LOG_INFO_("BCN ");
                break;
                case 1:
                LOG_INFO_("DATA ");
                break;
                case 2:
                LOG_INFO_("ACK ");
                break;
            }
            LOG_INFO_("for node 0x%X\n", txPacket.dest_addr);
            LOG_INFO("* Timestamp: %lds and %ldus\n", txPacket.timestamp_s, txPacket.timestamp_us);
            LOG_INFO("* CHN: %d", txPacket.channel);
            LOG_INFO_(" | LEN: %d", txPacket.len);
            LOG_INFO_(" | SQN: %d\n", txPacket.sqn);
            if(txPacket.power == 0){
                LOG_INFO("* Power: 3dBm\n");          // page 106 of datasheet
            } else {
                LOG_INFO("* Power %d\n", txPacket.power);
            }
            LOG_INFO("\n");
        }
    }
    if(rxBuffer.count != 0){
        LOG_INFO("--- RX PACKET STATISTICS ---\n");       
        while(rxBuffer.count != 0){
            STATS_get_rx_packet(&rxPacket);

            LOG_INFO("Packet %d was: ", (rxPacket.count)); 
            switch(rxPacket.type){
                case 0:
                LOG_INFO_("eBCN ");
                LOG_INFO_("from node 0x%X\n", rxPacket.source_addr);
                break;
                case 1:
                LOG_INFO_("DATA ");
                LOG_INFO_("from node 0x%X\n", rxPacket.source_addr);
                break;
                case 2:
                LOG_INFO_("ACK \n");
                // No address when receiving ACK
                break;
                default:
                LOG_INFO_("Unknown type (3 = CMD, 7 = BCN_REQ");
            }
            LOG_INFO("* Timestamp: %lds and %ldus\n", rxPacket.timestamp_s, rxPacket.timestamp_us);
            LOG_INFO("* CHN: %d", rxPacket.channel);
            LOG_INFO_(" | LEN: %d", rxPacket.len);
            LOG_INFO_(" | SQN: %d\n", rxPacket.sqn);
            LOG_INFO("* RSSI: %d", rxPacket.rssi);
            LOG_INFO_(" | LQI: %d\n", rxPacket.lqi);
            LOG_INFO("\n");
        }
    }
}


/*------------------------------------------------------------
    T 14 [ 61:122827] B 0xFFFF (C 15 L 35 S205 | P3.0) M
    T 15 [ 61:193676] D 0x4CF8 (C 20 L 94 S205 | P3.0) U

    R 16 [ 53: 10711] D 0x D01 (C 20 L 83 S 50 | R-67 Q255)
    R 17 [ 53: 81672] A        (C 26 L 17 S 46 | R-67 Q255)
---------------------------------------------------------------*/
/** 
 * @brief Costum function for stats-app.c.
 */
void
STATS_print_packet_stats(void){
    rxPacket_t rxPacket;
    txPacket_t txPacket;

    if(txBuffer.count != 0){
        printf("\n");
        while(txBuffer.count != 0){
            STATS_get_tx_packet(&txPacket);
            
            printf("T%3d ",txPacket.count);
            printf("[%3ld:%6ld] ", txPacket.timestamp_s, txPacket.timestamp_us);
            switch(txPacket.type){
                case 0:
                printf("B 0x%4X ", txPacket.dest_addr);
                break;
                case 1:
                printf("D 0x%4X ", txPacket.dest_addr);
                break;
                case 2:
                printf("A 0x%4X ", txPacket.dest_addr);
                break;
                default:
                printf("Undef ");
                break;
            }
                       
            printf("(C%3d L%3d S%3d | P", txPacket.channel, txPacket.len, txPacket.sqn);
            switch(txPacket.power){
                case 0x0:
                    printf("3.0"); break;
                case 0x1:
                    printf("2.8"); break;
                case 0x2:
                    printf("2.3"); break;
                case 0x3:
                    printf("1.8"); break;
                case 0x4:
                    printf("1.3"); break;
                case 0x5:
                    printf("0.7"); break;
                case 0x6:
                    printf("0.0"); break;
                case 0x7:
                    printf("-1"); break;
                case 0x8:
                    printf("-2"); break;
                case 0x9:
                    printf("-3"); break;
                case 0xa:
                    printf("-4"); break;
                case 0xb:
                    printf("-5"); break;
                case 0xc:
                    printf("-7"); break;
                case 0xd:
                    printf("-9"); break;
                case 0xe:
                    printf("-12"); break;
                case 0xf:
                    printf("-17"); break;
            }
            if(txPacket.unicast) printf(") U\n");
            else printf(") M\n");
        }
    }
    if(rxBuffer.count != 0){  
        printf("\n");
        while(rxBuffer.count != 0){
            STATS_get_rx_packet(&rxPacket);
            
            printf("R%3d ",rxPacket.count);
            printf("[%3ld:%6ld] ", rxPacket.timestamp_s, rxPacket.timestamp_us);
            switch(rxPacket.type){
                case 0:
                printf("B 0x%4X ", rxPacket.source_addr);
                break;
                case 1:
                printf("D 0x%4X ", rxPacket.source_addr);
                break;
                case 2:
                printf("A        ");
                break;
                default:
                printf("Undef");
                break;
            }
            printf("(C%3d L%3d S%3d | R%3d Q%3d)\n",rxPacket.channel, rxPacket.len, rxPacket.sqn, rxPacket.rssi, rxPacket.lqi);
        }
    }
}
/* -----------------------------
        T3 0 D 0xFFFF
        253:580174
        C15 L24 S205 | P0

        R5 B 0xD01
        252:184481
        C15 L35 S205 | R-47 Q255
 ---------------------------------
void
STATS_print_packet_stats(void){
    rxPacket_t rxPacket;
    txPacket_t txPacket;

    if(txBuffer.count != 0){
        while(txBuffer.count != 0){
            STATS_get_tx_packet(&txPacket);
            printf("\n");
            printf("T%d %d ",txPacket.count, txPacket.unicast);
            switch(txPacket.type){
                case 0:
                printf("B 0x%X\n", txPacket.dest_addr);
                break;
                case 1:
                printf("D 0x%X\n", txPacket.dest_addr);
                break;
                case 2:
                printf("A 0x%X\n", txPacket.dest_addr);
                break;
                default:
                printf("Un\n");
                break;
            }
            printf("%ld:%ld\n", txPacket.timestamp_s, txPacket.timestamp_us);
            printf("C%d L%d S%d | P%d\n", txPacket.channel, txPacket.len, txPacket.sqn, txPacket.power);
        }
    }
    if(rxBuffer.count != 0){  
        while(rxBuffer.count != 0){
            STATS_get_rx_packet(&rxPacket);
            printf("\n");
            printf("R%d ",rxPacket.count);
            switch(rxPacket.type){
                case 0:
                printf("B 0x%X\n", rxPacket.source_addr);
                break;
                case 1:
                printf("D 0x%X\n", rxPacket.source_addr);
                break;
                case 2:
                printf("A\n");
                break;
                default:
                printf("Un\n");
                break;
            }
            printf("%ld:%ld\n", rxPacket.timestamp_s, rxPacket.timestamp_us);
            printf("C%d L%d S%d | R%d Q%d\n",rxPacket.channel, rxPacket.len, rxPacket.sqn, rxPacket.rssi, rxPacket.lqi);
        }
    }
}
*/


/** 
 * @brief Reset buffer variables - just read the values and don't display them
 */
void
STATS_clear_packet_stats(void){

    rxPacket_t rxPacket;
    txPacket_t txPacket;

    if(txBuffer.count != 0){
        while(txBuffer.count != 0){
            STATS_get_tx_packet(&txPacket);
        }
    }
    if(rxBuffer.count != 0){  
        while(rxBuffer.count != 0){
            STATS_get_rx_packet(&rxPacket);
        }
    }
    rxPacketCount = 1;
    txPacketCount = 1;
}



/**********************************************************************************************************
 * BACKGROUND NOISE
 **********************************************************************************************************/

static uint16_t channel_stats_index;   
static buffer_t buffer[16];

/** 
 * @brief Prepare buffer for storing RSSI values
 *
 * Allocate memory for ring buffers, depending on channel number and input 
 * capacity - number of valuesto be stored. Reset all variables of ring buffer.
 * Add channel to the channel index.
 *
 * @param  b  array of channel buffers
 * @param  ch number of buffer (= channel number)
 * @param  capacity size of buffer
 * @return 1 if success, 0 if not
 */
uint8_t
STATS_init_channel_buffer(buffer_t *b, uint8_t ch, uint16_t capacity){

    b[ch].size = sizeof(bgNoise_t); //12B
    b[ch].buffer_start = heapmem_alloc(capacity * b[ch].size );

    if(b[ch].buffer_start == NULL){
        printf("RSSI Memory buffer for channel %d could not be allocated!..not enough heap memory. \n", ch+11);
        while(1){} //TODO ce nemore inicializirat kanala, naj neha provat ponovno init
        return 0;
    }
    b[ch].buffer_end = (char *)b[ch].buffer_start + capacity * b[ch].size;

    b[ch].capacity = capacity;
    b[ch].head = b[ch].buffer_start;
    b[ch].tail = b[ch].buffer_start;
    b[ch].count = 0;

    // Add channel to channel_index - so we know it is allready initilazed :)
    uint16_t tmp = 0x0001;
    tmp = tmp << ch;
    channel_stats_index |= tmp;
    return 1;
}

/** 
 * @brief Free allocated memory
 */
void
STATS_free_channel_buffer(buffer_t *b, uint8_t ch){
    heapmem_free(b[ch].buffer_start);

    // Remove channel from channel_index
    uint16_t tmp = 0x0001;
    tmp = tmp << ch;
    tmp = ~tmp;
    channel_stats_index &= tmp;
}

/** 
 * @brief When measured, put new value into buffer of that channel
 * @param b  array of channel buffers
 * @param ch channel on which value is measured
 * @param bgn struct with rssi value and time-stamp
 * @return 1 if success, 0 if not
 */
uint8_t
STATS_put_channel_rssi(buffer_t *b, uint8_t ch, bgNoise_t *bgn){

    if((b[ch].count == b[ch].capacity) || (b[ch].head == b[ch].tail-(b[ch].size/4))){
        LOG_WARN("RSSI Buffer on channel %d is full! \n", ch+11);
        return 0;
    }
   
    memcpy(b[ch].head, bgn, b[ch].size);

    b[ch].head = b[ch].head + (b[ch].size/4);
    if (b[ch].head == b[ch].buffer_end){
        b[ch].head = b[ch].buffer_start;
    }
    b[ch].count++;
    return 1;
}

/** 
 * @brief Get value from buffer of selected channel
 * @param b  array of channel buffers
 * @param ch channel on which value is measured
 * @param bgn struct with rssi value and time-stamp
 * @return 1 if success, 0 if not
 */
uint8_t
STATS_get_channel_rssi(buffer_t *b, uint8_t ch, bgNoise_t *bgn){

    if(b[ch].count == 0){
        LOG_WARN("RSSI buffer on channel %d is empty!\n", ch);
        return 0;
    }
    memcpy(bgn, b[ch].tail, b[ch].size);

    b[ch].tail = b[ch].tail + (b[ch].size/4);    
    if (b[ch].tail == b[ch].buffer_end){
        b[ch].tail = b[ch].buffer_start;
    }
    b[ch].count--;
    return 1;
}





// Added so it won't print "Buffer full" all the time if something goes wrong
static uint8_t suc_update = 1;

/** 
 * @brief Check if buffer of selected channel is init and store the value into it.
 *        If buffer is not yet initialized, first init it and then store value.
 * @param capacity of buffer
 */
void 
STATS_update_background_noise(uint16_t capacity){
    uint8_t suc_init = 0;
    bgNoise_t bgn;

    // Channel 11 is stored in channel_buffer[0], 12->[1] and so on
    uint8_t channel = bitRead(SR_CHANNEL) - 11;     
    bgn.rssi = bitRead(SR_RSSI);
    vsnTime_preciseUptime(&bgn.timestamp_s, &bgn.timestamp_us);

    if(channel < 16 && channel >= 0){
        
        // If channel was not yet initialized
        if(!STATS_is_channel_init(channel)){
            
            // Init channel
            suc_init = STATS_init_channel_buffer(buffer, channel, capacity);
            // Store the RSSI value
            if(suc_init) suc_update = STATS_put_channel_rssi(buffer, channel, &bgn);  
       
        } else {
            // Just store the RSSI value
            if(suc_update) suc_update = STATS_put_channel_rssi(buffer, channel, &bgn);   
        }
    }
}

/** 
 * @brief Read data from buffer for each channel and send it via UART
 */
void
STATS_print_background_noise(void){
    bgNoise_t data;
    int rssi, first_us, first_s;

    if(channel_stats_index != 0){
        // Go through all channels
        for(uint8_t i=0; i<16; i++){

            // If channel was initialized
            if(STATS_is_channel_init(i)){

                // If there is data in buffer            
                if(buffer[i].count != 0){
                    printf("\n");
                    printf("CH%d:",(i+11));

                    // Get first measurment - its timestamp
                    STATS_get_channel_rssi(buffer, i, &data);
                    printf("[%3ld:%6ld] ", data.timestamp_s, data.timestamp_us);

                    first_us = data.timestamp_us;
                    first_s = data.timestamp_s;

                    // Print count of all measurements
                    printf("(%3d) ", buffer[i].count);

                    // Print firs measured RSSI
                    rssi =(3 * (data.rssi - 1) + RSSI_BASE_VAL);
                    printf("[0]%d ", rssi);

                    // Go through buffer
                    while(buffer[i].count != 0){
                        
                        STATS_get_channel_rssi(buffer, i, &data);

                        // Print deviation from first timestamp
                        if(data.timestamp_s > first_s){
                            // If seconds changed during measurments, us counter
                            // starts counting from zero...its max value is 1000 000
                            uint32_t seconds_passed = data.timestamp_s - first_s;
                            printf("[%ld]", (((1000000 - first_us) + data.timestamp_us) + (seconds_passed * 1000000)));
                        } else{
                            printf("[%ld]", (data.timestamp_us - first_us));
                        }
                        // Print its RSSI
                        rssi =(3 * (data.rssi - 1) + RSSI_BASE_VAL);
                        printf("%d ", rssi);
                    }
                }
            }
        }
        printf("\n");
        suc_update = 1;
    }
}

/** 
 * @brief Clear buffers - read values but don't display them 
 */
void
STATS_clear_background_noise(void){
    // Go through all channels
    for(uint8_t i=0; i<16; i++){
        // If channel was initialized
        if(STATS_is_channel_init(i)){
            STATS_free_channel_buffer(buffer, i);
        }
    }
}

/** 
 * @brief Check if buffer was initialized for certain channel
 * @param  channel number to check 
 */
uint8_t
STATS_is_channel_init(uint8_t channel){
    uint16_t tmp = channel_stats_index;
    tmp = tmp >> channel;
    tmp &= 0x0001;

    return ((tmp == 1)? 1 : 0);
}


/**********************************************************************************************************
 * DRIVER STATISTICS
 **********************************************************************************************************/

extern const struct radio_driver rf2xx_driver;

/** @brief Costum function for stats-app.c. --> HUMAN READABLE
 *         Print all driver statistics.
 * 
 *  ------- TX STATISTICS -------
    Success: 36 | Error: 0
    * Beac: 4
    * Data: 20
    * Ackn: 12
    ------- RX STATISTICS -------
    Success: 39 | Detected: 39
    * Beac: 6
    * Data: 16
    * Ackn: 17 -> Requested 17
 */
void
STATS_display_driver_stats(void){
    LOG_INFO("------- TX STATISTICS -------\n");
    LOG_INFO("Success: %ld | Error: %ld\n",RF2XX_STATS_GET(txCount),RF2XX_STATS_GET(txError));  //txCount == txSuccess in TSCH
    LOG_INFO(" * Beac: %ld\n", RF2XX_STATS_GET(txBeacon));
    LOG_INFO(" * Data: %ld\n", RF2XX_STATS_GET(txData));
    LOG_INFO(" * Ackn: %ld\n", RF2XX_STATS_GET(txAck));
    
    LOG_INFO("------- RX STATISTICS -------\n");
    LOG_INFO("Success: %ld | Detected: %ld\n", RF2XX_STATS_GET(rxSuccess), RF2XX_STATS_GET(rxDetected));
    LOG_INFO(" * Beac: %ld\n",RF2XX_STATS_GET(rxBeacon));
    LOG_INFO(" * Data: %ld\n",RF2XX_STATS_GET(rxData));
    LOG_INFO(" * Ackn: %ld -> Requested %ld\n", RF2XX_STATS_GET(rxAck),RF2XX_STATS_GET(txReqAck));
    LOG_INFO_("\n");
}

/** @brief Costum function for stats-app.c.--> FOR SERIAL MONITOR
 *         Print all driver statistics.
 * 
 *  TX suc36 err0  cnt: B4 D20 A12
    RX suc39 det39 cnt: B6 D16 A17 -> req(17)
 */
void
STATS_print_driver_stats(void){
    printf("\n");
    printf("TX suc%ld err%ld  cnt: B%ld D%ld A%ld\n",
            RF2XX_STATS_GET(txCount),
            RF2XX_STATS_GET(txError),
            RF2XX_STATS_GET(txBeacon),
            RF2XX_STATS_GET(txData),
            RF2XX_STATS_GET(txAck)
    );
    printf("RX suc%ld det%ld cnt: B%ld D%ld A%ld -> req(%ld)\n",
            RF2XX_STATS_GET(rxSuccess), 
            RF2XX_STATS_GET(rxDetected),
            RF2XX_STATS_GET(rxBeacon),
            RF2XX_STATS_GET(rxData),
            RF2XX_STATS_GET(rxAck),
            RF2XX_STATS_GET(txReqAck)
    );
}

/** @brief Costum function for stats-app.c. --> FOR DEBUGGING PURPOSE
 *         Print small in-line staistics.
 * 
 *  CH 26 [currRSSI -94, lastRSSI -82, LQI 255]  R[38(38)]  T[35(35)]
    CH 20 [currRSSI -94, lastRSSI -82, LQI 255]  R[39(39)]  T[36(36)]
    CH 15 [currRSSI -94, lastRSSI -82, LQI 255]  R[42(42)]  T[37(37)]
 */
void
STATS_display_driver_stats_inline(void){
    radio_value_t channel;
    radio_value_t lastRssi;
    radio_value_t currRssi;
    radio_value_t lqi;

    rf2xx_driver.get_value(RADIO_PARAM_CHANNEL, &channel);
    rf2xx_driver.get_value(RADIO_PARAM_RSSI, &currRssi);
    rf2xx_driver.get_value(RADIO_PARAM_LAST_RSSI, &lastRssi);
    rf2xx_driver.get_value(RADIO_PARAM_LAST_LINK_QUALITY, &lqi);

    LOG_INFO("CH %2d [currRSSI %d, lastRSSI %d, LQI %d]  R[%ld(%ld)]  T[%ld(%ld)]\n",
        channel,                        // Current channel
        currRssi,                       // Current RSSI value
        lastRssi,                       // Last RSSI measured
        lqi,                            // Last LQI measured

        RF2XX_STATS_GET(rxDetected),    // Num of detected packets
        RF2XX_STATS_GET(rxSuccess),     // Successfully received packets

        RF2XX_STATS_GET(txTry),         // Num of transmissions
        RF2XX_STATS_GET(txSuccess)      // Successfull transmissions
    );
}

#endif