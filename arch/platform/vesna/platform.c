#include <malloc.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "vsn.h"
#include "vsncrc32.h"
#include "vsnpm.h"
#include "vsnsetup.h"
#include "vsnusart.h"
#include "vsntime.h"

#include "contiki.h"
#include "dev/uart1.h"
#include "dev/serial-line.h"
#include "net/netstack.h"
#include "net/mac/framer/frame802154.h"
#include "net/linkaddr.h"
#include "lib/crc16.h"

#include "sys/platform.h"
#include "sys/node-id.h"

#include "sys/log.h"
#define LOG_MODULE "Platform"
#define LOG_LEVEL LOG_LEVEL_MAIN


void
platform_init_stage_one(void)
{
	// justinc, TOP_PAD is incorrect when first calling newlib malloc.
    mallopt(-2, 0); // #define M_TOP_PAD           -2

    // Reset clock configuration
    SystemInit();

    // Enable power management (enable/disable features/pins/hw)
    vsnPM_init();

    // VESNA uses internal clock generator, that's why is limited to 64MHz.
    // For up to 72MHz it would require external oscilator/clock generator.
    // VESNA SNC doesn't have one. 64MHz
    vsnSetup_intClk(SNC_CLOCK_64MHZ);

    // - Enable GPIO clocks
    // - Enable interrupts
    // - Enable RTC, Tick,
    vsnSetup_initSnc();

	// Workaround: vesna-drivers have issue with calibration, if clock != 8MHz
	// TODO: Is this still relevant?
    vsnSetup_intClk(SNC_CLOCK_8MHZ);
    vsnSetup_calibHsi();
    vsnSetup_intClk(SNC_CLOCK_64MHZ);

    vsnPM_mesureAdcBitVolt();
    vsnCRC32_init();
}


void
platform_init_stage_two(void)
{
	#if UART_CONF_ENABLE
	uart1_init(BAUD2UBR(UART1_BAUDRATE));
	uart1_set_input(serial_line_input_byte);
	#endif

	serial_line_init();

	const uint16_t id = crc16_data(STM32F1_UUID.u8, sizeof(STM32F1_UUID.u8), 0);
	const uint8_t extAddr[8] = { 0, 0x12, 0x4B, 0, 0, 0x06, (id & 0xff), (id >> 8) };

	// populate linkaddr_node_addr. Maintain endianess;
	// This sets MAC address
	memcpy(linkaddr_node_addr.u8, extAddr + 8 - LINKADDR_SIZE, LINKADDR_SIZE);
}


void
platform_init_stage_three(void)
{
	//set_rf_parameters();
	//const uint8_t extAddr[8] = { 0, 0x12, 0x4B, 0, 0, 0x06, (node_id & 0xff), (node_id >> 8) };

	NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, IEEE802154_PANID);
	NETSTACK_RADIO.set_value(RADIO_PARAM_16BIT_ADDR, node_id);
	NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, IEEE802154_DEFAULT_CHANNEL);
	NETSTACK_RADIO.set_object(RADIO_PARAM_64BIT_ADDR, linkaddr_node_addr.u8, LINKADDR_SIZE);
}


void
platform_idle()
{
	// lpm_enter();
}
