CONTIKI_PROJECT = gateway
all: $(CONTIKI_PROJECT)

TARGET = vesna

CONTIKI =  ../../..
CFLAGS += -DRF2XX_DEBUG


ifeq ($(HOST_ADDR),)
HOST_ADDR = aaaa::1/64
endif

PROJECT_SOURCEFILES += slip-bridge.c

UART_ENABLE = 1
USART1_BAUDRATE = 115200
CFLAGS += -DUSART1_BAUDRATE=$(USART1_BAUDRATE)


AT86RF2XX_BOARD = SNR

WITH_SLIP = 1

include $(CONTIKI)/Makefile.include

$(CONTIKI)/tools/tunslip6:	$(CONTIKI)/tools/tunslip6.c
	(cd $(CONTIKI)/tools && $(MAKE) tunslip6)


#	sudo sysctl -w net.ipv6.conf.all.forwarding=1

slip: $(TUNSLIP6)
	sudo $(TUNSLIP6) $(PREFIX) -L -v4 -s /dev/ttyUSB1 -B $(USART1_BAUDRATE) $(HOST_ADDR)
