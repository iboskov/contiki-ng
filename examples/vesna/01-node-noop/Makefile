CONTIKI_PROJECT = node
all: $(CONTIKI_PROJECT)

TARGET = vesna

CONTIKI =  ../../..

#CFLAGS += -DRF2XX_DEBUG


ifeq ($(HOST_ADDR),)
HOST_ADDR = aaaa::1/64
endif

#UART_ENABLE = 1
#USART1_BAUDRATE = 115200
#CFLAGS += -DUSART1_BAUDRATE=$(USART1_BAUDRATE)

AT86RF2XX_BOARD = ISMTV_v1_1


include $(CONTIKI)/Makefile.include

$(CONTIKI)/tools/tunslip6:	$(CONTIKI)/tools/tunslip6.c
	(cd $(CONTIKI)/tools && $(MAKE) tunslip6)


#	sudo sysctl -w net.ipv6.conf.all.forwarding=1

connect-router:	$(CONTIKI)/tools/tunslip6
	sudo $(CONTIKI)/tools/tunslip6 -L -v4 -s /dev/ttyUSB0 -B $(USART1_BAUDRATE) $(HOST_ADDR)
