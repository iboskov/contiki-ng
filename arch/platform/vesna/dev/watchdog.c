#include <stdio.h>
#include "contiki-conf.h"
#include "dev/watchdog.h"
#include "vesna-conf.h"
#include "stm32f10x_iwdg.h"

#include "sys/log.h"
#define LOG_MODULE "Watchdog"
#define LOG_LEVEL LOG_CONF_LEVEL_WATCHDOG


#if WATCHDOG_CONF_ENABLED
#warning "Included watchdog ..."

void watchdog_init(void) {
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload(0xFFFF);
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
    LOG_INFO("HW watchdog initialized.\n")
}

void watchdog_start(void) {
    watchdog_init();
    IWDG_Enable();
    LOG_INFO("HW watchdog started.\n");
}

void watchdog_stop(void) {
    // practically stopping a watchdog is impossible, so we just use huge values here
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    IWDG_SetReload(0xFFFF);
    // should be about 26 seconds...
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
    IWDG_ReloadCounter();
}

void watchdog_periodic(void) {
    IWDG_ReloadCounter();
    LOG_INFO("watchdog_periodic triggered");
}

void watchdog_reboot(void) {
    // Intentional reboot with watchdog
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_4);
    IWDG_SetReload(0x0000);
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
    while (1);
}

#else // !WATCHDOG_CONF_ENABLED

void watchdog_init(void) {}
void watchdog_start(void) {}
void watchdog_periodic(void) {}
void watchdog_stop(void) {}
void watchdog_reboot(void) {}

#endif // WATCHDOG_CONF_ENABLED
