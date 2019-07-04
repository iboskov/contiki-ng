#ifndef RTIMER_ARCH_H_
#define RTIMER_ARCH_H_

#include "sys/rtimer.h"

// Hint: See rtimer-arch.c for explanation why such value
#define RTIMER_ARCH_SECOND		(64000)


rtimer_clock_t rtimer_arch_now(void);
uint32_t rtimer_arch_us_to_rtimerticks(int32_t us);
uint32_t rtimer_arch_rtimerticks_to_us(int32_t ticks);

// converts micro-seconds to rtimer ticks (1 tick ~ 15.625us -> 0.064 tick/us)
#define US_TO_RTIMERTICKS(us)   (((us) * 64) / 1000)
//#define US_TO_RTIMERTICKS       rtimer_arch_us_to_rtimerticks

// converts rtimer ticks to micro-seconds (1/64kHz ~ 15.625us/tick)
#define RTIMERTICKS_TO_US(t)    (((t) * 15625) / 1000)

//#define RTIMERTICKS_TO_US       rtimer_arch_rtimerticks_to_us

// converts rtimer ticks to micro-seconds (64-bit version)
#define RTIMERTICKS_TO_US_64(t) (((t) * 15625) / 1000)
//#define RTIMERTICKS_TO_US_64    rtimer_arch_rtimerticks_to_us


/* A 64-bit version because the 32-bit one cannot handle T >= 4295 ticks.
   Intended only for positive values of T. */
//#define RTIMERTICKS_TO_US_64(T)  ((uint32_t)(((uint64_t)(T) * 1000000 + ((RTIMER_ARCH_SECOND) / 2)) / (RTIMER_ARCH_SECOND)))

void contiki_rtimer_isr(void);




#endif
