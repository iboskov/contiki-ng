#ifndef RTIMER_ARCH_H_
#define RTIMER_ARCH_H_

#include "sys/rtimer.h"

// Rtimer is 16bit counter with freq of 65.506 kHZ
// The timer is not 100% accurate (it does not run at 65.536 kHz)
// See rtimer-arch.c for explanation why such value
#define RTIMER_ARCH_SECOND		(65536)

#define RTIMER_ARCH_PPM         (594 + 1500)

// Converts micro seconds to rtimer ticks (65536 ticks/s ---> 1us = 0.065536 tick)
// Eqn.: T = (us * 0.065536) +- 1/2
#define US_TO_RTIMERTICKS(us)   ((us) >= 0 ? \
                                (uint32_t)(((((int64_t)(us)) * (RTIMER_ARCH_SECOND)) + 500000) / 1000000L) : \
                                (uint32_t)(((((int64_t)(us)) * (RTIMER_ARCH_SECOND)) - 500000) / 1000000L)) 

// Converts rtimer ticks to micro seconds (1/65536 Hz ---> 1 tick ~ 15.25879 us)
// Eqn.: us = (T * 15.258) +- 1/2
#define RTIMERTICKS_TO_US(t)    ((t) >= 0? \
                                ((((int32_t)(t)) * 1000000L + ((RTIMER_ARCH_SECOND) / 2)) / (RTIMER_ARCH_SECOND)) : \
                                ((((int32_t)(t)) * 1000000L - ((RTIMER_ARCH_SECOND) / 2)) / (RTIMER_ARCH_SECOND)))

// Converts rtimer ticks to micro-seconds (64-bit version) because the 32-bit one cannot handle T >= 4294 ticks.
// Intended only for positive values of t
#define RTIMERTICKS_TO_US_64(t) ((uint32_t)(((int64_t)(t)) * 1000000L + ((RTIMER_ARCH_SECOND) / 2)) / (RTIMER_ARCH_SECOND)) 


void contiki_rtimer_isr(void);
rtimer_clock_t rtimer_arch_now(void);

uint32_t rtimer_arch_us_to_rtimerticks(int32_t us);
uint32_t rtimer_arch_rtimerticks_to_us(int32_t ticks);

#endif
