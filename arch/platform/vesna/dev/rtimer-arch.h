#ifndef RTIMER_ARCH_H_
#define RTIMER_ARCH_H_

#include "sys/rtimer.h"

// CC26xx has 65536. Why so?
#define RTIMER_ARCH_SECOND		(1024)
//#define RTIMER_ARCH_SECOND    65536

rtimer_clock_t rtimer_arch_now(void);

void contiki_rtimer_isr(void);

#endif
