/*
    Real-time middleware for this architecture/platform
    Contiki requires to define functions:
      - rtimer_arch_now()
      - void rtimer_arch_init(void)
      - void rtimer_arch_schedule(rtimer_clock_t t);
      - RTIMER_ARCH_SECOND

    TODO: Timers & RTimers are usualy CPU specific.
        At some point move this to cpu/.
*/
#include "contiki.h"
#include "sys/energest.h"
#include "sys/rtimer.h"

#include "rtimer-arch.h"

// STM libraries
#include "misc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

// DEBUG option
#ifndef RTIMER_ARCH_DEBUG
#define RTIMER_ARCH_DEBUG 0
#endif

#if RTIMER_ARCH_DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


// Was timer5 selected for some specific reason???
#define RTIMER_TIM   TIM5
#define RTIMER_IRQN  TIM5_IRQn



// rtimer_clock_t is unsigned short == 16bits
// but we changed it to uint32 in contiki-conf.h
// 1 "second" = value of 1024
// 1 tick on timer == 1/1024 second
// f_timer = 1kHz; f_clock = 8MHz
// prescaler: f_clock/f_timer = 8000
#define PRESCALER   7999 // 0-based counting


void rtimer_arch_init(void) {
    PRINTF("rtimer_arch_init\n");

    // for real-time counter we will use timer5 (16-bit; Enable it.
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    // Stop it for initialization
    TIM_Cmd(RTIMER_TIM, DISABLE);


    // Initialization
    TIM_TimeBaseInitTypeDef tim_TimeBaseStructure;
    tim_TimeBaseStructure.TIM_Period = 0xFFFF; // Upper limit; max value of 16-bit counter
    tim_TimeBaseStructure.TIM_Prescaler = PRESCALER;
    tim_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    tim_TimeBaseStructure.TIM_RepetitionCounter = 0; // we don't care anyway

    TIM_TimeBaseInit(RTIMER_TIM, &tim_TimeBaseStructure);
    TIM_PrescalerConfig(RTIMER_TIM, PRESCALER, TIM_PSCReloadMode_Immediate);

	TIM_SetCounter(RTIMER_TIM, 0); // Set initial value

	TIM_OC1PreloadConfig(RTIMER_TIM, TIM_OCPreload_Disable);

	TIM_ITConfig(RTIMER_TIM, TIM_IT_CC1, DISABLE);

	TIM_ClearITPendingBit(RTIMER_TIM, TIM_IT_CC1);

    TIM_Cmd(RTIMER_TIM, ENABLE); // Now, start counting
}


rtimer_clock_t rtimer_arch_now(void) {
    return TIM_GetCounter(RTIMER_TIM);
}


void rtimer_arch_schedule(rtimer_clock_t t) {
    TIM_ClearITPendingBit(RTIMER_TIM, TIM_IT_CC1);
    TIM_ITConfig(RTIMER_TIM, TIM_IT_CC1, DISABLE);

    TIM_OCInitTypeDef tim_oc1_init;
    tim_oc1_init.TIM_OCMode = TIM_OCMode_Timing;
    tim_oc1_init.TIM_OCPolarity = TIM_OCPolarity_High;
    tim_oc1_init.TIM_Pulse = t;
    tim_oc1_init.TIM_OutputState = TIM_OutputState_Enable;

    TIM_OC1Init(RTIMER_TIM, &tim_oc1_init);
    TIM_ITConfig(RTIMER_TIM, TIM_IT_CC1, ENABLE);
}


void contiki_rtimer_isr(void) {
    // disable interrupt; if there is any new event,
    // then schedule will enable it
    TIM_ITConfig(RTIMER_TIM, TIM_IT_CC1, DISABLE);
    rtimer_run_next();
}
