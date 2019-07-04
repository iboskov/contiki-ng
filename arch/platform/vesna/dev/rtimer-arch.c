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
#include <stdio.h>
#include <stdlib.h>
#include "contiki.h"
#include "sys/energest.h"
#include "sys/rtimer.h"

#include "rtimer-arch.h"

// STM libraries
#include "misc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"


#define RTIMER_TIM  TIM5
#define RTIMER_IRQN TIM5_IRQn
#define RTIMER_APB1 RCC_APB1Periph_TIM5


volatile uint32_t overflow = 0;


uint32_t rtimer_arch_us_to_rtimerticks(int32_t us) {
    uint32_t ticks = (abs(us) * 64) / 1000;
    //if (ticks > 64000) ticks -= 64000;
    //printf("%luus -> %luticks\n", us, ticks);
    return ticks;
}

uint32_t rtimer_arch_rtimerticks_to_us(int32_t ticks) {
    uint32_t us = (abs(ticks) * 15625) / 1000;
    //printf("%luticks -> %luus\n", ticks, us);
    return us;
}

/*
    Contiki(-ng) uses several timers. We implemented rtimer (r is for real-time)
    using TIM5 general purpose timer found.

    TIM5 properties:
        - 16-bit up/down counter,
        - interrupt can be triggered on overflow or specific value,

    (optional) It is possible to bring in external clock from a AT86RF2xx radio,
    which is suppose to be more stable.

    When VESNA is running on internal scilator at 64MHz, the TIM5 receives
    gets 64MHz clock. Our goal is to get >= 32kHz triggers (see tsch-slot-operation.c)
    We set goal to have 64kHz triggers.

    So the equation is:
        event [Hz] = TIMx_CLK / ((PRSC + 1) * (PERIOD + 1))
*/

void rtimer_arch_init(void) {

    TIM_TimeBaseInitTypeDef timerInitStructure = {
        .TIM_Prescaler = 1000 - 1, // 0 = run @ 64MHz, 63 = run at 1MHz; 999 = run at 64kHz
        .TIM_CounterMode = TIM_CounterMode_Up,
        .TIM_Period = 64000 - 1, // upper bound value of counter
        .TIM_ClockDivision = TIM_CKD_DIV1,
        .TIM_RepetitionCounter = 0, // Not available on TIM5 anyway
    };


    // TIM5 clock enable
    RCC_APB1PeriphClockCmd(RTIMER_APB1, ENABLE);

    // Disable to configure it
    TIM_Cmd(RTIMER_TIM, DISABLE);

    // Initialize timer
    TIM_TimeBaseInit(RTIMER_TIM, &timerInitStructure);

    //TIM_PrescalerConfig(TIM5, 1000 - 1, TIM_PSCReloadMode_Immediate);

    // Set initial value (value could be random at power on)
    TIM_SetCounter(RTIMER_TIM, 0);

    // Not really sure what it is, but is used for PWM usecase. Disable it.
    //TIM_OC1PreloadConfig(RTIMER_TIM, TIM_OCPreload_Disable);

    //TIM_ARRPreloadConfig(RTIMER_TIM, ENABLE);

    // We are not controlling any pins
    //TIM_CtrlPWMOutputs(TIM5, DISABLE);

    // Disable interrupts from compare register 1
    TIM_ITConfig(RTIMER_TIM, 0xFF, DISABLE);

    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    // Clear interrupt bit, if it was triggered for some reason
    TIM_ClearITPendingBit(RTIMER_TIM, TIM_IT_CC1);
    TIM_ClearITPendingBit(RTIMER_TIM, TIM_IT_Update);


    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = RTIMER_IRQN;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    /*
    TIM_OCInitTypeDef tim_oc2_init;
    tim_oc2_init.TIM_OCMode = TIM_OCMode_Timing;
    tim_oc2_init.TIM_OCPolarity = TIM_OCPolarity_High;
    tim_oc2_init.TIM_Pulse = 0; // To be compared against
    tim_oc2_init.TIM_OutputState = TIM_OutputState_Disable;

    TIM_OC1Init(RTIMER_TIM, &tim_oc2_init);
    TIM_ClearITPendingBit(RTIMER_TIM, TIM_IT_CC2);
    TIM_ITConfig(RTIMER_TIM, TIM_IT_CC2, ENABLE);
    */



    // Enable timer back, since it was configured
    TIM_Cmd(RTIMER_TIM, ENABLE);

    /*
    // for real-time counter we will use TIM5 counter.
    // 16-bit up/down counter.
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    // Stop it for initialization
    TIM_Cmd(RTIMER_TIM, DISABLE);

    // Initialization
    TIM_TimeBaseInitTypeDef tim_TimeBaseStructure;
    tim_TimeBaseStructure.TIM_Period = 0xFFFF; // Upper limit; max value of 16-bit counter (65536)
    tim_TimeBaseStructure.TIM_Prescaler = PRESCALER;
    tim_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    tim_TimeBaseStructure.TIM_RepetitionCounter = 0; // we don't care anyway

    TIM_TimeBaseInit(RTIMER_TIM, &tim_TimeBaseStructure);

	TIM_SetCounter(RTIMER_TIM, 0); // Set initial value

	TIM_OC1PreloadConfig(RTIMER_TIM, TIM_OCPreload_Disable);

	TIM_ITConfig(RTIMER_TIM, TIM_IT_CC1, DISABLE);

	TIM_ClearITPendingBit(RTIMER_TIM, TIM_IT_CC1);

    TIM_Cmd(RTIMER_TIM, ENABLE); // Start counting ...
    */



}


rtimer_clock_t
rtimer_arch_now(void)
{   
    //rtimer_clock_t t = TIM_GetCounter(TIM5);
    //printf("%uticks\n", t);
    return (rtimer_clock_t)TIM_GetCounter(RTIMER_TIM) + overflow * RTIMER_ARCH_SECOND;
    //return t;
}


void
rtimer_arch_schedule(rtimer_clock_t t)
{
    t = t % RTIMER_ARCH_SECOND;

    //printf("NOW=%lu FUT=%lu\n", rtimer_arch_now(), t);
    TIM_ITConfig(RTIMER_TIM, TIM_IT_CC1, DISABLE);
    TIM_ClearITPendingBit(RTIMER_TIM, TIM_IT_CC1);

    TIM_OCInitTypeDef tim_oc1_init;
    tim_oc1_init.TIM_OCMode = TIM_OCMode_Timing;
    tim_oc1_init.TIM_OCPolarity = TIM_OCPolarity_High;
    tim_oc1_init.TIM_Pulse = t; // To be compared against
    tim_oc1_init.TIM_OutputState = TIM_OutputState_Disable;

    TIM_OC1Init(RTIMER_TIM, &tim_oc1_init);
    TIM_ITConfig(RTIMER_TIM, TIM_IT_CC1, ENABLE);
}


void
contiki_rtimer_isr(void)
{
    if (TIM_GetITStatus(RTIMER_TIM, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(RTIMER_TIM, TIM_IT_Update);
        overflow += 1;
    }

    if (TIM_GetITStatus(RTIMER_TIM, TIM_IT_CC1) != RESET) {
        // Comparator 1 trigger
        TIM_ITConfig(RTIMER_TIM, TIM_IT_CC1, DISABLE);
        TIM_ClearITPendingBit(RTIMER_TIM, TIM_IT_CC1);

        rtimer_run_next();
    }




/*
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
        // Overflow happened
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        overflow += 1;
   }
*/

    /*if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
        // Disable interrupt. Will be enabled again if needed
        TIM_ITConfig(TIM5, TIM_IT_CC1, DISABLE);
        //TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        rtimer_run_next();
    }*/
}


