#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"



#define TIM4_CH1  //PB6
#define TIM4_CH2  //PB7
#if (leg_count==3)  
#define TIM4_CH3  //PB8
#endif

extern unsigned long cpu_run_time;

extern unsigned int  run_ms;


extern void IWDG_Init(u8 prer,u16 rlr);


extern void TIM2_Int_Init(u16 arr,u16 psc);
extern void TIM4_Init(u16 arr,u16 psc);


extern unsigned int time_get_ms(void);
extern unsigned int time_diff_ms(unsigned int lltimer_stamp);


#endif
