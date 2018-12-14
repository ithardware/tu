#include "pwr.h"
#include "control.h"
#include "usart.h"
#include "24cxx.h"
#include "mortor.h"
#include "save_data.h"

u8 diaodianceshi_count = 10;

void PVD_Config(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    EXTI_InitStructure.EXTI_Line = EXTI_Line16;														//PVD连接中断线16
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 									//使能中断模式
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;							//下降沿触发
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;								//上升沿触发
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;			//电压上升或下降越过设定阀值时都产生中断
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 														//使能中断线
    EXTI_Init(&EXTI_InitStructure); 																			//初始化

    PWR_PVDCmd(ENABLE); 																									//使能PVD
    PWR_PVDLevelConfig(PWR_PVDLevel_2V9);																	//设定监控阀值
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	
		NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	
}

void PVD_IRQHandler(void)
{
//		u8 dssdsdfs = 0;
		EXTI_ClearITPendingBit(EXTI_Line16);
		
		if(PWR_GetFlagStatus(PWR_FLAG_PVDO)) 
		{
				mortor_stop();
				
//			if(diaodianceshi_count == 1)
//				AT24CXX_WriteOneByte(14,0);
//			else
			{
				//printf("11111:%d",motor_running);
				if((motor_running == 1) || (diaodian == 1))
					AT24CXX_WriteOneByte(0xff,1);	
				
			}
				


//				printf("%d",dssdsdfs);
////				puts("Has entered the interrupt!!!");
				PWR_ClearFlag(PWR_FLAG_PVDO);
		}
}
 
