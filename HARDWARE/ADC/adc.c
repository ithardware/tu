#include "stm32f10x.h"
#include "adc.h"

__IO uint16_t Adc1_temp[5] = {0,0,0,0,0};
void ADC1_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure;//定义结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;				// ????0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;			// ????
  GPIO_Init(GPIOA, &GPIO_InitStructure);				//??A??
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//工作在单独模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//模数转换工作在扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//模数转换工作连续模式下
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//规定数据是右边对齐
	ADC_InitStructure.ADC_NbrOfChannel = 4;//
	ADC_Init(ADC1, &ADC_InitStructure);//保存设置

	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_55Cycles5);
	//设置指定ADC的规则组通道，设置他们的转化顺序和转化时间
	ADC_DMACmd(ADC1, ENABLE);//使能指定的ADC的DMA请求
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);//重置指定的ADc的校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1));//获取ADC重置校准寄存器的状态
	ADC_StartCalibration(ADC1);//开始指定ADC的校准状态
	while(ADC_GetCalibrationStatus(ADC1));//获取指定的校准器状态
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能指定的ADc的软件转换功能
}
//-----------------//
void ADC_DMA(void)
{
	DMA_InitTypeDef DMA_InitStructure; //定义结构体
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//??MDA
	
	DMA_DeInit(DMA1_Channel1);//将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;//外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Adc1_temp;//内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//外设作为数据传输的来源
	DMA_InitStructure.DMA_BufferSize = 4;//定义指定DMA通道的DMA缓存的大小，单位为数据单位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内设地址寄存器不变
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据宽度为16
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//内存数据宽度为16
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//工作在循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//通道具有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA通道没有设置为内存到内存传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);//初始化通道1的寄存器
	DMA_Cmd(DMA1_Channel1, ENABLE);//使能通道1
}

