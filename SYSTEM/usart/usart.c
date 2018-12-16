#include "sys.h"
#include "usart.h"	  
#include "save_data.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif




//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
    if(para_value.debug_mode == 1)
    {
        while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
          USART2->DR = (u8) ch;      
    }
		else
		{
        while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
          USART1->DR = (u8) ch;   
		}
    
    return ch;
}
#endif 


/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 

//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	

//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  

void (*usart3_irq_callback) (u8 data); 

uint8_t USART1_TX_BUF[SENDBUFF_SIZE]; 
uint8_t USART2_TX_BUF[SENDBUFF_SIZE]; 


unsigned char USART1_RX_BUF[USART1_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
unsigned char USART2_RX_BUF[USART2_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.


//串口发送缓存区 	
unsigned char USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//发送缓冲,最大USART2_MAX_SEND_LEN字节
								//如果使能了接收   	  
//串口接收缓存区 	
unsigned char USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//接收缓冲,最大USART2_MAX_RECV_LEN个字节.



CIRCULATION_BUFF usart1_rx_cir_buff;
CIRCULATION_BUFF usart2_rx_cir_buff;




void USART_RX_DMA_Config(DMA_Channel_TypeDef* DMAy_Channelx, u32 p_addr, u32 m_addr, u16 len)
{
		DMA_InitTypeDef DMA_InitStructure;
	
	  if(((u32)DMAy_Channelx) < (AHBPERIPH_BASE + 0x0400))
		{
		    /*开启DMA时钟*/
		    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
		}
		else
		{
		    /*开启DMA时钟*/
		    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	
		}

		/*设置DMA源：串口数据寄存器地址*/
		DMA_InitStructure.DMA_PeripheralBaseAddr = p_addr;	   

		/*内存地址(要传输的变量的指针)*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)m_addr;

		/*方向：从外设到内存*/		
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	

		/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/	
		DMA_InitStructure.DMA_BufferSize = len;

		/*外设地址不增*/	    
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 

		/*内存地址自增*/
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	

		/*外设数据单位*/	
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

		/*内存数据单位 8bit*/
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 

		/*DMA模式：不断循环*/
	  //DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 

		/*优先级：高级*/	
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  

		/*禁止内存到内存的传输	*/
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

		/*配置DMA1的4通道*/		   
		DMA_Init(DMAy_Channelx, &DMA_InitStructure); 	  
		
		/*使能DMA*/
		DMA_Cmd (DMAy_Channelx,ENABLE);		
}


void USART_TX_DMA_Config(DMA_Channel_TypeDef* DMAy_Channelx, u32 p_addr, u32 m_addr)
{
		DMA_InitTypeDef DMA_InitStructure;
		//NVIC_InitTypeDef NVIC_InitStructure; 
	
	  if(((u32)DMAy_Channelx) < (AHBPERIPH_BASE + 0x0400))
		{
		    /*开启DMA时钟*/
		    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
		}
		else
		{
		    /*开启DMA时钟*/
		    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	
		}

		/*设置DMA源：串口数据寄存器地址*/
		DMA_InitStructure.DMA_PeripheralBaseAddr = p_addr;	   

		/*内存地址(要传输的变量的指针)*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)m_addr;

		/*方向：从内存到外设*/		
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	

		/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/	
		DMA_InitStructure.DMA_BufferSize = 0;

		/*外设地址不增*/	    
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 

		/*内存地址自增*/
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	

		/*外设数据单位*/	
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

		/*内存数据单位 8bit*/
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 

		/*DMA模式：发送完一次就停止*/
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
		//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 

		/*优先级：中*/	
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  

		/*禁止内存到内存的传输	*/
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

		/*配置DMA1的4通道*/		   
		DMA_Init(DMAy_Channelx, &DMA_InitStructure); 	  
		
		/*使能DMA*/
		DMA_Cmd (DMAy_Channelx,ENABLE);		
}




  
void uart1_init(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  //NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	//NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	
	/* Write to DMAy Channelx CMAR */
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	
  USART_Cmd(USART1, ENABLE);                    //使能串口1 
	
	
	usart1_rx_cir_buff.busy = 0;
	usart1_rx_cir_buff.remain = USART1_REC_LEN;
	usart1_rx_cir_buff.buff = USART1_RX_BUF;
	usart1_rx_cir_buff.rptr = USART1_RX_BUF;
	usart1_rx_cir_buff.wptr = USART1_RX_BUF;
	usart1_rx_cir_buff.len  = USART1_REC_LEN;
	
	USART_TX_DMA_Config(USART1_TX_DMA_CH, USART1_DR_Base, (u32)USART1_TX_BUF);
	USART_RX_DMA_Config(USART1_RX_DMA_CH, USART1_DR_Base, (u32)USART1_RX_BUF, USART1_REC_LEN);

}

void uart2_init(USART_PARA para)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  //NVIC_InitTypeDef NVIC_InitStructure;
	   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 
    
	  //USART2_TX   GPIOA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.2
     
    //USART2_RX	  GPIOA.3初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.3 
    
    //Usart2 NVIC 配置
    //NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
	  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	  //NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	  
    
    USART_InitStructure.USART_BaudRate = (para.baurd);//串口波特率
		
	  if(para.data_bits == 9)
	  {
	      USART_InitStructure.USART_WordLength = USART_WordLength_9b;  
	  }
	  else
	  {
	      USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
	  }
    
	  if(para.stop == 3)
	  {
	      USART_InitStructure.USART_StopBits = USART_StopBits_2;  
	  }
	  else if(para.stop == 2)
	  {
	      USART_InitStructure.USART_StopBits = USART_StopBits_1_5;  
	  }
	  else if(para.stop == 0)
	  {
	      USART_InitStructure.USART_StopBits = USART_StopBits_0_5;  
	  }
	  else
	  {
	      USART_InitStructure.USART_StopBits = USART_StopBits_1;  
	  }
    
	  if(para.parity == 2)
	  {
	      USART_InitStructure.USART_Parity = USART_Parity_Odd;  
	  }
	  else if(para.parity == 1)
	  {
	      USART_InitStructure.USART_Parity = USART_Parity_Even;  
	  }
	  else
	  {
	      USART_InitStructure.USART_Parity = USART_Parity_No;
	  }
    
	  if(para.flow_ctr == 2)
	  {
	      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;  
	  }
	  else if(para.flow_ctr == 1)
	  {
	      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;  
	  }
	  else
	  {
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
	  }
    
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
    USART_Init(USART2, &USART_InitStructure);   
	  
	  /* Write to DMAy Channelx CMAR */
	  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	  
    USART_Cmd(USART2, ENABLE);                    //使能串口1 
	  
	  
	  usart2_rx_cir_buff.busy = 0;
	  usart2_rx_cir_buff.remain = USART2_REC_LEN;
	  usart2_rx_cir_buff.buff = USART2_RX_BUF;
	  usart2_rx_cir_buff.rptr = USART2_RX_BUF;
	  usart2_rx_cir_buff.wptr = USART2_RX_BUF;
	  usart2_rx_cir_buff.len  = USART2_REC_LEN;
	  
	  USART_TX_DMA_Config(USART2_TX_DMA_CH, USART2_DR_Base, (u32)USART2_TX_BUF);
	  USART_RX_DMA_Config(USART2_RX_DMA_CH, USART2_DR_Base, (u32)USART2_RX_BUF, USART2_REC_LEN);

}


void uart3_init(void)//串口初始化函数
{  

    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);// GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    
    USART_DeInit(USART3);                                //复位串口1
     //USART2_TX   PB10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;            //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	     //复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);               //初始化PA2
     
    //USART2_RX	  PB11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;            //PA.3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);               //初始化PA3
    
    USART_InitStructure.USART_BaudRate = 115200;                                    //一般设置为9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	               //收发模式
     
    USART_Init(USART3, &USART_InitStructure); //初始化串口	3
    USART_Cmd(USART3, ENABLE);                                         //使能串口 
    	
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断   
    
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
		
    USART3_RX_STA=0;		//清零
	 
}	





#if 0
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
}

#endif


u16 USART3_RX_STA=0;   	 


void USART3_IRQHandler(void)//STM32
{
    u8 res;	    
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//接收到数据
    {
        res =USART_ReceiveData(USART3);		
        if(usart3_irq_callback != NULL)
				{
				    (*usart3_irq_callback)(res);
				}					
        
    }  											 
}   



void usart_send_byte(USART_TypeDef * USARTx, u8 str)
{
    while((USARTx->SR&0X40)==0);
		USARTx->DR = str;  
}


void usart_send_str(USART_TypeDef * USARTx, u8 * str, u16 len)
{
    u16 i = 0;
    
	  for(i=0; i<len; i++)
	  {
		    while((USARTx->SR&0X40)==0);//循环发送,直到发送完毕  
        USARTx->DR = (u8) *str++; 				
		}
}




//开启一次DMA传输
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx, unsigned char send_len)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //关闭USART1 TX DMA1 所指示的通道      
 	DMA_SetCurrDataCounter(DMA_CHx, send_len);//DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA_CHx, ENABLE);  //使能USART1 TX DMA1 所指示的通道 
}	  


/*******************************************************************************  
* Function Name  : USART1_dma_send  
* Description    : This function DMA send data.  
* Input          : DMAy_Channelx is dma cm, str is send content, len is long
* Output         : None  
* Return         : None  
*******************************************************************************/  
u16 USART_dma_send(DMA_Channel_TypeDef* DMAy_Channelx, u8 *str, u16 len)
{
    u16 i=0;
    
    if(len > SENDBUFF_SIZE)
    {
        return 1;
    }
    if(DMAy_Channelx == USART1_TX_DMA_CH)
		{
        for(i=0; i<len; i++)
        {
            USART1_TX_BUF[i] = *str++;
		    }
				while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	  }
		else if(DMAy_Channelx == USART2_TX_DMA_CH)
		{
        for(i=0; i<len; i++)
        {
            USART2_TX_BUF[i] = *str++;
		    }
				while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	  }	
   
    while(DMA_GetCurrDataCounter(DMAy_Channelx));
    MYDMA_Enable(DMAy_Channelx, len);
	  return 0;
}


u16 circulation_buff_available(DMA_Channel_TypeDef* DMAy_Channelx, CIRCULATION_BUFF cir_buff)
{
	  u16 res = 0;
	  cir_buff.remain = DMA_GetCurrDataCounter(DMAy_Channelx);
	  cir_buff.wptr = cir_buff.buff + (cir_buff.len - cir_buff.remain);
    if(cir_buff.wptr >  cir_buff.rptr)
    {
			  res = (u32)cir_buff.wptr - (u32)cir_buff.rptr;
    }
    else if(cir_buff.wptr <  cir_buff.rptr)
    {
			  res = cir_buff.len - ((u32)cir_buff.rptr - (u32)cir_buff.wptr);
    }
		else
		{
		    res = 0;
		}
		return res;
}

int circulation_buff_read(DMA_Channel_TypeDef* DMAy_Channelx, CIRCULATION_BUFF* cir_buff,unsigned char* read_buff,unsigned int len)
{
	  unsigned int i = 0,rlen = 0;
	
	  cir_buff->remain = DMA_GetCurrDataCounter(DMAy_Channelx);
	  cir_buff->wptr = cir_buff->buff + (cir_buff->len - cir_buff->remain);
	  if(cir_buff->busy == 0)
		{
		    cir_buff->busy = 1;
        for(i=0; i< len; i++)
	      {
			      //if(circulation_buff_available(cir_buff))  
					  if(cir_buff->rptr !=  cir_buff->wptr)
				    {
				        read_buff[i] = *cir_buff->rptr++;
							  rlen++;
							  
		            if(cir_buff->rptr == (cir_buff->buff+cir_buff->len))
		            {
		                cir_buff->rptr = cir_buff->buff;
		            }
				    }
						else
						{
							  cir_buff->busy = 0;
						    return rlen;
						}
		    }
		    cir_buff->busy = 0;
			  return rlen;

		}
		else
		{
		    return CIRCULATION_BUFF_BUSY;
		}

}





