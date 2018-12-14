#include "SIM800C.h"
//#include "led.h"
#include "string.h"
#include "timer.h"
//#include "crc32.h"
#include "delay.h"
#include "usart.h"
#include "control.h"

#define countof(a) (sizeof(a) / sizeof(*(a)))//计算数组内的成员个数
	
u8 tcp_udp_rx_data_flog = 0;
u8 first_tcp_udp_connect_flog = 0;

static u8 http_buff[HTTP_BUFF_LEN] = {0};
u8 http_buff_rx[HTTP_BUFF_LEN] = {0};


u8 http_get_buff[HTTP_GET_BUFF_LEN] = {0};
u16 http_get_rx_len = 0;



unsigned int sim800c_rx_stime = 0;



u8 	RSSI = 0;//信号强度
u8 CSQ_RSSI[2] = {0};//信号强度缓存数组




volatile u8	Heartbeat_Server = 0;//表示心跳正常
volatile u8 hbeaterrcnt=0;		 //心跳错误计数器,连续5次心跳信号无应答,则重新连接
volatile u8 connectsta=0;		 //0,正在连接;1,连接成功;2,连接关闭; 



u8	IMEI [15] = {0};//IMEI数组，不含M	
u8	IMEI_CRC16[2] = {0};//IMEI号的CRC检验值
u8	Token[16] = {0};//TOKEN数组
u16	RX_buf_lenth = 0;//接收字符串长度与协议有关
u16	RX_buf_sum = 0;
u8	RX_buf_sum_lastbit = 0;
u16 CHECK_SUM = 0;



volatile u32 RELAY_TIME = 0;//设置网络开锁时间，每次网络开锁时都会更新
volatile u32 RELAY_TIME_temp = 0;
volatile u32	Set_Relay_Open_Time = 0;//设置本地开锁时间，默认6秒，储存在EEPROM中。否者掉电后失效。


u8  RELAY_STA = 0;
u8	RELAY_NET_STA = 0;//用户回传服务器检测锁头是否已经关闭
u8  Order_old = 0;//上一次服务器的命令
u8  TIM_STA = 0;//定时器状态，当定时器1从运行到停止时，判断为关锁命令执行结束



u8	Rigster_ERR_Count = 0;//注册失败计数
u8  RELAY_Local_Open_STA = 0;//本地开锁状态，用于上报服务器。
u8  RELAY_Local_Close_STA = 0;//本地关锁状态，用于上报服务器。


u8  onceString[] = "523";//电源ADC电压PA3
u8	deviceName[] = "test";//设备名
u8  secret[] = "0123456789123456";//密码16位
u8  CRCbuf[16+3+4+15] = {0};
u8  sign[4] = {0};
u16 CRC_SIGN = 0;


char *Receive_Data = "0";
u16 timex=0;
u8 IMEI_CHECK = 1;//表示正确0表示错误	

void sim800c_usart_callback(u8 data);

void SIM800C_Init(void)//高电平开启
{
	
    GPIO_InitTypeDef  GPIO_InitStructure;
    uart3_init();//串口配置 
	  usart3_irq_callback = sim800c_usart_callback;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PA,PD端口时钟
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;				 //LED0-->PA.8 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.8
    SIM_PWR = 0;
    USART3_RX_STA=0;		//清零

}

//向sim900a发送命令
//cmd:发送的命令字符串(不需要添加回车了),当cmd<0XFF的时候,发送数字(比如发送0X1A),大于的时候发送字符串.
u8 sim900a_send_data_nck(u8 *cmd,u16 len)
{
    u8 res=0,i=0; 
    USART3_RX_STA=0;
    memset(USART3_RX_BUF,'\0',USART3_MAX_RECV_LEN);
    if(len>1)
    {
        for(i=0;i<len;i++)
        {
            usart_send_byte(SIM800C_USART,cmd[i]);
        }
    }
    return res;
} 



//向sim900a发送命令
//cmd:发送的命令字符串(不需要添加回车了),当cmd<0XFF的时候,发送数字(比如发送0X1A),大于的时候发送字符串.
//ack:期待的应答结果,如果为空,则表示不需要等待应答
//waittime:等待时间(单位:10ms)
//返回值:0,发送成功(得到了期待的应答结果)
//       1,发送失败
u8 sim900a_send_data_ack(u8 *cmd,u16 len,u8 *ack,u16 waittime)
{
    u8 res=0; 
		u16 i = 0;
    USART3_RX_STA=0;
    if(USART3_RX_STA != 0)
    {
        SIM8XX_User_Receive();
    }
    memset(USART3_RX_BUF,'\0',USART3_MAX_RECV_LEN);
	
    for(i=0;i<len;i++)
    {
        usart_send_byte(SIM800C_USART,cmd[i]);
    }

    if(ack&&waittime)		//需要等待应答
    {
        while(--waittime)	//等待倒计时
        {
            delay_ms(10);//重点函数
					
					  if(sim800c_rx_stime != 0)
						{
						    if(time_diff_ms(sim800c_rx_stime) > SIM800C_RX_TIMEOUT)
                {
                	  sim800c_rx_stime = 0;
                    USART3_RX_STA|=(unsigned int)1<<15; //标记接收完成
                }
						}
            
					
            if(USART3_RX_STA&0X8000)//接收到期待的应答结果
            {
							  USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//添加结束符
                if(strstr((const char*)USART3_RX_BUF,"CIPRXGET"))
                {
				        	  tcp_udp_rx_data_flog = 1;
                		DEBUG("TCP or UDP receive data\r\n");
                }
								
                if(sim900a_check_cmd(ack))
                {
                    USART3_RX_STA=0;
                    break;//得到有效数据 
                }
            } 
        }
				
        if(waittime==0)
				{
				    if(USART3_RX_STA == 0)
				    {
				        res = 2;
				    }
						else
						{
						    res=1;
						}
				} 
    }
    return res;
} 


u8 read_imei(u8 *data)
{
    u8 res = 0,i = 0;
    res = sim900a_send_data_ack("AT+GSN\r\n", 8, "OK", 10);
	  if(res == 0)
		{
		    while(USART3_RX_BUF[2+i] != '\r')
			  {
				    data[i] = USART3_RX_BUF[2+i];
					  i++;
				}
		}
	  return i;
	
}

void SIM_PWR_Open(void)
{
	  u8 i = 0, res = 0;
	  usart_send_byte(SIM800C_USART,0x1B);//防止上次对话异常关闭，卡死在发送状态
	  usart_send_byte(SIM800C_USART,0x1B);//防止上次对话异常关闭，卡死在发送状态，第二次发送，第一次可能同步串口波特率
	  usart_send_byte(SIM800C_USART,0x1B);  
  	while(1)
		{
	      if(i > 5)
		    {
					  DEBUG("SIM800C start up\r\n");
					  i = 0;
            SIM_PWR = 1;//输入高电平
            delay_ms(500);
					  delay_ms(500);
					  delay_ms(500);
            SIM_PWR = 0;//输入低电平
					  delay_ms(500);
					  delay_ms(500);
					  delay_ms(500);
		    }
				else
				{
					  res = sim900a_send_data_ack("AT\r\n",4,"OK",10);
				    if(res == 1)  //有数据但是返回值不对
						{

						}
						else if(res == 2) //没有返回数据，SIM800可能关机
						{
						    i++;
						}
						else  //正确，说明sim800正常，开机
						{
							  delay_ms(200);
						    break;
						}
						delay_ms(200);
						delay_ms(200);
						delay_ms(200);
						delay_ms(200);
						delay_ms(200);
				}
		}

		if(sim900a_send_data_ack("AT\r\n",4,"OK",500) == 0)//自带1S延时
		{
			  DEBUG("SIM800C open system ok\r\n");
		}
	  else
		{
			  DEBUG("SIM800C open file\r\n");
		}
		delay_ms(200);
    if(sim900a_send_data_ack("ATE0&W\r\n",4,"ATE0",100) == 0)//自带1S延时
    {
    	  DEBUG("ATE close success\r\n");
    }
    else
    {
        DEBUG("ATE close file\r\n");
    }
		delay_ms(200);
		//回显关闭成功后，第二条指令才生效
		if(sim900a_send_data_ack("AT\r\n",4,"OK",100) == 0)//自带1S延时
		{
			
		}
	  else
		{
			
		}
		delay_ms(200);
		off_line();
		delay_ms(200);
		//设置TCP输出方式为手动获取
		if(sim900a_send_data_ack("AT+CIPRXGET=1\r\n",15,"OK",100) == 0)//自带1S延时
		{
			  DEBUG("手动获取TCP数据修改成功\r\n");
		}
	  else
		{
			  DEBUG("手动获取TCP数据修改失败\r\n");
		}

}

void SIM_PWR_Close(void)
{
		usart_send_byte(SIM800C_USART,0x1B);
		delay_ms(100);
		if(sim900a_send_data_ack("AT\r\n",4,"OK",100) == 1)//自带1S延时
		{
			  //SIM868无响应
		}
	  else
		{
				//SIMXX已开机
				SIM_PWR = 1;//输入高电平
			  delay_ms(5000);
			  SIM_PWR = 0;//输入低电平
		}
}

//sim900a发送命令后,检测接收到的应答
//str:期待的应答结果
//返回值:0,没有得到期待的应答结果
//    其他,期待应答结果的位置(str的位置)
u8* sim900a_check_cmd(u8 *str)
{
    char *strx=0;

    if(USART3_RX_STA&0X8000)		//接收到一次数据了
    { 
        USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//添加结束符
        strx=strstr((const char*)USART3_RX_BUF,(const char*)str);
    } 
    return (u8*)strx;
}




void sim800c_usart_callback(u8 data)
{
    if(USART3_RX_STA<USART3_MAX_RECV_LEN)		//还可以接收数据
    {
				sim800c_rx_stime = time_get_ms();
        USART3_RX_BUF[USART3_RX_STA++]=data;		//记录接收到的值	 
    }
    else 
    {
    	USART3_RX_STA|=(unsigned int)1<<15;					//强制标记接收完成
    } 
}



//将1个字符转换为16进制数字
//chr:字符,0~9/A~F/a~F
//返回值:chr对应的16进制数值
u8 sim900a_chr2hex(u8 chr)
{
	if(chr>='0'&&chr<='9')return chr-'0';
	if(chr>='A'&&chr<='F')return (chr-'A'+10);
	if(chr>='a'&&chr<='f')return (chr-'a'+10); 
	return 0;
}
//将1个16进制数字转换为字符
//hex:16进制数字,0~15;
//返回值:字符
u8 sim900a_hex2chr(u8 hex)
{
	if(hex<=9)return hex+'0';
	if(hex>=10&&hex<=15)return (hex-10+'A'); 
	return '0';
}

u8 check_tcp_udp_connet(void)
{
	  u8 res = 0;

    if(sim900a_send_data_ack("AT+CIPSTATUS\r\n", 14, "CONNECT", 10) == 0)//发送数据
    {
        res = 1;
    }
    else
    {
        res = 0;
    }
		return res;
}

u8 connect_tcp_udp(LINK_PARA link)
{
	  int i = 0;
	  u8 res = 0;
	  u8 temp_str[64] = {0};
		

    if(check_tcp_udp_connet() == 1)
		{
		    off_line();
		}
		
    if(link.connect_type == 0)
    {
        res = sprintf((char *)temp_str,"AT+CIPSTART=\"%s\",\"%s\",\"%d\"\r\n", "TCP",
        	                        link.ip_addr, link.port);
    }
    else if(link.connect_type == 1)
    {
        res = sprintf((char *)temp_str,"AT+CIPSTART=\"%s\",\"%s\",\"%d\"\r\n", "UDP",
        	                        link.ip_addr, link.port);
    }
		else
		{
		    res = 0;
		}
    
    if(res > 0)
    {
        if(sim900a_send_data_ack(temp_str, res, "OK", 1000) == 0)//发送数据
        {
        	  res = 1;
	          while(1)
		        {
							  if(check_tcp_udp_connet() == 1)
								{
								    break;
								}
		        		i++;
		        		if (i>=4)
		        		{
		        			  res = 0;
		        			  break;
		        		}
		        }
        }
        else
        {
            res = 0;
        }
    }
		if(res == 0)
		{
        off_line();
		}
		else
		{
		    first_tcp_udp_connect_flog = 1;
		}
		
		return (u8)res;
}

u16 tcp_udp_rx_data(u8 *data)
{
	  u16 res = 0,i = 0,j = 0;
    if(sim900a_send_data_ack("AT+CIPRXGET=2,1460\r\n",20,"OK",500)==0)		//发送数据
    {
			  while(USART3_RX_BUF[15+j] != ',')
				{
				    res = res*10 + (USART3_RX_BUF[15+j] - 0x30);
					  j++;
				}
				if(res != 0)
				{
				    for(i=0; i<res; i++)
					  {
                data[i] = USART3_RX_BUF[19+i+j];
						}
				}
        
    }
    else 
    {

    }
		
		return res;
}


int tcp_udp_send(u8 *data, u16 len)
{
	  u8 cmd = 0;
		int res = 0;
		
		if(res >= 0)
		{
	      //
		    if(sim900a_send_data_ack("AT+CIPSEND\r\n",12,">",500)==0)		//发送数据
		    {
		    		sim900a_send_data_nck(data, len);
		    		delay_ms(100);
		    		cmd = 0x1A;
		    		if(sim900a_send_data_ack(&cmd,1,"SEND OK",5000)==0)
		    		{
            
		    		}                
		    		else
		    		{
		    				res = -2;
		    		}                           
		    		delay_ms(200); 
		    }
		    else 
		    {
		    		cmd = 0x1B;
		    		sim900a_send_data_ack(&cmd,1,0,0);	//ESC,取消发送 
		    		res = -3; 
		    }
		}
		
		return res;
		
}

u8 off_line(void)
{
	  u8 res = 0;
    if(sim900a_send_data_ack("AT+CIPCLOSE\r\n",13,"OK",50) == 0)//
    {
    	  res = 1;
    }
    else
    {
        res = 0;
    }
    if(sim900a_send_data_ack("AT+SAPBR=0,1\r\n",14,"OK",10) == 0)//关闭HTTP承载
    {
        //success
			  res = 1;
    }
    else
    {		
        res = 0;
    }
    if(sim900a_send_data_ack("AT+CIPSHUT\r\n",12,"OK",50) == 0)//发送数据
    {
    	  res = 1;
    }
    else
    {
        res = 0;
    }
		return (u8)res;
}

int init_http(void)
{
	  int res = 0;
    if(sim900a_send_data_ack("AT+CGATT?\r\n",11,"+CGATT",200) == 0)//发送数据
    {
			  if(USART3_RX_BUF[10] == 0x31)
				{
				    res = 1;
				}
				else
				{
				    res = (char)-1;
				}
    }
		
		if(res > 0)
		{
        if(sim900a_send_data_ack("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n",31,"\r\n",100) == 0)
        {
            //success
					  res = 1;
        }
				else
				{
				    res = (char)-2;
				}
		}
		
		if(res > 0)
		{
        if(sim900a_send_data_ack("AT+SAPBR=3,1,\"APN\",\"CMNET\"\r\n",28,"\r\n",100) == 0)
        {
            //success
					  res = 1;
        }
				else
				{
				    res = (char)-3;
				}
		}
		
		if(res > 0)
		{
        if(sim900a_send_data_ack("AT+SAPBR=1,1\r\n",14,"OK",100) == 0)
        {
            //success
					  res = 1;
        }
				else
				{
				    res = (char)-4;
				}
		}
	  return res;
}

int http_get(LINK_PARA link, HTTP_PARA http_para, u8 *in_data)
{
    int res = 0, i = 0;  
    int status_code = 0;
    int http_rx_len = 0;
    memset(http_buff,'\0',HTTP_BUFF_LEN);
    
    init_http();
	
    if(sim900a_send_data_ack("AT+HTTPTERM\r\n",13,"OK",100) == 0)//关闭HTTP服务
    {
        //success
			  res = 1;
    }
    else
    {		
        res = (char)-8;
    }
	
    if(sim900a_send_data_ack("AT+HTTPINIT\r\n",13,"OK",100) == 0)
	  {
	      //success
			  res = 1;
	  }
    else
    {
        res = (char)-1;
    }
		
    if(res > 0)
    {
        if(link.connect_type == 0)//TCP连接
        {
    		memset(http_buff,'\0',HTTP_BUFF_LEN);
    		if((u32)in_data == (u32)(para_value.modbus_qry_get_para.buff))
    		{
    			res = sprintf((char *)http_buff,"AT+HTTPPARA=\"URL\",\"%s:%d%s\"\r\n", link.ip_addr,link.port, http_para.url);
    		}
    		else
    		{
    			res = sprintf((char *)http_buff,"AT+HTTPPARA=\"URL\",\"%s:%d%s?%s\"\r\n", link.ip_addr,link.port, http_para.url, in_data);
    		}
    
    		//DEBUG("%s\r\n",http_buff);
    		if(sim900a_send_data_ack(http_buff, res, "OK", 100) == 0)
    		{
    			res = 1;
    		}
    		else
    		{
    			res = (char)-2;
    		}
    
    		if((strlen((char *)http_para.head) != 0) && (res > 0))
    		{
    			memset(http_buff,'\0',HTTP_BUFF_LEN);
    			res = sprintf((char *)http_buff,"AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n",http_para.head);
    			//DEBUG("%s\r\n",http_buff);
    			if(sim900a_send_data_ack(http_buff, res, "OK", 100) == 0)
    			{
    				res = 1;
    			}
    			else
    			{
    				res = (char)-2;
    			}
    		}
    	}
    	else
    	{
    		res = (char)-3;
    	}
    }

    if(res > 0)
    {
        if(sim900a_send_data_ack("AT+HTTPACTION=0\r\n",17,"OK",300) == 0)//等待3S中GET服务器数据,但是获取数据可能存在延时
        {
            //success
    	    res = 1;
    	    //这里不发送，等待回应
    	    if(sim900a_send_data_ack("",0,"+HTTPACTION:",500) == 0)
    	    {
    	        res = 1;
    	    	status_code = (USART3_RX_BUF[17]-0x30)*100 + (USART3_RX_BUF[18]-0x30)*10 + (USART3_RX_BUF[19]-0x30);
    	    	i=0;
    	    }
    	    else
    	    {
    	        res = (char)-4;
    	    }
        }
        else
        {
            res = (char)-5;
        }
    }
		
#if 0
    //输出http的head
    if((res > 0) && ((http_para.out_head == 1) || (status_code != 200)))
    {
        if(sim900a_send_data_ack("AT+HTTPHEAD\r\n",13,"+HTTPHEAD",300) == 0)
        {
            http_rx_len = 0;
            while(1)
            {
            	  if(USART3_RX_BUF[13+i] == 0x0d)
            	  {
            	      break;
            	  }
            	  else
            	  {
            	  	  if(USART3_RX_BUF[13+i] == ' ')
            	  	  {
            	  	      i++;
            	  	  }
            	  	  else
            	  	  {
            	  	      http_rx_len = http_rx_len*10 + (USART3_RX_BUF[13+i] - 0x30);
            	  	  }
            	  }
            	  i++;
            }
            usart_send_str(HOSTIF_USART, &USART3_RX_BUF[i+15], http_rx_len);
            res = 1;
        }
        else
        {
            res = (char)-2;
        }
    }
#endif
    //输出http的body
    if((res > 0) && (status_code == 200))
    {
        if(sim900a_send_data_ack("AT+HTTPREAD\r\n",13,"+HTTPREAD",300) == 0)
        {
            http_rx_len = 0;
            while(1)
            {
                if(USART3_RX_BUF[13+i] == 0x0d)
                {
                    break;
                }
                else
                {
                    if(USART3_RX_BUF[13+i] == ' ')
                    {
                        i++;
                    }
                    else
                    {
                        http_rx_len = http_rx_len*10 + (USART3_RX_BUF[13+i] - 0x30);
                    }
                }
                i++;
            }

            //usart_send_str(HOSTIF_USART, &USART3_RX_BUF[i+15], http_rx_len);			
			memcpy((char *)http_get_buff, (char *)(&USART3_RX_BUF[i+15]), http_rx_len);
            http_get_rx_len = http_rx_len;
						
            res = 1;
        }
        else
        {
            res = (char)-2;
        }
    }
		
    if(sim900a_send_data_ack("AT+SAPBR=0,1\r\n",14,"OK",100) == 0)//关闭HTTP承载
    {
        //success
	    //res = 1;
    }
    else
    {		
        //res = (char)-7;
    }
    delay_ms(100);
    if(sim900a_send_data_ack("AT+HTTPTERM\r\n",13,"OK",100) == 0)//关闭HTTP服务
    {
        //success
	    //res = 1;
    }
    else
    {		
        //res = (char)-8;
    }
	  DEBUG("\r\nGET Result %d\r\n",res);
    return res;
}

int http_post(LINK_PARA link, HTTP_PARA http_para, u8 *in_data, u16 len)
{
    int res = 0, i = 0;  
    int status_code = 0;
    int http_rx_len = 0;
    //u32 
    memset(http_buff,'\0',HTTP_BUFF_LEN);
    //初始化http
    init_http();
	
    if(sim900a_send_data_ack("AT+HTTPINIT\r\n",13,"OK",100) == 0)
    {
        //success
    	res = 1;
    }
    else
    {
        res = (char)-1;
    }
	
    //设置url
    if(res > (int)0)
    {
        if(link.connect_type == 0)//TCP连接
        {
            if(sim900a_send_data_ack("AT+HTTPPARA=\"CID\",1\r\n", 21, "OK", 100) == 0)
            {
                res = 1;
            }
            else
            {
                res = (char)-2;
            }
            res = sprintf((char *)http_buff,"AT+HTTPPARA=\"URL\",\"%s:%d%s\"\r\n", link.ip_addr,link.port, http_para.url);
            //DEBUG("%s\r\n",http_buff);
            if(sim900a_send_data_ack(http_buff, res, "OK", 100) == 0)
            {
                res = 1;
            }
            else
            {
                res = (char)-3;
            }
        }
        else
        {
            res = (char)-4;
        }
    }
		
    //设置HTTP的HEAD
    if((strlen((char *)http_para.head) != 0) && (res > (int)0))
    {
        memset(http_buff,'\0',HTTP_BUFF_LEN);
        res = sprintf((char *)http_buff,"AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n",http_para.head);
        //DEBUG("%s\r\n",http_buff);
        if(sim900a_send_data_ack(http_buff, res, "OK", 100) == 0)
        {
            res = 1;
        }
        else
        {
            res = (char)-2;
        }
    }
		
	//设置发送的数据
	if(res > (int)0)
	{
	    memset(http_buff,'\0',HTTP_BUFF_LEN);
	    res = sprintf((char *)http_buff,"AT+HTTPDATA=%d,10000\r\n", (len));
      //DEBUG("%s",http_buff);
	    if(sim900a_send_data_ack(http_buff, res,  "DOWNLOAD", 1000) == 0)
	    {
	        res = 1;
	    }
	    else
	    {
	        res = (char)-6;
	    }
	    delay_ms(100);
	    if(sim900a_send_data_ack(in_data, len, "OK", 5000) == 0)
	    {
	        res = 1;
	    }
	    else
	    {
	        res = (char)-7;
	    }

	}
    //设置post发送数据
	if(res > (int)0)
	{
	    //设置为post模式
        if(sim900a_send_data_ack("AT+HTTPACTION=1\r\n",17,"OK",300) == 0)
        {
            //success
			//这里不发送，等待回应
		    res = 1;
		    if(sim900a_send_data_ack("",0,"+HTTPACTION:",500) == 0)
		    {
		        res = 1;
		    	status_code = (USART3_RX_BUF[17]-0x30)*100 + (USART3_RX_BUF[18]-0x30)*10 + (USART3_RX_BUF[19]-0x30);
		    	i=0;
		    	http_rx_len = 0;
		    	while(1)
		    	{
		    		  if(USART3_RX_BUF[20+i] == '\r')
		    			{
		    				//DEBUG("status_code is %d\r\n",status_code);
		    				//DEBUG("http_rx_len is %d\r\n",http_rx_len);
		    			    break;
		    			}
		    			else
		    			{
		    				  if((USART3_RX_BUF[20+i] == ' ') ||(USART3_RX_BUF[20+i] == ','))
		    				  {
		    				      i++;
		    				  }
		    				  else
		    				  {
		    				      http_rx_len = http_rx_len*10 + (USART3_RX_BUF[20+i] - 0x30);
		    				  }
		    			}
		    			i++;
		    	    
		    	}
		    }
		    else
		    {
		        res = (char)-8;
		    }
					  
        }
        else
        {
            res = (char)-9;
        }
	}
	//输出http的head
#if 0
	if((http_para.out_head == 1) || (status_code != 200))
	{
        if(sim900a_send_data_ack("AT+HTTPHEAD\r\n",13,"+HTTPHEAD",300) == 0)
        {
            http_rx_len = 0;
			//这里处理可能有问题
            while(1)
            {
                if(USART3_RX_BUF[13+i] == 0x0d)
                {
                    break;
                }
                else
                {
                    if(USART3_RX_BUF[13+i] == ' ')
                    {
                        i++;
                    }
                    else
                    {
                        http_rx_len = http_rx_len*10 + (USART3_RX_BUF[13+i] - 0x30);
                    }
                }
                i++;
            }
            usart_send_str(HOSTIF_USART, &USART3_RX_BUF[i+15], http_rx_len);
            res = 1;
        }
        else
        {
            res = (char)-2;
        }
	}
#endif
		
		
    if(sim900a_send_data_ack("AT+SAPBR=0,1\r\n",14,"OK",10) == 0)//关闭HTTP承载
    {
        //success
	    res = 1;
    }
    else
    {
        res = (char)-7;
    }
		
    if(sim900a_send_data_ack("AT+HTTPTERM\r\n",13,"OK",10) == 0)//关闭HTTP服务
    {
        //success
	    res = 1;
    }
    else
    {		
        res = (char)-8;
    }
	  DEBUG("\r\n POST Result %d\r\n",res);
    return res;
}


u8 SIM8XX_User_Receive(void)
{

    u8 i = 0;
    u8 m = 0;
    u8 cmd = 0;

    if(sim800c_rx_stime != 0)
    {
        if(time_diff_ms(sim800c_rx_stime) > SIM800C_RX_TIMEOUT)
        {
        	sim800c_rx_stime = 0;
            USART3_RX_STA|=(unsigned int)1<<15; //标记接收完成
        }
    }
	

    if(USART3_RX_STA&0X8000)//串口接收函数
    {
       //	USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;	//添加结束符 
    
      /*SIM800C关键反馈函数*/
        if(hbeaterrcnt)//需要检测心跳应答
        {
            if(strstr((const char*)USART3_RX_BUF,"SEND OK"))
            {
            	hbeaterrcnt=0;//心跳正常
            }
        }
        
        //接收处理函数，连接异常关闭，重启TCP链接，关键词"CLOSE"。
        Receive_Data = strstr((const char*)USART3_RX_BUF,"CLOSED");
        if(Receive_Data)
        {
            return 110;//异常关闭
        }
        /*SIM800C关键反馈函数*/
        //说明TCP或者UDP有数据返回来了，需要接收
        Receive_Data = strstr((const char*)USART3_RX_BUF,"CIPRXGET");
        if(Receive_Data)
        {
					tcp_udp_rx_data_flog = 1;
					DEBUG("TCP 或UDP 接收到了数据\r\n");
        }
    }
    
    USART3_RX_STA=0;
    return 0;
}

