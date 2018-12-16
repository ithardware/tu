#include "hostif.h"
#include "save_data.h"
#include "timer.h"
#include "delay.h"
#include "string.h"
#include "SIM800C.h"
#include "control.h"


u8 host_rx_buff[HOST_RX_BUFF_LEN];
u16 host_rx_len = 0;

u8 host_tx_buff[HOST_TX_BUFF_LEN];

u8 dtu_hw_version[] = "V1.2";
u8 dtu_sw_version[] = "V2.1.3 Test GET ONLY";


APN_GATEWAT temp_apn_para;

u8 test_gsm(u8 cmd);


u8 assic_hex_tab[16] = {0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x41,0x42,0x43,0x44,0x45,0x46};

static u8 temp_str[130] = {0};
static int temp[5] = {0};

//sim900a发送命令后,检测接收到的应答
//str:期待的应答结果
//返回值:0,没有得到期待的应答结果
//    其他,期待应答结果的位置(str的位置)
//u8 SIM8XX_check_cmd(u8 *str)
//{
//		strx=strstr((const char*)USART_RX_BUF,(const char*)str);
//}
//系统命令
u8 *hostif_cmd_tab[]=
{
	"WKMODE",	
	"GPRSMODE",
	"DBGMODE",
	"DTUID",
	"UART",
	"DFI",
	"HBTIME",
	"HBHEAD",
	"REGPKG",
	"REGHEAD",
	"SERVER1",
	"SERVER2",
	"IDLTIME",
	"APN",
	"HTTPS",
	"HTTPHD",
	"QRYPTM",
	"QRYPOST",
	"QRYPPA",
	"QRYGTM",
	"QRYGET",
	"QRYGPA",

	"SYSPWD",
	"BUFMODE",

	"WAKEUP",
	"OFFLINE",
	"VERSION",
	"LIST",
	"CSQ",
	"GSTATE",
	"DTUIP",
	"NET",
	"RESET",
	"RESTART",
	"CSMINS",
	"TEST_GSM",
};



u8 cmd_cmp(u8 *str1,u8 *str2)
{
	u8 res = 0;
	while(1)
	{
		if(*str2=='\0')
		{
		    if(*str1=='?')
		    {
		    	  DEBUG("query cmd\r\n");
		        res = 1;
		    	  break;
		    }
		    else if(*str1=='=')
		    {
		    	  DEBUG("set cmd\r\n");
		        res = 2;
		    	  break;
		    }
        else if(*str1=='#')
		    {
		    	  DEBUG("operation cmd\r\n");
		        res = 2;
		    	  break;
		    }
				else
				{
					  res = 3;
		    	  break;
				}
		}
		
		if(*str1!=*str2)
		{
			  res = 0;
		    break;
		}
		
		str1++;
		str2++;
	}
	return res;//两个字符串相等
}

//查询命令
void quert_cmd_process(u8 cmd, u8 *para)
{
	  int res = 0, i = 0;
	  temp[0] = 0;
	  temp[1] = 0;
	  temp[2] = 0;
	  temp[3] = 0;
	  temp[4] = 0;
	  memset(temp_str, 0, 130);
	
    switch(cmd)
    {
    	  case HOST_CMD_WKMODE://工作模式
					  res = sprintf((char *)host_tx_buff,"+WKMODE:%d\r\n",para_value.word_mode);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_GPRSMODE://在线模式
            res = sprintf((char *)host_tx_buff,"+GPRSMODE:%d\r\n",para_value.on_line_mode);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_DBGMODE://调试模式
					  res = sprintf((char *)host_tx_buff,"+DBGMODE:%d\r\n",para_value.debug_mode);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_DTUID://ID配置
            res = sprintf((char *)host_tx_buff,"+DTUID:%s\r\n",para_value.id);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_UART://串口参数
            res = sprintf((char *)host_tx_buff,"+UART:%d,%d,%d,%d,%d\r\n",para_value.usart_para.baurd, para_value.usart_para.data_bits, 
							             para_value.usart_para.parity, para_value.usart_para.stop ,para_value.usart_para.flow_ctr);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_DFI://串口数据包时间间隔
            res = sprintf((char *)host_tx_buff,"+DFI:%d\r\n",para_value.usart_rx_interval);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_HBTIME://心跳时间
            res = sprintf((char *)host_tx_buff,"+HBTIME:%d\r\n",para_value.heartbeat_time);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_HBHEAD://心跳包头
					  if(para_value.heartbeat_head_type == 0)
						{
						    if(para_value.heartbeat_head_len >0)
                {
								    res = sprintf((char *)host_tx_buff,"+HBHEAD:HEX%02X ",para_value.heartbeat_head[0]);
							      for(i=0; i<(para_value.heartbeat_head_len-1); i++)
							      {
								        res = res + sprintf((char *)(&host_tx_buff[14+i*3]),"%02X ",para_value.heartbeat_head[i+1]);
								    }
								}
                else
								{
								    res = sprintf((char *)host_tx_buff,"+HBHEAD: ");
								}									
							  
								host_tx_buff[res-1] = '\r';
								host_tx_buff[res] = '\n';
								res++;
						}
						else if(para_value.heartbeat_head_type == 1)
						{
						    res = sprintf((char *)host_tx_buff,"+HBHEAD:%s\r\n",para_value.heartbeat_head);
						}
            
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_REGPKG://注册包开关
					  res = sprintf((char *)host_tx_buff,"+REGPKG:%d\r\n",para_value.reg_packet_switch);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_REGHEAD://注册包头
					  if(para_value.reg_head_type == 0)
						{
							  if(para_value.reg_head_len >0)
								{
									  res = sprintf((char *)host_tx_buff,"+REGHEAD:HEX%02X ",para_value.reg_head[0]);
							      for(i=0; i<(para_value.reg_head_len-1); i++)
							      {
								        res = res + sprintf((char *)(&host_tx_buff[14+i*3]),"%02X ",para_value.reg_head[i+1]);
								    }
								}
								else
								{
								    res = sprintf((char *)host_tx_buff,"+REGHEAD: ");
								}
								host_tx_buff[res-1] = '\r';
								host_tx_buff[res] = '\n';
								res++;
								
						}
						else if(para_value.reg_head_type == 1)
						{
						    res = sprintf((char *)host_tx_buff,"+REGHEAD:%s\r\n",para_value.heartbeat_head);
						}
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_SERVER1://连接设置1
            res = sprintf((char *)host_tx_buff,"+SERVER1:%d,%s,%d\r\n",para_value.link1.connect_type,
							                                       para_value.link1.ip_addr, para_value.link1.port);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_SERVER2://连接设置2
            res = sprintf((char *)host_tx_buff,"+SERVER2:%d,%s,%d\r\n",para_value.link2.connect_type,
							                                       para_value.link2.ip_addr, para_value.link2.port);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_IDLTIME://自动下线时间
					  res = sprintf((char *)host_tx_buff,"+IDLTIME:%d\r\n",para_value.auto_off_line_time);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_APN://GPRS APN网关
					  res = sprintf((char *)host_tx_buff,"+APN:%s,%s,%s\r\n",para_value.apn_para.apn_name, 
							                      para_value.apn_para.user_name, para_value.apn_para.user_pwd);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_HTTPS://HTTP设置
					  res = sprintf((char *)host_tx_buff,"+HTTPS:%d,%d,%s\r\n", para_value.http_para.type,
							                    para_value.http_para.out_head, para_value.http_para.url);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
					  break;
		    case HOST_CMD_HTTPHD://HTTP 头设置
					  if(res != -1)
						{
						    res = 1;
							  temp[0] = 0;
							  temp[1] = 0;
							  //这里需要判断加转义符，不然sim800c识别不了
							  while(para_value.http_para.head[temp[1]] != 0)
							  {
									  if((para_value.http_para.head[temp[1]] == 0x5c) && (para_value.http_para.head[temp[1]+1] == 'r'))
										{
										    temp_str[temp[0]] = '\r';
											  temp[1]++;
										}
									  else if((para_value.http_para.head[temp[1]] == 0x5c) && (para_value.http_para.head[temp[1]+1] == 'n'))
										{
											  temp_str[temp[0]] = '\n';
											  temp[1]++;
										}
										else
										{
										    temp_str[temp[0]] = para_value.http_para.head[temp[1]];
										}
						        
										temp[0]++;
										temp[1]++;
								}
						}	
					  res = sprintf((char *)host_tx_buff,"+HTTPHD:%s\r\n", temp_str);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
					  break;
						
				case HOST_CMD_QRYPTM://轮询POST间隔时间
					  res = sprintf((char *)host_tx_buff,"+QRYPTM:%d,%d\r\n", para_value.modbus_qry_post_time, para_value.modbus_qry_cmd_time);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
					  break;
				case HOST_CMD_QRYPOST://轮询POST
					  para = para + 7;
				    //先把第一个参数读取回来
						while(*para != '?')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == '?')
								{
								    break;
								}
						}
						if(temp[0] < 16)
						{
						    res = sprintf((char *)host_tx_buff,"+QRYPOST:%d,", temp[0]);
								for(i=0; i<para_value.modbus_qry_post_para[temp[0]].len;i++)
								{
										host_tx_buff[res+i*2] = assic_hex_tab[para_value.modbus_qry_post_para[temp[0]].buff[i] >> 4];
										host_tx_buff[res+i*2+1] = assic_hex_tab[para_value.modbus_qry_post_para[temp[0]].buff[i] & 0x0f];
								}
								host_tx_buff[res+i*2] = '\r';
								host_tx_buff[res+i*2+1] = '\n';
								i++;
				        usart_send_str(HOSTIF_USART, host_tx_buff, (res+i*2));	
						}
					  break;
				case HOST_CMD_QRYPPA://设置轮询post http参数
						res = sprintf((char *)host_tx_buff,"+QRYPPA:%s\r\n", para_value.modbus_http_post_para.url);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
						break;
				case HOST_CMD_QRYGTM://轮询GET间隔时间
					  res = sprintf((char *)host_tx_buff,"+QRYGTM:%d\r\n", para_value.modbus_qry_get_time);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
					  break;
				case HOST_CMD_QRYGET://轮询GET
						res = sprintf((char *)host_tx_buff,"+QRYGET:");
						for(i=0; i<para_value.modbus_qry_get_para.len;i++)
						{
								host_tx_buff[res+i*2] = assic_hex_tab[para_value.modbus_qry_get_para.buff[i] >> 4];
								host_tx_buff[res+i*2+1] = assic_hex_tab[para_value.modbus_qry_get_para.buff[i] & 0x0f];
						}
						host_tx_buff[res+i*2] = '\r';
						host_tx_buff[res+i*2+1] = '\n';
						i++;
				    usart_send_str(HOSTIF_USART, host_tx_buff, (res+i*2));	
					  break;
				case HOST_CMD_QRYGPA://设置轮询get http参数
						res = sprintf((char *)host_tx_buff,"+QRYGPA:%s\r\n", para_value.modbus_http_get_para.url);
						usart_send_str(HOSTIF_USART, host_tx_buff, res);
				case HOST_CMD_BUFMODE://数据缓存模式
            
    		    break;	
				case HOST_CMD_VERSION://查询DTU版本
					  res = sprintf((char *)host_tx_buff,"+VERSION:%s,%s\r\n",dtu_hw_version,dtu_sw_version);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_LIST://查询DTU所以参数
					  //
					  res = sprintf((char *)host_tx_buff,"+LIST:\r\n");
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //工作模式
					  res = sprintf((char *)host_tx_buff,"+WKMODE:%d\r\n",para_value.word_mode);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //在线模式
            res = sprintf((char *)host_tx_buff,"+GPRSMODE:%d\r\n",para_value.on_line_mode);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //DTU ide号
            res = sprintf((char *)host_tx_buff,"+DTUID:%s\r\n",para_value.id);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //串口输出参数
            res = sprintf((char *)host_tx_buff,"+UART:%d,%d,%d,%d,%d\r\n",para_value.usart_para.baurd, para_value.usart_para.data_bits, 
							             para_value.usart_para.parity, para_value.usart_para.stop ,para_value.usart_para.flow_ctr);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //串口数据包时间间隔 
            res = sprintf((char *)host_tx_buff,"+DFI:%d\r\n",para_value.usart_rx_interval);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //心跳时间
            res = sprintf((char *)host_tx_buff,"+HBTIME:%d\r\n",para_value.heartbeat_time);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //心跳包头
					  if(para_value.heartbeat_head_type == 0)
						{
						    if(para_value.heartbeat_head_len >0)
                {
								    res = sprintf((char *)host_tx_buff,"+HBHEAD:HEX%02X ",para_value.heartbeat_head[0]);
							      for(i=0; i<(para_value.heartbeat_head_len-1); i++)
							      {
								        res = res + sprintf((char *)(&host_tx_buff[14+i*3]),"%02X ",para_value.heartbeat_head[i+1]);
								    }
								}
                else
								{
								    res = sprintf((char *)host_tx_buff,"+HBHEAD: ");
								}									
							  
								host_tx_buff[res-1] = '\r';
								host_tx_buff[res] = '\n';
								res++;
						}
						else if(para_value.heartbeat_head_type == 1)
						{
						    res = sprintf((char *)host_tx_buff,"+HBHEAD:%s\r\n",para_value.heartbeat_head);
						}
            
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //注册包开关
					  res = sprintf((char *)host_tx_buff,"+REGPKG:%d\r\n",para_value.reg_packet_switch);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
				    //注册包头
					  if(para_value.reg_head_type == 0)
						{
							  if(para_value.reg_head_len >0)
								{
									  res = sprintf((char *)host_tx_buff,"+REGHEAD:HEX%02X ",para_value.reg_head[0]);
							      for(i=0; i<(para_value.reg_head_len-1); i++)
							      {
								        res = res + sprintf((char *)(&host_tx_buff[14+i*3]),"%02X ",para_value.reg_head[i+1]);
								    }
								}
								else
								{
								    res = sprintf((char *)host_tx_buff,"+REGHEAD: ");
								}
								host_tx_buff[res-1] = '\r';
								host_tx_buff[res] = '\n';
								res++;
								
						}
						else if(para_value.reg_head_type == 1)
						{
						    res = sprintf((char *)host_tx_buff,"+REGHEAD:%s\r\n",para_value.heartbeat_head);
						}
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //连接设置1
            res = sprintf((char *)host_tx_buff,"+SERVER1:%d,%s,%d\r\n",para_value.link1.connect_type,
							                                       para_value.link1.ip_addr, para_value.link1.port);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    //连接设置2
            res = sprintf((char *)host_tx_buff,"+SERVER2:%d,%s,%d\r\n",para_value.link2.connect_type,
							                                       para_value.link2.ip_addr, para_value.link2.port);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    //自动下线时间
					  res = sprintf((char *)host_tx_buff,"+IDLTIME:%d\r\n",para_value.auto_off_line_time);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    //GPRS APN网关
					  res = sprintf((char *)host_tx_buff,"+APN:%s,%s,%s\r\n",para_value.apn_para.apn_name, 
							                      para_value.apn_para.user_name, para_value.apn_para.user_pwd);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
						
						
				case HOST_CMD_CSQ://查询网络信号质量
            //获取网络信号质量
            if(sim900a_send_data_ack("AT+CSQ\r\n",8,"+CSQ",50) == 0)//查询信号强度
            {
            	  RSSI = (USART3_RX_BUF[8]-0X30)*10+(USART3_RX_BUF[9]-0X30);
            }
            else
            {
                //无法获取信号强度
							  RSSI = 99;
            }
				    temp[0] = RSSI;

					  res = sprintf((char *)host_tx_buff,"+CSQ:%d\r\n", temp[0]);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
    		    break;	
				case HOST_CMD_GSTATE://查询GPRS在线状态
						if(sim900a_send_data_ack("AT+CGATT?\r\n",11,"+CGATT",200) == 0)//发送数据
					  {
							  temp[0] = USART3_RX_BUF[10] - 0x30;
						}
						else
						{
						    temp[0] = 0;
						}
					  res = sprintf((char *)host_tx_buff,"+GPRS:%d\r\n", temp[0]);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
    		    break;	
				case HOST_CMD_DTUIP://查询DTU IP地址
					  //因为这个指令直接返回IP，所以要判断是否ERROR
						if(sim900a_send_data_ack("AT+CIFSR\r\n",10,"ERROR",200) == 0)//发送数据
					  {
							  res = 0;
							  res = sprintf((char *)host_tx_buff,"+DTUIP:%s\r\n", "0.0.0.0");
						}
						else
						{
							  
						    res = sprintf((char *)host_tx_buff,"+DTUIP:%s", &USART3_RX_BUF[2]);
						}

					  
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
    		    break;	
				case HOST_CMD_NET://查询网络注册状态
						if(sim900a_send_data_ack("AT+CREG=2\r\n",11,"OK",200) == 0)//发送数据
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
						delay_ms(100);
						if(sim900a_send_data_ack("AT+CREG?\r\n",10,"+CREG",200) == 0)//发送数据
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
					  res = sprintf((char *)host_tx_buff,"+NET:%s\r\n", &USART3_RX_BUF[11]);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
						if(sim900a_send_data_ack("AT+CREG=0\r\n",11,"OK",200) == 0)//发送数据
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
    		    break;	
				
				case HOST_CMD_CSMINS://查询SIM卡是否存在
						if(sim900a_send_data_ack("AT+CSMINS?\r\n", 12, "+CSMINS", 200) == 0)//发送数据
					  {
							  temp[0] = USART3_RX_BUF[13]-0X30;
						}
						else
						{
						    temp[0] = 0;
						}

					  res = sprintf((char *)host_tx_buff,"+CSMINS:%d\r\n", temp[0]);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
    		    break;	

    }
		if(res > 0)
		{
        res = sprintf((char *)host_tx_buff,"OK\r\n");
        usart_send_str(HOSTIF_USART, host_tx_buff, res);
		}
		else
		{
		    res = sprintf((char *)host_tx_buff,"ERROR\r\n");
        usart_send_str(HOSTIF_USART, host_tx_buff, res);
		}
}
//设置命令
void set_cmd_process(u8 cmd, u8 *para)
{
	  int res = 0, i = 0;

    temp[0] = 0;
    temp[1] = 0;
    temp[2] = 0;
    temp[3] = 0;
    temp[4] = 0;
		memset(temp_str,0,130);
    switch(cmd)
    {
    	  case HOST_CMD_WKMODE://工作模式
					  res = para[7] - 0x30;
				    if(res > 8)
						{
						    res = -1;
						}
						else
						{
						    para_value.word_mode = res;
						}
    		    break;	
				case HOST_CMD_GPRSMODE://在线模式
				 	  res = para[9] - 0x30;
				    if(res > 3)
						{
						    res = -1;
						}
				    else
						{
						    para_value.on_line_mode = res;
						}
    		    break;	
				case HOST_CMD_DBGMODE://调试模式
				 	  res = para[8] - 0x30;
				    if(res > 3)
						{
						    res = -1;
						}
				    else
						{
						    para_value.debug_mode = res;
						}
    		    break;	
				case HOST_CMD_DTUID://ID配置
					  para  = para + 6;
            while(*para != HOST_REVEIVE_END_STR)
						{
						    temp_str[i] = *para++;
							  i++;
							  if(i > 32)
								{
								    res = -1;
									  break;
								}
								
						}

						
						if(res != -1)
						{
							  para_value.id_len = i;
							  res = i;
							  memset((para_value.id),0,34);
							  for(i=0; i<34; i++)
								{
								    para_value.id[i] =  temp_str[i];
								}
						}
						
    		    break;	
				case HOST_CMD_UART://串口参数
				    para = para + 5;
				    
            //检查参数是否完整
				    while(*para != HOST_REVEIVE_END_STR)
						{
							  if(*para++ == ',')
								{
								    res++;
								}
						    i++;
						}
						if(res == 4)
						{
						    res = 0;
							  para = para-i;
						}
						else
						{
						    res = -1;
						}
				    //读取数据到缓冲变量
						if(res != -1)
						{
						    for(res=0; res<5; res++)
				        {
						    	  
						        while(*para != ',')
						    		{
						    		    temp[res] = temp[res]*10 + (*para++ - 0x30);
						    			  if(*para == HOST_REVEIVE_END_STR)
						    				{
						    				    break;
						    				}
						    		}
						    		para++;
						    }
						    //检查参数的完整性
                if((temp[0] == 1200 ) || (temp[0] == 2400 ) || (temp[0] == 4800) 
                || (temp[0] == 9600 ) || (temp[0] == 14400) || (temp[0] == 19200)
                || (temp[0] == 38400) || (temp[0] == 57600) || (temp[0] == 115200))
                {
                
                }
                else
                {
                     res = -1;
                }
                
                if((temp[1] == 8) || (temp[1] == 9))
                {
                
                }
                else
                {
                    res = -1;
                }
                
                if(temp[2] > 2)
                {
                    res = -1;
                }
                
                if(temp[3] > 3)
                {
                    res = -1;
                }
                
                if(temp[4] > 2)
                {
                    res = -1;
                }
								//参数赋值
						    if(res != -1)
						    {
						        para_value.usart_para.baurd = temp[0];
						    	  para_value.usart_para.data_bits = temp[1];
						    	  para_value.usart_para.parity = temp[2];
						    	  para_value.usart_para.stop = temp[3];
						    	  para_value.usart_para.flow_ctr = temp[4];
									
									  uart2_init(para_value.usart_para);	 //串口初始化为115200
						    }		
						}
						
    		    break;	
				case HOST_CMD_DFI://串口数据包时间间隔
					  para = para + 4;
						while(*para != ',')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
				    if(res > 5000)
						{
						    res = -1;
						}
				    else
						{
						    para_value.usart_rx_interval = temp[0];
							  res = temp[0];
						}
    		    break;	
				case HOST_CMD_HBTIME://心跳时间
					  para = para + 7;
						while(*para != ',')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
				    if(res > 3600)
						{
						    res = -1;
						}
				    else
						{
						    para_value.heartbeat_time = temp[0];
							  res = temp[0];
						}
						
    		    break;	
				case HOST_CMD_HBHEAD://心跳包头
					  para = para + 7;
					  if((*para == 'H') && (*(para+1) == 'E') && (*(para+2) == 'X'))
						{
							  temp[0] = 0;
						    para = para + 3;
							  while(*para != HOST_REVEIVE_END_STR)
								{
								    if(*para == ' ')
										{
										    para++;
										}
										temp_str[i] = (*para++ - 0x30)<<4 ;
										temp_str[i]  |= (*para++ - 0x30) ;
										i++;
										if(i > 32)
										{
											  res = -1;
											  break;
										}
								}
						}
						else
						{
							  temp[0] = 1;
                while(*para != HOST_REVEIVE_END_STR)
						    {
						        temp_str[i] = *para++;
						    	  i++;
						    	  if(i > 32)
						    		{
											  res = -1;
						    			  break;
						    		}
						    }
						}
						
						if(res != -1)
						{
		            para_value.heartbeat_head_len = i;
			          para_value.heartbeat_head_type = temp[0];
							  res = i;
							  memset((para_value.heartbeat_head),0,34);
							  for(i=0; i<34; i++)
								{
								    para_value.heartbeat_head[i] = temp_str[i];
								}
						    
						}
						
    		    break;	
				case HOST_CMD_REGPKG://注册包开关
					  res = para[7] - 0x30;
				    if(res > 1)
						{
						    res = -1;
						}
						else
						{
						    para_value.reg_packet_switch = res;
						}
    		    break;	
				case HOST_CMD_REGHEAD://注册包头
					  para = para + 8;
					  if((*para == 'H') && (*(para+1) == 'E') && (*(para+2) == 'X'))
						{
							  temp[0] = 0;
						    para = para + 3;
							  while(*para != HOST_REVEIVE_END_STR)
								{
								    if(*para == ' ')
										{
										    para++;
										}
										temp_str[i] = (*para++ - 0x30)<<4 ;
										temp_str[i]  |= (*para++ - 0x30) ;
										i++;
										if(i > 32)
										{
											  res = -1;
											  break;
										}
								}
						}
						else
						{
							  temp[0] = 1;
                while(*para != HOST_REVEIVE_END_STR)
						    {
						        temp_str[i] = *para++;
						    	  i++;
						    	  if(i > 32)
						    		{
											  res = -1;
						    			  break;
						    		}
						    }
						}
						
						if(res != -1)
						{
		            para_value.reg_head_len = i;
			          para_value.reg_head_type = temp[0];
							  res = i;
							  memset((para_value.reg_head),0,34);
							 for(i=0; i<34; i++)
								{
								    para_value.reg_head[i] = temp_str[i];
								}
						    
						}
    		    break;	
				case HOST_CMD_SERVER1://连接设置1
					  para = para + 8;
				    //检查参数是否完整
				    while(*para != HOST_REVEIVE_END_STR)
						{
							  if(*para++ == ',')
								{
								    res++;
								}
								i++;
						}
						if(res == 2)
						{
						    res = 0;
							  para = para-i;
							  i = 0;
						}
						else
						{
						    res = -1;
						}
				    //获取参数
						if(res != -1)
						{
							  //读取第一个参数
							  while(*para != ',')
								{
								    temp[0] = temp[0]*10 + (*para++ - 0x30);
								}
								
								//判断参数的合法性
								if(temp[0] > 3)
		            {
						    	  res = -1;
		            }
								//读取第二个参数
								if(res != -1)
								{
									  para++;
								    while(*para != ',')
								    {
								    	  temp_str[i] = *para++;
								        i++;
											  //判断数据长度的合法性
								    	  if(i > 64)
								    		{
								    		    res = -1;
								    			  break;
								    		}
								    }
								}
								//读取第三个参数
								if(res != -1)
								{
									  para++;
								    while(*para != HOST_REVEIVE_END_STR)
								    {
								    	  temp[1] = temp[1]*10 + (*para++ - 0x30);
								    }
								}
								
				        if(res != -1)
						    {
									  res = temp[0];
		                para_value.link1.connect_type = temp[0];
			              para_value.link1.port = temp[1];
									  for(i=0; i<66; i++)
										{
										    para_value.link1.ip_addr[i] = temp_str[i];
										}
						    }
						}
    		    break;	
				case HOST_CMD_SERVER2://连接设置2
					  para = para + 8;
				    //检查参数是否完整
				    while(*para != HOST_REVEIVE_END_STR)
						{
							  if(*para++ == ',')
								{
								    res++;
								}
								i++;
						}
						if(res == 2)
						{
						    res = 0;
							  para = para-i;
							  i = 0;
						}
						else
						{
						    res = -1;
						}
				    //获取参数
						if(res != -1)
						{
							  //读取第一个参数
							  while(*para != ',')
								{
								    temp[0] = temp[0]*10 + (*para++ - 0x30);
								}
								
								//判断参数的合法性
								if(temp[0] > 3)
		            {
						    	  res = -1;
		            }
								//读取第二个参数
								if(res != -1)
								{
									  para++;
								    while(*para != ',')
								    {
								    	  temp_str[i] = *para++;
								        i++;
											  //判断数据长度的合法性
								    	  if(i > 64)
								    		{
								    		    res = -1;
								    			  break;
								    		}
								    }
								}
								//读取第三个参数
								if(res != -1)
								{
									  para++;
								    while(*para != HOST_REVEIVE_END_STR)
								    {
								    	  temp[1] = temp[1]*10 + (*para++ - 0x30);
								    }
								}
								
				        if(res != -1)
						    {
									  res = temp[0];
		                para_value.link2.connect_type = temp[0];
			              para_value.link2.port = temp[1];
									  for(i=0; i<66; i++)
										{
										    para_value.link2.ip_addr[i] = temp_str[i];
										}
						    }
						}
    		    break;	
				case HOST_CMD_IDLTIME://自动下线时间
					  para = para + 8;
						while(*para != ',')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
				    if(res > 5000)
						{
						    res = -1;
						}
				    else
						{
						    para_value.auto_off_line_time = temp[0];
							  res = temp[0];
						}
    		    break;	
				case HOST_CMD_APN://GPRS APN网关
					  para = para + 4;
				    //检查参数是否完整
				    while(*para != HOST_REVEIVE_END_STR)
						{
							  if(*para == ',')
								{
								    res++;
								}
								para++;
								i++;
						}
						if(res == 2)
						{
						    res = 0;
							  para = para-i;
							  i = 0;
						}
						else
						{
						    res = -1;
						}
				    //获取参数
						if(res != -1)
						{
							  //读取第一个参数
							  while(*para != ',')
								{
								    temp_apn_para.apn_name[i] = *para++;
								    i++;
								    //判断数据长度的合法性
								    if(i > 16)
								    {
								        res = -1;
								    	  break;
								    }
								}
								
								if(res != -1)
								{
									  para++;
									  temp[0] = i;
									  i = 0;
								    while(*para != ',')
								    {
								    	  temp_apn_para.user_name[i] = *para++;
								        i++;
											  //判断数据长度的合法性
								    	  if(i > 32)
								    		{
								    		    res = -1;
								    			  break;
								    		}
								    }
								}
								if(res != -1)
								{
									  para++;
									  temp[1] = i;
									  i = 0;
								    while(*para != HOST_REVEIVE_END_STR)
								    {
								    	  temp_apn_para.user_pwd[i] = *para++;
								        i++;
											  //判断数据长度的合法性
								    	  if(i > 32)
								    		{
								    		    res = -1;
								    			  break;
								    		}
								    }
								}
								
				        if(res != -1)
						    {
									  temp[2] = i;
									  res = temp[0];
									  memset((para_value.apn_para.apn_name),0,18);
									  memset((para_value.apn_para.user_name),0,34);
									  memset((para_value.apn_para.user_pwd),0,34);
									  while(temp[0]--)
										{
										    para_value.apn_para.apn_name[temp[0]] = temp_apn_para.apn_name[temp[0]];
										}
									  while(temp[1]--)
										{
										    para_value.apn_para.user_name[temp[1]] = temp_apn_para.user_name[temp[1]];
										}
									  while(temp[2]--)
										{
										    para_value.apn_para.user_pwd[temp[2]] = temp_apn_para.user_pwd[temp[2]];
										}
										if((temp_apn_para.user_name[0] == 0) && (temp_apn_para.user_pwd[0] == 0))
										{
										    res = sprintf((char *)temp_str,"AT+CSTT=\"%s\"\r\n", para_value.apn_para.apn_name);
										}
										else
										{
										    res = sprintf((char *)temp_str,"AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", para_value.apn_para.apn_name,
										    	                                  para_value.apn_para.user_name, para_value.apn_para.user_pwd);
										}
										
										
						        //if(sim900a_send_data_ack(temp_str,res,"OK",200) == 0)//发送数据
					          //{
						        //	  res = 1;
						        //}
						        //else
						        //{
						        //    res = -1;
						        //}
										
						    }
						}
				
				
    		    break;	
						
				case HOST_CMD_HTTPS://HTTP设置
					  para = para + 6;
				    while(*para != HOST_REVEIVE_END_STR)
						{
							  if(*para++ == ',')
								{
								    res++;
								}
								i++;
						}
						if(res == 2)
						{
						    res = 0;
							  para = para-i;
							  i = 0;
						}
						else
						{
						    res = -1;
						}
				    //获取参数
						if(res != -1)
						{
							  //读取第一个参数
							  while(*para != ',')
								{
								    temp[0] = temp[0]*10 + (*para++ - 0x30);
								}
								//判断参数的合法性
								if(temp[0] > 2)
		            {
						    	  res = -1;
		            }
								//读取第二个参数
								if(res != -1)
								{
									  para++;
							      while(*para != ',')
								    {
								        temp[1] = temp[1]*10 + (*para++ - 0x30);
								    }
								    //判断参数的合法性
								    if(temp[1] > 2)
		                {
						        	  res = -1;
		                }
								}
								//读取第三个参数
								if(res != -1)
								{
									  para++;
								    while(*para != HOST_REVEIVE_END_STR)
								    {
								    	  temp_str[i] = *para++;
								        i++;
											  //判断数据长度的合法性
								    	  if(i > 100)
								    		{
								    		    res = -1;
								    			  break;
								    		}
								    }
								}
								
				        if(res != -1)
						    {
									  res = temp[0];
		                para_value.http_para.type = temp[0];
			              para_value.http_para.out_head = temp[1];
									  memset(para_value.http_para.url, 0, 130);
									  for(i=0; i<130; i++)
										{
										    para_value.http_para.url[i] = temp_str[i];
										}
						    }
						}
					  break;
				
				case HOST_CMD_HTTPHD://HTTP设置
            para = para + 7;
						//读取第一个参数
				    while(*para != HOST_REVEIVE_END_STR)
						{
						    temp_str[i] = *para++;
						    i++;
						    //判断数据长度的合法性
						    if(i > 100)
						    {
						        res = -1;
						    	  break;
						    }
						}
				    if(res != -1)
						{
						    res = 1;
							  temp[0] = 0;
							  temp[1] = 0;
							  memset(para_value.http_para.head, 0, 130);
							  //这里需要判断加转义符，不然sim800c识别不了
							  for(temp[0]=0; temp[0]<i;)
							  {
									  if(temp_str[temp[0]] == '\r')
										{
										    para_value.http_para.head[temp[1]] = 0x5c;//   '\'
											  temp[1]++;
                        para_value.http_para.head[temp[1]] = 'r';//
											  temp[1]++;
										}
										else if(temp_str[temp[0]] == '\n')
										{
										    para_value.http_para.head[temp[1]] = 0x5c;//   '\'
											  temp[1]++;
                        para_value.http_para.head[temp[1]] = 'n';//   
											  temp[1]++;
										}
										else
										{
						            para_value.http_para.head[temp[1]] = temp_str[temp[0]];
										    temp[1]++;
										}
										temp[0]++;
								}
								
								for(i=0; i<temp[1]; i++)
								{
								     para_value.modbus_http_get_para.head[i]  = para_value.http_para.head[i];
									   para_value.modbus_http_post_para.head[i]  = para_value.http_para.head[i];
								}
						}

					  break;
				case HOST_CMD_QRYPTM://轮询POST间隔时间
					  para = para + 7;
				    //获取第一个参数
						while(*para != ',')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
					  para++;
				    //获取第二个参数
						while(*para != ',')
						{

						    temp[1] = temp[1]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
				    if((temp[0] > 3600) || (temp[1] > 3000))
						{
						    res = -1;
						}
				    else
						{
						    para_value.modbus_qry_post_time = temp[0];
							  para_value.modbus_qry_cmd_time = temp[1];
							  res = temp[0];
						}
					  break;
				case HOST_CMD_QRYPOST://轮询POST
					  para = para + 8;
				    //先把第一个参数读取回来
						while(*para != ',')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
						para++;
						
						if(temp[0] >=16)
						{
						    res = -1;
						}
						
						//判断第二个参数的合法性
						
						if(res != -1)
						{
						    i = 0;
				        while(*para != HOST_REVEIVE_END_STR)
						    {
						        para++;
						    	  i++;
						    }
						    if((i%2 != 0) || (i > 64))
						    {
						        res = -1;
						    }
						}

				    //读取第二个参数
				    if(res != -1)
						{
							  para = para - i;
							  i = 0;
							  while(*para != HOST_REVEIVE_END_STR)
								{
								    if(*para == ' ')
										{
										    para++;
										}
										
										for(temp[1]=0;temp[1]<16;temp[1]++)
										{
										    if(assic_hex_tab[temp[1]] == *para)
												{
													   break;
												}
										}
										para++;
										for(temp[2]=0;temp[2]<16;temp[2]++)
										{
										    if(assic_hex_tab[temp[2]] == *para)
												{
													   break;
												}
										}
										para++;
										temp_str[i] = (temp[1]<<4) | (temp[2]);

										i++;
										if(i > 32)
										{
											  res = -1;
											  break;
										}
								}
						}
						
						//把获取的数据赋值给  para_value
						if(res != -1)
						{
							  res = i;
							  para_value.modbus_qry_post_para[temp[0]].len = i;
						    for(i=0; i<34; i++)
								{
								    para_value.modbus_qry_post_para[temp[0]].buff[i] = temp_str[i];
								}
						}
					  break;
				case HOST_CMD_QRYPPA://设置轮询post http参数
					  para = para + 7;
						//读取第一个参数
				    while(*para != HOST_REVEIVE_END_STR)
						{
						    temp_str[i] = *para++;
						    i++;
						    //判断数据长度的合法性
						    if(i > 128)
						    {
						        res = -1;
						    	  break;
						    }
						}
						//把获取的数据赋值给  para_value
						if(res != -1)
						{
							  res = i;
							  para_value.modbus_http_post_para.type = 0;
							  para_value.modbus_http_post_para.out_head = 0;
							  for(i=0; i<sizeof(para_value.http_para.head); i++)
							  {
								    para_value.modbus_http_post_para.head[i] = para_value.http_para.head[i];
								}
						    for(i=0; i<130; i++)
								{
								    para_value.modbus_http_post_para.url[i] = temp_str[i];
								}
						}
						break;
				case HOST_CMD_QRYGTM://轮询GET间隔时间
					  para = para + 7;
						while(*para != ',')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
				    if(res > 3600)
						{
						    res = -1;
						}
				    else
						{
						    para_value.modbus_qry_get_time = temp[0];
							  res = temp[0];
						}
					  break;
				case HOST_CMD_QRYGET://轮询GET
					  para = para + 7;  
						//判断第一个参数的合法性
						i = 0;
				    while(*para != HOST_REVEIVE_END_STR)
						{
						    para++;
							  i++;
						}
						if((i%2 != 0) || (i > 64))
						{
						    res = -1;
						}
				    //读取第一个参数
				    if(res != -1)
						{
							  para = para - i;
							  i = 0;
							  while(*para != HOST_REVEIVE_END_STR)
								{
								    if(*para == ' ')
										{
										    para++;
										}

								    if(*para == ' ')
										{
										    para++;
										}
										
										for(temp[1]=0;temp[1]<16;temp[1]++)
										{
										    if(assic_hex_tab[temp[1]] == *para)
												{
													   break;
												}
										}
										para++;
										for(temp[2]=0;temp[2]<16;temp[2]++)
										{
										    if(assic_hex_tab[temp[2]] == *para)
												{
													   break;
												}
										}
										para++;
										temp_str[i] = (temp[1]<<4) | (temp[2]);
										
										i++;
										if(i > 32)
										{
											  res = -1;
											  break;
										}
								}
						}
						
						//把获取的数据赋值给  para_value
						if(res != -1)
						{
							  res = i;
							  para_value.modbus_qry_get_para.len = i;
						    for(i=0; i<34; i++)
								{
								    para_value.modbus_qry_get_para.buff[i] = temp_str[i];
								}
						}
					  break;
				case HOST_CMD_QRYGPA://设置轮询get http参数
					  para = para + 7;
						//读取第一个参数
				    while(*para != HOST_REVEIVE_END_STR)
						{
						    temp_str[i] = *para++;
						    i++;
						    //判断数据长度的合法性
						    if(i > 128)
						    {
						        res = -1;
						    	  break;
						    }
						}
						//把获取的数据赋值给  para_value
						if(res != -1)
						{
							  res = i;
							  para_value.modbus_http_get_para.type = 1;
							  para_value.modbus_http_get_para.out_head = 0;
							  for(i=0; i<sizeof(para_value.http_para.head); i++)
							  {
								    para_value.modbus_http_get_para.head[i] = para_value.http_para.head[i];
								}
						    for(i=0; i<130; i++)
								{
								    para_value.modbus_http_get_para.url[i] = temp_str[i];
								}
						}
						break;
						
						
						
				
				case HOST_CMD_SYSPWD://DTU密码修改
            para = para + 7;
						//读取第一个参数
				    while(*para != ',')
						{
						    temp_str[i] = *para++;
						    i++;
						    //判断数据长度的合法性
						    if(i > 6)
						    {
						        res = -1;
						    	  break;
						    }
						}
						//判断密码是否正确
						if(res != -1)
						{
							  i = para_value.password_len;
	              while(i--)
	              {
	              	  if(para_value.password[i] != temp_str[i])
	              	  {
	              	  	  res = -1;
	              	      break;
	              	  }
	              }
								//获取新的密码
								if(res != -1)
								{
								    para++;
									  i=0;
				            while(*para != HOST_REVEIVE_END_STR)
						        {
						            temp_str[i] = *para++;
						            i++;
						            //判断数据长度的合法性
						            if(i > 6)
						            {
						                res = -1;
						            	  break;
						            }
						        }
								}
								//赋值新的密码
								if(res != -1)
								{
									  para_value.password_len = i;
								    res = i;
									  memset((para_value.password),0,8);
									  for(i=0; i<8; i++)
										{
										    para_value.password[i] = temp_str[i];
										}
								}
						}
    		    break;		
				case HOST_CMD_BUFMODE:
						
    		    break;	
						
				case HOST_CMD_WAKEUP://唤醒命令
					  //设置TCP连接参数
				    if(para_value.on_line_mode != 0)
						{
						    res = connect_tcp_udp(para_value.link1);
						}
    		    break;	
				case HOST_CMD_OFFLINE://离线命令
            res = off_line();
    		    break;	
				
	
				case HOST_CMD_RESET://DTU恢复出厂设置
            para = para + 6;
						//读取第一个参数
				    while(*para != HOST_REVEIVE_END_STR)
						{
						    temp_str[i] = *para++;
						    i++;
						    //判断数据长度的合法性
						    if(i > 6)
						    {
						        res = -1;
						    	  break;
						    }
						}
						//判断密码是否正确
						if(res != -1)
						{
							  i = para_value.password_len;
	              while(i--)
	              {
	              	  if(para_value.password[i] != temp_str[i])
	              	  {
	              	  	  res = -1;
	              	      break;
	              	  }
	              }
						}
						
						if(res != -1)
						{
						    define_para_value();
							  res = 1;
						}
    		    break;	
				case HOST_CMD_RESTART://DTU重启
                    res = sprintf((char *)host_tx_buff,"OK\r\n");
                    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //重启SIM800C
				    
				    //重启
				    NVIC_SystemReset();
    		    break;	
				
				case HOST_CMD_TEST_GSM:
					  para = para + 9;
				    temp[0] = (*para - 0x30);
				    para++;
				    temp[0] = temp[0]*10 + (*para - 0x30);
				    temp[1] = test_gsm(temp[0]);
            res = sprintf((char *)host_tx_buff,"test gsm %d\r\n",temp[1]);
            usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    break;

    }

		if(res >= 0)
		{
        res = sprintf((char *)host_tx_buff,"OK\r\n");
        usart_send_str(HOSTIF_USART, host_tx_buff, res);
		}
		else
		{
		    res = sprintf((char *)host_tx_buff,"ERROR\r\n");
        usart_send_str(HOSTIF_USART, host_tx_buff, res);
		}

		
}

u8 test_gsm(u8 cmd)
{
	  char res = 0;
    switch (cmd)
		{
			  case 0:
						if(sim900a_send_data_ack("AT&F0\r\n",7,"OK",200) == 0)//发送数据
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
					  break;
				case 1:
						res = read_imei(host_tx_buff);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
						break;
				case 2:

						break;
				case 3:
            res = http_post(para_value.link1, para_value.http_para, "act=get_token", 15,1);
						break;
				case 4:
            res = sprintf((char *)host_tx_buff,"test gsm %d  \\r   \\n    \r\n",temp[1]);
            usart_send_str(HOSTIF_USART, host_tx_buff, res);
					  break;
				case 5:

						break;
				case 6:
            if(sim900a_send_data_ack("AT+CGDCONT=1,\"IP\",\"CMNET\"", 26, "OK", 1000) == 0)//设置PDP上下文,互联网接协议,接入点等信息
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
						break;
				case 7:

						break;

				case 8://激活场景
						if(sim900a_send_data_ack("AT+CIICR\r\n",10,"OK",2000) == 0)//发送数据
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
						break;
				case 9://
						if(sim900a_send_data_ack("AT+CSPN?\r\n",10,"OK",200) == 0)//发送数据
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
						break;
				case 10://查询连接状态
						if(sim900a_send_data_ack("AT+CIPSTATUS\r\n",14,"CONNECT OK",500) == 0)	
						{
							//connectsta=1;//连接正常
						}
						else
						{
							//connectsta=2;//其他返回全部都是链接异常，直接重启TCP
						}
						break;
				case 11:
					
				
						break;
				case 12://发起TCP连接
					  res = connect_tcp_udp(para_value.link1);
				    if(res > 0)
						{
						    res = tcp_udp_send("qwertyu", strlen((char *)"qwertyu"));
						}
						
						break;
						

		}

		usart_send_str(HOSTIF_USART, USART3_RX_BUF, USART3_MAX_RECV_LEN);
		return (u8)res;
}


u8 cmd_process(u8 *str, u16 len)
{
	  u8 i = 0;
	  u8 res = 0;
    for(i=0;i<sizeof(hostif_cmd_tab)/4;i++)//支持的系统指令
		{
			  res = cmd_cmp(str,hostif_cmd_tab[i]);
				if(res != 0)
				{
				    break;
				}
		}
		if((i == 17) && (res == 3))
		{
		    DEBUG("query cmd\r\n");
		    res = 1;
		}
		DEBUG("CMD is %s\r\n",hostif_cmd_tab[i]);
		//查询命令
		if(res == 1)
		{
			  quert_cmd_process(i, str);

		}
		//设置命令
		else if(res == 2)
		{
			  if(str[len-3] == '#')
				{
			      set_cmd_process(i, str);
					  if(i<=HOST_CMD_BUFMODE)
						{
						    write_para_to_flash();
						}
				}
				else
				{
				
				}
			  
		}
		return res;
		
}

u8 tcp_udp_auto_line(void)
{
	  u8 res = 0;
	
	  if(check_tcp_udp_connet() == 0)
		{
        if((para_value.link1.connect_type == 0) || (para_value.link1.connect_type == 1))
        {
        	  if(connect_tcp_udp(para_value.link1) == 0)
        		{
        		    if((para_value.link2.connect_type == 0) || (para_value.link2.connect_type == 1))
        				{
        				    if(connect_tcp_udp(para_value.link2))
										{
										    res = 0; 
										}
										else
										{
										    res = 2;
										}
		    					  
        				}
        		}
						else
						{
						    res = 1;
						}
        }
        else if((para_value.link2.connect_type == 0) || (para_value.link2.connect_type == 1))
        {
            if(connect_tcp_udp(para_value.link1) == 0)
						{
						    res = 0;
						}
						else
						{
						    res = 2;
						}
		    	  
        }
		}
		else
		{
		    res = 3;
		}

		return res;
}



void host_receive_packet(void)
{
    static unsigned int time_stamp = 0;
	static u16 old_receive_len = 0, new_receive_len = 0;
	u16 i = 0;
	u8 res = 0;
	
    new_receive_len = circulation_buff_available(HOSTIF_USART_RX_DMA_CH, HOSTIF_USART_CIR_BUFF);
	if(old_receive_len != new_receive_len)
	{
		old_receive_len = new_receive_len;
	    time_stamp=run_ms;
	}
	
	if(new_receive_len != 0)
	{
		  
	    if(time_diff_ms(time_stamp) > HOST_UASRT_RX_TIME_OUT)
	    {
		    //DEBUG("HOST time_stamp is %d\r\n", time_stamp);
		    //DEBUG("HOST old_receive_len is %d\r\n", old_receive_len);
			new_receive_len = 0;
			old_receive_len = 0;
			//接收完成
	    	host_rx_len = circulation_buff_read(HOSTIF_USART_RX_DMA_CH, &HOSTIF_USART_CIR_BUFF, host_rx_buff, 512);
				//DEBUG("cmd is %s \r\n", host_rx_buff);
			//在AT指令模式，不进行数据发送
			if((host_rx_buff[0] == 'A')  && (host_rx_buff[1] == 'T') && (host_rx_buff[host_rx_len-2] == '\r') && (host_rx_buff[host_rx_len-1] == '\n'))
			{
			    if(host_rx_len == 4)
				  {
				      USART_dma_send(HOSTIF_USART_TX_DMA_CH, "OK\r\n", 4);
				  }
				  else if(host_rx_len > 7)
				  {
				      //识别命令
				  	  res = cmd_process(&host_rx_buff[3], (host_rx_len -3));
							//DEBUG("\r\nres=%d\r\n",res);
				  }
			}
			
			//处理接收的数据
			if(para_value.word_mode == WOKE_MODE_AT)
			{
			
			}
			if(para_value.word_mode == WOKE_MODE_TS)
			{
			    //通过TCP/UDP发送数据，判断出来非命令数据，将其发送出去
			    if((res == 0) && (para_value.on_line_mode != 0))
			    {
						  host_rx_buff[host_rx_len-2] = 0;
						  host_rx_buff[host_rx_len-1] = 0;
						  if(para_value.on_line_mode == 2)
						  {
			                if(check_tcp_udp_connet() == 1)
							  {
							      tcp_udp_send(host_rx_buff, host_rx_len);
							  }
						  }
						  else
						  {
			                if(tcp_udp_auto_line() != 0)
						      {
                               tcp_udp_send(host_rx_buff, host_rx_len);      
						      }
						  }
			    }
			}
			else if(para_value.word_mode == WOKE_MODE_HTTP)
			{
			    //通过HTTP发送数据，判断出来非命令数据，将其发送出去
			    if(res == 0)
			    {
					  host_rx_buff[host_rx_len-2] = '\0';
					  host_rx_buff[host_rx_len-1] = '\0';
			        if(para_value.http_para.type == 0) //post
					  {
					      http_post(para_value.link1, para_value.http_para, host_rx_buff, sizeof(host_rx_buff),1);
					  }
					  else if(para_value.http_para.type == 1) //get
					  {
					  	  if(http_get(para_value.link1, para_value.http_para, host_rx_buff) == 1)
						  {
						      usart_send_str(HOSTIF_USART, http_get_buff, http_get_rx_len);
						  }
					  }
			    }
			}
			else if(para_value.word_mode == WOKE_MODE_MODBUS_HTTP)
			{
			    //HTTP MODBUS 模式，这里要判断是什么数据，
			    if((res == 0) && (modbus_get_flog == 0))
			    {
                    for(i=0;i<(host_rx_len);i++)
				    {
				        modbus_http_post_buff[modbus_http_post_len++] = host_rx_buff[i];
				    }
			    }
			}
			
			memset(host_rx_buff, 0, host_rx_len);
			host_rx_len = 0;
				
				  
	    		//DEBUG("HOST usart rx time out\r\n");
	    }
	}
	
}

