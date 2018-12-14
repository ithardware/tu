#include "control.h"
#include "timer.h"
#include "delay.h"
#include "save_data.h"
#include "sim800c.h"
#include "hostif.h"
#include "string.h"
#define Modbus_Commands_Length 16
#define Post_Buffer_Length 4096
#define Get_Modbus_Commands_Split_Zero_Count 8

unsigned int post_action_send_modbus_commands_Index = 0;
unsigned int modbus_get_flog = 0;

unsigned char modbus_http_post_buff[Post_Buffer_Length] = {0x00};
unsigned int modbus_http_post_len = 0;


void woke_mode_at_main(void)
{
	//初始化该模式数据
	
    while(1)
    {
        host_receive_packet();
        SIM8XX_User_Receive();

        if(para_value.word_mode != WOKE_MODE_AT)
        {
            break;
        }
    }
}

void woke_mode_ts_main(void)
{
	  int res = 0,i = 0;
	  static unsigned int check_tcp_udp_connect_stime = 0;
	  static unsigned int heart_beat_stime = 0;
	  static unsigned int tcp_udp_off_line_stime = 0;
	  //初始化该模式数据
	  check_tcp_udp_connect_stime = 0;
	  heart_beat_stime = 0;
	  check_tcp_udp_connect_stime = 0;
	  while(1)
		{
			  res = 0;
			  i = 0;
	      host_receive_packet();
		    SIM8XX_User_Receive();
	      if(para_value.on_line_mode ==1)//永远在线模式
		    {
		        if(time_diff_ms(check_tcp_udp_connect_stime) > CHECK_TCP_UDP_CONNECT_TIMEOUT)
		    		{
							  check_tcp_udp_connect_stime = time_get_ms();
                tcp_udp_auto_line();
		    		    tcp_udp_off_line_stime = time_get_ms();
		    		}
		    }
		    //唤醒在线模式或者按需在线模式
		    //这两种模式要隔一段时间自动下线
		    else if((para_value.on_line_mode == 2) || (para_value.on_line_mode == 3))
		    {
		        //判断自动下线时间是否到了
		    	  if(time_diff_ms(tcp_udp_off_line_stime) > (para_value.auto_off_line_time * 1000))
		    		{
		    			  if(check_tcp_udp_connet() == 1)//如果tcp在线，则离线
		    				{
		    				    off_line();
		    				}
		    			  
		    		    tcp_udp_off_line_stime = time_get_ms();
		    		}
		    }
		    if(tcp_udp_rx_data_flog == 1)
		    {
		    	  memset(host_tx_buff, 0, HOST_TX_BUFF_LEN);
		        res = tcp_udp_rx_data(host_tx_buff);
		    	  usart_send_str(HOSTIF_USART, host_tx_buff, (u16)res);
		    	  tcp_udp_rx_data_flog = 0;
		    }
		    if(first_tcp_udp_connect_flog == 1)
		    {
		    	  //发送注册包信息
		    	  if(para_value.reg_packet_switch == 1)
		    		{
		    		    for(i=0; i<(para_value.reg_head_len+para_value.id_len); i++)
		    			  {
		    					  if(i < para_value.reg_head_len)
		    						{
		    						    host_tx_buff[i] = para_value.reg_head[i];
		    						}
		    						else
		    						{
		    						    host_tx_buff[i] = para_value.id[i-para_value.reg_head_len];
		    						}
		    				    
		    					
		    				}
		    			  tcp_udp_send(host_tx_buff, (para_value.reg_head_len+para_value.id_len));
		    		}
		    	  
		    	  
		    	  //读取建立连接的时间
		    	  tcp_udp_off_line_stime = time_get_ms();
		    		if(para_value.heartbeat_time > 0)
		        {
		    			  heart_beat_stime = time_get_ms();
		        }
		    		
		        first_tcp_udp_connect_flog = 0;
		    }

		    if(para_value.heartbeat_time == 0)
		    {
		        heart_beat_stime = 0;
		    }
		    else
		    {
		        if(heart_beat_stime == 0)
		    		{
		    		    heart_beat_stime = time_get_ms();
		    		}
		    }

		    //判断心跳包建立时间，发送心跳包
		    if((heart_beat_stime != 0) && (para_value.word_mode != 0))
		    {
		        if(time_diff_ms(heart_beat_stime) > (para_value.heartbeat_time * 1000))
		    		{
		    			
		    			  if(check_tcp_udp_connet() != 0)
		    				{
		    		        for(i=0; i<(para_value.heartbeat_head_len+para_value.id_len); i++)
		    			      {
		    				    	  if(i < para_value.heartbeat_head_len)
		    				    		{
		    				    		    host_tx_buff[i] = para_value.heartbeat_head[i];
		    				    		}
		    				    		else
		    				    		{
		    				    		    host_tx_buff[i] = para_value.id[i-para_value.heartbeat_head_len];
		    				    		}
		    				    }
		    			      tcp_udp_send(host_tx_buff, (para_value.heartbeat_head_len+para_value.id_len));
		    				}
		    				heart_beat_stime = time_get_ms();
		    				tcp_udp_off_line_stime = time_get_ms();
		    		}
		    }
				if(para_value.word_mode != WOKE_MODE_TS)
				{
				    break;
				}
		
		}
	

		
		
}

void woke_mode_http_main(void)
{
	  //初始化该模式数据
	
    while(1)
		{
	      host_receive_packet();
		    SIM8XX_User_Receive();
			
			
			  if(para_value.word_mode != WOKE_MODE_HTTP)
				{
				    break;
				}
		}
}


void modbus_http_get_send(u8 *data, u16 len)
{
    unsigned int wait_modbus_get_stime = 0;
    u16 i=0;
	u16 count = 0;
	if((len%8) == 0)
	{
	    count = len/8;
	}
	else
	{
	    count = len/8+1;
	}
	for(i=0; i<count; i++)
	{
		usart_send_str(HOSTIF_USART, &data[i*8], 8);
        wait_modbus_get_stime = time_get_ms();
        while(time_diff_ms(wait_modbus_get_stime) < MODBUS_GET_RX_TIMEOUT);
        host_rx_len = circulation_buff_read(HOSTIF_USART_RX_DMA_CH, &HOSTIF_USART_CIR_BUFF, host_rx_buff, HOST_RX_BUFF_LEN);
        //清除接收的数据
        memset(host_rx_buff, 0, host_rx_len);
        host_rx_len = 0;
	}
}
void modbus_http_get_send2(u8 *data, u16 len)
{
  unsigned int wait_modbus_get_stime = 0;
	unsigned int zero_count = 0;
  u16 i=0;
	u16 index =0;
	u8 x = 0;
	for(i=0; i<len; i++)
	{
		if(data[i]!=0){
			zero_count = 0;
		}else{
			zero_count++;
		}
		if(Get_Modbus_Commands_Split_Zero_Count == zero_count)
		{
			if(Get_Modbus_Commands_Split_Zero_Count < i){
				for(x=0; x< i-7-index ;x++){
					DEBUG("%2d ",data[index+x]);
				}
				usart_send_str(HOSTIF_USART, &data[index], i-7-index);
				wait_modbus_get_stime = time_get_ms();
        while(time_diff_ms(wait_modbus_get_stime) < MODBUS_GET_RX_TIMEOUT);
        host_rx_len = circulation_buff_read(HOSTIF_USART_RX_DMA_CH, &HOSTIF_USART_CIR_BUFF, host_rx_buff, HOST_RX_BUFF_LEN);
        memset(host_rx_buff, 0, host_rx_len);
        host_rx_len = 0;
			}
			zero_count = 0;
			index = i+1;
		}
	}	
}

void woke_mode_modbus_http_main(void)
{
		static unsigned int work_interval = 0;//Post间隔计数
		static unsigned int modbus_cmd_interval = 0;//modbus命令发送间隔计数
		unsigned int work_flag = 0;//post大周期到来标识 0未到来 1到来
		unsigned int dealy = 0;//延迟
		u16 i = 0;
	//断电测试flag
		u8 power_flag = 0;
    
		unsigned int timer = 0;
    //初始化该模式数据
		work_interval = time_get_ms();
    post_action_send_modbus_commands_Index = 0;
		
		
    while(1)
    {
			host_receive_packet();
      SIM8XX_User_Receive();
			
			
			//1 检测POST时间设置 大于0则进入工作模式
			if(para_value.modbus_qry_post_time > 0){
				if(time_diff_ms(work_interval) > (para_value.modbus_qry_post_time * 1000)){ //检测POST时间间隔的到来 大周期
					//DEBUG("\r\nwork is coming \r\n");
					//POST ACTION 发送串口查询指令开始
					modbus_cmd_interval = time_get_ms(); //初始化modbus 命令间隔计数
					work_interval = time_get_ms(); //重置POST时间间隔计数器，用于不停检测下次大周期是否到来
					work_flag = 1;
					
					/** 断电测试
					if(power_flag){
						power_flag = 0;
						DEBUG("断电\r\n");
						SIM_PWR_Close();
						para_value.modbus_qry_post_time=300;
					}
					else{
						power_flag = 1;
						DEBUG("上电\r\n");
						SIM_PWR_Open();
						para_value.modbus_qry_post_time=30;
					}
					**/
					/**
					char 打印测试
					**/
					DEBUG("char -1->%d\r\n",((char)-1));
					DEBUG("char -2->%d\r\n",((char)-2));
					DEBUG("char -3->%d\r\n",((char)-3));
					DEBUG("char -4->%d\r\n",((char)-4));
					DEBUG("char -5->%d\r\n",((char)-5));
					DEBUG("char -6->%d\r\n",((char)-6));
					DEBUG("char -7->%d\r\n",((char)-7));
					DEBUG("char -8->%d\r\n",((char)-8));
					continue;
				}
				if(work_flag && time_diff_ms(modbus_cmd_interval) > para_value.modbus_qry_cmd_time){//检测大周期到来且发送modbus命令间隔时间的到来
					//DEBUG("\r\ncmd index %d\r\n",post_action_send_modbus_commands_Index);
					if(Modbus_Commands_Length == post_action_send_modbus_commands_Index){//如果命令索引超过总数，命令索引归0,调用HTTP POST
						post_action_send_modbus_commands_Index = 0;
						work_flag = 0;
						DEBUG("\r\nhttp post data %d server1 %d server2 %d \r\n",modbus_http_post_len,para_value.link1.connect_type,para_value.link2.connect_type);
						//判断是否需要执行HTTP POST请求
						//sprintf("");
						if(modbus_http_post_len){//接收modbus查询命令返回的数据不为空
							if(para_value.link1.connect_type == 0)//TCP连接有效，执行HTTP POST
							{
								http_post(para_value.link1, para_value.modbus_http_post_para, modbus_http_post_buff, modbus_http_post_len);
								/**
								清理HTTP POST 缓冲区，不可使用HTTP GET中的（清理HTTP POST 缓冲区）操作替换。
								因为HTTP GET 行为并不一定会发生
								**/
								modbus_http_post_len = 0;
								memset(modbus_http_post_buff, 0, Post_Buffer_Length);
								DEBUG("\r\nPOST\r\n");
								dealy = time_get_ms();
								while(time_diff_ms(dealy) < 5000);
							}
							//清理接收的modbus查询命令返回的数据
							modbus_http_post_len = 0;
							memset(modbus_http_post_buff, 0, Post_Buffer_Length);
						}
						//执行get请求
						if(para_value.link2.connect_type == 0)//TCP连接
        		{
							if(http_get(para_value.link2, para_value.modbus_http_get_para, para_value.modbus_qry_get_para.buff) == 1)
							{
								DEBUG("\r\nGET\r\n");
								modbus_http_get_send(http_get_buff, http_get_rx_len);
								/**
								GET 获取指令发送后，延迟MODBUS_GET_RX_TIMEOUT后清理HTTP POST 数据缓冲区。
								在此之前存在GET到的指令发送后，设备反馈数据污染HTTP POST数据缓冲区的情况。
								**/
								dealy = time_get_ms();
								while(time_diff_ms(dealy) < MODBUS_GET_RX_TIMEOUT);
								modbus_http_post_len = 0;
								memset(modbus_http_post_buff, 0, Post_Buffer_Length);
							}
						}
					}
					else{
						if(para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len > 0){//命令长度有效发送命令
							usart_send_str(HOSTIF_USART, 
								para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].buff, 
								para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len);
								post_action_send_modbus_commands_Index++;//命令计数++
						}else{
							post_action_send_modbus_commands_Index = Modbus_Commands_Length;//遇到空命令直接跳过
						}
						modbus_cmd_interval = time_get_ms(); //重置modbus 命令间隔计数，用于不停检测下次命令发送间隔是否到来
					}
				}
			}
		
			if(para_value.word_mode != WOKE_MODE_MODBUS_HTTP)
			{
				break;
			}
		}
}
