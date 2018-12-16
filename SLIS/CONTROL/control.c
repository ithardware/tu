#include "control.h"
#include "timer.h"
#include "delay.h"
#include "save_data.h"
#include "sim800c.h"
#include "hostif.h"
#include "string.h"


unsigned int post_action_send_modbus_commands_Index = 0;
unsigned int get_send_modbus_command_working_flag = 0; //get方式正在发送modbus命令标示，该标示不为0时 post不能工作
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
				delay_ms(100);//延迟100ms退出，防止设备返回数据污染POST数据缓存区
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
		unsigned int all_post_count = 0;//所有post次数
		unsigned int all_get_count = 0;//所有get次数
		
	//http_connect_count 一个周期内 http 连接次数
	u8 http_connect_count = 0;
    
		//unsigned int timer = 0;
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
					//统计周期内http连接次数，确保在完成必要的连接次数后关闭 http 连接承载
					 http_connect_count = (para_value.link1.connect_type==0?1:0)
					+
					(para_value.link2.connect_type==0?1:0);
					//DEBUG("http count %d \r\n",http_connect_count);
					//进入大周期后，先清理POST缓冲区，防止数据污染
					modbus_http_post_len = 0;
					memset(modbus_http_post_buff, 0, Post_Buffer_Length);
					
					work_flag = 1;
					//POST ACTION 发送串口查询指令开始
					modbus_cmd_interval = time_get_ms(); //初始化modbus 命令间隔计数
					work_interval = time_get_ms(); //重置POST时间间隔计数器，用于不停检测下次大周期是否到来
				}
				//检测大周期到来且发送modbus命令间隔时间的到来
				if(work_flag && time_diff_ms(modbus_cmd_interval) > para_value.modbus_qry_cmd_time){
					//DEBUG("\r\ncmd index %d\r\n",post_action_send_modbus_commands_Index);
					//如果命令索引超过总数，命令索引归0,调用HTTP POST
					if(Modbus_Commands_Length == post_action_send_modbus_commands_Index){
						post_action_send_modbus_commands_Index = 0;
						work_flag = 0;
						//DEBUG("\r\nhttp post data %d server1 %d server2 %d \r\n",modbus_http_post_len,para_value.link1.connect_type,para_value.link2.connect_type);
						//判断是否需要执行HTTP POST请求
						if(modbus_http_post_len){//接收modbus查询命令返回的数据不为空
							modbus_get_flog=0;//修改get操作标识
							all_post_count++;
							DEBUG(",all:%d,this code:%d\r\n",all_post_count,
							http_post(para_value.link1, para_value.modbus_http_post_para, modbus_http_post_buff, modbus_http_post_len,http_connect_count));
								/**
								清理HTTP POST 缓冲区，不可使用HTTP GET中的（清理HTTP POST 缓冲区）操作替换。
								因为HTTP GET 行为并不一定会发生
								**/
								modbus_http_post_len = 0;
								memset(modbus_http_post_buff, 0, Post_Buffer_Length);
								delay_ms(2000);
						}
							//清理接收的modbus查询命令返回的数据
							modbus_http_post_len = 0;
							memset(modbus_http_post_buff, 0, Post_Buffer_Length);
						
						//执行get请求
							all_get_count++;
							modbus_get_flog = 1;//修改get操作标识，防止uart2收到的数据放入modbus_http_post_buff;
							int code = http_get(para_value.link2, para_value.modbus_http_get_para, para_value.modbus_qry_get_para.buff);
							DEBUG(",all:%d,this code:%d\r\n",all_get_count,code);
							if(!code)
							{
								DEBUG("http_get_rx_len:%d\r\n",http_get_rx_len);
								modbus_http_get_send(http_get_buff, http_get_rx_len);
							}
							modbus_get_flog = 0;
						
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

