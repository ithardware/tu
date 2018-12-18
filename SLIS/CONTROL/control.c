#include "control.h"
#include "timer.h"
#include "delay.h"
#include "save_data.h"
#include "sim800c.h"
#include "hostif.h"
#include "string.h"

#define Setting_Modbus_Command_Length 13 //Modbus 设置命令的基本长度，1个命令需要满足13个字节
#define Setting_Modbus_Command_Type_Single 0x06 //modbus单点位设置指令
#define Setting_Modbus_Command_Type_Double 0x06 //modbus双点位设置指令

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

#if 1
//以13个字节做基本指令长度，10指令操作一个浮点数点位需要13个字节
void modbus_http_get_send(u8 *data, u16 len)
{
		DEBUG("haha1\r\n");
    unsigned int wait_modbus_get_stime = 0;
    u16 i=0,x =0,lenx = 0;
    u16 count = count = len/Setting_Modbus_Command_Length;;
    /**
	if((len%Setting_Modbus_Command_Length) == 0)
    {
        count = len/Setting_Modbus_Command_Length;
    }
	**/
		DEBUG("count:%d\r\n",count);
    for(i=0; i<count; i++)
    {
				lenx = 0;
        //如果单点位设置指令
        if(Setting_Modbus_Command_Type_Single == data[i*Setting_Modbus_Command_Length+1]) {
            usart_send_str(HOSTIF_USART, &data[i*Setting_Modbus_Command_Length], 8);
						delay_ms(200);
						DEBUG("\r\n single command:");
						for(x=i*Setting_Modbus_Command_Length; lenx < 8 ; x++,lenx++) {
							DEBUG("%02x ",data[x]);
						}
						DEBUG("\r\n");
        }
        else if(Setting_Modbus_Command_Type_Double == data[i*Setting_Modbus_Command_Length+1])
        {
            usart_send_str(HOSTIF_USART, &data[i*Setting_Modbus_Command_Length], 13);
						DEBUG("\r\n double command:");
					for(x=i*Setting_Modbus_Command_Length; lenx < 13 ; x++,lenx++) {
						DEBUG("%02x ",data[x]);
          }
					DEBUG("\r\n");
        }
        else {
						DEBUG("\r\nno cmd\r\n");
            continue;
        }
        delay_ms(200);
        host_rx_len = circulation_buff_read(HOSTIF_USART_RX_DMA_CH, &HOSTIF_USART_CIR_BUFF, host_rx_buff, HOST_RX_BUFF_LEN);
        //清除接收的数据
        memset(host_rx_buff, 0, host_rx_len);
        host_rx_len = 0;
#if 0
        usart_send_str(HOSTIF_USART, &data[i*8], 8);
        //wait_modbus_get_stime = time_get_ms();
        //while(time_diff_ms(wait_modbus_get_stime) < MODBUS_GET_RX_TIMEOUT);
        delay_ms(200);
        host_rx_len = circulation_buff_read(HOSTIF_USART_RX_DMA_CH, &HOSTIF_USART_CIR_BUFF, host_rx_buff, HOST_RX_BUFF_LEN);
        //清除接收的数据
        memset(host_rx_buff, 0, host_rx_len);
        host_rx_len = 0;
        //delay_ms(100);//延迟100ms退出，防止设备返回数据污染POST数据缓存区
#endif
    }
}
#endif
#if 0
void modbus_http_get_send(u8 *data, u16 len)
{
    unsigned int wait_modbus_get_stime = 0;
    unsigned int zero_count = 0;
    u16 i=0;
    u16 index =0;
    u8 x = 0;
    for(i=0; i<len; i++)
    {
        if(data[i]!=0) {
            zero_count = 0;
        } else {
            zero_count++;
        }
        if(Get_Modbus_Commands_Split_Zero_Count == zero_count)
        {
            if(Get_Modbus_Commands_Split_Zero_Count < i) {
                for(x=0; x< i-7-index ; x++) {
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
#endif

void woke_mode_modbus_http_main(void)
{
    static unsigned int work_interval = 0;//Post间隔计数
    static unsigned int modbus_cmd_interval = 0;//modbus命令发送间隔计数
    unsigned int work_flag = 0;//post大周期到来标识 0未到来 1到来
    unsigned int all_post_count = 0;//所有post次数
    unsigned int all_get_count = 0;//所有get次数
    unsigned int all_init_gprs_count = 0;//所有init gprs的次数
    unsigned int init_gprs_success_count = 0;//init gprs成功的次数
    u8 http_connect_count = 0;

    //初始化该模式数据
    work_interval = time_get_ms();
    post_action_send_modbus_commands_Index = 0;


    while(1)
    {
        host_receive_packet();
        SIM8XX_User_Receive();


        //1 检测POST时间设置 大于0则进入工作模式
        if(para_value.modbus_qry_post_time > 0) {

            if(!work_flag && time_diff_ms(work_interval) > (para_value.modbus_qry_post_time * 1000)) { //检测POST时间间隔的到来 大周期

                //进入大周期后，先清理POST缓冲区，防止数据污染
                modbus_http_post_len = 0;
                memset(modbus_http_post_buff, 0, Post_Buffer_Length);

                work_flag = 1;
                //POST ACTION 发送串口查询指令开始
                modbus_cmd_interval = time_get_ms(); //初始化modbus 命令间隔计数
                work_interval = time_get_ms(); //重置POST时间间隔计数器，用于不停检测下次大周期是否到来
            }
            //检测大周期到来且发送modbus命令间隔时间的到来
            if(work_flag && time_diff_ms(modbus_cmd_interval) > para_value.modbus_qry_cmd_time) {
                //DEBUG("\r\ncmd index %d\r\n",post_action_send_modbus_commands_Index);
                //如果命令索引超过总数，命令索引归0,调用HTTP POST
                if(Modbus_Commands_Length == post_action_send_modbus_commands_Index) {
                    post_action_send_modbus_commands_Index = 0;
                    work_flag = 0;
                    //DEBUG("\r\nhttp post data %d server1 %d server2 %d \r\n",modbus_http_post_len,para_value.link1.connect_type,para_value.link2.connect_type);
                    if(!para_value.link1.connect_type || !para_value.link2.connect_type)
                    {
                        all_init_gprs_count++;
                        if(!init_gprs())//打开grps承载失败
                        {
                            init_gprs_success_count++;
                            if(!para_value.link1.connect_type && modbus_http_post_len)
                            {
                                modbus_get_flog = 0;//修改get操作标识，防止uart2收到的数据放入modbus_http_post_buff;
                                all_post_count++;
                                u8 code = http_post(para_value.link1, para_value.modbus_http_post_para, modbus_http_post_buff, modbus_http_post_len);
                                modbus_http_post_len = 0;
                                memset(modbus_http_post_buff, 0, Post_Buffer_Length);
                                DEBUG(",all:%d,this code:%d\r\n",all_post_count,code);
                            }
                            if(!para_value.link2.connect_type)
                            {
                                modbus_get_flog = 1;
                                all_get_count++;
                                modbus_get_flog = 1;//修改get操作标识，防止uart2收到的数据放入modbus_http_post_buff;

                                u8 code = http_get(para_value.link2, para_value.modbus_http_get_para, para_value.modbus_qry_get_para.buff);
                                DEBUG(",all:%d,this code:%d",all_get_count,code);
                                if(!code)
                                {
                                    DEBUG("\r\nhttp_get_rx_len:%d\r\n",http_get_rx_len);
                                    modbus_http_get_send(http_get_buff, http_get_rx_len);
                                } else
                                {
                                    DEBUG("\r\n");
                                }
                                modbus_get_flog = 0;
                            }
                        }
												close_gprs();
                        DEBUG("\r\ninit gprs:%d success:%d\r\n",all_init_gprs_count,init_gprs_success_count);
                    }
                }
                else {
                    if(para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len > 0) { //命令长度有效发送命令
                        usart_send_str(HOSTIF_USART,
                                       para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].buff,
                                       para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len);
                        post_action_send_modbus_commands_Index++;//命令计数++
                    } else {
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
#if 0
void woke_mode_modbus_http_main(void)
{
    static unsigned int work_interval = 0;//Post间隔计数
    static unsigned int modbus_cmd_interval = 0;//modbus命令发送间隔计数
    u8 main_loop_idle_flag = 1;//主循环是否处于空闲状态，默认为空闲状态
    unsigned int all_post_count = 0;//所有post次数
    unsigned int all_get_count = 0;//所有get次数
    unsigned int all_init_gprs_count = 0;//所有init gprs的次数
    unsigned int init_gprs_fail_count = 0;//init gprs错误的次数
    //初始化该模式数据
    work_interval = time_get_ms();
    post_action_send_modbus_commands_Index = 0;


    while(1)
    {
        host_receive_packet();
        SIM8XX_User_Receive();

        //1 检测POST时间设置 大于0则进入工作模式
        if(para_value.modbus_qry_post_time > 0) {
            //主循环空闲且大循环周期已达到
            if(main_loop_idle_flag && time_diff_ms(work_interval)>para_value.modbus_qry_post_time * 1000) {
                memset(modbus_http_post_buff, 0, Post_Buffer_Length);//清理数据缓存
                main_loop_idle_flag = 0;//不在空闲，进入工作状态
                modbus_cmd_interval = time_get_ms(); //初始化modbus 命令间隔计数
                DEBUG("modbus_qry_cmd_time %d\r\n",para_value.modbus_qry_cmd_time);
            }
            if(!main_loop_idle_flag && time_diff_ms(modbus_cmd_interval) > para_value.modbus_qry_cmd_time) {
                //如果modbus指令发送完毕，进入http请求发送环节
                if(Modbus_Commands_Length == post_action_send_modbus_commands_Index) {
                    post_action_send_modbus_commands_Index = 0;
                    if(!para_value.link1.connect_type || !para_value.link2.connect_type)
                    {
                        all_init_gprs_count++;
                        if(!init_gprs())//打开grps承载失败
                        {
                            if(!para_value.link1.connect_type && modbus_http_post_len)
                            {
                                all_post_count++;
                                modbus_get_flog = 0;//修改get操作标识，防止uart2收到的数据放入modbus_http_post_buff;
                                int code = http_post(para_value.link1, para_value.modbus_http_post_para, modbus_http_post_buff, modbus_http_post_len);
                                DEBUG(",all:%d,this code:%d\r\n",all_post_count,code);
                            }

                            if(!para_value.link2.connect_type)
                            {
                                all_get_count++;
                                modbus_get_flog = 1;//修改get操作标识，防止uart2收到的数据放入modbus_http_post_buff;
                                int code = http_get(para_value.link2, para_value.modbus_http_get_para, para_value.modbus_qry_get_para.buff);
                                DEBUG(",all:%d,this code:%d",all_get_count,code);
                                if(!code)
                                {
                                    DEBUG(",http get len:%d\r\n",http_get_rx_len);
                                    modbus_http_get_send(http_get_buff, http_get_rx_len);
                                }
                                else
                                {
                                    DEBUG("\r\n");
                                }
                                modbus_get_flog = 0;
                            }
                        }
                        else
                        {
                            init_gprs_fail_count++;
                        }
                        DEBUG("init grps count:%d,fail:%d\r\n",all_init_gprs_count,init_gprs_fail_count);
                        close_gprs();
                        main_loop_idle_flag = 1;//在此进入空闲状态
                        work_interval = time_get_ms();
                        //DEBUG("2222222222222222222\r\n");
                    }
                }
                else//发送modbus命令
                {
                    //命令长度有效则发送命令
                    if(para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len > 0) {
                        /**usart_send_str(HOSTIF_USART,
                                         para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].buff,
                                         para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len);
                        **/
                        DEBUG("\r\ncmd %d len %d %s\r\n",
                              post_action_send_modbus_commands_Index,
                              para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len,
                              para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].buff);
                        post_action_send_modbus_commands_Index++;//命令计数++
                    }
                    else//无效命令直接导致，命令发送退出
                    {
                        post_action_send_modbus_commands_Index = Modbus_Commands_Length;//遇到空命令直接跳过
                    }
                    //重置modbus 命令间隔计数，用于不停检测下次命令发送间隔是否到来
                    modbus_cmd_interval = time_get_ms();
                }
            }
        }
        if(para_value.word_mode != WOKE_MODE_MODBUS_HTTP)
        {
            break;
        }
    }
}
#endif

