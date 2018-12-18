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

//以13个字节做基本指令长度，10指令操作一个浮点数点位需要13个字节
void modbus_http_get_send(u8 *data, u16 len)
{
    u16 i=0;
    DEBUG("\r\nsend get commands:");
    for(i=0; i < len ; i++) {
        DEBUG("%02x ",data[i]);
    }
    DEBUG("\r\n");
    u16 count = count = len/Setting_Modbus_Command_Length;
    for(i=0; i<count; i++)
    {
        //如果单点位设置指令
        if(Setting_Modbus_Command_Type_Single == data[i*Setting_Modbus_Command_Length+1]) {
            usart_send_str(HOSTIF_USART, &data[i*Setting_Modbus_Command_Length], 8);
        }
        else if(Setting_Modbus_Command_Type_Double == data[i*Setting_Modbus_Command_Length+1])
        {
            usart_send_str(HOSTIF_USART, &data[i*Setting_Modbus_Command_Length], 13);
        }
        else {
            continue;
        }
        delay_ms(200);
        host_rx_len = circulation_buff_read(HOSTIF_USART_RX_DMA_CH, &HOSTIF_USART_CIR_BUFF, host_rx_buff, HOST_RX_BUFF_LEN);
        //清除接收的数据
        memset(host_rx_buff, 0, host_rx_len);
        host_rx_len = 0;
    }
}

void woke_mode_modbus_http_main(void)
{
    unsigned int work_interval = 0;//Post间隔计数
    unsigned int modbus_cmd_interval = 0;//modbus命令发送间隔计数
    unsigned int work_flag = 0;//post大周期到来标识 0未到来 1到来
		unsigned int http_post_count = 0;
		unsigned int http_get_count = 0;
		unsigned int gprs_init_count = 0;
		unsigned int gprs_init_fail_count = 0;
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
								DEBUG("\r\nwork starting\r\n");
            }
            //检测大周期到来且发送modbus命令间隔时间的到来
            if(work_flag && time_diff_ms(modbus_cmd_interval) > para_value.modbus_qry_cmd_time) {
                
                //如果命令索引超过总数，命令索引归0,调用HTTP POST
                if(Modbus_Commands_Length == post_action_send_modbus_commands_Index) {
										DEBUG("\r\nwork cmds end\r\n");
                    post_action_send_modbus_commands_Index = 0;
                    //work_flag = 0;
                    if(!para_value.link1.connect_type || !para_value.link2.connect_type)
                    {
												gprs_init_count ++;
                        if(!init_gprs())//打开grps承载失败
                        {
                            if(!para_value.link1.connect_type && modbus_http_post_len)
                            {
																http_post_count++;
																DEBUG("\r\ndo http post %d\r\n",http_post_count);
															
                                modbus_get_flog = 0;//修改get操作标识，防止uart2收到的数据放入modbus_http_post_buff;
                                
                                u8 code = http_post(para_value.link1, para_value.modbus_http_post_para, modbus_http_post_buff, modbus_http_post_len);
                                modbus_http_post_len = 0;
                                memset(modbus_http_post_buff, 0, Post_Buffer_Length);
                            }
                            if(!para_value.link2.connect_type)
                            {
																http_get_count++;
																DEBUG("\r\ndo http get %d\r\n",http_get_count);
                                modbus_get_flog = 1;//修改get操作标识，防止uart2收到的数据放入modbus_http_post_buff;

                                u8 code = http_get(para_value.link2, para_value.modbus_http_get_para, para_value.modbus_qry_get_para.buff);
                                if(!code)
                                {
                                    modbus_http_get_send(http_get_buff, http_get_rx_len);
                                } else
                                {
                                    DEBUG("\r\n");
                                }
                                modbus_get_flog = 0;
                            }
                        }
												else
												{
														gprs_init_fail_count++;
												}
                        close_gprs();//无论是否init_gprs成功，都要执行关闭。如只在成功init_gprs后执行，会出现频繁断网的情况。
												DEBUG("\r\ngrps init %d,fail %d\r\n",gprs_init_count,gprs_init_fail_count);
										}
										DEBUG("\r\nwork stop\r\n");
										work_flag = 0;
								}
                else {
                    if(para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len > 0) { //命令长度有效发送命令
                      //DEBUG("\r\nwork cmd is %02d\r\n",post_action_send_modbus_commands_Index);  
											usart_send_str(HOSTIF_USART,
                                       para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].buff,
                                       para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len);
                        post_action_send_modbus_commands_Index++;//命令计数++
                    } else {
												DEBUG("\r\nwork cmd break in %02d\r\n",post_action_send_modbus_commands_Index);
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
