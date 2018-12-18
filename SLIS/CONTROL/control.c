#include "control.h"
#include "timer.h"
#include "delay.h"
#include "save_data.h"
#include "sim800c.h"
#include "hostif.h"
#include "string.h"

#define Setting_Modbus_Command_Length 13 //Modbus ��������Ļ������ȣ�1��������Ҫ����13���ֽ�
#define Setting_Modbus_Command_Type_Single 0x06 //modbus����λ����ָ��
#define Setting_Modbus_Command_Type_Double 0x06 //modbus˫��λ����ָ��

unsigned int post_action_send_modbus_commands_Index = 0;
unsigned int get_send_modbus_command_working_flag = 0; //get��ʽ���ڷ���modbus�����ʾ���ñ�ʾ��Ϊ0ʱ post���ܹ���
unsigned int modbus_get_flog = 0;
unsigned char modbus_http_post_buff[Post_Buffer_Length] = {0x00};
unsigned int modbus_http_post_len = 0;

void woke_mode_at_main(void)
{
    //��ʼ����ģʽ����
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

//��13���ֽ�������ָ��ȣ�10ָ�����һ����������λ��Ҫ13���ֽ�
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
        //�������λ����ָ��
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
        //������յ�����
        memset(host_rx_buff, 0, host_rx_len);
        host_rx_len = 0;
    }
}

void woke_mode_modbus_http_main(void)
{
    unsigned int work_interval = 0;//Post�������
    unsigned int modbus_cmd_interval = 0;//modbus����ͼ������
    unsigned int work_flag = 0;//post�����ڵ�����ʶ 0δ���� 1����
		unsigned int http_post_count = 0;
		unsigned int http_get_count = 0;
		unsigned int gprs_init_count = 0;
		unsigned int gprs_init_fail_count = 0;
    //��ʼ����ģʽ����
    work_interval = time_get_ms();
    post_action_send_modbus_commands_Index = 0;


    while(1)
    {
        host_receive_packet();
        SIM8XX_User_Receive();


        //1 ���POSTʱ������ ����0����빤��ģʽ
        if(para_value.modbus_qry_post_time > 0) {

            if(!work_flag && time_diff_ms(work_interval) > (para_value.modbus_qry_post_time * 1000)) { //���POSTʱ�����ĵ��� ������
								//��������ں�������POST����������ֹ������Ⱦ
                modbus_http_post_len = 0;
                memset(modbus_http_post_buff, 0, Post_Buffer_Length);

                work_flag = 1;
                //POST ACTION ���ʹ��ڲ�ѯָ�ʼ
                modbus_cmd_interval = time_get_ms(); //��ʼ��modbus ����������
                work_interval = time_get_ms(); //����POSTʱ���������������ڲ�ͣ����´δ������Ƿ���
								DEBUG("\r\nwork starting\r\n");
            }
            //�������ڵ����ҷ���modbus������ʱ��ĵ���
            if(work_flag && time_diff_ms(modbus_cmd_interval) > para_value.modbus_qry_cmd_time) {
                
                //�������������������������������0,����HTTP POST
                if(Modbus_Commands_Length == post_action_send_modbus_commands_Index) {
										DEBUG("\r\nwork cmds end\r\n");
                    post_action_send_modbus_commands_Index = 0;
                    //work_flag = 0;
                    if(!para_value.link1.connect_type || !para_value.link2.connect_type)
                    {
												gprs_init_count ++;
                        if(!init_gprs())//��grps����ʧ��
                        {
                            if(!para_value.link1.connect_type && modbus_http_post_len)
                            {
																http_post_count++;
																DEBUG("\r\ndo http post %d\r\n",http_post_count);
															
                                modbus_get_flog = 0;//�޸�get������ʶ����ֹuart2�յ������ݷ���modbus_http_post_buff;
                                
                                u8 code = http_post(para_value.link1, para_value.modbus_http_post_para, modbus_http_post_buff, modbus_http_post_len);
                                modbus_http_post_len = 0;
                                memset(modbus_http_post_buff, 0, Post_Buffer_Length);
                            }
                            if(!para_value.link2.connect_type)
                            {
																http_get_count++;
																DEBUG("\r\ndo http get %d\r\n",http_get_count);
                                modbus_get_flog = 1;//�޸�get������ʶ����ֹuart2�յ������ݷ���modbus_http_post_buff;

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
                        close_gprs();//�����Ƿ�init_gprs�ɹ�����Ҫִ�йرա���ֻ�ڳɹ�init_gprs��ִ�У������Ƶ�������������
												DEBUG("\r\ngrps init %d,fail %d\r\n",gprs_init_count,gprs_init_fail_count);
										}
										DEBUG("\r\nwork stop\r\n");
										work_flag = 0;
								}
                else {
                    if(para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len > 0) { //�������Ч��������
                      //DEBUG("\r\nwork cmd is %02d\r\n",post_action_send_modbus_commands_Index);  
											usart_send_str(HOSTIF_USART,
                                       para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].buff,
                                       para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len);
                        post_action_send_modbus_commands_Index++;//�������++
                    } else {
												DEBUG("\r\nwork cmd break in %02d\r\n",post_action_send_modbus_commands_Index);
                        post_action_send_modbus_commands_Index = Modbus_Commands_Length;//����������ֱ������
                    }
                    modbus_cmd_interval = time_get_ms(); //����modbus ���������������ڲ�ͣ����´�����ͼ���Ƿ���
                }
            }
        }
        if(para_value.word_mode != WOKE_MODE_MODBUS_HTTP)
        {
            break;
        }
    }
}
