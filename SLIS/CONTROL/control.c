#include "control.h"
#include "timer.h"
#include "delay.h"
#include "save_data.h"
#include "sim800c.h"
#include "hostif.h"
#include "string.h"


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
        //������յ�����
        memset(host_rx_buff, 0, host_rx_len);
        host_rx_len = 0;
				delay_ms(100);//�ӳ�100ms�˳�����ֹ�豸����������ȾPOST���ݻ�����
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
		static unsigned int work_interval = 0;//Post�������
		static unsigned int modbus_cmd_interval = 0;//modbus����ͼ������
		unsigned int work_flag = 0;//post�����ڵ�����ʶ 0δ���� 1����
		unsigned int all_post_count = 0;//����post����
		unsigned int all_get_count = 0;//����get����
		
	//http_connect_count һ�������� http ���Ӵ���
	u8 http_connect_count = 0;
    
		//unsigned int timer = 0;
    //��ʼ����ģʽ����
		work_interval = time_get_ms();
    post_action_send_modbus_commands_Index = 0;
		
		
    while(1)
    {
			host_receive_packet();
      SIM8XX_User_Receive();
			
			
			//1 ���POSTʱ������ ����0����빤��ģʽ
			if(para_value.modbus_qry_post_time > 0){

				if(time_diff_ms(work_interval) > (para_value.modbus_qry_post_time * 1000)){ //���POSTʱ�����ĵ��� ������
					//ͳ��������http���Ӵ�����ȷ������ɱ�Ҫ�����Ӵ�����ر� http ���ӳ���
					 http_connect_count = (para_value.link1.connect_type==0?1:0)
					+
					(para_value.link2.connect_type==0?1:0);
					//DEBUG("http count %d \r\n",http_connect_count);
					//��������ں�������POST����������ֹ������Ⱦ
					modbus_http_post_len = 0;
					memset(modbus_http_post_buff, 0, Post_Buffer_Length);
					
					work_flag = 1;
					//POST ACTION ���ʹ��ڲ�ѯָ�ʼ
					modbus_cmd_interval = time_get_ms(); //��ʼ��modbus ����������
					work_interval = time_get_ms(); //����POSTʱ���������������ڲ�ͣ����´δ������Ƿ���
				}
				//�������ڵ����ҷ���modbus������ʱ��ĵ���
				if(work_flag && time_diff_ms(modbus_cmd_interval) > para_value.modbus_qry_cmd_time){
					//DEBUG("\r\ncmd index %d\r\n",post_action_send_modbus_commands_Index);
					//�������������������������������0,����HTTP POST
					if(Modbus_Commands_Length == post_action_send_modbus_commands_Index){
						post_action_send_modbus_commands_Index = 0;
						work_flag = 0;
						//DEBUG("\r\nhttp post data %d server1 %d server2 %d \r\n",modbus_http_post_len,para_value.link1.connect_type,para_value.link2.connect_type);
						//�ж��Ƿ���Ҫִ��HTTP POST����
						if(modbus_http_post_len){//����modbus��ѯ����ص����ݲ�Ϊ��
							modbus_get_flog=0;//�޸�get������ʶ
							all_post_count++;
							DEBUG(",all:%d,this code:%d\r\n",all_post_count,
							http_post(para_value.link1, para_value.modbus_http_post_para, modbus_http_post_buff, modbus_http_post_len,http_connect_count));
								/**
								����HTTP POST ������������ʹ��HTTP GET�еģ�����HTTP POST �������������滻��
								��ΪHTTP GET ��Ϊ����һ���ᷢ��
								**/
								modbus_http_post_len = 0;
								memset(modbus_http_post_buff, 0, Post_Buffer_Length);
								delay_ms(2000);
						}
							//������յ�modbus��ѯ����ص�����
							modbus_http_post_len = 0;
							memset(modbus_http_post_buff, 0, Post_Buffer_Length);
						
						//ִ��get����
							all_get_count++;
							modbus_get_flog = 1;//�޸�get������ʶ����ֹuart2�յ������ݷ���modbus_http_post_buff;
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
						if(para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len > 0){//�������Ч��������
							usart_send_str(HOSTIF_USART, 
								para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].buff, 
								para_value.modbus_qry_post_para[post_action_send_modbus_commands_Index].len);
								post_action_send_modbus_commands_Index++;//�������++
						}else{
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

