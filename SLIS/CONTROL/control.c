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

void woke_mode_ts_main(void)
{
	  int res = 0,i = 0;
	  static unsigned int check_tcp_udp_connect_stime = 0;
	  static unsigned int heart_beat_stime = 0;
	  static unsigned int tcp_udp_off_line_stime = 0;
	  //��ʼ����ģʽ����
	  check_tcp_udp_connect_stime = 0;
	  heart_beat_stime = 0;
	  check_tcp_udp_connect_stime = 0;
	  while(1)
		{
			  res = 0;
			  i = 0;
	      host_receive_packet();
		    SIM8XX_User_Receive();
	      if(para_value.on_line_mode ==1)//��Զ����ģʽ
		    {
		        if(time_diff_ms(check_tcp_udp_connect_stime) > CHECK_TCP_UDP_CONNECT_TIMEOUT)
		    		{
							  check_tcp_udp_connect_stime = time_get_ms();
                tcp_udp_auto_line();
		    		    tcp_udp_off_line_stime = time_get_ms();
		    		}
		    }
		    //��������ģʽ���߰�������ģʽ
		    //������ģʽҪ��һ��ʱ���Զ�����
		    else if((para_value.on_line_mode == 2) || (para_value.on_line_mode == 3))
		    {
		        //�ж��Զ�����ʱ���Ƿ���
		    	  if(time_diff_ms(tcp_udp_off_line_stime) > (para_value.auto_off_line_time * 1000))
		    		{
		    			  if(check_tcp_udp_connet() == 1)//���tcp���ߣ�������
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
		    	  //����ע�����Ϣ
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
		    	  
		    	  
		    	  //��ȡ�������ӵ�ʱ��
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

		    //�ж�����������ʱ�䣬����������
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
	  //��ʼ����ģʽ����
	
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
        //������յ�����
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
		static unsigned int work_interval = 0;//Post�������
		static unsigned int modbus_cmd_interval = 0;//modbus����ͼ������
		unsigned int work_flag = 0;//post�����ڵ�����ʶ 0δ���� 1����
		unsigned int dealy = 0;//�ӳ�
		u16 i = 0;
	//�ϵ����flag
		u8 power_flag = 0;
    
		unsigned int timer = 0;
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
					//DEBUG("\r\nwork is coming \r\n");
					//POST ACTION ���ʹ��ڲ�ѯָ�ʼ
					modbus_cmd_interval = time_get_ms(); //��ʼ��modbus ����������
					work_interval = time_get_ms(); //����POSTʱ���������������ڲ�ͣ����´δ������Ƿ���
					work_flag = 1;
					
					/** �ϵ����
					if(power_flag){
						power_flag = 0;
						DEBUG("�ϵ�\r\n");
						SIM_PWR_Close();
						para_value.modbus_qry_post_time=300;
					}
					else{
						power_flag = 1;
						DEBUG("�ϵ�\r\n");
						SIM_PWR_Open();
						para_value.modbus_qry_post_time=30;
					}
					**/
					/**
					char ��ӡ����
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
				if(work_flag && time_diff_ms(modbus_cmd_interval) > para_value.modbus_qry_cmd_time){//�������ڵ����ҷ���modbus������ʱ��ĵ���
					//DEBUG("\r\ncmd index %d\r\n",post_action_send_modbus_commands_Index);
					if(Modbus_Commands_Length == post_action_send_modbus_commands_Index){//�������������������������������0,����HTTP POST
						post_action_send_modbus_commands_Index = 0;
						work_flag = 0;
						DEBUG("\r\nhttp post data %d server1 %d server2 %d \r\n",modbus_http_post_len,para_value.link1.connect_type,para_value.link2.connect_type);
						//�ж��Ƿ���Ҫִ��HTTP POST����
						//sprintf("");
						if(modbus_http_post_len){//����modbus��ѯ����ص����ݲ�Ϊ��
							if(para_value.link1.connect_type == 0)//TCP������Ч��ִ��HTTP POST
							{
								http_post(para_value.link1, para_value.modbus_http_post_para, modbus_http_post_buff, modbus_http_post_len);
								/**
								����HTTP POST ������������ʹ��HTTP GET�еģ�����HTTP POST �������������滻��
								��ΪHTTP GET ��Ϊ����һ���ᷢ��
								**/
								modbus_http_post_len = 0;
								memset(modbus_http_post_buff, 0, Post_Buffer_Length);
								DEBUG("\r\nPOST\r\n");
								dealy = time_get_ms();
								while(time_diff_ms(dealy) < 5000);
							}
							//������յ�modbus��ѯ����ص�����
							modbus_http_post_len = 0;
							memset(modbus_http_post_buff, 0, Post_Buffer_Length);
						}
						//ִ��get����
						if(para_value.link2.connect_type == 0)//TCP����
        		{
							if(http_get(para_value.link2, para_value.modbus_http_get_para, para_value.modbus_qry_get_para.buff) == 1)
							{
								DEBUG("\r\nGET\r\n");
								modbus_http_get_send(http_get_buff, http_get_rx_len);
								/**
								GET ��ȡָ��ͺ��ӳ�MODBUS_GET_RX_TIMEOUT������HTTP POST ���ݻ�������
								�ڴ�֮ǰ����GET����ָ��ͺ��豸����������ȾHTTP POST���ݻ������������
								**/
								dealy = time_get_ms();
								while(time_diff_ms(dealy) < MODBUS_GET_RX_TIMEOUT);
								modbus_http_post_len = 0;
								memset(modbus_http_post_buff, 0, Post_Buffer_Length);
							}
						}
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
