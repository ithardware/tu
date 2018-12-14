#include "save_data.h"
#include "stmflash.h"
#include "string.h"
#include "sim800c.h"

SAVE_DATA para_value;

void define_para_value(void)
{
	  u8 i = 0;
	  memset(&para_value,0,sizeof(para_value));
	
    para_value.word_mode = 0;
	  para_value.on_line_mode = 1;
	  para_value.debug_mode = 0;
	  //设置模块的IMEI号为默认ID
	  memset((para_value.id),0,34);
    para_value.id_len = read_imei(&para_value.id[0]);
		para_value.usart_para.baurd = 9600;
		para_value.usart_para.data_bits = 8;
		para_value.usart_para.parity = 0;
		para_value.usart_para.stop = 1;
		para_value.usart_para.flow_ctr = 0;
		para_value.usart_rx_interval = 100;
		para_value.heartbeat_time = 60;
		para_value.heartbeat_head_len = 0;
	  para_value.heartbeat_head_type = 0;
	  para_value.reg_packet_switch = 0;
	  para_value.reg_head_len = 0;
    para_value.reg_head_type = 0;
		para_value.link1.connect_type = 3;
		para_value.link2.connect_type = 3;
		para_value.auto_off_line_time = 300;
		strcpy((char *)(para_value.apn_para.apn_name), "CMNET");
		para_value.password_len = 6;
		strcpy((char *)(para_value.password), "123456");
		para_value.http_para.type = 0;
		para_value.http_para.out_head = 0;
		memset((para_value.http_para.url),0,130);
		memset((para_value.http_para.head),0,130);
		para_value.modbus_qry_post_time = 0;
		para_value.modbus_qry_cmd_time = 500;

		for(i=0;i<16;i++)
		{
		    para_value.modbus_qry_post_para[i].len = 0;
		}
	  para_value.modbus_http_post_para.type = 0;
		para_value.modbus_http_post_para.out_head = 0;
		memset((para_value.modbus_http_post_para.url),0,130);
		memset((para_value.modbus_http_post_para.head),0,130);
		
		para_value.modbus_qry_get_time = 0;
		para_value.modbus_qry_get_para.len = 0;
	  para_value.modbus_http_get_para.type = 0;
		para_value.modbus_http_get_para.out_head = 0;
		memset((para_value.modbus_http_get_para.url),0,130);
		memset((para_value.modbus_http_get_para.head),0,130);
		
		//串口初始化
		
}


//将数据保存到FLASH中
void write_para_to_flash(void)
{
    STMFLASH_Write(SAVE_PARA_ADDR,(u16*)&para_value,(sizeof(SAVE_DATA)));
}

//将数据从FLASH中读出
void read_para_from_flash(void)
{
	  u8 i = 0;
    STMFLASH_Read(SAVE_PARA_ADDR,(u16*)&para_value,sizeof(SAVE_DATA));
	  
	  if(para_value.word_mode > 8)
		{
		    para_value.word_mode = 0;
		}
		
		if(para_value.on_line_mode > 3)
		{
		    para_value.on_line_mode = 1;
		}
		
		if(para_value.debug_mode > 2)
		{
		    para_value.debug_mode =1;
		}
		
		if(para_value.id_len > 32)
		{
			  memset((para_value.id),0,34);
			  para_value.id_len = read_imei(&para_value.id[0]);
		}
		
		if((para_value.usart_para.baurd == 1200 ) || (para_value.usart_para.baurd == 2400 ) || (para_value.usart_para.baurd == 4800) 
		|| (para_value.usart_para.baurd == 9600 ) || (para_value.usart_para.baurd == 14400) || (para_value.usart_para.baurd == 19200)
	  || (para_value.usart_para.baurd == 38400) || (para_value.usart_para.baurd == 57600) || (para_value.usart_para.baurd == 115200))
		{
		
		}
		else
		{
		     para_value.usart_para.baurd = 9600;
		}
		
		if((para_value.usart_para.data_bits == 8) || (para_value.usart_para.data_bits == 9))
		{
		
		}
		else
		{
		    para_value.usart_para.data_bits = 8;
		}
		
		if(para_value.usart_para.parity > 2)
		{
		    para_value.usart_para.parity = 0;
		}
		
		if(para_value.usart_para.stop > 3)
		{
		    para_value.usart_para.stop = 1;
		}
		
		if(para_value.usart_para.flow_ctr > 2)
		{
		    para_value.usart_para.flow_ctr = 0;
		}
		
		if(para_value.usart_rx_interval > 200)
		{
		    para_value.usart_rx_interval = 100;
		}
		
		if(para_value.heartbeat_time > 3600)
		{
		    para_value.heartbeat_time = 60;
		}
		
		if(para_value.heartbeat_head_len > 32)
		{
			  memset((para_value.heartbeat_head),0,34);
		    para_value.heartbeat_head_len = 0;
			  para_value.heartbeat_head_type = 0;
		}
		
		if(para_value.heartbeat_head_type > 1)
		{
			  memset((para_value.heartbeat_head),0,34);
		    para_value.heartbeat_head_len = 0;
			  para_value.heartbeat_head_type = 0;
		}
		
		if(para_value.reg_packet_switch > 1)
		{
		    para_value.reg_packet_switch = 0;
		}
	  
		if(para_value.reg_head_len > 32)
		{
			  memset((para_value.reg_head),0,34);
		    para_value.reg_head_len = 0;
        para_value.reg_head_type = 0;
		}
		
		if(para_value.reg_head_type > 1)
		{
			  memset((para_value.reg_head),0,34);
		    para_value.reg_head_len = 0;
        para_value.reg_head_type = 0;
		}
	  
		if(para_value.link1.connect_type > 3)
		{
		    para_value.link1.connect_type = 3;
			  memset((para_value.link1.ip_addr),0,66);
			  para_value.link1.port = 0;
		}
		
		if(para_value.link2.connect_type > 3)
		{
		    para_value.link2.connect_type = 3;
			  memset((para_value.link2.ip_addr),0,66);
			  para_value.link2.port = 0;
		}
		
		if(para_value.auto_off_line_time > 3600)
		{
		    para_value.auto_off_line_time = 300;
		}
		
		//assic 数字和字母的范围是  0x30(48)-0x7a(122) 超过这个范围就不正常
		if((para_value.apn_para.apn_name[0] > 122) || (para_value.apn_para.apn_name[0] < 48))
		{
			  memset((para_value.apn_para.apn_name),0,18);
        memset((para_value.apn_para.user_name),0,34);
			  memset((para_value.apn_para.user_pwd),0,34);
		    strcpy((char *)(&para_value.apn_para.apn_name), "connet");
			  
		}
		
		if(para_value.password_len > 6)
		{
		    para_value.password_len = 6;
			  strcpy((char *)(para_value.password), "123456");
		}
		
		if(para_value.http_para.type > 1)
		{
		    para_value.http_para.type = 0;
			  para_value.http_para.out_head = 0;
			  memset((para_value.http_para.url),0,130);
			  memset((para_value.http_para.head),0,130);
		}
		
		if(para_value.modbus_qry_post_time > 3600)
		{
		    para_value.modbus_qry_post_time = 0;
		}
		if(para_value.modbus_qry_cmd_time > 3000)
		{
		    para_value.modbus_qry_cmd_time = 500;
		}
		for(i=0;i<16;i++)
		{
			  if(para_value.modbus_qry_post_para[i].len > 32)
				{
				    para_value.modbus_qry_post_para[i].len = 0;
					  memset((para_value.modbus_qry_post_para[i].buff),0,34);
				}
		}
		if(para_value.modbus_http_post_para.type > 1)
		{
		    para_value.modbus_http_post_para.type = 0;
			  para_value.modbus_http_post_para.out_head = 0;
			  memset((para_value.modbus_http_post_para.url),0,130);
			  memset((para_value.modbus_http_post_para.head),0,130);
		}
		
		
		if(para_value.modbus_qry_get_time > 3600)
		{
		    para_value.modbus_qry_get_time = 0;
		}
    if(para_value.modbus_qry_get_para.len > 32)
    {
        para_value.modbus_qry_get_para.len = 0;
			  memset((para_value.modbus_qry_get_para.buff),0,34);
    }
		if(para_value.modbus_http_get_para.type > 1)
		{
		    para_value.modbus_http_get_para.type = 0;
			  para_value.modbus_http_get_para.out_head = 0;
			  memset((para_value.modbus_http_get_para.url),0,130);
			  memset((para_value.modbus_http_get_para.head),0,130);
		}

}



