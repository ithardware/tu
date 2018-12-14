#ifndef __SAVE_DATA_H
#define __SAVE_DATA_H 
#include "sys.h"
#include "usart.h"	

#define SAVE_PARA_ADDR  0X0800e000		//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)

typedef struct 
{
	  //  0-none   1--tcp   2--udp
    u8  connect_type;
	  //max len is 64
	  u8  ip_addr[66];
	  u16 port;
	
 }LINK_PARA;

typedef struct 
{
	  //max len is 16
    u8  apn_name[18];
	  //max len is 32
	  u8  user_name[34];
	  //max len is 32
	  u8  user_pwd[34];
}APN_GATEWAT;

typedef struct 
{
	  //  0-POST     1--GET
    u8  type;
	  //  0-不输出   1--输出
	  u8  out_head;
	  //max len is 64
	  u8  head[130];
	  u8  url[130];
	
}HTTP_PARA;


typedef struct 
{
	  //命令的长度
    u8  len;
	  //缓冲modbus数据
	  u8  buff[34];
	
}MODBUS_PARA;

 
typedef struct 
{
	  //工作模式  
    u8  word_mode;
	  //在线模式  0--off online   1--alway online  
	  //          2--wake online  3--need online
	  u8  on_line_mode;
	  //调试模式  0--close   1--gprs state  2trace(define 0)
	  u8  debug_mode;
	  //id号的长度  max  32
	  u8  id_len;
	  //id号
	  u8  id[34];
	  //串口设置参数  usart para define  baurd--3--9600  data_bits--8
	  //               parity--0     stop--1    flow_ctr---0
	  USART_PARA  usart_para;          
	  //串口间隔时间max 5000ms    (define  100)
	  u16  usart_rx_interval;
	  //心跳时间   max 3600s  define 60
	  u16 heartbeat_time;
	  //心跳包的长度 max 32
	  u8  heartbeat_head_len;
	  //心跳包类型    0--hex   1--string
	  u8  heartbeat_head_type;
		//心跳包头内容
	  u8  heartbeat_head[34];
	  //注册包开关  0--close  1--open
	  u8  reg_packet_switch;
	  //注册包长度  max 32
	  u8  reg_head_len;
		//注册包类型    --hex   1--string
	  u8  reg_head_type;
		//注册包头内容
		u8  reg_head[34];
	  //连接设置1   define 3--none
		LINK_PARA link1;
		//连接设置2   define 3--none
		LINK_PARA link2;
		//自动下线时间    max 3600s   define 300s
		u16 auto_off_line_time;
		//apn网关设置
		APN_GATEWAT apn_para;
		
		HTTP_PARA http_para;
		u16 modbus_qry_cmd_time;
		u16 modbus_qry_post_time;
		MODBUS_PARA modbus_qry_post_para[16];
		HTTP_PARA modbus_http_post_para;
		
		u16 modbus_qry_get_time;
		MODBUS_PARA modbus_qry_get_para;
		HTTP_PARA modbus_http_get_para;
		
		u8 password_len;
		u8 password[8];
	
}SAVE_DATA;



extern SAVE_DATA para_value;


extern void define_para_value(void);

extern void write_para_to_flash(void);
extern void read_para_from_flash(void);


#endif
