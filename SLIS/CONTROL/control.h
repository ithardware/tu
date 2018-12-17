#ifndef __CONTROL_H
#define __CONTROL_H	 
#include "sys.h"


#define Get_Modbus_Commands_Split_Zero_Count 8
#define SIM800C_RX_TIMEOUT                         10
#define CHECK_TCP_UDP_CONNECT_TIMEOUT              10000
#define MODBUS_GET_RX_TIMEOUT                      200
#define Post_Buffer_Length												 Modbus_Commands_Length*256

extern unsigned int modbus_post_count;
extern unsigned int modbus_get_flog;

extern unsigned char modbus_http_post_buff[];
extern unsigned int modbus_http_post_len;


extern void woke_mode_at_main(void); 			
extern void woke_mode_modbus_http_main(void);

#endif
