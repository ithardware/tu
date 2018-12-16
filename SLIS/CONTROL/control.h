#ifndef __CONTROL_H
#define __CONTROL_H	 
#include "sys.h"




#define Modbus_Commands_Length 16
#define Post_Buffer_Length 4096
#define Get_Modbus_Commands_Split_Zero_Count 8
#define SIM800C_RX_TIMEOUT                         10
#define CHECK_TCP_UDP_CONNECT_TIMEOUT              10000
#define MODBUS_GET_RX_TIMEOUT                      200

extern unsigned int modbus_post_count;
extern unsigned int modbus_get_flog;

extern unsigned char modbus_http_post_buff[Post_Buffer_Length];
extern unsigned int modbus_http_post_len;


extern void woke_mode_at_main(void); 			
extern void woke_mode_modbus_http_main(void);

#endif
