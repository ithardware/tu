
#ifndef __SIM800C_H
#define __SIM800C_H
#include "sys.h"
#include "save_data.h"

#define HTTP_BUFF_LEN             512
#define HTTP_GET_BUFF_LEN         512
#define HTTP_POST_BUFF_LEN        1024







#define SIM_PWR PBout(6)

extern u8 http_buff_rx[HTTP_BUFF_LEN];

extern u8 http_get_buff[HTTP_GET_BUFF_LEN];
extern u16 http_get_rx_len;


extern u8 RSSI;//信号强度

void SIM_PWR_Open(void);
void SIM_PWR_Close(void);



u8 SIM8XX_User(void);
u8 SIM8XX_Register(void);


void SIM800C_Init(void);
u8* sim900a_check_cmd(u8 *str);
extern u8 sim900a_send_data_nck(u8 *cmd,u16 len);
extern u8 sim900a_send_data_ack(u8 *cmd,u16 len,u8 *ack,u16 waittime);


u8 sim900a_chr2hex(u8 chr);
u8 sim900a_hex2chr(u8 hex);
u8 sim900a_tcpudp_test(void);
u8 Set_Register(void);//成功获取返回1；获取失败返回0；


extern u8 read_imei(u8 *data);

extern u8 check_tcp_udp_connet(void);
extern u8 connect_tcp_udp(LINK_PARA link);
extern u16 tcp_udp_rx_data(u8 *data);
extern int tcp_udp_send(u8 *data, u16 len);

extern u8 off_line(void);
extern int init_http(void);
extern int http_get(LINK_PARA link, HTTP_PARA http_para, u8 *in_data);
extern int http_post(LINK_PARA link, HTTP_PARA http_para, u8 *in_data, u16 len);



extern u8 SIM8XX_User_Receive(void);//while(1)




extern u8 tcp_udp_rx_data_flog;
extern u8 first_tcp_udp_connect_flog;


extern volatile u32 RELAY_TIME;
extern u8  RELAY_NET_STA;
extern u8  Rigster_ERR_Count;//注册失败计数
extern volatile u32 Set_Relay_Open_Time;//设置本地开锁时间，默认6秒，储存在EEPROM中。否者掉电后失效。
extern u8  RELAY_Local_Open_STA;//本地开锁状态，用于上报服务器。
extern u8  RELAY_Local_Close_STA;//本地关锁状态，用于上报服务器。

extern volatile u8	Heartbeat_Server ;//表示心跳正常
extern volatile u8  hbeaterrcnt; //心跳错误计数器,连续5次心跳信号无应答,则重新连接
extern volatile u8  connectsta;//0,正在连接;1,连接成功;2,连接关闭; 









#endif

