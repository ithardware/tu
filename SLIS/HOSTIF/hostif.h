#ifndef __HOST_IF_H
#define __HOST_IF_H	 
#include "sys.h"

#define HOST_UASRT_RX_TIME_OUT             para_value.usart_rx_interval


#define HOST_REVEIVE_END_STR               '#'

#define HOST_RX_BUFF_LEN              512
#define HOST_TX_BUFF_LEN              512


#define WOKE_MODE_AT                   0
#define WOKE_MODE_TS                   1

#define WOKE_MODE_MODBUS_HTTP          6
#define WOKE_MODE_HTTP                 8

enum CMD_LIST 
{ 
    HOST_CMD_WKMODE = 0,   //工作模式
    HOST_CMD_GPRSMODE,     //在线模式
    HOST_CMD_DBGMODE,      //调试模式
    HOST_CMD_DTUID,        //ID配置 
    HOST_CMD_UART,         //串口参数
    HOST_CMD_DFI,          //串口数据包时间间隔
    HOST_CMD_HBTIME,       //心跳时间
    HOST_CMD_HBHEAD,       //心跳包头
    HOST_CMD_REGPKG,       //注册包开关
    HOST_CMD_REGHEAD,      //注册包头
    HOST_CMD_SERVER1,      //连接设置1
    HOST_CMD_SERVER2,      //连接设置2
    HOST_CMD_IDLTIME,      //自动下线时间
    HOST_CMD_APN,          //GPRS APN网关
	HOST_CMD_HTTPS,        //HTTP设置
	HOST_CMD_HTTPHD,       //HTTP 头设置
	HOST_CMD_QRYPTM,       //轮询POST间隔时间
	HOST_CMD_QRYPOST,      //轮询POST
	HOST_CMD_QRYPPA,       //设置轮询post http参数
	HOST_CMD_QRYGTM,       //轮询GET间隔时间
	HOST_CMD_QRYGET,       //轮询GET
	HOST_CMD_QRYGPA,       //设置轮询get http参数
		
    HOST_CMD_SYSPWD,       //DTU密码修改
    HOST_CMD_BUFMODE,      //数据缓存模式
    //以下命令不保存信息
    HOST_CMD_WAKEUP,       //唤醒命令
    HOST_CMD_OFFLINE,      //离线命令
    HOST_CMD_VERSION,      //查询DTU版本
    HOST_CMD_LIST,         //查询DTU所以参数
    HOST_CMD_CSQ,          //查询网络信号质量
    HOST_CMD_GSTATE,       //查询GPRS在线状态
    HOST_CMD_DTUIP,        //查询DTU IP地址
    HOST_CMD_NET,          //查询网络注册状态
    HOST_CMD_RESET,        //DTU恢复出厂设置
    HOST_CMD_RESTART,      //DTU重启
    HOST_CMD_CSMINS,       //查询SIM卡是否存在
    HOST_CMD_TEST_GSM,
};

extern u8 host_rx_buff[HOST_RX_BUFF_LEN];
extern u16 host_rx_len;


extern u8 host_tx_buff[HOST_TX_BUFF_LEN];

extern void host_receive_packet(void);
extern u8 tcp_udp_auto_line(void);


#endif
