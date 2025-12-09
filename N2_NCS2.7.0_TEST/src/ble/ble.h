/****************************************Copyright (c)************************************************
** File name:			    BLE.c
** Last modified Date:          
** Last Version:		   
** Descriptions:		   	使用的ncs版本-2.7.0		
** Created by:				谢彪
** Created date:			2025-11-25
** Version:			    	1.0
** Descriptions:			BLE事件处理源文件
******************************************************************************************************/
#ifndef __BLE_H__
#define __BLE_H__

#define COM_BLE_SET_OPEN		"OPEN:"
#define COM_BLE_SET_CLOSE		"CLOSE:"
#define COM_BLE_GET_VER			"VER:"
#define COM_BLE_GET_MAC			"MAC:"
#define COM_BLE_GET_STATUS		"STATUS:"
#define COM_BLE_GET_SCAN_AP		"SCAN_AP:"
#define COM_BLE_SET_CONNECT		"CONNECT:"
#define COM_BLE_SET_DISCONNECT	"DISCONNECT:"
#define COM_BLE_SEND_DATA		"SEND_DATA:"
#define COM_BLE_RECE_DATA		"RECE_DATA:"

#define PACKET_HEAD	0xAB
#define PACKET_END	0x88

#define BLE_WORK_MODE_ID		0xFF10			//5340工作状态正常
#define HEART_RATE_ID			0xFF31			//心率
#define BLOOD_OXYGEN_ID			0xFF32			//血氧
#define BLOOD_PRESSURE_ID		0xFF33			//血压
#define TEMPERATURE_ID			0xFF62			//体温
#define	ONE_KEY_MEASURE_ID		0xFF34			//一键测量
#define	PULL_REFRESH_ID			0xFF35			//下拉刷新
#define	SLEEP_DETAILS_ID		0xFF36			//睡眠详情
#define	FIND_DEVICE_ID			0xFF37			//查找手环
#define SMART_NOTIFY_ID			0xFF38			//智能提醒
#define	ALARM_SETTING_ID		0xFF39			//闹钟设置
#define USER_INFOR_ID			0xFF40			//用户信息
#define	SEDENTARY_ID			0xFF41			//久坐提醒
#define	SHAKE_SCREEN_ID			0xFF42			//抬手亮屏
#define	MEASURE_HOURLY_ID		0xFF43			//整点测量设置
#define	SHAKE_PHOTO_ID			0xFF44			//摇一摇拍照
#define	LANGUAGE_SETTING_ID		0xFF45			//中英日文切换
#define	TIME_24_SETTING_ID		0xFF46			//12/24小时设置
#define	FIND_PHONE_ID			0xFF47			//查找手机回复
#define	WEATHER_INFOR_ID		0xFF48			//天气信息下发
#define	TIME_SYNC_ID			0xFF49			//时间同步
#define	TARGET_STEPS_ID			0xFF50			//目标步数
#define	BATTERY_LEVEL_ID		0xFF51			//电池电量
#define	FIRMWARE_INFOR_ID		0xFF52			//固件版本号
#define	FACTORY_RESET_ID		0xFF53			//清除手环数据
#define	ECG_ID					0xFF54			//心电
#define	LOCATION_ID				0xFF55			//获取定位信息
#define	DATE_FORMAT_ID			0xFF56			//年月日格式设置
#define NOTIFY_CONTENT_ID		0xFF57			//智能提醒内容
#define CHECK_WHITELIST_ID		0xFF58			//判断手机ID是否在手环白名单
#define INSERT_WHITELIST_ID`	0xFF59			//将手机ID插入白名单
#define DEVICE_SEND_128_RAND_ID	0xFF60			//手环发送随机的128位随机数
#define PHONE_SEND_128_AES_ID	0xFF61			//手机发送AES 128 CBC加密数据给手环

#define	BLE_CONNECT_ID			0xFFB0			//BLE断连提醒
#define	CTP_NOTIFY_ID			0xFFB1			//CTP触屏消息
#define GET_BLE_VER_ID			0xFFB2			//获取BLE版本号
#define GET_BLE_MAC_ADDR_ID		0xFFB3			//获取BLE MAC地址
#define GET_BLE_STATUS_ID		0xFFB4			//获取BLE当前工作状态	0:关闭 1:休眠 2:广播 3:连接
#define SET_BEL_WORK_MODE_ID	0xFFB5			//设置BLE工作模式		0:关闭 1:打开 2:唤醒 3:休眠

//0:关闭 1:休眠 2:广播 3:连接
typedef enum
{
	BLE_STATUS_OFF,
	BLE_STATUS_SLEEP,
	BLE_STATUS_BROADCAST,
	BLE_STATUS_CONNECTED,
	BLE_STATUS_MAX
}ENUM_BLE_STATUS;

//0:关闭 1:打开 2:唤醒 3:休眠
typedef enum
{
	BLE_MODE_TURN_OFF,
	BLE_MODE_TURN_ON,
	BLE_MODE_WAKE_UP,
	BLE_MODE_GOTO_SLEEP,
	BLE_MODE_MAX
}ENUM_BLE_MODE;

typedef enum
{
	BLE_WORK_NORMAL,
	BLE_WORK_DFU,
	BLE_WORK_MAX
}ENUM_BLE_WORK_MODE;

extern bool g_ble_connected;
extern ENUM_BLE_STATUS g_ble_status;
extern ENUM_BLE_MODE g_ble_mode;

extern uint8_t g_ble_mac_addr[20];
extern uint8_t g_nrf52810_ver[128];

extern void uart_ble_test(void);
extern void MCU_get_ble_app_ver(void);
extern void MCU_get_ble_mac_address(void);
extern void MCU_get_ble_status(void);
extern void MCU_set_ble_work_mode(uint8_t work_mode);
extern void MCU_send_find_phone(void);
#endif/*__BLE_H__*/