/****************************************Copyright (c)************************************************
** File Name:				transfer_cache.h
** Descriptions:			Data transfer cache pool head file
** Created By:				xie biao
** Created Date:			2021-03-25
** Modified Date:			2021-03-25 
** Version:					V1.0
******************************************************************************************************/
#ifndef __UART_BLE_H__
#define __UART_BLE_H__

#define PPG_DATA_HEAD	"PPG:"
#define ECG_DATA_HEAD	"ECG:"
#define TEMP_DATA_HEAD	"TEMP:"
#define WIFI_DATA_HEAD	"WIFI:"
#define AUDIO_DATA_HEAD	"AUDIO:"
#define BLE_DATA_HEAD	"BLE:"

#define COM_OPEN		"OPEN"
#define COM_CLOSE		"CLOSE"

#define COM_PPG_GET_INFOR		"INFOR:"
#define COM_PPG_GET_HR			"HR:"
#define COM_PPG_GET_SPO2		"SPO2:"
#define COM_PPG_GET_BPT			"BPT:"
#define COM_PPG_GET_DATA		"PPG_DATA:"
#define COM_PPG_GET_CAL			"GET_CAL:"
#define COM_PPG_SAVE_CAL		"SAVE_CAL:"
#define COM_PPG_UPGRADE					"UPGRADE:"
#define COM_PPG_UPGRADE_OK				"UPGRADE_OK:"
#define COM_PPG_UPGRADE_FAIL			"UPGRADE_FAIL:"
#define COM_PPG_UPGRADE_PAGE_NUM		"SET_PAGE_NUM:"
#define COM_PPG_UPGRADE_VECTOR_BYTES	"SET_VECTOR_BYTES:"
#define COM_PPG_UPGRADE_AUTH_BYTES		"SET_AUTH_BYTES:"
#define COM_PPG_UPGRADE_FLASH_PAGE		"SET_FLASH_PAGE:"


#define COM_ECG_GET_DATA		"ECG_DATA:"

#define COM_TEMP_GET_DATA		"TEMP_DATA:"

#define COM_WIFI_GET_INFOR		"INFOR:"
#define COM_WIFI_GET_SCAN_AP	"SCAN_AP:"
#define COM_WIFI_SEND_DATA		"SEND_DATA:"
#define COM_WIFI_RECE_DATA		"RECE_DATA:"

#define COM_AUDIO_PLAY			"PLAY"
#define COM_AUDIO_STOP			"STOP"
#define COM_AUDIO_PAUSE			"PAUSE"
#define COM_AUDIO_RESUME		"RESUME"
#define COM_AUDIO_NEXT			"NEXT"
#define COM_AUDIO_PRE			"PRE"
#define COM_AUDIO_VOL_INC		"VOL_INC"
#define COM_AUDIO_VOL_DEC		"VOL_DEC"

#define COM_BLE_GET_INFOR		"INFOR:"
#define COM_BLE_GET_STATUS		"STATUS:"
#define COM_BLE_SET_CONNECT		"CONNECT"
#define COM_BLE_SET_DISCONNECT	"DISCONNECT"
#define COM_BLE_SEND_DATA		"SEND_DATA:"
#define COM_BLE_RECE_DATA		"RECE_DATA:"


typedef enum
{
	UART_DATA_PPG,
	UART_DATA_ECG,
	UART_DATA_TEMP,
	UART_DATA_WIFI,
	UART_DATA_AUIOD,
	UART_DATA_BLE,
	UART_DATA_MAX
}UART_DATA_TYPE;

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
extern uint8_t g_nrf5340_ver[128];

extern void uart_ble_test(void);
extern void MCU_get_nrf52810_ver(void);
extern void MCU_get_ble_mac_address(void);
extern void MCU_get_ble_status(void);
extern void MCU_set_ble_work_mode(uint8_t work_mode);
extern void MCU_send_find_phone(void);

#endif/*__UART_BLE_H__*/
