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

#define UART_DATA_HEAD_MAX_LEN	8

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

extern uint8_t g_ble_mac_addr[20];
extern uint8_t g_ble_app_ver[64];

extern void uart_ble_test(void);
extern void MCU_get_nrf52810_ver(void);
extern void MCU_get_ble_mac_address(void);
extern void MCU_get_ble_status(void);
extern void MCU_set_ble_work_mode(uint8_t work_mode);
extern void MCU_send_find_phone(void);

#endif/*__UART_BLE_H__*/
