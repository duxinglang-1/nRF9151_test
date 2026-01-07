/****************************************Copyright (c)************************************************
** File Name:			    ecg.c
** Descriptions:			ecg function main source file
** Created By:				xie biao
** Created Date:			2024-04-11
** Modified Date:      		2024-04-11
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include "external_flash.h"
#include "lcd.h"
#include "screen.h"
#include "logger.h"
#include "uart.h"
#include "ecg.h"
#ifdef CONFIG_BLE_SUPPORT
#include "Ble.h"
#endif

uint8_t g_ecg_trigger = 0;

ECG_WORK_STATUS g_ecg_status = ECG_STATUS_PREPARE;

static bool ecg_start_flag = false;
static bool ecg_test_flag = false;
static bool ecg_stop_flag = false;
static bool menu_start_ecg = false;
static bool ft_start_ecg = false;
static bool app_start_ecg = false;

#ifdef CONFIG_FACTORY_TEST_SUPPORT
uint8_t ecg_test_info[256] = {0};
#endif

static void ecg_auto_stop_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(ecg_stop_timer, ecg_auto_stop_timerout, NULL);
static void ecg_menu_stop_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(ecg_menu_stop_timer, ecg_menu_stop_timerout, NULL);

static void ecg_auto_stop_timerout(struct k_timer *timer_id)
{
	if((g_ecg_trigger&ECG_TRIGGER_BY_MENU) == 0)
		ecg_stop_flag = true;
}

static void ecg_menu_stop_timerout(struct k_timer *timer_id)
{
	ecg_stop_flag = true;
	
	if(screen_id == SCREEN_ID_ECG)
	{
		g_ecg_status = ECG_STATUS_MEASURE_FAIL;
		
		scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_TEMP;
		scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
	}
}

void StartECG(ECG_TRIGGER_SOUCE trigger_type)
{
	notify_infor infor = {0};

	infor.x = 0;
	infor.y = 0;
	infor.w = LCD_WIDTH;
	infor.h = LCD_HEIGHT;
	infor.align = NOTIFY_ALIGN_CENTER;
	infor.type = NOTIFY_TYPE_POPUP;


	if(1
	#ifdef CONFIG_FACTORY_TEST_SUPPORT
		&& !IsFTECGTesting()
	#endif
		)
	{
		StartSCC();
	}

	switch(trigger_type)
	{
	case ECG_TRIGGER_BY_HOURLY:
	case ECG_TRIGGER_BY_APP:
	case ECG_TRIGGER_BY_FT:
		if(!is_wearing())
		{
			return;
		}
		break;
		
	case ECG_TRIGGER_BY_MENU:
		if(!is_wearing())
		{
			infor.img[0] = IMG_WRIST_OFF_ICON_ADDR;
			infor.img_count = 1;

			DisplayPopUp(infor);
			return;
		}

		break;
	}

	g_ecg_trigger |= trigger_type;

	ecg_start_flag = true;
}

void MenuStartECG(void)
{
	menu_start_ecg = true;
}

void MenuStopECG(void)
{
	ecg_stop_flag = true;
}

void APPStartECG(void)
{
	app_start_ecg = true;
}

#ifdef CONFIG_FACTORY_TEST_SUPPORT
void FTStartECG(void)
{
	ft_start_ecg = true;
}

void FTStopECG(void)
{
	ecg_stop_flag = true;
}
#endif

void ECGDataProcess(uint8_t *data, uint32_t data_len)
{
}

void UartECGEventHandle(uint8_t *data, uint32_t data_len)
{
	uint8_t *ptr;
	static uint32_t page_num=0,flash_partial=0;

	if(data == NULL || data_len == 0)
		return;

	ptr = strstr(data, ECG_DATA_HEAD);
	if(ptr != NULL)
	{
		uint8_t *ptr1,*ptr2;

		ptr += strlen(ECG_DATA_HEAD);
		if((ptr1 = strstr(ptr, COM_ECG_SET_OPEN)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_ECG_SET_CLOSE)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_ECG_GET_DATA)) != NULL)
		{
			ptr1 += strlen(COM_ECG_GET_DATA);
			ECGDataProcess(ptr1, data_len-(ptr1-data));
		}
		else if((ptr1 = strstr(ptr, COM_ECG_GET_INFOR)) != NULL)
		{
		}
	}
}

void ECG_init(void)
{
}

void ECGMsgProcess(void)
{
	if(menu_start_ecg)
	{
		StartTemp(ECG_TRIGGER_BY_MENU);
		menu_start_ecg = false;
	}

	if(app_start_ecg)
	{
		StartTemp(ECG_TRIGGER_BY_APP);
		app_start_ecg = false;
	}
	
	if(ft_start_ecg)
	{
		StartTemp(ECG_TRIGGER_BY_FT);
		ft_start_ecg = false;
	}

	if(ecg_start_flag)
	{
		ecg_start_flag = false;
		
		CopcsSendData(UART_DATA_ECG, COM_ECG_GET_DATA, strlen(COM_ECG_GET_DATA));
	
		if((g_ecg_trigger&ECG_TRIGGER_BY_HOURLY) == ECG_TRIGGER_BY_HOURLY)
		{
			k_timer_start(&ecg_stop_timer, K_MSEC(ECG_CHECK_TIMELY*60*1000), K_NO_WAIT);
		}
		else if((g_ecg_trigger&ECG_TRIGGER_BY_MENU) == ECG_TRIGGER_BY_MENU)
		{
			k_timer_start(&ecg_menu_stop_timer, K_SECONDS(ECG_CHECK_MENU), K_NO_WAIT);
		}
	}

	if(ecg_stop_flag)
	{
		ecg_stop_flag = false;
	}
}
