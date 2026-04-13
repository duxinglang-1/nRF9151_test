/****************************************Copyright (c)************************************************
** File name:			    screen.c
** Last modified Date:          
** Last Version:		   
** Descriptions:		   	使用的ncs版本-1.2		
** Created by:				谢彪
** Created date:			2020-12-16
** Version:			    	1.0
** Descriptions:			屏幕UI管理C文件
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include "settings.h"
#include "lcd.h"
#include "font.h"
#include "img.h"
#include "key.h"
#include "mainmenu.h"
#include "datetime.h"
#include "max20353.h"
#ifdef CONFIG_PPG_SUPPORT
#include "max32674.h"
#endif
#ifdef CONFIG_IMU_SUPPORT
#include "lsm6dso.h"
#include "fall.h"
#ifdef CONFIG_SLEEP_SUPPORT
#include "sleep.h"
#endif/*CONFIG_SLEEP_SUPPORT*/
#endif
#include "external_flash.h"
#include "screen.h"
#include "ucs2.h"
#include "nb.h"
#include "sos.h"
#ifdef CONFIG_ALARM_SUPPORT
#include "alarm.h"
#endif
#include "gps.h"
#include "uart.h"
#ifdef CONFIG_ECG_SUPPORT
#include "ecg.h"
#endif
#ifdef CONFIG_TOUCH_SUPPORT
#include "CST816.h"
#endif
#ifdef CONFIG_BLE_SUPPORT
#include "Ble.h"
#endif
#ifdef CONFIG_FOTA_DOWNLOAD
#include "fota_mqtt.h"
#endif/*CONFIG_FOTA_DOWNLOAD*/
#ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
#include "data_download.h"
#endif/*CONFIG_DATA_DOWNLOAD_SUPPORT*/
#ifdef CONFIG_WIFI_SUPPORT
#include "esp8266.h"
#endif/*CONFIG_WIFI_SUPPORT*/
#ifdef CONFIG_SYNC_SUPPORT
#include "sync.h"
#endif/*CONFIG_SYNC_SUPPORT*/
#ifdef CONFIG_TEMP_SUPPORT
#include "temp.h"
#endif/*CONFIG_TEMP_SUPPORT*/
#ifdef CONFIG_FACTORY_TEST_SUPPORT
#include "ft_main.h"
#include "ft_aging.h"
#endif/*CONFIG_FACTORY_TEST_SUPPORT*/
#include "logger.h"

// External ECG lead status variable
#ifdef CONFIG_ECG_SUPPORT
extern ECG_LEAD_STATUS g_ecg_lead_status;
#endif

static uint8_t scr_index = 0;
static uint8_t bat_charging_index = 0;
static bool exit_notify_flag = false;
static bool entry_idle_flag = false;
static bool entry_setting_bk_flag = false;

static void NotifyTimerOutCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(notify_timer, NotifyTimerOutCallBack, NULL);
static void MainMenuTimerOutCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(mainmenu_timer, MainMenuTimerOutCallBack, NULL);
#ifdef CONFIG_PPG_SUPPORT
static void PPGStatusTimerOutCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(ppg_status_timer, PPGStatusTimerOutCallBack, NULL);
#endif
#ifdef CONFIG_TEMP_SUPPORT
static void TempStatusTimerOutCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(temp_status_timer, TempStatusTimerOutCallBack, NULL);
#endif

SCREEN_ID_ENUM screen_id = SCREEN_ID_BOOTUP;
SCREEN_ID_ENUM history_screen_id = SCREEN_ID_BOOTUP;
screen_msg scr_msg[SCREEN_ID_MAX] = {0};
notify_infor notify_msg = {0};

extern bool key_pwroff_flag;
extern uint8_t g_rsrp;

static void EnterHRScreen(void);

#ifdef IMG_FONT_FROM_FLASH
static uint32_t logo_img[] = 
{
	IMG_ID_PWRON_ANI_0,
	IMG_ID_PWRON_ANI_1,
	IMG_ID_PWRON_ANI_2,
	IMG_ID_PWRON_ANI_3,
	IMG_ID_PWRON_ANI_4,
	IMG_ID_PWRON_ANI_5
};
#else
static char *logo_img[] = 
{
	IMG_PWRON_ANI_1_ADDR,
	IMG_PWRON_ANI_1_ADDR,
	IMG_PWRON_ANI_1_ADDR,
	IMG_PWRON_ANI_1_ADDR,
	IMG_PWRON_ANI_1_ADDR
};
#endif

void EnterSettingsScreen(void);
#ifdef CONFIG_SYNC_SUPPORT
void EnterSyncDataScreen(void);
#endif/*CONFIG_SYNC_SUPPORT*/
#ifdef CONFIG_TEMP_SUPPORT
void EnterTempScreen(void);
#endif/*CONFIG_TEMP_SUPPORT*/
#ifdef CONFIG_PPG_SUPPORT
void EnterSPO2Screen(void);
void EnterBPScreen(void);
#endif/*CONFIG_PPG_SUPPORT*/
#ifdef CONFIG_ECG_SUPPORT
void EnterEcgScreen(void);
#endif/*CONFIG_ECG_SUPPORT*/
#ifdef CONFIG_SLEEP_SUPPORT
void EnterSleepScreen(void);
#endif/*CONFIG_SLEEP_SUPPORT*/
#ifdef CONFIG_STEP_SUPPORT
void EnterStepsScreen(void);
#endif/*CONFIG_STEP_SUPPORT*/

void ShowBootUpLogoFinished(void)
{
	EnterIdleScreen();
}

void ShowBootUpLogo(void)
{
	uint8_t i,count=0;
	uint16_t x,y,w,h;

#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaShow(PWRON_LOGO_X, PWRON_LOGO_Y, logo_img, ARRAY_SIZE(logo_img), 200, false, ShowBootUpLogoFinished);
#else
  #ifdef IMG_FONT_FROM_FLASH
	LCD_ShowImg_From_Flash(PWRON_LOGO_X, PWRON_LOGO_Y, IMG_PWRON_ANI_6_ADDR);
  #else
	LCD_ShowImg(PWRON_LOGO_X, PWRON_LOGO_Y, IMG_PWRON_ANI_6_ADDR);
  #endif
  
	k_sleep(K_MSEC(1000));
	ShowBootUpLogoFinished();
#endif
}

void MainMenuTimerOutCallBack(struct k_timer *timer_id)
{
	if(screen_id == SCREEN_ID_GPS_TEST)
	{
		MenuStartGPS();
	}
	else if(screen_id == SCREEN_ID_NB_TEST)
	{
		MenuStartNB();
	}
	else if(screen_id == SCREEN_ID_BLE_TEST)
	{
		
	}
#ifdef CONFIG_WIFI_SUPPORT	
	else if(screen_id == SCREEN_ID_WIFI_TEST)
	{
		MenuStartWifi();
	}
#endif	
#ifdef CONFIG_FOTA_DOWNLOAD	
	else if(screen_id == SCREEN_ID_FOTA)
	{
		MenuStartFOTA();
	}
#endif
#ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
	else if(screen_id == SCREEN_ID_DL)
	{
		switch(g_dl_data_type)
		{
		case DL_DATA_IMG:
			switch(get_dl_status())
			{
			case DL_STATUS_PREPARE:
				dl_start();
				break;
				
			case DL_STATUS_FINISHED:
			case DL_STATUS_ERROR:
				if((strcmp(g_new_font_ver,g_font_ver) != 0) && (strlen(g_new_font_ver) > 0))
					dl_font_start();
				else if((strcmp(g_new_str_ver,g_str_ver) != 0) && (strlen(g_new_str_ver) > 0))
					dl_str_start();
			#if defined(CONFIG_PPG_SUPPORT)
				else if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
					dl_ppg_start();
			#endif
				else
				{
					dl_reboot_confirm();
				}
				break;
			}
			break;

		case DL_DATA_FONT:
			switch(get_dl_status())
			{
			case DL_STATUS_PREPARE:
				dl_start();
				break;

			case DL_STATUS_FINISHED:
			case DL_STATUS_ERROR:
				if((strcmp(g_new_str_ver,g_str_ver) != 0) && (strlen(g_new_str_ver) > 0))
					dl_str_start();
			#if defined(CONFIG_PPG_SUPPORT)
				else if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
					dl_ppg_start();
				else
			#endif		
				{
					dl_reboot_confirm();
				}
				break;
			}
			break;

		case DL_DATA_STR:
			switch(get_dl_status())
			{
			case DL_STATUS_PREPARE:
				dl_start();
				break;

			case DL_STATUS_FINISHED:
			case DL_STATUS_ERROR:
			#if defined(CONFIG_PPG_SUPPORT)
				if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
					dl_ppg_start();
				else
			#endif		
				{
					dl_reboot_confirm();
				}
				break;
			}
			break;	
			
		case DL_DATA_PPG:
			switch(get_dl_status())
			{
			case DL_STATUS_PREPARE:
				dl_start();
				break;

			case DL_STATUS_FINISHED:
			case DL_STATUS_ERROR:
				dl_reboot_confirm();
				break;
			}
			break;
		}
	}
#endif
#ifdef CONFIG_SYNC_SUPPORT
	else if(screen_id == SCREEN_ID_SYNC)
	{
		MenuStartSync();
	}
#endif
#ifdef CONFIG_TEMP_SUPPORT
	else if(screen_id == SCREEN_ID_TEMP)
	{
		if(get_temp_ok_flag)
		{
			EntryIdleScr();
		}
		else
		{
			MenuStartTemp();
		}
	}
#endif
	else if(screen_id == SCREEN_ID_SETTINGS)
	{
		ExitSettingsScreen();
	}
	else if(screen_id == SCREEN_ID_POWEROFF)
	{
		EntryIdleScr();
	}
}

#ifdef CONFIG_BLE_SUPPORT
void IdleShowBleStatus(void)
{
	uint16_t x,y,w,h;
	uint16_t ble_link_str[] = {0x0042,0x004C,0x0045,0x0000};//BLE

	LCD_SetFontSize(FONT_SIZE_28);

	if(g_ble_connected)
	{
		LCD_MeasureUniString(ble_link_str, &w, &h);
		LCD_ShowUniString(IDLE_BLE_X+(IDLE_BLE_W-w)/2, IDLE_BLE_Y+(IDLE_BLE_H-w)/2, ble_link_str);
	}
	else
	{
		LCD_FillColor(IDLE_BLE_X, IDLE_BLE_Y, IDLE_BLE_W, IDLE_BLE_H, BLACK);
	}
}
#endif

void IdleShowSystemDate(void)
{
	uint16_t x,y,w,h,str_w,str_h;;
	uint8_t str_date[20] = {0};
	uint16_t str_mon[12] = {
								STR_ID_MONTH_JAN,
								STR_ID_MONTH_FEB,
								STR_ID_MONTH_MAR,
								STR_ID_MONTH_APR,
								STR_ID_MONTH_MAY,
								STR_ID_MONTH_JUN,
								STR_ID_MONTH_JUL,
								STR_ID_MONTH_AUG,
								STR_ID_MONTH_SEPT,
								STR_ID_MONTH_OCT,
								STR_ID_MONTH_NOV,
								STR_ID_MONTH_DEC,
							};

	uint16_t str_mon_cn[2] = {0x6708,0x0000};
	uint16_t str_day_cn[2] = {0x65E5,0x0000};
	uint16_t str_mon_kr[2] = {0xC6D4,0x0000};//?
	uint16_t str_day_kr[2] = {0xC77C,0x0000};//?
	uint16_t *str_m,*str_d;
	uint32_t img_45_num[10] = {IMG_ID_FONT_45_NUM_0,IMG_ID_FONT_45_NUM_1,IMG_ID_FONT_45_NUM_2,IMG_ID_FONT_45_NUM_3,IMG_ID_FONT_45_NUM_4,
							   IMG_ID_FONT_45_NUM_5,IMG_ID_FONT_45_NUM_6,IMG_ID_FONT_45_NUM_7,IMG_ID_FONT_45_NUM_8,IMG_ID_FONT_45_NUM_9};

	POINT_COLOR=WHITE;
	BACK_COLOR=BLACK;

	LCD_SetFontSize(FONT_SIZE_52);

	switch(global_settings.language)
	{
 #if defined(LANGUAGE_CN_ENABLE)||defined(LANGUAGE_JA_ENABLE)||defined(LANGUAGE_KR_ENABLE)
   #ifdef LANGUAGE_CN_ENABLE
	case LANGUAGE_CN:
   #endif
   #ifdef LANGUAGE_JA_ENABLE
	case LANGUAGE_JA:
   #endif
   #ifdef LANGUAGE_KR_ENABLE
	case LANGUAGE_KR:
   #endif
		switch(global_settings.language)
		{
	   #if defined(LANGUAGE_CN_ENABLE)||defined(LANGUAGE_JA_ENABLE)
	   #ifdef LANGUAGE_CN_ENABLE
		case LANGUAGE_CN:
	   #endif
	   #ifdef LANGUAGE_JA_ENABLE
		case LANGUAGE_JA:
	   #endif
			x = IDLE_DATE_MON_CN_X;
			y = IDLE_DATE_MON_CN_Y;
			w = IDLE_DATE_MON_CN_W;
			h = IDLE_DATE_MON_CN_H;
			str_m = str_mon_cn;
			str_d = str_day_cn;
			break;
	   #endif
	   	   
	   #ifdef LANGUAGE_KR_ENABLE
		case LANGUAGE_KR:
			x = IDLE_DATE_MON_CN_X-10;
			y = IDLE_DATE_MON_CN_Y;
			w = IDLE_DATE_MON_CN_W+10;
			h = IDLE_DATE_MON_CN_H;
			str_m = str_mon_kr;
			str_d = str_day_kr;
			break;
	   #endif
		}
		
		LCD_FillColor(x, y, w, h, BLACK);
		if(date_time.month > 9)
		{
			LCD_ShowImage(x, y+2, img_45_num[date_time.month/10]);
			x += IDLE_DATE_NUM_CN_W;
		}
		LCD_ShowImage(x, y+2, img_45_num[date_time.month%10]);

		x += IDLE_DATE_NUM_CN_W;
		LCD_ShowUniString(x, y, str_m);
		LCD_MeasureUniString(str_m, &w, &h);
		
		x += w;
		if(date_time.day > 9)
		{
			LCD_ShowImage(x, y+2, img_45_num[date_time.day/10]);
			x += IDLE_DATE_NUM_CN_W;
		}
		LCD_ShowImage(x, y+2, img_45_num[date_time.day%10]);
		
		x += IDLE_DATE_NUM_CN_W;
		LCD_ShowUniString(x, y, str_d);
		break;
 #endif/*LANGUAGE_CN_ENABLE||LANGUAGE_JA_ENABLE||LANGUAGE_KR_ENABLE*/		
	
	default:
		x = IDLE_DATE_DAY_EN_X;
		LCD_ShowImage(x+0*IDLE_DATE_NUM_EN_W, IDLE_DATE_DAY_EN_Y, img_45_num[date_time.day/10]);
		LCD_ShowImage(x+1*IDLE_DATE_NUM_EN_W, IDLE_DATE_DAY_EN_Y, img_45_num[date_time.day%10]);

		LCD_MeasureUniStr(str_mon[date_time.month-1], &str_w, &str_h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
		{
			switch(global_settings.language)
			{
		   #ifdef LANGUAGE_AR_ENABLE
			case LANGUAGE_AR:
				x = IDLE_WEEK_EN_X;
				y = IDLE_WEEK_EN_Y;
				w = IDLE_DATE_MON_EN_W;
				h = IDLE_DATE_MON_EN_H;
				break;
		   #endif
		   
			default:
				x = IDLE_DATE_MON_EN_X;
				y = IDLE_DATE_MON_EN_Y;
				w = IDLE_DATE_MON_EN_W;
				h = IDLE_DATE_MON_EN_H;
				break;
			}
			LCD_FillColor(x, y, w, h, BLACK);
			
			if(w > str_w)
				LCD_ShowUniStrRtoL(x+(w+str_w)/2, y, str_mon[date_time.month-1]);
			else
				LCD_ShowUniStrRtoL(x+w, y, str_mon[date_time.month-1]);
		}
		else
	#endif		
		{
			LCD_FillColor(IDLE_DATE_MON_EN_X, IDLE_DATE_MON_EN_Y, IDLE_DATE_MON_EN_W, IDLE_DATE_MON_EN_H, BLACK);
			
			if(IDLE_DATE_MON_EN_W > str_w)
				LCD_ShowUniStr(IDLE_DATE_MON_EN_X+(IDLE_DATE_MON_EN_W-str_w)/2, IDLE_DATE_MON_EN_Y, str_mon[date_time.month-1]);
			else
				LCD_ShowUniStr(IDLE_DATE_MON_EN_X, IDLE_DATE_MON_EN_Y, str_mon[date_time.month-1]);
		}
		break;
	}
}

void IdleShowSystemTime(void)
{
	static bool flag = false;
	uint32_t img_num[10] = {IMG_ID_FONT_129_NUM_0,IMG_ID_FONT_129_NUM_1,IMG_ID_FONT_129_NUM_2,IMG_ID_FONT_129_NUM_3,IMG_ID_FONT_129_NUM_4,
							IMG_ID_FONT_129_NUM_5,IMG_ID_FONT_129_NUM_6,IMG_ID_FONT_129_NUM_7,IMG_ID_FONT_129_NUM_8,IMG_ID_FONT_129_NUM_9};

	flag = !flag;

	LCD_ShowImage(IDLE_TIME_X+0*IDLE_TIME_NUM_W, IDLE_TIME_Y, img_num[date_time.hour/10]);
	LCD_ShowImage(IDLE_TIME_X+1*IDLE_TIME_NUM_W, IDLE_TIME_Y, img_num[date_time.hour%10]);
	
	if(flag)
		LCD_ShowImage(IDLE_TIME_X+2*IDLE_TIME_NUM_W, IDLE_TIME_Y, IMG_ID_FONT_129_COLON_Y);
	else
		LCD_ShowImage(IDLE_TIME_X+2*IDLE_TIME_NUM_W, IDLE_TIME_Y, IMG_ID_FONT_129_COLON_N);

	LCD_ShowImage(IDLE_TIME_X+2*IDLE_TIME_NUM_W+IDLE_TIME_COLON_W, IDLE_TIME_Y, img_num[date_time.minute/10]);
	LCD_ShowImage(IDLE_TIME_X+3*IDLE_TIME_NUM_W+IDLE_TIME_COLON_W, IDLE_TIME_Y, img_num[date_time.minute%10]);
}

void IdleShowSystemWeek(void)
{
	uint16_t x,y,w,h,str_w,str_h;
#ifdef FONTMAKER_UNICODE_FONT	
	uint16_t str_week[7] = {
								STR_ID_WEEK_SUN,
								STR_ID_WEEK_MON,
								STR_ID_WEEK_TUE,
								STR_ID_WEEK_WED,
								STR_ID_WEEK_THU,
								STR_ID_WEEK_FRI,
								STR_ID_WEEK_SAT,
							};
#else
	uint8_t str_week[128] = {0};
#endif
	
	POINT_COLOR=WHITE;
	BACK_COLOR=BLACK;

#ifdef FONTMAKER_UNICODE_FONT
	LCD_SetFontSize(FONT_SIZE_52);

	switch(global_settings.language)
	{
   #ifdef LANGUAGE_CN_ENABLE
	case LANGUAGE_CN:
		x = IDLE_WEEK_CN_X;
		y = IDLE_WEEK_CN_Y;
		w = IDLE_WEEK_CN_W;
		h = IDLE_WEEK_CN_H;
		break;
   #endif
   
   #if defined(LANGUAGE_JA_ENABLE)||defined(LANGUAGE_KR_ENABLE)
   #ifdef LANGUAGE_JA_ENABLE
	case LANGUAGE_JA:
   #endif
   #ifdef LANGUAGE_KR_ENABLE
	case LANGUAGE_KR:
   #endif		
		x = IDLE_WEEK_CN_X;
		y = IDLE_WEEK_CN_Y;
		w = IDLE_WEEK_CN_W;
		h = IDLE_WEEK_CN_H;
		break;
   #endif/*LANGUAGE_JA_ENABLE||LANGUAGE_KR_ENABLE*/
   
   #ifdef LANGUAGE_AR_ENABLE		
	case LANGUAGE_AR:
		x = IDLE_DATE_MON_EN_X;
		y = IDLE_DATE_MON_EN_Y;
		w = IDLE_DATE_MON_EN_W;
		h = IDLE_DATE_MON_EN_H;
		break;
   #endif

	default:
		x = IDLE_WEEK_EN_X;
		y = IDLE_WEEK_EN_Y;
		w = IDLE_WEEK_EN_W;
		h = IDLE_WEEK_EN_H;
		break;
	}	

	LCD_FillColor(x, y, w, h, BLACK);
	LCD_MeasureUniStr(str_week[date_time.week], &str_w, &str_h);
  #ifdef LANGUAGE_AR_ENABLE	
	if(g_language_r2l)
	{
		if(w > str_w)
			LCD_ShowUniStrRtoL(x+(w+str_w)/2, y, str_week[date_time.week]);
		else
			LCD_ShowUniStrRtoL(x+w, y, str_week[date_time.week]);
	}
	else
  #endif		
	{
		if(w > str_w)
			LCD_ShowUniStr(x+(w-str_w)/2, y, str_week[date_time.week]);
		else
			LCD_ShowUniStr(x, y, str_week[date_time.week]);
	}
#else
	LCD_SetFontSize(FONT_SIZE_32);
	GetSystemWeekStrings(str_week);
	LCD_ShowString(IDLE_WEEK_EN_X, IDLE_WEEK_EN_Y, str_week);
#endif
}

void IdleShowDateTime(void)
{
	IdleShowSystemTime();
	IdleShowSystemDate();
	IdleShowSystemWeek();
}

void IdleUpdateBatSoc(void)
{
	static bool flag = true;
	uint16_t w,h;
	uint8_t strbuf[128] = {0};

	if(g_bat_soc >= 100)
		sprintf(strbuf, "%d%%", g_bat_soc);
	else if(g_bat_soc >= 10)
		sprintf(strbuf, "   %d%%", g_bat_soc);
	else
		sprintf(strbuf, "      %d%%", g_bat_soc);

#ifdef FONTMAKER_UNICODE_FONT
	LCD_SetFontSize(FONT_SIZE_28);
#else	
	LCD_SetFontSize(FONT_SIZE_16);
#endif

	LCD_MeasureString(strbuf, &w, &h);
	LCD_ShowString((IDLE_BAT_X-w)-4, IDLE_BAT_PERCENT_Y, strbuf);

	if(charger_is_connected && (g_chg_status == BAT_CHARGING_PROGRESS))
	{
		if(flag)
		{
			flag = false;
			LCD_ShowImage(IDLE_BAT_X, IDLE_BAT_Y, IMG_ID_BAT_RECT_WHITE);
		}
		
		bat_charging_index++;
		if(bat_charging_index > 10)
		 bat_charging_index = 0;

		if(bat_charging_index == 0)
			LCD_Fill(IDLE_BAT_INNER_RECT_X, IDLE_BAT_INNER_RECT_Y, IDLE_BAT_INNER_RECT_W, IDLE_BAT_INNER_RECT_H, BLACK);
		else
			LCD_Fill(IDLE_BAT_INNER_RECT_X, IDLE_BAT_INNER_RECT_Y, (bat_charging_index*IDLE_BAT_INNER_RECT_W)/10, IDLE_BAT_INNER_RECT_H, GREEN);
	}
	else
	{
		flag = true;
		bat_charging_index = g_bat_soc/10;
		
		if(g_bat_soc >= 7)
		{
			LCD_ShowImage(IDLE_BAT_X, IDLE_BAT_Y, IMG_ID_BAT_RECT_WHITE);
			LCD_Fill(IDLE_BAT_INNER_RECT_X, IDLE_BAT_INNER_RECT_Y, (g_bat_soc*IDLE_BAT_INNER_RECT_W)/100, IDLE_BAT_INNER_RECT_H, GREEN);
		}
		else
		{
			LCD_ShowImage(IDLE_BAT_X, IDLE_BAT_Y, IMG_ID_BAT_RECT_RED);
			LCD_Fill(IDLE_BAT_INNER_RECT_X, IDLE_BAT_INNER_RECT_Y, (g_bat_soc*IDLE_BAT_INNER_RECT_W)/100, IDLE_BAT_INNER_RECT_H, RED);
		}
	}
}

void IdleShowBatSoc(void)
{
	uint16_t w,h;
	uint8_t strbuf[128] = {0};

	if(g_bat_soc >= 100)
		sprintf(strbuf, "%d%%", g_bat_soc);
	else if(g_bat_soc >= 10)
		sprintf(strbuf, "   %d%%", g_bat_soc);
	else
		sprintf(strbuf, "      %d%%", g_bat_soc);

#ifdef FONTMAKER_UNICODE_FONT
	LCD_SetFontSize(FONT_SIZE_28);
#else
	LCD_SetFontSize(FONT_SIZE_16);
#endif

	LCD_MeasureString(strbuf, &w, &h);
	LCD_ShowString((IDLE_BAT_X-w)-4, IDLE_BAT_PERCENT_Y, strbuf);

	bat_charging_index = g_bat_soc/10;
	
	if(charger_is_connected && (g_chg_status == BAT_CHARGING_PROGRESS))
	{
		LCD_ShowImage(IDLE_BAT_X, IDLE_BAT_Y, IMG_ID_BAT_RECT_WHITE);
		LCD_Fill(IDLE_BAT_INNER_RECT_X, IDLE_BAT_INNER_RECT_Y, (g_bat_soc*IDLE_BAT_INNER_RECT_W)/100, IDLE_BAT_INNER_RECT_H, GREEN);
	}
	else
	{
		if(g_bat_soc >= 7)
		{
			LCD_ShowImage(IDLE_BAT_X, IDLE_BAT_Y, IMG_ID_BAT_RECT_WHITE);
			LCD_Fill(IDLE_BAT_INNER_RECT_X, IDLE_BAT_INNER_RECT_Y, (g_bat_soc*IDLE_BAT_INNER_RECT_W)/100, IDLE_BAT_INNER_RECT_H, GREEN);
		}
		else
		{
			LCD_ShowImage(IDLE_BAT_X, IDLE_BAT_Y, IMG_ID_BAT_RECT_RED);
			LCD_Fill(IDLE_BAT_INNER_RECT_X, IDLE_BAT_INNER_RECT_Y, (g_bat_soc*IDLE_BAT_INNER_RECT_W)/100, IDLE_BAT_INNER_RECT_H, RED);
		}
	}
}

void IdleShowSignal(void)
{
	uint32_t img_add[5] = {IMG_ID_SIG_0, IMG_ID_SIG_1, IMG_ID_SIG_2, IMG_ID_SIG_3, IMG_ID_SIG_4};

	if(nb_is_connected())
	{
		LCD_ShowImage(IDLE_SIGNAL_X, IDLE_SIGNAL_Y, img_add[g_nb_sig]);
	}
	else
	{
		LCD_ShowImage(IDLE_SIGNAL_X, IDLE_SIGNAL_Y, img_add[0]);
	}
}

void IdleShowNetMode(void)
{
	if(nb_is_connected())
	{
		if(g_net_mode == NET_MODE_NB)
		{
			LCD_ShowImage(IDLE_NET_MODE_X, IDLE_NET_MODE_Y, IMG_ID_NET_NB);
		}
		else
		{
			LCD_ShowImage(IDLE_NET_MODE_X, IDLE_NET_MODE_Y, IMG_ID_NET_LTEM);
		}
	}
	else
	{
		LCD_Fill(IDLE_NET_MODE_X, IDLE_NET_MODE_Y, IDLE_NET_MODE_W, IDLE_NET_MODE_H, BLACK);
	}
}

#if defined(CONFIG_IMU_SUPPORT)&&defined(CONFIG_STEP_SUPPORT)
void IdleUpdateSportData(void)
{
	uint16_t bg_color = 0x00c3;
	uint8_t i,count=1;
	uint16_t steps_show;
	uint32_t divisor=10;
	uint32_t img_num[10] = {IMG_FONT_20_NUM_0_ADDR,IMG_FONT_20_NUM_1_ADDR,IMG_FONT_20_NUM_2_ADDR,IMG_FONT_20_NUM_3_ADDR,IMG_FONT_20_NUM_4_ADDR,
							IMG_FONT_20_NUM_5_ADDR,IMG_FONT_20_NUM_6_ADDR,IMG_FONT_20_NUM_7_ADDR,IMG_FONT_20_NUM_8_ADDR,IMG_FONT_20_NUM_9_ADDR};

	steps_show = last_sport.step_rec.steps;

	LCD_Fill(IDLE_STEPS_STR_X, IDLE_STEPS_STR_Y, IDLE_STEPS_STR_W, IDLE_STEPS_STR_H, bg_color);

	while(1)
	{
		if(steps_show/divisor > 0)
		{
			count++;
			divisor = divisor*10;
		}
		else
		{
			divisor = divisor/10;
			break;
		}
	}

	for(i=0;i<count;i++)
	{
		LCD_ShowImage(IDLE_STEPS_STR_X+(IDLE_STEPS_STR_W-count*IDLE_STEPS_NUM_W)/2+i*IDLE_STEPS_NUM_W, IDLE_STEPS_STR_Y, img_num[steps_show/divisor]);
		steps_show = steps_show%divisor;
		divisor = divisor/10;
	}
}

void IdleShowSportData(void)
{
	uint16_t bg_color = 0x00c3;
	uint8_t i,count=1;
	uint16_t steps_show;
	uint32_t divisor=10;
	uint32_t img_num[10] = {IMG_FONT_20_NUM_0_ADDR,IMG_FONT_20_NUM_1_ADDR,IMG_FONT_20_NUM_2_ADDR,IMG_FONT_20_NUM_3_ADDR,IMG_FONT_20_NUM_4_ADDR,
							IMG_FONT_20_NUM_5_ADDR,IMG_FONT_20_NUM_6_ADDR,IMG_FONT_20_NUM_7_ADDR,IMG_FONT_20_NUM_8_ADDR,IMG_FONT_20_NUM_9_ADDR};

	steps_show = last_sport.step_rec.steps;
	
	LCD_ShowImg_From_Flash(IDLE_STEPS_BG_X, IDLE_STEPS_BG_Y, IMG_IDLE_STEP_BG_ADDR);
	LCD_dis_pic_trans_from_flash(IDLE_STEPS_ICON_X, IDLE_STEPS_ICON_Y, IMG_IDLE_STEP_ICON_ADDR, bg_color);

	while(1)
	{
		if(steps_show/divisor > 0)
		{
			count++;
			divisor = divisor*10;
		}
		else
		{
			divisor = divisor/10;
			break;
		}
	}

	for(i=0;i<count;i++)
	{
		LCD_ShowImg_From_Flash(IDLE_STEPS_STR_X+(IDLE_STEPS_STR_W-count*IDLE_STEPS_NUM_W)/2+i*IDLE_STEPS_NUM_W, IDLE_STEPS_STR_Y, img_num[steps_show/divisor]);
		steps_show = steps_show%divisor;
		divisor = divisor/10;
	}
}
#endif

#ifdef CONFIG_PPG_SUPPORT
void IdleUpdateHrData(void)
{
	uint16_t bg_color = 0x1820;
	uint8_t i,hr_show,count=1;
	uint32_t divisor=10;
	uint32_t img_num[10] = {IMG_ID_IDLE_HR_NUM_0,IMG_ID_IDLE_HR_NUM_1,IMG_ID_IDLE_HR_NUM_2,IMG_ID_IDLE_HR_NUM_3,IMG_ID_IDLE_HR_NUM_4,
							IMG_ID_IDLE_HR_NUM_5,IMG_ID_IDLE_HR_NUM_6,IMG_ID_IDLE_HR_NUM_7,IMG_ID_IDLE_HR_NUM_8,IMG_ID_IDLE_HR_NUM_9};

	hr_show = last_health.hr_rec.hr;
	
	LCD_Fill(IDLE_HR_STR_X, IDLE_HR_STR_Y, IDLE_HR_STR_W, IDLE_HR_STR_H, bg_color);

	while(1)
	{
		if(hr_show/divisor > 0)
		{
			count++;
			divisor = divisor*10;
		}
		else
		{
			divisor = divisor/10;
			break;
		}
	}

	for(i=0;i<count;i++)
	{
		LCD_ShowImage(IDLE_HR_STR_X+(IDLE_HR_STR_W-count*IDLE_HR_NUM_W)/2+i*IDLE_HR_NUM_W, IDLE_HR_STR_Y, img_num[hr_show/divisor]);
		hr_show = hr_show%divisor;
		divisor = divisor/10;
	}
}

void IdleShowHrData(void)
{
	uint16_t bg_color = 0x1820;
	uint8_t i,hr_show,count=1;
	uint32_t divisor=10;
	uint32_t img_num[10] = {IMG_ID_IDLE_HR_NUM_0,IMG_ID_IDLE_HR_NUM_1,IMG_ID_IDLE_HR_NUM_2,IMG_ID_IDLE_HR_NUM_3,IMG_ID_IDLE_HR_NUM_4,
							IMG_ID_IDLE_HR_NUM_5,IMG_ID_IDLE_HR_NUM_6,IMG_ID_IDLE_HR_NUM_7,IMG_ID_IDLE_HR_NUM_8,IMG_ID_IDLE_HR_NUM_9};

	hr_show = last_health.hr_rec.hr;

	LCD_ShowImage(IDLE_HR_BG_X, IDLE_HR_BG_Y, IMG_ID_HR_IDLE_BG);
	LCD_ShowImageTrans(IDLE_HR_ICON_X, IDLE_HR_ICON_Y, bg_color, IMG_ID_HR_IDLE_ICON);

	while(1)
	{
		if(hr_show/divisor > 0)
		{
			count++;
			divisor = divisor*10;
		}
		else
		{
			divisor = divisor/10;
			break;
		}
	}

	for(i=0;i<count;i++)
	{
		LCD_ShowImage(IDLE_HR_STR_X+(IDLE_HR_STR_W-count*IDLE_HR_NUM_W)/2+i*IDLE_HR_NUM_W, IDLE_HR_STR_Y, img_num[hr_show/divisor]);
		hr_show = hr_show%divisor;
		divisor = divisor/10;
	}
}

void IdleUpdateSPO2Data(void)
{
	uint16_t bg_color = 0x1820;
	uint8_t i,spo2_show,count=1;
	uint32_t divisor=10;
	uint32_t img_num[10] = {IMG_ID_IDLE_SPO2_NUM_0,IMG_ID_IDLE_SPO2_NUM_1,IMG_ID_IDLE_SPO2_NUM_2,IMG_ID_IDLE_SPO2_NUM_3,IMG_ID_IDLE_SPO2_NUM_4,
							IMG_ID_IDLE_SPO2_NUM_5,IMG_ID_IDLE_SPO2_NUM_6,IMG_ID_IDLE_SPO2_NUM_7,IMG_ID_IDLE_SPO2_NUM_8,IMG_ID_IDLE_SPO2_NUM_9};

	spo2_show = last_health.spo2_rec.spo2;
	
	LCD_Fill(IDLE_SPO2_STR_X, IDLE_SPO2_STR_Y, IDLE_SPO2_STR_W, IDLE_SPO2_STR_H, bg_color);

	while(1)
	{
		if(spo2_show/divisor > 0)
		{
			count++;
			divisor = divisor*10;
		}
		else
		{
			divisor = divisor/10;
			break;
		}
	}

	for(i=0;i<count;i++)
	{
		LCD_ShowImage(IDLE_SPO2_STR_X+(IDLE_SPO2_STR_W-(IDLE_SPO2_PERC_W+count*IDLE_SPO2_NUM_W))/2+i*IDLE_SPO2_NUM_W, IDLE_SPO2_STR_Y, img_num[spo2_show/divisor]);
		spo2_show = spo2_show%divisor;
		divisor = divisor/10;
	}
	LCD_ShowImage(IDLE_SPO2_STR_X+(IDLE_SPO2_STR_W-(IDLE_SPO2_PERC_W+count*IDLE_SPO2_NUM_W))/2+i*IDLE_SPO2_NUM_W, IDLE_SPO2_STR_Y, IMG_ID_IDLE_SPO2_PERNECT);
}

void IdleShowSPO2Data(void)
{
	uint16_t bg_color = 0x1820;
	uint8_t i,spo2_show,count=1;
	uint32_t divisor=10;
	uint32_t img_num[10] = {IMG_ID_IDLE_SPO2_NUM_0,IMG_ID_IDLE_SPO2_NUM_1,IMG_ID_IDLE_SPO2_NUM_2,IMG_ID_IDLE_SPO2_NUM_3,IMG_ID_IDLE_SPO2_NUM_4,
							IMG_ID_IDLE_SPO2_NUM_5,IMG_ID_IDLE_SPO2_NUM_6,IMG_ID_IDLE_SPO2_NUM_7,IMG_ID_IDLE_SPO2_NUM_8,IMG_ID_IDLE_SPO2_NUM_9};

	spo2_show = last_health.spo2_rec.spo2;

	LCD_ShowImage(IDLE_SPO2_BG_X, IDLE_SPO2_BG_Y, IMG_ID_SPO2_IDLE_BG);
	LCD_ShowImageTrans(IDLE_SPO2_ICON_X, IDLE_SPO2_ICON_Y, bg_color, IMG_ID_SPO2_IDLE_ICON);

	while(1)
	{
		if(spo2_show/divisor > 0)
		{
			count++;
			divisor = divisor*10;
		}
		else
		{
			divisor = divisor/10;
			break;
		}
	}

	for(i=0;i<count;i++)
	{
		LCD_ShowImage(IDLE_SPO2_STR_X+(IDLE_SPO2_STR_W-(IDLE_SPO2_PERC_W+count*IDLE_SPO2_NUM_W))/2+i*IDLE_SPO2_NUM_W, IDLE_SPO2_STR_Y, img_num[spo2_show/divisor]);
		spo2_show = spo2_show%divisor;
		divisor = divisor/10;
	}
	LCD_ShowImage(IDLE_SPO2_STR_X+(IDLE_SPO2_STR_W-(IDLE_SPO2_PERC_W+count*IDLE_SPO2_NUM_W))/2+i*IDLE_SPO2_NUM_W, IDLE_SPO2_STR_Y, IMG_ID_IDLE_SPO2_PERNECT);
}

#endif

#ifdef CONFIG_TEMP_SUPPORT
void IdleUpdateTempData(void)
{
	uint16_t x,y,w,h;
	uint16_t bg_color = 0x00c3;
	uint8_t i,count=1;
	uint16_t temp_show;
	uint32_t divisor=10;
	uint32_t img_num[10] = {IMG_ID_IDLE_TEMP_NUM_0,IMG_ID_IDLE_TEMP_NUM_1,IMG_ID_IDLE_TEMP_NUM_2,IMG_ID_IDLE_TEMP_NUM_3,IMG_ID_IDLE_TEMP_NUM_4,
							IMG_ID_IDLE_TEMP_NUM_5,IMG_ID_IDLE_TEMP_NUM_6,IMG_ID_IDLE_TEMP_NUM_7,IMG_ID_IDLE_TEMP_NUM_8,IMG_ID_IDLE_TEMP_NUM_9};

	if(global_settings.temp_unit == TEMP_UINT_C)
	{
		LCD_ShowImageTrans(IDLE_TEMP_ICON_X, IDLE_TEMP_ICON_Y, bg_color, IMG_ID_TEMP_IDLE_ICON);
		temp_show = last_health.temp_rec.deca_temp;
	}
	else
	{
		LCD_ShowImageTrans(IDLE_TEMP_ICON_X, IDLE_TEMP_ICON_Y, bg_color, IMG_ID_TEMP_IDLE_ICON);
		temp_show = round((32+1.8*(last_health.temp_rec.deca_temp/10.0))*10.0);
	}

	while(1)
	{
		if(temp_show/divisor > 0)
		{
			count++;
			divisor = divisor*10;
		}
		else
		{
			divisor = divisor/10;
			break;
		}
	}

	x = IDLE_TEMP_STR_X+(IDLE_TEMP_STR_W-count*IDLE_TEMP_NUM_W-IDLE_TEMP_DOT_W)/2;
	y = IDLE_TEMP_STR_Y;

	if(count == 1)
	{
		x = IDLE_TEMP_STR_X+(IDLE_TEMP_STR_W-2*IDLE_TEMP_NUM_W-IDLE_TEMP_DOT_W)/2;
		LCD_ShowImage(x, y, img_num[temp_show/10]);
		x += IDLE_TEMP_NUM_W;
		LCD_ShowImage(x, y, IMG_ID_IDLE_TEMP_DOT);
		x += IDLE_TEMP_DOT_W;
		LCD_ShowImage(x, y, img_num[temp_show%10]);
	}
	else
	{
		for(i=0;i<(count+1);i++)
		{
			if(i == count-1)
			{
				LCD_ShowImage(x, y, IMG_ID_IDLE_TEMP_DOT);
				x += IDLE_TEMP_DOT_W;
			}
			else
			{
				LCD_ShowImage(x, y, img_num[temp_show/divisor]);
				temp_show = temp_show%divisor;
				divisor = divisor/10;
				x += IDLE_TEMP_NUM_W;
			}
		}	
	}
}

void IdleShowTempData(void)
{
	uint16_t x,y,w,h;
	uint16_t bg_color = 0x00c3;
	uint8_t i,count=1;
	uint16_t temp_show;
	uint32_t divisor=10;
	uint32_t img_num[10] = {IMG_ID_IDLE_TEMP_NUM_0,IMG_ID_IDLE_TEMP_NUM_1,IMG_ID_IDLE_TEMP_NUM_2,IMG_ID_IDLE_TEMP_NUM_3,IMG_ID_IDLE_TEMP_NUM_4,
							IMG_ID_IDLE_TEMP_NUM_5,IMG_ID_IDLE_TEMP_NUM_6,IMG_ID_IDLE_TEMP_NUM_7,IMG_ID_IDLE_TEMP_NUM_8,IMG_ID_IDLE_TEMP_NUM_9};

	LCD_ShowImage(IDLE_TEMP_BG_X, IDLE_TEMP_BG_Y, IMG_ID_TEMP_IDLE_BG);
	if(global_settings.temp_unit == TEMP_UINT_C)
	{
		LCD_ShowImageTrans(IDLE_TEMP_ICON_X, IDLE_TEMP_ICON_Y, bg_color, IMG_ID_TEMP_IDLE_ICON);
		temp_show = last_health.temp_rec.deca_temp;
	}
	else
	{
		LCD_ShowImageTrans(IDLE_TEMP_ICON_X, IDLE_TEMP_ICON_Y, bg_color, IMG_ID_TEMP_IDLE_ICON);
		temp_show = round((32+1.8*(last_health.temp_rec.deca_temp/10.0))*10.0);
	}

	while(1)
	{
		if(temp_show/divisor > 0)
		{
			count++;
			divisor = divisor*10;
		}
		else
		{
			divisor = divisor/10;
			break;
		}
	}

	x = IDLE_TEMP_STR_X+(IDLE_TEMP_STR_W-count*IDLE_TEMP_NUM_W-IDLE_TEMP_DOT_W)/2;
	y = IDLE_TEMP_STR_Y;

	if(count == 1)
	{
		x = IDLE_TEMP_STR_X+(IDLE_TEMP_STR_W-2*IDLE_TEMP_NUM_W-IDLE_TEMP_DOT_W)/2;
		LCD_ShowImage(x, y, img_num[temp_show/10]);
		x += IDLE_TEMP_NUM_W;
		LCD_ShowImage(x, y, IMG_ID_IDLE_TEMP_DOT);
		x += IDLE_TEMP_DOT_W;
		LCD_ShowImage(x, y, img_num[temp_show%10]);
	}
	else
	{
		for(i=0;i<(count+1);i++)
		{
			if(i == count-1)
			{
				LCD_ShowImage(x, y, IMG_ID_IDLE_TEMP_DOT);
				x += IDLE_TEMP_DOT_W;
			}
			else
			{
				LCD_ShowImage(x, y, img_num[temp_show/divisor]);
				temp_show = temp_show%divisor;
				divisor = divisor/10;
				x += IDLE_TEMP_NUM_W;
			}
		}
	}
}
#endif

void IdleScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_IDLE].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_IDLE].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_IDLE].status = SCREEN_STATUS_CREATED;

		LCD_Clear(BLACK);
		IdleShowSignal();
		IdleShowNetMode();
		IdleShowBatSoc();
		IdleShowDateTime();
	#ifdef CONFIG_BLE_SUPPORT	
		//IdleShowBleStatus();
	#endif	
	#ifdef CONFIG_PPG_SUPPORT
		IdleShowHrData();
		IdleShowSPO2Data();
	#endif
	#if 0//defined(CONFIG_IMU_SUPPORT)&&defined(CONFIG_STEP_SUPPORT)
		IdleShowSportData();
	#endif
	#ifdef CONFIG_TEMP_SUPPORT
		IdleShowTempData();
	#endif
		break;
		
	case SCREEN_ACTION_UPDATE:
		if(scr_msg[SCREEN_ID_IDLE].para&SCREEN_EVENT_UPDATE_SIG)
		{
			scr_msg[SCREEN_ID_IDLE].para &= (~SCREEN_EVENT_UPDATE_SIG);
			IdleShowSignal();
		}
		if(scr_msg[SCREEN_ID_IDLE].para&SCREEN_EVENT_UPDATE_NET_MODE)
		{
			scr_msg[SCREEN_ID_IDLE].para &= (~SCREEN_EVENT_UPDATE_NET_MODE);	
			IdleShowNetMode();
		}
		if(scr_msg[SCREEN_ID_IDLE].para&SCREEN_EVENT_UPDATE_BAT)
		{
			scr_msg[SCREEN_ID_IDLE].para &= (~SCREEN_EVENT_UPDATE_BAT);
			IdleUpdateBatSoc();
		}
		if(scr_msg[SCREEN_ID_IDLE].para&SCREEN_EVENT_UPDATE_TIME)
		{
			scr_msg[SCREEN_ID_IDLE].para &= (~SCREEN_EVENT_UPDATE_TIME);
			IdleShowSystemTime();
		}
		if(scr_msg[SCREEN_ID_IDLE].para&SCREEN_EVENT_UPDATE_DATE)
		{
			scr_msg[SCREEN_ID_IDLE].para &= (~SCREEN_EVENT_UPDATE_DATE);
			IdleShowSystemDate();
		}
		if(scr_msg[SCREEN_ID_IDLE].para&SCREEN_EVENT_UPDATE_WEEK)
		{
			scr_msg[SCREEN_ID_IDLE].para &= (~SCREEN_EVENT_UPDATE_WEEK);
			IdleShowSystemWeek();
		}
	#ifdef CONFIG_BLE_SUPPORT	
		if(scr_msg[SCREEN_ID_IDLE].para&SCREEN_EVENT_UPDATE_BLE)
		{
			scr_msg[SCREEN_ID_IDLE].para &= (~SCREEN_EVENT_UPDATE_BLE);
			IdleShowBleStatus();
		}
	#endif
	#if 0//defined(CONFIG_IMU_SUPPORT)&&defined(CONFIG_STEP_SUPPORT)
		if(scr_msg[SCREEN_ID_IDLE].para&SCREEN_EVENT_UPDATE_SPORT)
		{
			scr_msg[SCREEN_ID_IDLE].para &= (~SCREEN_EVENT_UPDATE_SPORT);
			IdleUpdateSportData();
		}
	#endif
	#ifdef CONFIG_PPG_SUPPORT
		if(scr_msg[SCREEN_ID_IDLE].para&SCREEN_EVENT_UPDATE_HR)
		{
			scr_msg[SCREEN_ID_IDLE].para &= (~SCREEN_EVENT_UPDATE_HR);
			IdleUpdateHrData();
		}
		if(scr_msg[SCREEN_ID_IDLE].para&SCREEN_EVENT_UPDATE_SPO2)
		{
			scr_msg[SCREEN_ID_IDLE].para &= (~SCREEN_EVENT_UPDATE_SPO2);
			IdleUpdateSPO2Data();
		}
	#endif
	#ifdef CONFIG_TEMP_SUPPORT
		if(scr_msg[SCREEN_ID_IDLE].para&SCREEN_EVENT_UPDATE_TEMP)
		{
			scr_msg[SCREEN_ID_IDLE].para &= (~SCREEN_EVENT_UPDATE_TEMP);
			IdleUpdateTempData();
		}
	#endif
		if(scr_msg[SCREEN_ID_IDLE].para == SCREEN_EVENT_UPDATE_NO)
			scr_msg[SCREEN_ID_IDLE].act = SCREEN_ACTION_NO;
	}
}

bool IsInIdleScreen(void)
{
	if(screen_id == SCREEN_ID_IDLE)
		return true;
	else
		return false;
}

void poweroff_confirm(void)
{
	k_timer_stop(&mainmenu_timer);
	ClearAllKeyHandler();

	if(screen_id == SCREEN_ID_POWEROFF)
	{
		scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
	}

	key_pwroff_flag = true;
}

void poweroff_cancel(void)
{
	k_timer_stop(&mainmenu_timer);
	EnterIdleScreen();
}

void EnterPoweroffScreen(void)
{
	if(screen_id == SCREEN_ID_POWEROFF)
		return;

#ifdef NB_SIGNAL_TEST
	if(gps_is_working())
		MenuStopGPS();
#endif

#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(TempIsWorking()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif

#ifdef CONFIG_ANIMATION_SUPPORT	
	AnimaStop();
#endif

	k_timer_stop(&mainmenu_timer);
	k_timer_start(&mainmenu_timer, K_SECONDS(5), K_NO_WAIT);

	LCD_Set_BL_Mode(LCD_BL_AUTO);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_POWEROFF;	
	scr_msg[SCREEN_ID_POWEROFF].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_POWEROFF].status = SCREEN_STATUS_CREATING;		
}

void PowerOffUpdateStatus(void)
{
	uint32_t img_anima[4] = {IMG_ID_PROGRASS_ANI_0, IMG_ID_PROGRASS_ANI_1, IMG_ID_PROGRASS_ANI_2, IMG_ID_PROGRASS_ANI_3};

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
#endif

#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaShow(POW_OFF_RUNNING_ANI_X, POW_OFF_RUNNING_ANI_Y, img_anima, ARRAY_SIZE(img_anima), 1000, true, NULL);
#endif	
}

void PowerOffShowStatus(void)
{
	uint16_t x,y,w,h;

	LCD_Clear(BLACK);

	LCD_ShowImage(PWR_OFF_ICON_X, PWR_OFF_ICON_Y, IMG_ID_PWROFF_BUTTON);

#ifdef CONFIG_FACTORY_TEST_SUPPORT
	ClearAllKeyHandler();
#endif
	SetLeftKeyUpHandler(poweroff_cancel);
	SetLeftKeyLongPressHandler(poweroff_confirm);
	
#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();

	register_touch_event_handle(TP_EVENT_LONG_PRESS, PWR_OFF_ICON_X, PWR_OFF_ICON_X+PWR_OFF_ICON_W, PWR_OFF_ICON_Y, PWR_OFF_ICON_Y+PWR_OFF_ICON_H, poweroff_confirm);
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterIdleScreen);

 #ifdef NB_SIGNAL_TEST
  #if 0	//xb add 2026-02-06
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterMainMenu);
  #else
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSettings);
  #endif
 #else
  #ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
  	if((strlen(g_ui_ver) == 0) 
		|| (strlen(g_font_ver) == 0)
		|| (strlen(g_str_ver) == 0)
		|| (strlen(g_ppg_algo_ver) == 0)
		)
	{
		SPIFlash_Read_DataVer(g_ui_ver, g_font_ver, g_str_ver, g_ppg_algo_ver);
	}

   #if defined(CONFIG_PPG_DATA_UPDATE)&&defined(CONFIG_PPG_SUPPORT)
   	if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
  	{
  		register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_ppg_start);
   	}
	else
   #endif	
	{
		if((strcmp(g_new_str_ver,g_str_ver) != 0) && (strlen(g_new_str_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
		{
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_str_start);
		}
		else if((strcmp(g_new_font_ver,g_font_ver) != 0) && (strlen(g_new_font_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
		{
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_font_start);
		}
		else if((strcmp(g_new_ui_ver,g_ui_ver) != 0) && (strlen(g_new_ui_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
		{
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_img_start);
		}
		else
		{
		#if 0	//xb add 2026-02-06
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterMainMenu);
		#else
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSettings);
		#endif
		}
	}
  #else
   #if 0	//xb add 2026-02-06
   	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterMainMenu);
   #else
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSettings);
   #endif
  #endif
 #endif 
#endif
}

void PowerOffScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_POWEROFF].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_POWEROFF].status = SCREEN_STATUS_CREATED;

		PowerOffShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		PowerOffUpdateStatus();
		break;
	}
	
	scr_msg[SCREEN_ID_POWEROFF].act = SCREEN_ACTION_NO;
}

void SettingsUpdateStatus(void)
{
	uint8_t i,count;
	uint16_t x,y,w,h;
	uint16_t bg_clor = 0x2124;
	uint16_t green_clor = 0x07e0;
	
	k_timer_stop(&mainmenu_timer);

	LCD_SetFontBgColor(bg_clor);

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
#endif

	switch(settings_menu.id)
	{
	case SETTINGS_MENU_MAIN:
		{
			uint16_t level_str[4] = {STR_ID_LEVEL_1, STR_ID_LEVEL_2, STR_ID_LEVEL_3, STR_ID_LEVEL_4};			
			uint32_t img_addr[2] = {IMG_ID_SET_TEMP_UNIT_C_ICON, IMG_ID_SET_TEMP_UNIT_F_ICON};

			entry_setting_bk_flag = false;
			
			LCD_Clear(BLACK);

			for(i=0;i<SETTINGS_MAIN_MENU_MAX_PER_PG;i++)
			{
				uint8_t copy_len;

				if((settings_menu.index + i) >= settings_menu.count)
					break;
				
				LCD_ShowImage(SETTINGS_MENU_BG_X, SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y), IMG_ID_SET_MENU_BG);
				
			#ifdef FONTMAKER_UNICODE_FONT
				LCD_SetFontSize(FONT_SIZE_28);
				LCD_SetFontColor(WHITE);
				
				LCD_MeasureUniStr(settings_menu.name[i+settings_menu.index], &w, &h);
			  #ifdef LANGUAGE_AR_ENABLE	
				if(g_language_r2l)
					LCD_SmartShowUniStr(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_STR_OFFSET_X),
									SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
									settings_menu.name[i+settings_menu.index]);
				else
			  #endif		
					LCD_ShowUniStr(SETTINGS_MENU_BG_X+SETTINGS_MENU_STR_OFFSET_X,
									SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
									settings_menu.name[i+settings_menu.index]);
			
				LCD_SetFontColor(green_clor);
				switch(settings_menu.index)
				{
				case 0:
					switch(i)
					{
					case 0:
						LCD_MeasureUniStr(STR_ID_LANGUAGE_NAME_SHOW, &w, &h);
					#ifdef LANGUAGE_AR_ENABLE	
						if(g_language_r2l)
							LCD_SmartShowUniStr(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-w),
												SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
												STR_ID_LANGUAGE_NAME_SHOW);
						else
					#endif		
							LCD_ShowUniStr(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-w,
												SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
												STR_ID_LANGUAGE_NAME_SHOW);
						break;
					case 1:
						LCD_MeasureUniStr(level_str[global_settings.backlight_level], &w, &h);
					#ifdef LANGUAGE_AR_ENABLE	
						if(g_language_r2l)
							LCD_SmartShowUniStr(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-w),
												SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
												level_str[global_settings.backlight_level]);
						else
					#endif		
							LCD_ShowUniStr(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-w,
												SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
												level_str[global_settings.backlight_level]);
						break;
					case 2:
					#ifdef LANGUAGE_AR_ENABLE	
						if(g_language_r2l)
							LCD_ShowImage(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X), 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_TEMP_UNIT_H)/2, 
											img_addr[global_settings.temp_unit]);
						else
					#endif		
							LCD_ShowImage(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-SETTINGS_MENU_TEMP_UNIT_W, 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_TEMP_UNIT_H)/2,
											img_addr[global_settings.temp_unit]);
						break;
					case 3:
					#ifdef LANGUAGE_AR_ENABLE
						if(g_language_r2l)
							LCD_ShowImage(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X),
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_LEFT_ARROW_H)/2, 
											IMG_ID_SET_L_ARROW);
						else
					#endif		
							LCD_ShowImage(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-SETTINGS_MENU_RIGHT_ARROW_W, 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_RIGHT_ARROW_H)/2,
											IMG_ID_SET_R_ARROW);
						break;	
					}
					break;
				case 4:
					switch(i)
					{
					case 0:
					#ifdef LANGUAGE_AR_ENABLE
						if(g_language_r2l)
							LCD_ShowImage(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X), 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_QR_ICON_H)/2, 
											IMG_ID_SET_QR_ICON);
						else
					#endif		
							LCD_ShowImage(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-SETTINGS_MENU_QR_ICON_W, 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_QR_ICON_H)/2,
											IMG_ID_SET_QR_ICON);
						break;
					case 1:
					case 2:
					#ifdef LANGUAGE_AR_ENABLE
						if(g_language_r2l)
							LCD_ShowImage(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X), 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_LEFT_ARROW_H)/2, 
											IMG_ID_SET_L_ARROW);
						else
					#endif		
							LCD_ShowImage(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-SETTINGS_MENU_RIGHT_ARROW_W, 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_RIGHT_ARROW_H)/2,
											IMG_ID_SET_R_ARROW);
						break;
					}
				}
			#endif

			#ifdef CONFIG_TOUCH_SUPPORT
				register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
											SETTINGS_MENU_BG_X, 
											SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W, 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y), 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+SETTINGS_MENU_BG_H, 
											settings_menu.sel_handler[i+SETTINGS_MAIN_MENU_MAX_PER_PG*(settings_menu.index/SETTINGS_MAIN_MENU_MAX_PER_PG)]);
			
			#endif
			}

		#ifdef CONFIG_TOUCH_SUPPORT
		 #ifdef NB_SIGNAL_TEST
			register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterPoweroffScreen);
		 #else
		  #ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
			if((strcmp(g_new_ui_ver,g_ui_ver) != 0) && (strlen(g_new_ui_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
			{
				register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_img_start);
			}
			else if((strcmp(g_new_font_ver,g_font_ver) != 0) && (strlen(g_new_font_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
			{
				register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_font_start);
			}
			else if((strcmp(g_new_str_ver,g_str_ver) != 0) && (strlen(g_new_str_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
			{
				register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_str_start);
			}
		   #if defined(CONFIG_PPG_DATA_UPDATE)&&defined(CONFIG_PPG_SUPPORT)
			else if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
			{
				register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_ppg_start);
			}
		   #endif
			else
		  #endif/*CONFIG_DATA_DOWNLOAD_SUPPORT*/		
			{
				register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterPoweroffScreen);
			}	
		 #endif/*NB_SIGNAL_TEST*/

		 #ifdef NB_SIGNAL_TEST
		  #ifdef CONFIG_WIFI_SUPPORT
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterWifiTestScreen);
		  #else
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterGPSTestScreen);
		  #endif
		 #elif defined(CONFIG_SYNC_SUPPORT)
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSyncDataScreen);
		 #elif defined(CONFIG_PPG_SUPPORT)
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterBPScreen);
		 #elif defined(CONFIG_TEMP_SUPPORT)
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterTempScreen);
		 #elif defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
		  #ifdef CONFIG_SLEEP_SUPPORT
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSleepScreen);
		  #elif defined(CONFIG_STEP_SUPPORT)
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterStepsScreen);
		  #endif
		 #else
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterIdleScreen);
		 #endif/*NB_SIGNAL_TEST*/
		#endif/*CONFIG_TOUCH_SUPPORT*/
		}
		break;
		
	case SETTINGS_MENU_LANGUAGE:
		{
			LCD_Clear(BLACK);
			LCD_SetFontColor(WHITE);
		#ifdef FONTMAKER_UNICODE_FONT
			LCD_SetFontSize(FONT_SIZE_28);
		#else
			LCD_SetFontSize(FONT_SIZE_16);
		#endif

			if(settings_menu.count > SETTINGS_SUB_MENU_MAX_PER_PG)
				count = (settings_menu.count - settings_menu.index >= SETTINGS_SUB_MENU_MAX_PER_PG) ? SETTINGS_SUB_MENU_MAX_PER_PG : settings_menu.count - settings_menu.index;
			else
				count = settings_menu.count;
			
			for(i=0;i<count;i++)
			{
				LCD_ShowImage(SETTINGS_MENU_BG_X, SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y), IMG_ID_SET_MENU_BG);

				if(LANG_MENU_ITEM[i+settings_menu.index] == global_settings.language)
				{
				#ifdef LANGUAGE_AR_ENABLE
					if(g_language_r2l)
						LCD_ShowImage(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X), 
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_SEL_DOT_H)/2, 
										IMG_ID_SELECT_ICON_YES);
					else
				#endif		
						LCD_ShowImage(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-SETTINGS_MENU_SEL_DOT_W, 
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_SEL_DOT_H)/2,
										IMG_ID_SELECT_ICON_YES);
				}
				else
				{
				#ifdef LANGUAGE_AR_ENABLE
					if(g_language_r2l)
						LCD_ShowImage(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X), 
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_SEL_DOT_H)/2, 
										IMG_ID_SELECT_ICON_NO);
					else
				#endif		
						LCD_ShowImage(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-SETTINGS_MENU_SEL_DOT_W, 
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_SEL_DOT_H)/2,
										IMG_ID_SELECT_ICON_NO);
				}

			#ifdef FONTMAKER_UNICODE_FONT
				LCD_MeasureUniStr(settings_menu.name[i+settings_menu.index], &w, &h);
			  #ifdef LANGUAGE_AR_ENABLE	
				if(g_language_r2l)
				{
					if(LANG_MENU_ITEM[i+settings_menu.index] == LANGUAGE_AR)
					{
						LCD_SmartShowUniStr(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_STR_OFFSET_X),
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
											settings_menu.name[i+settings_menu.index]);
					}
					else
					{
						LCD_ShowUniStr(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_STR_OFFSET_X)-w,
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
											settings_menu.name[i+settings_menu.index]);
					}
				}
				else
			  #endif		
				{
				 #ifdef LANGUAGE_AR_ENABLE
					if(LANG_MENU_ITEM[i+settings_menu.index] == LANGUAGE_AR)
					{
						//Arabic names should be displayed from right to left even in other language settings
						LCD_ShowUniStrRtoL(SETTINGS_MENU_BG_X+SETTINGS_MENU_STR_OFFSET_X+w,
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
										settings_menu.name[i+settings_menu.index]);
					}
					else
				 #endif		
					{
						LCD_ShowUniStr(SETTINGS_MENU_BG_X+SETTINGS_MENU_STR_OFFSET_X,
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
										settings_menu.name[i+settings_menu.index]);
					}
				}
			#endif

			#ifdef CONFIG_TOUCH_SUPPORT
				register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
											SETTINGS_MENU_BG_X, 
											SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W, 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y), 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+SETTINGS_MENU_BG_H, 
											settings_menu.sel_handler[i]);
			#endif
			}
		}
		break;
		
	case SETTINGS_MENU_FACTORY_RESET:
		{
			static RES_LANGUAGES_ID language_bk = LANGUAGE_MAX;
			
			LCD_Clear(BLACK);
			LCD_SetFontBgColor(BLACK);
		#ifdef FONTMAKER_UNICODE_FONT
			LCD_SetFontSize(FONT_SIZE_28);
		#else
			LCD_SetFontSize(FONT_SIZE_16);
		#endif

			if(language_bk == LANGUAGE_MAX)
				language_bk = global_settings.language;
			
			switch(g_reset_status)
			{
			case RESET_STATUS_IDLE:
				{
					LCD_ShowImage(SETTINGS_MENU_RESET_ICON_X, SETTINGS_MENU_RESET_ICON_Y, IMG_ID_RESET_DEFAULT_ANI_1);
					LCD_ShowImage(SETTINGS_MENU_RESET_NO_X, SETTINGS_MENU_RESET_NO_Y, IMG_ID_RESET_DEFAULT_SEL_N);
					LCD_ShowImage(SETTINGS_MENU_RESET_YES_X, SETTINGS_MENU_RESET_YES_Y, IMG_ID_RESET_DEFAULT_SEL_Y);

					LCD_MeasureUniStr(STR_ID_RESET_TO_FACTORY_SETTINGS, &w, &h);
				  #ifdef LANGUAGE_AR_ENABLE	
					if(g_language_r2l)
						LCD_SmartShowUniStr(LCD_WIDTH-(SETTINGS_MENU_RESET_STR_X+(SETTINGS_MENU_RESET_STR_W-w)/2), 
												SETTINGS_MENU_RESET_STR_Y+(SETTINGS_MENU_RESET_STR_H-h)/2, 
												STR_ID_RESET_TO_FACTORY_SETTINGS);
					else
				  #endif		
						LCD_ShowUniStr(SETTINGS_MENU_RESET_STR_X+(SETTINGS_MENU_RESET_STR_W-w)/2, 
											SETTINGS_MENU_RESET_STR_Y+(SETTINGS_MENU_RESET_STR_H-h)/2, 
											STR_ID_RESET_TO_FACTORY_SETTINGS);
				#ifdef CONFIG_TOUCH_SUPPORT
					register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
												SETTINGS_MENU_RESET_NO_X, 
												SETTINGS_MENU_RESET_NO_X+SETTINGS_MENU_RESET_NO_W, 
												SETTINGS_MENU_RESET_NO_Y, 
												SETTINGS_MENU_RESET_NO_Y+SETTINGS_MENU_RESET_NO_H, 
												settings_menu.sel_handler[0]);
					register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
												SETTINGS_MENU_RESET_YES_X, 
												SETTINGS_MENU_RESET_YES_X+SETTINGS_MENU_RESET_YES_W, 
												SETTINGS_MENU_RESET_YES_Y, 
												SETTINGS_MENU_RESET_YES_Y+SETTINGS_MENU_RESET_YES_H, 
												settings_menu.sel_handler[1]);
				#endif	
				}
				break;
				
			case RESET_STATUS_RUNNING:
				{
					uint32_t img_addr[8] = {
											IMG_ID_RESET_DEFAULT_ANI_1,
											IMG_ID_RESET_DEFAULT_ANI_2,
											IMG_ID_RESET_DEFAULT_ANI_3,
											IMG_ID_RESET_DEFAULT_ANI_4,
											IMG_ID_RESET_DEFAULT_ANI_5,
											IMG_ID_RESET_DEFAULT_ANI_6,
											IMG_ID_RESET_DEFAULT_ANI_7,
											IMG_ID_RESET_DEFAULT_ANI_8
										};

					LCD_MeasureUniStr(STR_ID_RESETTING_IN_PROGRESS, &w, &h);
				  #ifdef LANGUAGE_AR_ENABLE	
					if(g_language_r2l)
						LCD_SmartShowUniStr(LCD_WIDTH-(SETTINGS_MENU_RESET_NOTIFY_X+(SETTINGS_MENU_RESET_NOTIFY_W-w)/2), 
												SETTINGS_MENU_RESET_NOTIFY_Y+(SETTINGS_MENU_RESET_NOTIFY_H-h)/2, 
												STR_ID_RESETTING_IN_PROGRESS);
					else
				  #endif		
						LCD_ShowUniStr(SETTINGS_MENU_RESET_NOTIFY_X+(SETTINGS_MENU_RESET_NOTIFY_W-w)/2, 
											SETTINGS_MENU_RESET_NOTIFY_Y+(SETTINGS_MENU_RESET_NOTIFY_H-h)/2, 
											STR_ID_RESETTING_IN_PROGRESS);
				#ifdef CONFIG_ANIMATION_SUPPORT
					AnimaShow(SETTINGS_MENU_RESET_LOGO_X, SETTINGS_MENU_RESET_LOGO_Y, img_addr, ARRAY_SIZE(img_addr), 300, true, NULL);
				#endif	
				}
				break;
				
			case RESET_STATUS_SUCCESS:
				{
				#ifdef CONFIG_ANIMATION_SUPPORT
					AnimaStop();
				#endif

					LCD_ShowImage(SETTINGS_MENU_RESET_LOGO_X, SETTINGS_MENU_RESET_LOGO_Y, IMG_ID_RESET_DEFAULT_SUCCESS);

					LCD_MeasureUniStr(STR_ID_RESET_COMPLETED, &w, &h);
				  #ifdef LANGUAGE_AR_ENABLE	
					if(g_language_r2l)
						LCD_SmartShowUniStr(LCD_WIDTH-(SETTINGS_MENU_RESET_NOTIFY_X+(SETTINGS_MENU_RESET_NOTIFY_W-w)/2), 
												SETTINGS_MENU_RESET_NOTIFY_Y+(SETTINGS_MENU_RESET_NOTIFY_H-h)/2, 
												STR_ID_RESET_COMPLETED);
					else
				  #endif
						LCD_ShowUniStr(SETTINGS_MENU_RESET_NOTIFY_X+(SETTINGS_MENU_RESET_NOTIFY_W-w)/2, 
											SETTINGS_MENU_RESET_NOTIFY_Y+(SETTINGS_MENU_RESET_NOTIFY_H-h)/2, 
											STR_ID_RESET_COMPLETED);
				}
				break;
				
			case RESET_STATUS_FAIL:
				{
				#ifdef CONFIG_ANIMATION_SUPPORT
					AnimaStop();
				#endif

					LCD_ShowImage(SETTINGS_MENU_RESET_LOGO_X, SETTINGS_MENU_RESET_LOGO_Y, IMG_ID_RESET_DEFAULT_FAIL);

					LCD_MeasureUniString(STR_ID_RESET_FAILED, &w, &h);
				#ifdef LANGUAGE_AR_ENABLE	
					if(g_language_r2l)
						LCD_SmartShowUniStr(LCD_WIDTH-(SETTINGS_MENU_RESET_NOTIFY_X+(SETTINGS_MENU_RESET_NOTIFY_W-w)/2), 
										SETTINGS_MENU_RESET_NOTIFY_Y+(SETTINGS_MENU_RESET_NOTIFY_H-h)/2, 
										STR_ID_RESET_FAILED);
					else
				#endif
						LCD_ShowUniStr(SETTINGS_MENU_RESET_NOTIFY_X+(SETTINGS_MENU_RESET_NOTIFY_W-w)/2, 
										SETTINGS_MENU_RESET_NOTIFY_Y+(SETTINGS_MENU_RESET_NOTIFY_H-h)/2, 
										STR_ID_RESET_FAILED);
					language_bk = LANGUAGE_MAX;
				}
				break;
			}
		}
		break;
		
	case SETTINGS_MENU_OTA:
		{
			LCD_Clear(BLACK);
			LCD_SetFontBgColor(BLACK);
		#ifdef FONTMAKER_UNICODE_FONT
			LCD_SetFontSize(FONT_SIZE_28);
		#else
			LCD_SetFontSize(FONT_SIZE_16);
		#endif

			LCD_MeasureUniStr(STR_ID_LATEST_VERSION, &w, &h);
		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_SmartShowUniStr(LCD_WIDTH-(LCD_WIDTH-w)/2, (LCD_HEIGHT-h)/2, STR_ID_LATEST_VERSION);
			else
		#endif
				LCD_ShowUniStr((LCD_WIDTH-w)/2, (LCD_HEIGHT-h)/2, STR_ID_LATEST_VERSION);

		#ifdef CONFIG_TOUCH_SUPPORT
			register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
										0, 
										LCD_WIDTH, 
										0, 
										LCD_HEIGHT, 
										settings_menu.sel_handler[0]);
		#endif	
		}
		break;
		
	case SETTINGS_MENU_BRIGHTNESS:
		{
			uint32_t img_addr[4] = {IMG_ID_SCR_BL_1,IMG_ID_SCR_BL_2,IMG_ID_SCR_BL_3,IMG_ID_SCR_BL_4};

			if(entry_setting_bk_flag == false)
			{
				entry_setting_bk_flag = true;
				LCD_Clear(BLACK);
				LCD_ShowImage(SETTINGS_MENU_BK_DEC_X, SETTINGS_MENU_BK_DEC_Y, IMG_ID_SCR_BL_DEC);
				LCD_ShowImage(SETTINGS_MENU_BK_INC_X, SETTINGS_MENU_BK_INC_Y, IMG_ID_SCR_BL_INC);
			}
			
			LCD_ShowImage(SETTINGS_MENU_BK_LEVEL_X, SETTINGS_MENU_BK_LEVEL_Y, img_addr[global_settings.backlight_level]);

		#ifdef CONFIG_TOUCH_SUPPORT
			register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
										SETTINGS_MENU_BK_DEC_X, 
										SETTINGS_MENU_BK_DEC_X+SETTINGS_MENU_BK_DEC_W, 
										SETTINGS_MENU_BK_DEC_Y, 
										SETTINGS_MENU_BK_DEC_Y+SETTINGS_MENU_BK_DEC_H, 
										settings_menu.sel_handler[0]
										);
			register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
										SETTINGS_MENU_BK_INC_X, 
										SETTINGS_MENU_BK_INC_X+SETTINGS_MENU_BK_INC_W, 
										SETTINGS_MENU_BK_INC_Y, 
										SETTINGS_MENU_BK_INC_Y+SETTINGS_MENU_BK_INC_H, 
										settings_menu.sel_handler[1]
										);
			register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
										0, 
										LCD_WIDTH, 
										0, 
										SETTINGS_MENU_BK_INC_Y-10, 
										settings_menu.sel_handler[2]
										);			
		
			register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, settings_menu.pg_handler[2]);
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, settings_menu.pg_handler[3]);
		#endif
		}
		break;
		
	case SETTINGS_MENU_TEMP:
		{
			LCD_Clear(BLACK);
			LCD_SetFontColor(WHITE);
		#ifdef FONTMAKER_UNICODE_FONT
			LCD_SetFontSize(FONT_SIZE_28);
		#else
			LCD_SetFontSize(FONT_SIZE_16);
		#endif

			for(i=0;i<settings_menu.count;i++)
			{
				LCD_ShowImage(SETTINGS_MENU_BG_X, SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y), IMG_ID_SET_MENU_BG);

				if(i == global_settings.temp_unit)
				{
				#ifdef LANGUAGE_AR_ENABLE
					if(g_language_r2l)
						LCD_ShowImage(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-SETTINGS_MENU_SEL_DOT_W), 
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_SEL_DOT_H)/2, 
										IMG_ID_SELECT_ICON_YES);
					else
				#endif		
						LCD_ShowImage(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-SETTINGS_MENU_SEL_DOT_W, 
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_SEL_DOT_H)/2,
										IMG_ID_SELECT_ICON_YES);
				}
				else
				{
				#ifdef LANGUAGE_AR_ENABLE
					if(g_language_r2l)
						LCD_ShowImage(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-SETTINGS_MENU_SEL_DOT_W), 
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_SEL_DOT_H)/2, 
										IMG_ID_SELECT_ICON_NO);
					else
				#endif		
						LCD_ShowImage(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-SETTINGS_MENU_SEL_DOT_W, 
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_SEL_DOT_H)/2,
										IMG_ID_SELECT_ICON_NO);
				}
				
			#ifdef FONTMAKER_UNICODE_FONT
				LCD_MeasureUniStr(settings_menu.name[i], &w, &h);
			  #ifdef LANGUAGE_AR_ENABLE	
				if(g_language_r2l)
					LCD_SmartShowUniStr(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_STR_OFFSET_X),
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
										settings_menu.name[i]);
				else
			  #endif
					LCD_ShowUniStr(SETTINGS_MENU_BG_X+SETTINGS_MENU_STR_OFFSET_X,
										SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
										settings_menu.name[i]);
			#endif

			#ifdef CONFIG_TOUCH_SUPPORT
				register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
											SETTINGS_MENU_BG_X, 
											SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W, 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y), 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+SETTINGS_MENU_BG_H, 
											settings_menu.sel_handler[i]);
			#endif
			}
		}
		break;
		
	case SETTINGS_MENU_DEVICE:
		{
			uint16_t imei_str[IMEI_MAX_LEN+1] = {0};
			uint16_t imsi_str[IMSI_MAX_LEN+1] = {0};
			uint16_t mcu_str[20] = {0x0000};
			uint16_t *menu_sle_str[3] = {imei_str,imsi_str,mcu_str};
			uint16_t menu_color = 0x9CD3;

			LCD_Clear(BLACK);

			mmi_asc_to_ucs2((uint8_t*)imei_str, g_imei);
			mmi_asc_to_ucs2((uint8_t*)imsi_str, g_imsi);
			mmi_asc_to_ucs2((uint8_t*)mcu_str, g_fw_version);
		
			if(settings_menu.count > SETTINGS_SUB_MENU_MAX_PER_PG)
				count = (settings_menu.count - settings_menu.index >= SETTINGS_SUB_MENU_MAX_PER_PG) ? SETTINGS_SUB_MENU_MAX_PER_PG : settings_menu.count - settings_menu.index;
			else
				count = settings_menu.count;
			
			for(i=0;i<count;i++)
			{
				LCD_ShowImage(SETTINGS_MENU_BG_X, SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y), IMG_ID_SET_MENU_BG);

			#ifdef FONTMAKER_UNICODE_FONT
				LCD_SetFontColor(menu_color);
				LCD_SetFontSize(FONT_SIZE_28);
				
				LCD_ShowUniStr(SETTINGS_MENU_BG_X+SETTINGS_MENU_STR_OFFSET_X,
									SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+SETTINGS_MENU_STR_OFFSET_Y,
									settings_menu.name[i+settings_menu.index]);

				LCD_SetFontColor(WHITE);
		  		LCD_ShowUniString(SETTINGS_MENU_BG_X+SETTINGS_MENU_STR_OFFSET_X,
									SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+SETTINGS_MENU_STR_OFFSET_Y+30,
									menu_sle_str[i+settings_menu.index]);
			#endif

			#ifdef CONFIG_TOUCH_SUPPORT
				register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
											SETTINGS_MENU_BG_X, 
											SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W, 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y), 
											SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+SETTINGS_MENU_BG_H, 
											settings_menu.sel_handler[i+settings_menu.index]);
			#endif
			}
		}
		break;
		
	case SETTINGS_MENU_CAREMATE_QR:
		{
			uint16_t tmpbuf[128] = {0};

			LCD_Clear(BLACK);
			LCD_ReSetFontBgColor();
			LCD_ReSetFontColor();
		#ifdef FONTMAKER_UNICODE_FONT
			LCD_SetFontSize(FONT_SIZE_20);
		#else
			LCD_SetFontSize(FONT_SIZE_16);
		#endif

		#ifdef CONFIG_QRCODE_SUPPORT
			sprintf(tmpbuf, "%s%s", SETTINGS_CAREMATE_URL, g_imei);
			show_QR_code(strlen(tmpbuf), tmpbuf);
		#else
			LCD_MeasureUniStr(STR_ID_FUN_BEING_DEVELOPED, &w, &h);
		  #ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_SmartShowUniStr(LCD_WIDTH-(LCD_WIDTH-w)/2, (LCD_HEIGHT-h)/2, STR_ID_FUN_BEING_DEVELOPED);
			else
		  #endif
				LCD_ShowUniStr((LCD_WIDTH-w)/2, (LCD_HEIGHT-h)/2, STR_ID_FUN_BEING_DEVELOPED);
		#endif/*CONFIG_QRCODE_SUPPORT*/
		
		#ifdef CONFIG_TOUCH_SUPPORT
			register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 0, LCD_WIDTH, 0, LCD_HEIGHT, settings_menu.sel_handler[0]);
			register_touch_event_handle(TP_EVENT_MOVING_UP, 0, LCD_WIDTH, 0, LCD_HEIGHT, settings_menu.sel_handler[0]);
			register_touch_event_handle(TP_EVENT_MOVING_DOWN, 0, LCD_WIDTH, 0, LCD_HEIGHT, settings_menu.sel_handler[0]);
			register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, settings_menu.sel_handler[0]);
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, settings_menu.sel_handler[0]);
		#endif
		}
		break;
	}

	if(settings_menu.count > SETTINGS_MAIN_MENU_MAX_PER_PG)
	{		
		if(settings_menu.count > 5*SETTINGS_MAIN_MENU_MAX_PER_PG)
		{
			uint32_t page6_img[6] = {IMG_ID_PAGE6_1,IMG_ID_PAGE6_2,IMG_ID_PAGE6_3,IMG_ID_PAGE6_4,IMG_ID_PAGE6_5,IMG_ID_PAGE6_6};

		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowImage(LCD_WIDTH-SETTINGS_MEUN_PAGE6_DOT_X-SETTINGS_MEUN_PAGE6_DOT_W, SETTINGS_MEUN_PAGE6_DOT_Y, page6_img[settings_menu.index/SETTINGS_MAIN_MENU_MAX_PER_PG]);
			else
		#endif
				LCD_ShowImage(SETTINGS_MEUN_PAGE6_DOT_X, SETTINGS_MEUN_PAGE6_DOT_Y, page6_img[settings_menu.index/SETTINGS_MAIN_MENU_MAX_PER_PG]);
		}
		else if(settings_menu.count > 4*SETTINGS_MAIN_MENU_MAX_PER_PG)
		{
			uint32_t page5_img[5] = {IMG_ID_PAGE5_1,IMG_ID_PAGE5_2,IMG_ID_PAGE5_3,IMG_ID_PAGE5_4,IMG_ID_PAGE5_5};

		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowImage(LCD_WIDTH-SETTINGS_MEUN_PAGE5_DOT_X-SETTINGS_MEUN_PAGE5_DOT_W, SETTINGS_MEUN_PAGE5_DOT_Y, page5_img[settings_menu.index/SETTINGS_MAIN_MENU_MAX_PER_PG]);
			else
		#endif		
				LCD_ShowImage(SETTINGS_MEUN_PAGE5_DOT_X, SETTINGS_MEUN_PAGE5_DOT_Y, page5_img[settings_menu.index/SETTINGS_MAIN_MENU_MAX_PER_PG]);
		}
		else if(settings_menu.count > 3*SETTINGS_MAIN_MENU_MAX_PER_PG)
		{
			uint32_t page4_img[4] = {IMG_ID_PAGE4_1,IMG_ID_PAGE4_2,IMG_ID_PAGE4_3,IMG_ID_PAGE4_4};

		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowImage(LCD_WIDTH-SETTINGS_MEUN_PAGE4_DOT_X-SETTINGS_MEUN_PAGE4_DOT_W, SETTINGS_MEUN_PAGE4_DOT_Y, page4_img[settings_menu.index/SETTINGS_MAIN_MENU_MAX_PER_PG]);
			else
		#endif
				LCD_ShowImage(SETTINGS_MEUN_PAGE4_DOT_X, SETTINGS_MEUN_PAGE4_DOT_Y, page4_img[settings_menu.index/SETTINGS_MAIN_MENU_MAX_PER_PG]);
		}
		else if(settings_menu.count > 2*SETTINGS_MAIN_MENU_MAX_PER_PG)
		{
			uint32_t page3_img[3] = {IMG_ID_PAGE3_1,IMG_ID_PAGE3_2,IMG_ID_PAGE3_3};

		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowImage(LCD_WIDTH-SETTINGS_MEUN_PAGE3_DOT_X-SETTINGS_MEUN_PAGE3_DOT_W, SETTINGS_MEUN_PAGE3_DOT_Y, page3_img[settings_menu.index/SETTINGS_MAIN_MENU_MAX_PER_PG]);
			else
		#endif
				LCD_ShowImage(SETTINGS_MEUN_PAGE3_DOT_X, SETTINGS_MEUN_PAGE3_DOT_Y, page3_img[settings_menu.index/SETTINGS_MAIN_MENU_MAX_PER_PG]);
		}
		else
		{
			uint32_t page2_img[2] = {IMG_ID_PAGE2_1,IMG_ID_PAGE2_2};

		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowImage(LCD_WIDTH-SETTINGS_MEUN_PAGE2_DOT_X-SETTINGS_MEUN_PAGE2_DOT_W, SETTINGS_MEUN_PAGE2_DOT_Y, page2_img[settings_menu.index/SETTINGS_MAIN_MENU_MAX_PER_PG]);
			else
		#endif
				LCD_ShowImage(SETTINGS_MEUN_PAGE2_DOT_X, SETTINGS_MEUN_PAGE2_DOT_Y, page2_img[settings_menu.index/SETTINGS_MAIN_MENU_MAX_PER_PG]);
		}

	#ifdef CONFIG_TOUCH_SUPPORT
		register_touch_event_handle(TP_EVENT_MOVING_UP, 0, LCD_WIDTH, 0, LCD_HEIGHT, settings_menu.pg_handler[0]);
		register_touch_event_handle(TP_EVENT_MOVING_DOWN, 0, LCD_WIDTH, 0, LCD_HEIGHT, settings_menu.pg_handler[1]);
	#endif		
	}

	LCD_ReSetFontBgColor();
	LCD_ReSetFontColor();

	if(settings_menu.id == SETTINGS_MENU_CAREMATE_QR)
		k_timer_start(&mainmenu_timer, K_SECONDS(8), K_NO_WAIT);
	else
		k_timer_start(&mainmenu_timer, K_SECONDS(5), K_NO_WAIT);
}

void SettingsShowStatus(void)
{
	uint16_t i,x,y,w,h;
	uint16_t level_str[4] = {STR_ID_LEVEL_1, STR_ID_LEVEL_2, STR_ID_LEVEL_3, STR_ID_LEVEL_4};
	uint32_t img_addr[2] = {IMG_ID_SET_TEMP_UNIT_C_ICON, IMG_ID_SET_TEMP_UNIT_F_ICON};
	uint16_t bg_clor = 0x2124;
	uint16_t green_clor = 0x07e0;

	LCD_Clear(BLACK);
	LCD_SetFontBgColor(bg_clor);

	for(i=0;i<SETTINGS_MAIN_MENU_MAX_PER_PG;i++)
	{
		LCD_ShowImage(SETTINGS_MENU_BG_X, SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y), IMG_ID_SET_MENU_BG);

	#ifdef FONTMAKER_UNICODE_FONT
		LCD_SetFontSize(FONT_SIZE_28);
		LCD_SetFontColor(WHITE);

		LCD_MeasureUniStr(settings_menu.name[i], &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_SmartShowUniStr(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_STR_OFFSET_X),
							SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
							settings_menu.name[i]);
		else
	#endif
			LCD_ShowUniStr(SETTINGS_MENU_BG_X+SETTINGS_MENU_STR_OFFSET_X,
							SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
							settings_menu.name[i]);

		LCD_SetFontColor(green_clor);
		switch(i)
		{
		case 0:
			LCD_MeasureUniStr(STR_ID_LANGUAGE_NAME_SHOW, &w, &h);
		#ifdef LANGUAGE_AR_ENABLE	
			if(g_language_r2l)
				LCD_SmartShowUniStr(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-w),
									SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
									STR_ID_LANGUAGE_NAME_SHOW);
			else
		#endif		
				LCD_ShowUniStr(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-w,
									SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
									STR_ID_LANGUAGE_NAME_SHOW);
			break;
		case 1:
			LCD_MeasureUniStr(level_str[global_settings.backlight_level], &w, &h);
		#ifdef LANGUAGE_AR_ENABLE	
			if(g_language_r2l)
				LCD_SmartShowUniStr(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-w),
									SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
									level_str[global_settings.backlight_level]);
			else
		#endif		
				LCD_ShowUniStr(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-w,
									SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-h)/2,
									level_str[global_settings.backlight_level]);
			break;
		case 2:
		#ifdef LANGUAGE_AR_ENABLE	
			if(g_language_r2l)
				LCD_ShowImage(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X), 
								SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_TEMP_UNIT_H)/2, 
								img_addr[global_settings.temp_unit]);
			else
		#endif		
				LCD_ShowImage(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-SETTINGS_MENU_TEMP_UNIT_W, 
								SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_TEMP_UNIT_H)/2,
								img_addr[global_settings.temp_unit]);
			break;
		case 3:
		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowImage(LCD_WIDTH-(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X), 
								SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_LEFT_ARROW_H)/2, 
								IMG_ID_SET_L_ARROW);
			else
		#endif		
				LCD_ShowImage(SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W-SETTINGS_MENU_STR_OFFSET_X-SETTINGS_MENU_RIGHT_ARROW_W, 
								SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+(SETTINGS_MENU_BG_H-SETTINGS_MENU_RIGHT_ARROW_H)/2,
								IMG_ID_SET_R_ARROW);
			break;	
		}
	#endif	

	#ifdef CONFIG_TOUCH_SUPPORT
		register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
									SETTINGS_MENU_BG_X, 
									SETTINGS_MENU_BG_X+SETTINGS_MENU_BG_W, 
									SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y), 
									SETTINGS_MENU_BG_Y+i*(SETTINGS_MENU_BG_H+SETTINGS_MENU_BG_OFFSET_Y)+SETTINGS_MENU_BG_H, 
									settings_menu.sel_handler[i]);
	#endif
	}

	if(settings_menu.count > SETTINGS_MAIN_MENU_MAX_PER_PG)
	{
		if(settings_menu.count > 5*SETTINGS_MAIN_MENU_MAX_PER_PG)
		{
		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowImage(LCD_WIDTH-SETTINGS_MEUN_PAGE6_DOT_X-SETTINGS_MEUN_PAGE6_DOT_W, SETTINGS_MEUN_PAGE6_DOT_Y, IMG_ID_PAGE6_1);
			else
		#endif		
				LCD_ShowImage(SETTINGS_MEUN_PAGE6_DOT_X, SETTINGS_MEUN_PAGE6_DOT_Y, IMG_ID_PAGE6_1);
		}
		else if(settings_menu.count > 4*SETTINGS_MAIN_MENU_MAX_PER_PG)
		{
		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowImage(LCD_WIDTH-SETTINGS_MEUN_PAGE5_DOT_X-SETTINGS_MEUN_PAGE5_DOT_W, SETTINGS_MEUN_PAGE5_DOT_Y, IMG_ID_PAGE5_1);
			else
		#endif		
				LCD_ShowImage(SETTINGS_MEUN_PAGE5_DOT_X, SETTINGS_MEUN_PAGE5_DOT_Y, IMG_ID_PAGE5_1);
		}
		else if(settings_menu.count > 3*SETTINGS_MAIN_MENU_MAX_PER_PG)
		{
		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowImage(LCD_WIDTH-SETTINGS_MEUN_PAGE4_DOT_X-SETTINGS_MEUN_PAGE4_DOT_W, SETTINGS_MEUN_PAGE4_DOT_Y, IMG_ID_PAGE4_1);
			else
		#endif		
				LCD_ShowImage(SETTINGS_MEUN_PAGE4_DOT_X, SETTINGS_MEUN_PAGE4_DOT_Y, IMG_ID_PAGE4_1);
		}
		else if(settings_menu.count > 2*SETTINGS_MAIN_MENU_MAX_PER_PG)
		{
		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowImage(LCD_WIDTH-SETTINGS_MEUN_PAGE3_DOT_X-SETTINGS_MEUN_PAGE3_DOT_W, SETTINGS_MEUN_PAGE3_DOT_Y, IMG_ID_PAGE3_1);
			else
		#endif		
				LCD_ShowImage(SETTINGS_MEUN_PAGE3_DOT_X, SETTINGS_MEUN_PAGE3_DOT_Y, IMG_ID_PAGE3_1);
		}
		else
		{
		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowImage(LCD_WIDTH-SETTINGS_MEUN_PAGE2_DOT_X-SETTINGS_MEUN_PAGE2_DOT_W, SETTINGS_MEUN_PAGE2_DOT_Y, IMG_ID_PAGE2_1);
			else
		#endif		
				LCD_ShowImage(SETTINGS_MEUN_PAGE2_DOT_X, SETTINGS_MEUN_PAGE2_DOT_Y, IMG_ID_PAGE2_1);
		}

	#ifdef CONFIG_TOUCH_SUPPORT
		register_touch_event_handle(TP_EVENT_MOVING_UP, 0, LCD_WIDTH, 0, LCD_HEIGHT, settings_menu.pg_handler[0]);
		register_touch_event_handle(TP_EVENT_MOVING_DOWN, 0, LCD_WIDTH, 0, LCD_HEIGHT, settings_menu.pg_handler[1]);
	#endif		
	}

	entry_setting_bk_flag = false;
	LCD_ReSetFontBgColor();
	LCD_ReSetFontColor();

	k_timer_stop(&mainmenu_timer);
	k_timer_start(&mainmenu_timer, K_SECONDS(5), K_NO_WAIT);
}

void SettingsScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_SETTINGS].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_SETTINGS].status = SCREEN_STATUS_CREATED;
		SettingsShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		SettingsUpdateStatus();			
		break;
	}
	
	scr_msg[SCREEN_ID_SETTINGS].act = SCREEN_ACTION_NO;
}

void ExitSettingsScreen(void)
{
	EntryIdleScr();
}

void EnterSettingsScreen(void)
{
	if(screen_id == SCREEN_ID_SETTINGS)
		return;

	k_timer_stop(&mainmenu_timer);
#ifdef CONFIG_ANIMATION_SUPPORT	
	AnimaStop();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(TempIsWorking()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();
#endif
#ifdef CONFIG_ECG_SUPPORT
	if(IsInEcgScreen())
		ExitEcgScreen();
#endif
#ifdef CONFIG_WIFI_SUPPORT
	if(IsInWifiScreen()&&wifi_is_working())
		MenuStopWifi();
#endif

	LCD_Set_BL_Mode(LCD_BL_AUTO);
	
	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_SETTINGS;	
	scr_msg[SCREEN_ID_SETTINGS].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_SETTINGS].status = SCREEN_STATUS_CREATING;

#ifdef NB_SIGNAL_TEST
	SetLeftKeyUpHandler(EnterPoweroffScreen);
#else
 #ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
 	if((strlen(g_ui_ver) == 0) 
		|| (strlen(g_font_ver) == 0)
		|| (strlen(g_str_ver) == 0)
		|| (strlen(g_ppg_algo_ver) == 0)
		)
	{
		SPIFlash_Read_DataVer(g_ui_ver, g_font_ver, g_str_ver, g_ppg_algo_ver);
	}
 
  	if((strcmp(g_new_ui_ver,g_ui_ver) != 0) && (strlen(g_new_ui_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		SetLeftKeyUpHandler(dl_img_start);
	}
	else if((strcmp(g_new_font_ver,g_font_ver) != 0) && (strlen(g_new_font_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		SetLeftKeyUpHandler(dl_font_start);
	}
	else if((strcmp(g_new_str_ver,g_str_ver) != 0) && (strlen(g_new_str_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		SetLeftKeyUpHandler(dl_str_start);
	}
  #if defined(CONFIG_PPG_DATA_UPDATE)&&defined(CONFIG_PPG_SUPPORT)
	else if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
	{
		SetLeftKeyUpHandler(dl_ppg_start);
	}
  #endif
  	else
 #endif/*CONFIG_DATA_DOWNLOAD_SUPPORT*/
  	{
  		SetLeftKeyUpHandler(EnterPoweroffScreen);
  	}
#endif/*NB_SIGNAL_TEST*/
	SetRightKeyUpHandler(ExitSettingsScreen);

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();

 #ifdef NB_SIGNAL_TEST
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterPoweroffScreen);
 #else
  #ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
	if((strcmp(g_new_ui_ver,g_ui_ver) != 0) && (strlen(g_new_ui_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_img_start);
	}
	else if((strcmp(g_new_font_ver,g_font_ver) != 0) && (strlen(g_new_font_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_font_start);
	}
	else if((strcmp(g_new_str_ver,g_str_ver) != 0) && (strlen(g_new_str_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_str_start);
	}
   #if defined(CONFIG_PPG_DATA_UPDATE)&&defined(CONFIG_PPG_SUPPORT)
	else if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
	{
		register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_ppg_start);
	}
   #endif
	else
  #endif/*CONFIG_DATA_DOWNLOAD_SUPPORT*/
	{
		register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterPoweroffScreen);
 	}
 #endif/*NB_SIGNAL_TEST*/

 #ifdef NB_SIGNAL_TEST
  #ifdef CONFIG_WIFI_SUPPORT
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterWifiTestScreen);
  #else
 	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterGPSTestScreen);
  #endif
 #else
  #ifdef CONFIG_SYNC_SUPPORT
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSyncDataScreen);
  #elif defined(CONFIG_PPG_SUPPORT)
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterBPScreen);
  #elif defined(CONFIG_TEMP_SUPPORT)
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterTempScreen);
  #elif defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
   #ifdef CONFIG_SLEEP_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSleepScreen);
   #elif defined(CONFIG_STEP_SUPPORT)
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterStepsScreen);
   #endif
  #else
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterIdleScreen);
  #endif/*CONFIG_SYNC_SUPPORT*/
 #endif/*NB_SIGNAL_TEST*/
#endif
}

#ifdef CONFIG_SYNC_SUPPORT
void ExitSyncDataScreen(void)
{
	LCD_Set_BL_Mode(LCD_BL_AUTO);
	SyncDataStop();
	EnterIdleScreen();
}

void EnterSyncDataScreen(void)
{
	if(screen_id == SCREEN_ID_SYNC)
		return;

	k_timer_stop(&mainmenu_timer);
	k_timer_start(&mainmenu_timer, K_SECONDS(3), K_NO_WAIT);
#ifdef CONFIG_ANIMATION_SUPPORT	
	AnimaStop();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(TempIsWorking()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();
#endif
#ifdef CONFIG_ECG_SUPPORT
	if(IsInEcgScreen())
		ExitEcgScreen();
#endif
	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_SYNC;
	scr_msg[SCREEN_ID_SYNC].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_SYNC].status = SCREEN_STATUS_CREATING;

	SetLeftKeyUpHandler(EnterSettings);
	SetRightKeyUpHandler(ExitSyncDataScreen);

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();

	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSettings);

  #ifdef CONFIG_ECG_SUPPORT
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterEcgScreen);
  #elif defined(CONFIG_PPG_SUPPORT)
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterBPScreen);
  #elif defined(CONFIG_TEMP_SUPPORT)
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterTempScreen);
  #elif defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
   #ifdef CONFIG_SLEEP_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSleepScreen);
   #elif defined(CONFIG_STEP_SUPPORT)
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterStepsScreen);
   #endif
  #else
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterIdleScreen);
  #endif
#endif
}

void SyncUpdateStatus(void)
{
	uint32_t img_anima[3] = {IMG_ID_PROGRASS_ANI_1, IMG_ID_PROGRASS_ANI_2, IMG_ID_PROGRASS_ANI_3};
	
	switch(sync_state)
	{
	case SYNC_STATUS_IDLE:
		break;

	case SYNC_STATUS_LINKING:
	#ifdef CONFIG_ANIMATION_SUPPORT
		AnimaShow(SYNC_RUNNING_ANI_X, SYNC_RUNNING_ANI_Y, img_anima, ARRAY_SIZE(img_anima), 1000, true, NULL);
	#endif
		break;
		
	case SYNC_STATUS_SENT:
	#ifdef CONFIG_ANIMATION_SUPPORT 
		AnimaStop();
	#endif
	
		LCD_ShowImage(SYNC_ICON_X, SYNC_ICON_Y, IMG_ID_SYNC_SUCCESS);
		SetLeftKeyUpHandler(ExitSyncDataScreen);
		break;
		
	case SYNC_STATUS_FAIL:
	#ifdef CONFIG_ANIMATION_SUPPORT 
		AnimaStop();
	#endif
	
		LCD_ShowImage(SYNC_ICON_X, SYNC_ICON_Y, IMG_ID_SYNC_FAIL);
		SetLeftKeyUpHandler(ExitSyncDataScreen);
		break;
	}
}

void SyncShowStatus(void)
{
	LCD_Clear(BLACK);
	LCD_ShowImage(SYNC_ICON_X, SYNC_ICON_Y, IMG_ID_SYNV_READY);
	LCD_ShowImage(SYNC_RUNNING_ANI_X, SYNC_RUNNING_ANI_Y, IMG_ID_PROGRASS_ANI_0);
}

void SyncScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_SYNC].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_SYNC].status = SCREEN_STATUS_CREATED;
		SyncShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		SyncUpdateStatus();
		break;
	}

	scr_msg[SCREEN_ID_SYNC].act = SCREEN_ACTION_NO;
}
#endif/*CONFIG_SYNC_SUPPORT*/

#ifdef CONFIG_ECG_SUPPORT
void EcgUpdateStatus(void)
{
	uint16_t x,y,w,h;
	uint8_t tmpbuf[128] = {0};
	uint8_t strbuf[64] = {0};

	/*update ecg status*/	
}

// Show ECG lead status in bottom-left corner
void EcgShowLeadStatus(void)
{
	uint16_t x, y, w, h;
	uint8_t tmpbuf[128] = {0};
	const char *status_str;
	
	// Determine status string based on lead status
	switch(g_ecg_lead_status)
	{
		case ECG_LEAD_STATUS_OFF:
			status_str = "off";
			break;
		case ECG_LEAD_STATUS_ON:
			status_str = "on";
			break;
		case ECG_LEAD_STATUS_TIMEOUT:
			status_str = "timeout";
			break;
		default:
			status_str = "";
			break;
	}
	
	// Don't display if status is empty
	if(status_str[0] == '\0')
		return;
	
	// Set font size
#ifdef FONTMAKER_UNICODE_FONT
	LCD_SetFontSize(FONT_SIZE_20);
#else
	LCD_SetFontSize(FONT_SIZE_16);
#endif
	
	// Convert to UCS2
	mmi_asc_to_ucs2(tmpbuf, status_str);
	LCD_MeasureUniString(tmpbuf, &w, &h);
	
	// Position in bottom-left corner (adjust coordinates as needed)
	x = 10;  // Left margin
	y = LCD_HEIGHT - h - 5;  // Bottom margin
	
#ifdef LANGUAGE_AR_ENABLE
	if(g_language_r2l)
		LCD_ShowUniStringRtoL(x, y, tmpbuf);
	else
#endif
		LCD_ShowUniString(x, y, tmpbuf);
}

void EcgShowStatus(void)
{
	uint16_t i,x,y,w,h;
	uint8_t tmpbuf[128] = {0};

  #ifdef FONTMAKER_UNICODE_FONT
	LCD_SetFontSize(FONT_SIZE_28);
  #else	
	LCD_SetFontSize(FONT_SIZE_24);
  #endif

	mmi_asc_to_ucs2(tmpbuf, "");
	LCD_MeasureUniString(tmpbuf, &w, &h);
#ifdef LANGUAGE_AR_ENABLE	
	if(g_language_r2l)
		LCD_ShowUniStringRtoL(BP_NOTIFY_X+(BP_NOTIFY_W+w)/2, BP_NOTIFY_Y, tmpbuf);
	else
#endif		
		LCD_ShowUniString(BP_NOTIFY_X+(BP_NOTIFY_W-w)/2, BP_NOTIFY_Y, tmpbuf);

}

// ECG波形显示区域定义
#define ECG_WAVE_X_START    GRID_X_START        // 波形起始X与网格左边界对齐
#define ECG_WAVE_X_END      (LCD_WIDTH - 15)
#define ECG_WAVE_Y_START    75                  // 波形区域与网格顶部对??
#define ECG_WAVE_HEIGHT     285                 // 波形区域高度与网格一??
#define ECG_WAVE_COLOR      RED
#define GRID_SMALL_COLOR    GRAY
#define SMALL_GRID_SIZE     20                  // 小网格尺寸（格子内宽??
#define GRID_LINE_WIDTH     2                   // 网格线宽??

// 屏幕尺寸：宽390 x 高450
// 顶部预留15%空间：450 * 0.15 = 67.5 ≈ 68像素，取整后75像素
// 底部预留20%空间：450 * 0.2 = 90像素
// 网格可用高度：450 - 75 - 90 = 285像素
// Y方向：19个格子，每个格子13像素 + 20条线各2像素 = 247 + 40 = 287 ≈ 285
// X方向：24个格子，每个格子13像素 + 25条线各2像素 = 312 + 50 = 362 ≈ 360
#define GRID_CELLS_Y        13                  // Y方向13个格??
#define GRID_CELLS_X        16                  // X方向16个格??
#define GRID_PIXEL_HEIGHT   (GRID_CELLS_Y * SMALL_GRID_SIZE + (GRID_CELLS_Y + 1) * GRID_LINE_WIDTH)  // 287
#define GRID_PIXEL_WIDTH    (GRID_CELLS_X * SMALL_GRID_SIZE + (GRID_CELLS_X + 1) * GRID_LINE_WIDTH)  // 362
// 网格起始位置
#define GRID_Y_START        75                  // 顶部预留75像素
#define GRID_X_START        14                  // 左右边距14像素，使网格居中
// 网格结束位置
#define GRID_Y_END          (GRID_Y_START + GRID_PIXEL_HEIGHT - 1)  // 361
#define GRID_X_END          (GRID_X_START + GRID_PIXEL_WIDTH - 1)    // 375

// ECG数据环形缓冲区
#define ECG_LOCAL_BUFFER_SIZE  512
static int16_t ecg_local_buffer[ECG_LOCAL_BUFFER_SIZE];
static volatile uint16_t ecg_local_read_idx = 0;
static volatile uint16_t ecg_local_write_idx = 0;
static volatile bool ecg_local_buffer_full = false;

// 波形绘制状态
static uint16_t s_ecg_wave_x = ECG_WAVE_X_START;
static uint16_t s_ecg_prev_y = ECG_WAVE_Y_START + ECG_WAVE_HEIGHT / 2;
static uint16_t s_ecg_prev_x = ECG_WAVE_X_START;

// 用于记录上一轮波形每个位置的 y 值，以便新一轮清除旧波形
// 数组索引 = (x - GRID_X_START) / 2，0xFFFF表示该位置无波形
#define ECG_WAVE_HISTORY_SIZE  ((GRID_PIXEL_WIDTH / 2) + 1)  // 178
static uint16_t ecg_wave_y_history[ECG_WAVE_HISTORY_SIZE];         // 当前点的y值
static uint16_t ecg_wave_prev_x_history[ECG_WAVE_HISTORY_SIZE];    // 起点的x值，用于精确清除线段
static uint16_t ecg_wave_prev_y_history[ECG_WAVE_HISTORY_SIZE];    // 起点的y值，用于清除线段
#define ECG_WAVE_Y_INVALID  0xFFFF

// ECG倒计时显??
static uint8_t s_ecg_countdown_seconds = 60;
static bool s_ecg_countdown_first_show = true;
static void EcgCountdownTimerCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(ecg_countdown_timer, EcgCountdownTimerCallBack, NULL);
#define ECG_COUNTDOWN_Y     (GRID_Y_END + 10)   // 倒计时显示位置（网格下方??

// 网格列模板预计算
// 周期 = SMALL_GRID_SIZE + GRID_LINE_WIDTH = 13 + 2 = 15
#define ECG_GRID_STRIPE_PERIOD  (SMALL_GRID_SIZE + GRID_LINE_WIDTH)  // 15
// 每列模板高度多1行，防止LCD_DrawLine斜线在GRID_Y_END+1处残留
#define ECG_GRID_COL_HEIGHT     (GRID_PIXEL_HEIGHT + 1)              // 288
// 每个模板 1 列 * ECG_GRID_COL_HEIGHT * 2 字节
static uint8_t ecg_col_templates[ECG_GRID_STRIPE_PERIOD][ECG_GRID_COL_HEIGHT * 2];
static bool ecg_col_templates_ready = false;

// 在 EcgDisplayInit 时调用，预计算所有 15 种列模板
static void EcgPrecomputeColTemplates(void)
{
    const uint16_t GRAY_H  = GRID_SMALL_COLOR >> 8;
    const uint16_t GRAY_L  = GRID_SMALL_COLOR & 0xFF;
    const uint16_t BLACK_H = BLACK >> 8;
    const uint16_t BLACK_L = BLACK & 0xFF;

    for (uint16_t xp = 0; xp < ECG_GRID_STRIPE_PERIOD; xp++) {
        bool is_vcol = (xp < GRID_LINE_WIDTH);  // 该列是否为竖线
        uint8_t *tmpl = ecg_col_templates[xp];
        uint32_t idx = 0;

        for (uint16_t y = 0; y < ECG_GRID_COL_HEIGHT; y++) {
            uint16_t color_h, color_l;
            if (y < GRID_PIXEL_HEIGHT) {
                // 在网格有效高度内
                uint16_t yp = y % ECG_GRID_STRIPE_PERIOD;
                bool is_hrow = (yp < GRID_LINE_WIDTH);  // 该行是否为横线
                if (is_vcol || is_hrow) {
                    color_h = GRAY_H;
                    color_l = GRAY_L;
                } else {
                    color_h = BLACK_H;
                    color_l = BLACK_L;
                }
            } else {
                // 超出网格底部的1行：黑色（覆盖斜线残留）
                color_h = BLACK_H;
                color_l = BLACK_L;
            }
            tmpl[idx++] = color_h;
            tmpl[idx++] = color_l;
        }
    }
    ecg_col_templates_ready = true;
}

// 快速恢复指定列的网格（清除旧波形，恢复网格背景）
// 固定恢复宽度为 2 列，直接拼接两个预计算模板，一次 SPI 刷新
#define ECG_RESTORE_COL_WIDTH  2
static void RestoreGridColumns(uint16_t x, uint16_t width)
{
    (void)width;  // 固定宽度 2，忽略参数

    if (!ecg_col_templates_ready) {
        return;
    }
    if (x + ECG_RESTORE_COL_WIDTH > ECG_WAVE_X_END + 1) {
        return;
    }

    // 拼合两列模板到静态发送缓冲区（交错写入：每行先col0再col1）
    static uint8_t data_buf[ECG_RESTORE_COL_WIDTH * ECG_GRID_COL_HEIGHT * 2];

    uint16_t xp0 = (x     - GRID_X_START) % ECG_GRID_STRIPE_PERIOD;
    uint16_t xp1 = (x + 1 - GRID_X_START) % ECG_GRID_STRIPE_PERIOD;
    const uint8_t *t0 = ecg_col_templates[xp0];
    const uint8_t *t1 = ecg_col_templates[xp1];

    // LCD 行优先扫描：每行依次为 col0、col1
    uint32_t src = 0, dst = 0;
    for (uint16_t y = 0; y < ECG_GRID_COL_HEIGHT; y++) {
        data_buf[dst++] = t0[src];
        data_buf[dst++] = t0[src + 1];
        data_buf[dst++] = t1[src];
        data_buf[dst++] = t1[src + 1];
        src += 2;
    }

    BlockWrite(x, GRID_Y_START, ECG_RESTORE_COL_WIDTH, ECG_GRID_COL_HEIGHT);
    DispData(ECG_RESTORE_COL_WIDTH * ECG_GRID_COL_HEIGHT * 2, data_buf);
}

// 恢复单个波形点的网格背景（2x2像素）
// 对每个像素分别判断是否在网格线上，恢复为相应颜色
static void RestoreGridPoint(uint16_t x, uint16_t y)
{
    // 边界检查
    if (x < GRID_X_START || x + 1 > GRID_X_END || y < GRID_Y_START || y + 1 > GRID_Y_END) {
        return;
    }
    
    // 准备4个像素的颜色数据（RGB565，每个像素2字节）
    uint8_t data_buf[8];  // 2x2 * 2字节 = 8字节
    uint16_t idx = 0;
    
    // 按行优先顺序填充4个像素的颜色
    for (uint16_t dy = 0; dy < 2; dy++) {
        for (uint16_t dx = 0; dx < 2; dx++) {
            uint16_t px = x + dx;
            uint16_t py = y + dy;
            
            // 计算该像素在网格周期中的位置
            uint16_t xp = (px - GRID_X_START) % ECG_GRID_STRIPE_PERIOD;
            uint16_t yp = (py - GRID_Y_START) % ECG_GRID_STRIPE_PERIOD;
            
            // 判断是否在网格线上
            bool is_vcol = (xp < GRID_LINE_WIDTH);  // 是否在竖线上
            bool is_hrow = (yp < GRID_LINE_WIDTH);  // 是否在横线上
            
            uint16_t color;
            if (is_vcol || is_hrow) {
                color = GRID_SMALL_COLOR;  // 在网格线上
            } else {
                color = BLACK;  // 不在网格线上
            }
            
            // RGB565 高字节在前
            data_buf[idx++] = color >> 8;
            data_buf[idx++] = color & 0xFF;
        }
    }
    
    // 一次性写入2x2区域
    BlockWrite(x, y, 2, 2);
    DispData(8, data_buf);
}

// 恢复一条线段上所有点的网格背景（用于清除上一条波形线）
// 使用与 LCD_DrawLine 相同的 Bresenham 算法遍历所有点
static void RestoreGridLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    
    delta_x = x2 - x1;
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    
    if (delta_x > 0) incx = 1;
    else if (delta_x == 0) incx = 0;
    else { incx = -1; delta_x = -delta_x; }
    
    if (delta_y > 0) incy = 1;
    else if (delta_y == 0) incy = 0;
    else { incy = -1; delta_y = -delta_y; }
    
    if (delta_x > delta_y) distance = delta_x;
    else distance = delta_y;
    
    // 遍历线段上的所有点并恢复网格
    for (t = 0; t <= distance + 1; t++) {
        // 确保每个点都画 2x2 区域，需要每隔 2 像素调用一次
        // 但要保证覆盖整个线段，所以每个点都要处理
        RestoreGridPoint(uRow, uCol);
        
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
    
    // 额外处理终点，防止 Bresenham 算法遗漏
    // 因为循环条件是 t <= distance+1，可能会多走一步
    if (uRow != x2 || uCol != y2) {
        RestoreGridPoint(x2, y2);
    }
}

// 动态显示 ECG 波形（不清屏，只清除当前位置的旧波形）
static void DisplayDynamicECGPoint(int16_t value)
{
    // Y 轴映射 - 使用宏定义的网格区域

    // Y 轴范围
    static int16_t min_value = -5000, max_value = 5000;
    if (value < min_value) value = min_value;
    if (value > max_value) value = max_value;

    int range = max_value - min_value;
    if (range == 0) range = 1;

    uint16_t y = GRID_Y_START + GRID_PIXEL_HEIGHT - 
                 (uint16_t)(((int32_t)(value - min_value) * GRID_PIXEL_HEIGHT) / range);
    
    // 限制 Y 坐标在显示区间内
    if (y < GRID_Y_START) y = GRID_Y_START;
    if (y > GRID_Y_END) y = GRID_Y_END;

    /* 加锁：保证整个"清除旧点+画新点"是原子操作，不被主线程插入 */
    LCD_Lock();
    
    // 若到达右边界 → 重新从左边开始（不清屏）
    if (s_ecg_wave_x > GRID_X_END) {
        s_ecg_wave_x = GRID_X_START;
        s_ecg_prev_x = s_ecg_wave_x;
        s_ecg_prev_y = (GRID_Y_START + GRID_Y_END) / 2;
    }

    // 计算当前位置的索引
    uint16_t idx = (s_ecg_wave_x - GRID_X_START) / 2;
    
    // 检查该位置是否有上一轮的旧波形，如果有则清除
    if (idx < ECG_WAVE_HISTORY_SIZE && 
        ecg_wave_y_history[idx] != ECG_WAVE_Y_INVALID &&
        ecg_wave_prev_y_history[idx] != ECG_WAVE_Y_INVALID) {
        // 清除旧的波形线段（使用记录的精确端点）
        RestoreGridLine(ecg_wave_prev_x_history[idx], ecg_wave_prev_y_history[idx], 
                        s_ecg_wave_x, ecg_wave_y_history[idx]);
    }

    // 设置画笔颜色为红色并画新线段
    POINT_COLOR = ECG_WAVE_COLOR;
    LCD_DrawLine(s_ecg_prev_x, s_ecg_prev_y, s_ecg_wave_x, y);

    LCD_Unlock();

    // 保存当前位置的y值到历史数组（供下一轮清除使用）
    if (idx < ECG_WAVE_HISTORY_SIZE) {
        ecg_wave_prev_x_history[idx] = s_ecg_prev_x;  // 记录起点x坐标
        ecg_wave_prev_y_history[idx] = s_ecg_prev_y;  // 记录起点y坐标
        ecg_wave_y_history[idx] = y;                   // 记录终点y坐标
    }

    // 更新位置
    s_ecg_prev_x = s_ecg_wave_x;
    s_ecg_prev_y = y;
    s_ecg_wave_x += 2;
}

// 新的ECG数据显示函数 - 接收256字节数据并绘制波形
void EcgDisplayProcessData(const uint8_t *data, uint16_t length)
{
    // 不在ECG界面时不绘制
    if (screen_id != SCREEN_ID_ECG) {
        return;
    }

    // 验证输入参数
    if (data == NULL || length != 128) {
        LOGD("Invalid ECG data: data=%p, length=%d", data, length);
        return;
    }
    
    // 256字节 = 128个int16_t，每两字节一个样本，低字节在前（Little-Endian）
    // 手动解析避免对齐问题
    for (int i = 0; i < 64; i++) {
        int16_t value = (int16_t)((uint16_t)data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8));
        
        // Y 轴映射 - 使用宏定义的网格区域
        static int16_t min_value = -6000, max_value = 6000;
        if (value < min_value) value = min_value;
        if (value > max_value) value = max_value;

        int range = max_value - min_value;
        if (range == 0) range = 1;

        uint16_t y = GRID_Y_START + GRID_PIXEL_HEIGHT - 
                     (uint16_t)(((int32_t)(value - min_value) * GRID_PIXEL_HEIGHT) / range);
        
        // 限制 Y 坴标在显示区间内
        if (y < GRID_Y_START) y = GRID_Y_START;
        if (y > GRID_Y_END) y = GRID_Y_END;

        /* 加锁：保证整个"清除旧点+画新点"是原子操作 */
        //LCD_Lock();
        
        // 若到达右边界 → 重新从左边开始
        if (s_ecg_wave_x > GRID_X_END) {
            s_ecg_wave_x = GRID_X_START;
            s_ecg_prev_x = s_ecg_wave_x;
            s_ecg_prev_y = (GRID_Y_START + GRID_Y_END) / 2;
        }

        // 计算当前位置的索引
        uint16_t idx = (s_ecg_wave_x - GRID_X_START) / 2;
        
        // 检查该位置是否有上一轮的旧波形，如果有则清除
        if (idx < ECG_WAVE_HISTORY_SIZE && 
            ecg_wave_y_history[idx] != ECG_WAVE_Y_INVALID &&
            ecg_wave_prev_y_history[idx] != ECG_WAVE_Y_INVALID) {
            // 清除旧的波形线段（使用记录的精确端点）
            RestoreGridLine(ecg_wave_prev_x_history[idx], ecg_wave_prev_y_history[idx], 
                            s_ecg_wave_x, ecg_wave_y_history[idx]);
        }

        // 设置画笔颜色为红色并画新线段
        POINT_COLOR = ECG_WAVE_COLOR;
        LCD_DrawLine(s_ecg_prev_x, s_ecg_prev_y, s_ecg_wave_x, y);

        //LCD_Unlock();

        // 保存当前位置的y值到历史数组（供下一轮清除使用）
        if (idx < ECG_WAVE_HISTORY_SIZE) {
            ecg_wave_prev_x_history[idx] = s_ecg_prev_x;  // 记录起点x坐标
            ecg_wave_prev_y_history[idx] = s_ecg_prev_y;  // 记录起点y坐标
            ecg_wave_y_history[idx] = y;                   // 记录终点y坐标
        }

        // 更新位置
        s_ecg_prev_x = s_ecg_wave_x;
        s_ecg_prev_y = y;
        s_ecg_wave_x += 2;
    }
}

// ECG倒计时显示函?? - 只更新数字部??
static void EcgShowCountdown(void)
{
    uint8_t strbuf[32];
    uint8_t prefix[] = "Testing Wait ";
    uint16_t prefix_w, prefix_h;
    uint16_t num_w, num_h;
    uint16_t total_w, h;
    uint16_t start_x;
    
    // 测量前缀宽度
    LCD_MeasureString(prefix, &prefix_w, &prefix_h);
    
    if (s_ecg_countdown_first_show) {
        // 第一次显示完整文??
        LCD_Fill(0, ECG_COUNTDOWN_Y, LCD_WIDTH, 30, BLACK);
        if (s_ecg_countdown_seconds > 0) {
            sprintf(strbuf, "Testing Wait %ds", s_ecg_countdown_seconds);
        } else {
            sprintf(strbuf, "Testing Complete");
        }
        LCD_MeasureString(strbuf, &total_w, &h);
        POINT_COLOR = WHITE;
        LCD_ShowString((LCD_WIDTH - total_w) / 2, ECG_COUNTDOWN_Y, strbuf);
        s_ecg_countdown_first_show = false;
    } else {
        // 后续只更新数字部??
        if (s_ecg_countdown_seconds > 0) {
            // 计算数字区域的X坐标（居中）
            sprintf(strbuf, "%d", s_ecg_countdown_seconds);
            LCD_MeasureString(strbuf, &num_w, &num_h);
            total_w = prefix_w + num_w + 8; // 8??"s"的宽度估??
            start_x = (LCD_WIDTH - total_w) / 2 + prefix_w;
            
            // 清除固定宽度的数字区域（足以覆盖"60s"的最大宽度）
            // 测量"60s"的宽度作为最大清除区??
            uint16_t max_num_w, max_h;
            LCD_MeasureString("60s", &max_num_w, &max_h);
            LCD_Fill(start_x - 2, ECG_COUNTDOWN_Y, max_num_w + 4, prefix_h, BLACK);
            
            // 只显示数字和"s"
            POINT_COLOR = WHITE;
            sprintf(strbuf, "%ds", s_ecg_countdown_seconds);
            LCD_ShowString(start_x, ECG_COUNTDOWN_Y, strbuf);
        } else {
            // 倒计时结束，显示完成文本
            LCD_Fill(0, ECG_COUNTDOWN_Y, LCD_WIDTH, 30, BLACK);
            sprintf(strbuf, "Testing Complete");
            LCD_MeasureString(strbuf, &total_w, &h);
            POINT_COLOR = WHITE;
            LCD_ShowString((LCD_WIDTH - total_w) / 2, ECG_COUNTDOWN_Y, strbuf);
        }
    }
}

// 初始化ECG显示
void EcgDisplayInit(void)
{
    // 预计算网格列模板（仅在模板未就绪时执行，避免重复计算??
    if (!ecg_col_templates_ready) {
        EcgPrecomputeColTemplates();
    }

    // 绘制网格背景
    EcgDrawGrid();
    
    // 初始化波形绘制位置
    s_ecg_wave_x = GRID_X_START;
    s_ecg_prev_x = s_ecg_wave_x;
    s_ecg_prev_y = (GRID_Y_START + GRID_Y_END) / 2;
    
    // 初始化波形历史数组（全部设为无效）
    for (uint16_t i = 0; i < ECG_WAVE_HISTORY_SIZE; i++) {
        ecg_wave_y_history[i] = ECG_WAVE_Y_INVALID;
        ecg_wave_prev_x_history[i] = ECG_WAVE_Y_INVALID;
        ecg_wave_prev_y_history[i] = ECG_WAVE_Y_INVALID;
    }
    
    // 初始化并显示倒计??
    s_ecg_countdown_seconds = 35;
    s_ecg_countdown_first_show = true;
    EcgShowCountdown();
    
    // 启动倒计时定时器（每秒更新一次）
    k_timer_start(&ecg_countdown_timer, K_SECONDS(1), K_SECONDS(1));
    
    //LOGD("ECG display initialized");
}

void EcgDisplayDeinit(void)
{
    // 恢复默认颜色，避免影响其他界面
    POINT_COLOR = WHITE;
    BACK_COLOR = BLACK;
    
    //LOGD("ECG display deinitialized");
}

void EcgDrawGrid(void)
{
    // 1. 先填充整个网格区域为黑色背景
    //LCD_FillColor(GRID_X_START, GRID_Y_START, GRID_PIXEL_WIDTH, GRID_PIXEL_HEIGHT, BLACK);
    
    // 2. 画水平网格线
    for (uint8_t row = 0; row <= GRID_CELLS_Y; row++) {
        uint16_t y = GRID_Y_START + row * (SMALL_GRID_SIZE + GRID_LINE_WIDTH);
        LCD_FillColor(GRID_X_START, y, GRID_PIXEL_WIDTH, GRID_LINE_WIDTH, GRID_SMALL_COLOR);
    }
    
    // 3. 画垂直网格线
    for (uint8_t col = 0; col <= GRID_CELLS_X; col++) {
        uint16_t x = GRID_X_START + col * (SMALL_GRID_SIZE + GRID_LINE_WIDTH);
        LCD_FillColor(x, GRID_Y_START, GRID_LINE_WIDTH, GRID_PIXEL_HEIGHT, GRID_SMALL_COLOR);
    }
}

void EcgScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_ECG].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_ECG].status = SCREEN_STATUS_CREATED;

		LCD_Clear(BLACK);
		IdleShowSignal();
		IdleShowNetMode();
		IdleShowBatSoc();
		EcgShowStatus();
		// 初始化ECG显示
		EcgDisplayInit();
		// Reset lead status to unknown when entering ECG screen
		g_ecg_lead_status = ECG_LEAD_STATUS_UNKNOWN;
		break;
		
	case SCREEN_ACTION_UPDATE:
		if(scr_msg[SCREEN_ID_ECG].para&SCREEN_EVENT_UPDATE_SIG)
		{
			scr_msg[SCREEN_ID_ECG].para &= (~SCREEN_EVENT_UPDATE_SIG);
			IdleShowSignal();
		}
		if(scr_msg[SCREEN_ID_ECG].para&SCREEN_EVENT_UPDATE_NET_MODE)
		{
			scr_msg[SCREEN_ID_ECG].para &= (~SCREEN_EVENT_UPDATE_NET_MODE);	
			IdleShowNetMode();
		}
		if(scr_msg[SCREEN_ID_ECG].para&SCREEN_EVENT_UPDATE_BAT)
		{
			scr_msg[SCREEN_ID_ECG].para &= (~SCREEN_EVENT_UPDATE_BAT);
			IdleUpdateBatSoc();
		}
		if(scr_msg[SCREEN_ID_ECG].para&SCREEN_EVENT_UPDATE_BP)
		{
			scr_msg[SCREEN_ID_ECG].para &= (~SCREEN_EVENT_UPDATE_BP);
			EcgUpdateStatus();
		}
		if(scr_msg[SCREEN_ID_ECG].para&SCREEN_EVENT_UPDATE_ECG_TIMER)
		{
			scr_msg[SCREEN_ID_ECG].para &= (~SCREEN_EVENT_UPDATE_ECG_TIMER);
			EcgShowCountdown();
		}
		if(scr_msg[SCREEN_ID_ECG].para&SCREEN_EVENT_UPDATE_ECG_LEAD)
		{
			scr_msg[SCREEN_ID_ECG].para &= (~SCREEN_EVENT_UPDATE_ECG_LEAD);
			EcgShowLeadStatus();
		}
		break;
	}
	
	scr_msg[SCREEN_ID_ECG].act = SCREEN_ACTION_NO;
}

// ECG倒计时定时器回调 - 只减少计数，不操作LCD
static void EcgCountdownTimerCallBack(struct k_timer *timer_id)
{
    if (s_ecg_countdown_seconds > 0) {
        s_ecg_countdown_seconds--;
        // 只在ECG屏幕可见时发送更新事??
        if (screen_id == SCREEN_ID_ECG) {
            scr_msg[SCREEN_ID_ECG].act = SCREEN_ACTION_UPDATE;
            scr_msg[SCREEN_ID_ECG].para |= SCREEN_EVENT_UPDATE_ECG_TIMER;  
        }
    }
    
    // 如果倒计时结束，停止定时??
    if (s_ecg_countdown_seconds == 0) {
        k_timer_stop(&ecg_countdown_timer);
    }
}

void ExitEcgScreen(void)
{
	LOGD("Exit ECG screen");

	// 停止倒计时定时器
	k_timer_stop(&ecg_countdown_timer);

	// 停止ECG采集
	MenuStopECG();

	// 清理ECG显示（停止画图）
	EcgDisplayDeinit();

	// 重置波形绘制状态
	s_ecg_wave_x = ECG_WAVE_X_START;
	s_ecg_prev_x = ECG_WAVE_X_START;
	s_ecg_prev_y = ECG_WAVE_Y_START + (uint16_t)(ECG_WAVE_HEIGHT * 0.5);
	
	// 重置波形历史数组
	for (uint16_t i = 0; i < ECG_WAVE_HISTORY_SIZE; i++) {
		ecg_wave_y_history[i] = ECG_WAVE_Y_INVALID;
		ecg_wave_prev_x_history[i] = ECG_WAVE_Y_INVALID;
		ecg_wave_prev_y_history[i] = ECG_WAVE_Y_INVALID;
	}

	ecg_local_read_idx = 0;
	ecg_local_write_idx = 0;

	LCD_Set_BL_Mode(LCD_BL_AUTO);

	// 注意：不要在这里调用 EnterIdleScreen()，
	// 因为 EnterIdleScreen 内部会再次调?? ExitEcgScreen 导致递归栈溢??
	// 界面跳转应该由调用方（按键处理函数）负责
}

static void ExitEcgScreenAndGoIdle(void)
{
	ExitEcgScreen();
	EnterIdleScreen();
}

void EnterEcgScreen(void)
{
	if(screen_id == SCREEN_ID_ECG)
		return;

	k_timer_stop(&mainmenu_timer);

#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(TempIsWorking()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();
#endif

	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_ECG;	
	scr_msg[SCREEN_ID_ECG].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_ECG].status = SCREEN_STATUS_CREATING;

#ifdef CONFIG_SYNC_SUPPORT
	SetLeftKeyUpHandler(EnterSyncDataScreen);
#else
	SetLeftKeyUpHandler(EnterSettings);
#endif
	SetRightKeyUpHandler(ExitEcgScreenAndGoIdle);

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();

#ifdef CONFIG_SYNC_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSyncDataScreen);
#else
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSettings); 
#endif
 
  #ifdef CONFIG_PPG_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterBPScreen);
  #elif defined(CONFIG_TEMP_SUPPORT)
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterTempScreen);
  #elif defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
   #ifdef CONFIG_SLEEP_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSleepScreen);
   #elif defined(CONFIG_STEP_SUPPORT)
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterStepsScreen);
   #endif
  #else
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterIdleScreen);
  #endif
#endif
	// 直接启动ECG测量
	MenuStartECG();
}

#endif

#ifdef CONFIG_TEMP_SUPPORT
static uint8_t img_flag = 0;
static uint8_t temp_retry_left = 2;
static uint8_t tempdata[TEMP_REC2_MAX_DAILY*sizeof(temp_rec2_nod)] = {0};

void TempShowNumByImg(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t num_img_w, uint32_t *img_addr, uint32_t separa_img_addr, float data)
{
	uint8_t i,count=1;
	uint16_t x1,temp_body;
	uint32_t divisor=10;
	uint16_t img_w,img_h;
	
	if(global_settings.temp_unit == TEMP_UINT_C)
	{
		temp_body = round(data*10.0);
	}
	else
	{
		temp_body = round((32+1.8*data)*10.0);
	}

	while(1)
	{
		if(temp_body/divisor > 0)
		{
			count++;
			divisor = divisor*10;
		}
		else
		{
			divisor = divisor/10;
			break;
		}
	}

	LCD_Fill(x, y, w, h, BLACK);
	LCD_MeasureImage(separa_img_addr, &img_w, &img_h);
	x1 = x;
	if(count == 1)
	{
		LCD_ShowImage(x1, y, img_addr[temp_body/10]);
		x1 += num_img_w;
		LCD_ShowImage(x1, y, separa_img_addr);
		x1 += img_w;
		LCD_ShowImage(x1, y, img_addr[temp_body%10]);
	}
	else
	{
		for(i=0;i<(count+1);i++)
		{
			if(i == count-1)
			{
				LCD_ShowImage(x1, y, separa_img_addr);
				x1 += img_w;
			}
			else
			{
				LCD_ShowImage(x1, y, img_addr[temp_body/divisor]);
				temp_body = temp_body%divisor;
				divisor = divisor/10;
				x1 += num_img_w;
			}
		}
	}
}

static void TempStatusTimerOutCallBack(struct k_timer *timer_id)
{
	if(screen_id == SCREEN_ID_TEMP)
	{
		switch(g_temp_status)
		{
		case TEMP_STATUS_PREPARE:
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_TEMP;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			break;
			
		case TEMP_STATUS_MEASURING:
			if(get_temp_ok_flag)
			{
				g_temp_status = TEMP_STATUS_MEASURE_OK;
			}
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_TEMP;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			break;
			
		case TEMP_STATUS_MEASURE_FAIL:
			if(temp_retry_left > 0)
			{
				g_temp_status = TEMP_STATUS_NOTIFY;
				scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_TEMP;
				scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			}
			else
			{
				g_temp_status = TEMP_STATUS_MAX;
				EntryIdleScr();
			}
			break;

		case TEMP_STATUS_MEASURE_OK:
			g_temp_status = TEMP_STATUS_MAX;
			EntryIdleScr();
			break;

		case TEMP_STATUS_NOTIFY:
			g_temp_status = TEMP_STATUS_PREPARE;
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_TEMP;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			break;
		}
	}
}

void TempUpdateStatus(void)
{
	uint8_t i,count=1;
	uint16_t x,y,w,h;
	uint16_t temp_body;
	uint16_t str_id;
	uint32_t divisor=10;
	uint32_t img_anima[3] = {IMG_ID_TEMP_ANI_1,IMG_ID_TEMP_ANI_2,IMG_ID_TEMP_ANI_3};
	uint32_t img_num[10] = {IMG_ID_FONT_70_NUM_0,IMG_ID_FONT_70_NUM_1,IMG_ID_FONT_70_NUM_2,IMG_ID_FONT_70_NUM_3,IMG_ID_FONT_70_NUM_4,
							IMG_ID_FONT_70_NUM_5,IMG_ID_FONT_70_NUM_6,IMG_ID_FONT_70_NUM_7,IMG_ID_FONT_70_NUM_8,IMG_ID_FONT_70_NUM_9};
	uint32_t img_small_num[10] = {IMG_ID_FONT_45_NUM_0,IMG_ID_FONT_45_NUM_1,IMG_ID_FONT_45_NUM_2,IMG_ID_FONT_45_NUM_3,IMG_ID_FONT_45_NUM_4,
								  IMG_ID_FONT_45_NUM_5,IMG_ID_FONT_45_NUM_6,IMG_ID_FONT_45_NUM_7,IMG_ID_FONT_45_NUM_8,IMG_ID_FONT_45_NUM_9};

	switch(g_temp_status)
	{
	case TEMP_STATUS_PREPARE:
		LCD_Fill(TEMP_NOTIFY_X, TEMP_NOTIFY_Y, TEMP_NOTIFY_W, TEMP_NOTIFY_H, BLACK);
		
		LCD_SetFontSize(FONT_SIZE_36);

		LCD_MeasureUniStr(STR_ID_STAY_STILL, &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStrRtoL(TEMP_NOTIFY_X+(TEMP_NOTIFY_W+w)/2, TEMP_NOTIFY_Y, STR_ID_STAY_STILL);
		else
	#endif		
			LCD_ShowUniStr(TEMP_NOTIFY_X+(TEMP_NOTIFY_W-w)/2, TEMP_NOTIFY_Y, STR_ID_STAY_STILL);
		
		MenuStartTemp();
		g_temp_status = TEMP_STATUS_MEASURING;
		break;
		
	case TEMP_STATUS_MEASURING:
		img_flag++;
		if(img_flag >= 3)
			img_flag = 0;
		LCD_ShowImage(TEMP_ICON_X, TEMP_ICON_Y, img_anima[img_flag]);

		LCD_SetFontSize(FONT_SIZE_36);

		if(get_temp_ok_flag)
		{
			LCD_Fill(TEMP_NOTIFY_X, TEMP_NOTIFY_Y, TEMP_NOTIFY_W, TEMP_NOTIFY_H, BLACK);
			LCD_ShowImage(TEMP_ICON_X, TEMP_ICON_Y, IMG_ID_TEMP_ANI_3);

			if(global_settings.temp_unit == TEMP_UINT_C)
			{
				temp_body = round(g_temp_body*10.0);
			}
			else
			{
				temp_body = round((32+1.8*g_temp_body)*10.0);
			}
		
			while(1)
			{
				if(temp_body/divisor > 0)
				{
					count++;
					divisor = divisor*10;
				}
				else
				{
					divisor = divisor/10;
					break;
				}
			}
		
			x = TEMP_STR_X+(TEMP_STR_W-count*TEMP_NUM_W-TEMP_DOT_W)/2;
			y = TEMP_STR_Y;
		
			if(count == 1)
			{
				x = TEMP_STR_X+(TEMP_STR_W-2*TEMP_NUM_W-TEMP_DOT_W)/2;
				LCD_ShowImage(x, y, img_num[temp_body/10]);
				x += TEMP_NUM_W;
				LCD_ShowImage(x, y, IMG_ID_FONT_70_DOT);
				x += TEMP_DOT_W;
				LCD_ShowImage(x, y, img_num[temp_body%10]);
				x += TEMP_NUM_W;
			}
			else
			{
				for(i=0;i<(count+1);i++)
				{
					if(i == count-1)
					{
						LCD_ShowImage(x, y, IMG_ID_FONT_70_DOT);
						x += TEMP_DOT_W;
					}
					else
					{
						LCD_ShowImage(x, y, img_num[temp_body/divisor]);
						temp_body = temp_body%divisor;
						divisor = divisor/10;
						x += TEMP_NUM_W;
					}
				}
			}
			
			if(global_settings.temp_unit == TEMP_UINT_C)
				LCD_ShowImage(x, TEMP_UNIT_Y, IMG_ID_TEMP_UNIT_C);
			else
				LCD_ShowImage(x, TEMP_UNIT_Y, IMG_ID_TEMP_UNIT_F);
		
			MenuStopTemp();
			k_timer_start(&temp_status_timer, K_SECONDS(5), K_NO_WAIT);
		}
		break;
		
	case TEMP_STATUS_MEASURE_OK:
		LCD_ShowImage(TEMP_ICON_X, TEMP_ICON_Y, IMG_ID_TEMP_ANI_3);
		TempShowNumByImg(TEMP_UP_STR_X, TEMP_UP_STR_Y, TEMP_UP_STR_W, TEMP_UP_STR_H, TEMP_UP_NUM_W, img_small_num, IMG_ID_FONT_45_DOT, (float)last_health.deca_temp_max/10.0);
		TempShowNumByImg(TEMP_DOWN_STR_X, TEMP_DOWN_STR_Y, TEMP_DOWN_STR_W, TEMP_DOWN_STR_H, TEMP_DOWN_NUM_W, img_small_num, IMG_ID_FONT_45_DOT, (float)last_health.deca_temp_min/10.0);
		k_timer_start(&temp_status_timer, K_SECONDS(2), K_NO_WAIT);
		break;
		
	case TEMP_STATUS_MEASURE_FAIL:
		MenuStopTemp();

		LCD_Fill(TEMP_NOTIFY_X, TEMP_NOTIFY_Y, TEMP_NOTIFY_W, TEMP_NOTIFY_H, BLACK);
		LCD_ShowImage(TEMP_ICON_X, TEMP_ICON_Y, IMG_ID_TEMP_ANI_3);
		
		LCD_SetFontSize(FONT_SIZE_36);

		temp_retry_left--;
		if(temp_retry_left == 0)
		{
			str_id = STR_ID_INCONCLUSIVE_AND_RETRY_LATER;
		}
		else
		{
			str_id = STR_ID_INCONCLUSIVE;
		}
		
		LCD_MeasureUniStr(str_id, &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStrRtoL(TEMP_NOTIFY_X+(TEMP_NOTIFY_W+w)/2, TEMP_NOTIFY_Y, str_id);		
		else
	#endif		
			LCD_ShowUniStr(TEMP_NOTIFY_X+(TEMP_NOTIFY_W-w)/2, TEMP_NOTIFY_Y, str_id);		

		k_timer_start(&temp_status_timer, K_SECONDS(5), K_NO_WAIT);
		break;
		
	case TEMP_STATUS_NOTIFY:
		LCD_Fill(TEMP_NOTIFY_X, TEMP_NOTIFY_Y, TEMP_NOTIFY_W, TEMP_NOTIFY_H, BLACK);
		LCD_ShowImage(TEMP_ICON_X, TEMP_ICON_Y, IMG_ID_TEMP_ANI_3);

		LCD_SetFontSize(FONT_SIZE_36);

		LCD_MeasureUniStr(STR_ID_STAY_STILL_AND_RETRY, &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStrRtoL(TEMP_NOTIFY_X+(TEMP_NOTIFY_W+w)/2, TEMP_NOTIFY_Y, STR_ID_STAY_STILL_AND_RETRY);
		else
	#endif		
			LCD_ShowUniStr(TEMP_NOTIFY_X+(TEMP_NOTIFY_W-w)/2, TEMP_NOTIFY_Y, STR_ID_STAY_STILL_AND_RETRY);

		k_timer_start(&temp_status_timer, K_SECONDS(5), K_NO_WAIT);
		break;
	}
}

void TempShowStatus(void)
{
	uint16_t w,h;
	uint32_t img_num[10] = {IMG_ID_FONT_45_NUM_0,IMG_ID_FONT_45_NUM_1,IMG_ID_FONT_45_NUM_2,IMG_ID_FONT_45_NUM_3,IMG_ID_FONT_45_NUM_4,
							IMG_ID_FONT_45_NUM_5,IMG_ID_FONT_45_NUM_6,IMG_ID_FONT_45_NUM_7,IMG_ID_FONT_45_NUM_8,IMG_ID_FONT_45_NUM_9};

	LCD_ShowImage(TEMP_ICON_X, TEMP_ICON_Y, IMG_ID_TEMP_ANI_3);
	LCD_ShowImage(TEMP_UP_ARRAW_X, TEMP_UP_ARRAW_Y, IMG_ID_TEMP_MAX_REC);
	LCD_ShowImage(TEMP_DOWN_ARRAW_X, TEMP_DOWN_ARRAW_Y, IMG_ID_TEMP_MIN_REC);

	LCD_SetFontSize(FONT_SIZE_36);

	LCD_MeasureUniStr(STR_ID_BODY_TEMP, &w, &h);
#ifdef LANGUAGE_AR_ENABLE	
	if(g_language_r2l)
		LCD_ShowUniStrRtoL(TEMP_NOTIFY_X+(TEMP_NOTIFY_W+w)/2, TEMP_NOTIFY_Y, STR_ID_BODY_TEMP);
	else
#endif		
		LCD_ShowUniStr(TEMP_NOTIFY_X+(TEMP_NOTIFY_W-w)/2, TEMP_NOTIFY_Y, STR_ID_BODY_TEMP);

	TempShowNumByImg(TEMP_UP_STR_X, TEMP_UP_STR_Y, TEMP_UP_STR_W, TEMP_UP_STR_H, TEMP_UP_NUM_W, img_num, IMG_ID_FONT_45_DOT, (float)last_health.deca_temp_max/10.0);
	TempShowNumByImg(TEMP_DOWN_STR_X, TEMP_DOWN_STR_Y, TEMP_DOWN_STR_W, TEMP_DOWN_STR_H, TEMP_DOWN_NUM_W, img_num, IMG_ID_FONT_45_DOT, (float)last_health.deca_temp_min/10.0);

	k_timer_start(&temp_status_timer, K_SECONDS(2), K_NO_WAIT);
}

void TempScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_TEMP].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_TEMP].status = SCREEN_STATUS_CREATED;

		LCD_Clear(BLACK);
		IdleShowSignal();
		IdleShowNetMode();
		IdleShowBatSoc();
		TempShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		if(scr_msg[SCREEN_ID_TEMP].para&SCREEN_EVENT_UPDATE_SIG)
		{
			scr_msg[SCREEN_ID_TEMP].para &= (~SCREEN_EVENT_UPDATE_SIG);
			IdleShowSignal();
		}
		if(scr_msg[SCREEN_ID_TEMP].para&SCREEN_EVENT_UPDATE_NET_MODE)
		{
			scr_msg[SCREEN_ID_TEMP].para &= (~SCREEN_EVENT_UPDATE_NET_MODE);	
			IdleShowNetMode();
		}
		if(scr_msg[SCREEN_ID_TEMP].para&SCREEN_EVENT_UPDATE_BAT)
		{
			scr_msg[SCREEN_ID_TEMP].para &= (~SCREEN_EVENT_UPDATE_BAT);
			IdleUpdateBatSoc();
		}
		if(scr_msg[SCREEN_ID_TEMP].para&SCREEN_EVENT_UPDATE_TEMP)
		{
			scr_msg[SCREEN_ID_TEMP].para &= (~SCREEN_EVENT_UPDATE_TEMP);
			TempUpdateStatus();
		}
		break;
	}

	scr_msg[SCREEN_ID_TEMP].act = SCREEN_ACTION_NO;
}

void ExitTempScreen(void)
{
	k_timer_stop(&mainmenu_timer);
	k_timer_stop(&temp_status_timer);

#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif
	if(!TempIsWorkingTiming())
		MenuStopTemp();

	LCD_Set_BL_Mode(LCD_BL_AUTO);
	
	EnterIdleScreen();
}

void EnterTempScreen(void)
{
	if(screen_id == SCREEN_ID_TEMP)
		return;

	k_timer_stop(&mainmenu_timer);
	k_timer_stop(&temp_status_timer);
#ifdef CONFIG_PPG_SUPPORT	
	k_timer_stop(&ppg_status_timer);
#endif

#ifdef CONFIG_ANIMATION_SUPPORT	
	AnimaStop();
#endif
#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();
#endif
	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_TEMP;
	scr_msg[SCREEN_ID_TEMP].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_TEMP].status = SCREEN_STATUS_CREATING;

	get_temp_ok_flag = false;
	g_temp_status = TEMP_STATUS_PREPARE;
	img_flag = 0;
	temp_retry_left = 2;

#ifdef CONFIG_PPG_SUPPORT
	SetLeftKeyUpHandler(EnterSPO2Screen);
#elif defined(CONFIG_SYNC_SUPPORT)
	SetLeftKeyUpHandler(EnterSyncDataScreen);
#else
	SetLeftKeyUpHandler(EnterSettings);
#endif
	SetRightKeyUpHandler(ExitTempScreen);

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();

 #ifdef CONFIG_PPG_SUPPORT
 	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSPO2Screen);
 #elif defined(CONFIG_SYNC_SUPPORT)
 	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSyncDataScreen);
 #else
 	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSettings);
 #endif
 
 #ifdef CONFIG_PPG_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterHRScreen);
 #elif defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
   #ifdef CONFIG_SLEEP_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSleepScreen);
   #elif defined(CONFIG_STEP_SUPPORT)
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterStepsScreen);
   #endif
 #else
 	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterIdleScreen);
 #endif
#endif
}
#endif/*CONFIG_TEMP_SUPPORT*/

#ifdef CONFIG_PPG_SUPPORT
static uint8_t img_index = 0;
static uint8_t ppg_retry_left = 2;
static uint8_t ppgdata[PPG_REC2_MAX_DAILY*sizeof(bpt_rec2_nod)] = {0};

void PPGShowNumByImg(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t num_img_w, uint32_t *img_addr, uint8_t data)
{
	uint8_t i,count=1;
	uint32_t divisor=10;

	while(1)
	{
		if(data/divisor > 0)
		{
			count++;
			divisor = divisor*10;
		}
		else
		{
			divisor = divisor/10;
			break;
		}
	}

	LCD_Fill(x, y, w, h, BLACK);
	
	for(i=0;i<count;i++)
	{
		LCD_ShowImage(x+i*num_img_w, y, img_addr[data/divisor]);
		data = data%divisor;
		divisor = divisor/10;
	}
}

void PPGShowBpNumByImg(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t num_img_w, uint32_t *num_img_addr, uint32_t separa_img_addr, bpt_data data)
{
	uint8_t i,count1=1,count2=1;
	uint16_t x1;
	uint32_t divisor1=10,divisor2=10;

	while(1)
	{
		if(data.systolic/divisor1 > 0)
		{
			count1++;
			divisor1 = divisor1*10;
		}
		else
		{
			divisor1 = divisor1/10;
			break;
		}
	}
	while(1)
	{
		if(data.diastolic/divisor2 > 0)
		{
			count2++;
			divisor2 = divisor2*10;
		}
		else
		{
			divisor2 = divisor2/10;
			break;
		}
	}

	LCD_Fill(x, y, w, h, BLACK);
	
	x1 = x;
	for(i=0;i<(count1+count2+1);i++)
	{
		if(i < count1)
		{
			LCD_ShowImage(x1, y, num_img_addr[data.systolic/divisor1]);
			x1 += num_img_w;
			data.systolic = data.systolic%divisor1;
			divisor1 = divisor1/10;
		}
		else if(i == count1)
		{
			uint16_t img_w,img_h;
			
			LCD_MeasureImage(separa_img_addr, &img_w, &img_h);
			LCD_ShowImage(x1, y, separa_img_addr);
			x1 += img_w;
		}
		else
		{
			LCD_ShowImage(x1, y, num_img_addr[data.diastolic/divisor2]);
			x1 += num_img_w;
			data.diastolic = data.diastolic%divisor2;
			divisor2 = divisor2/10;
		}
	}
}

void PPGShowNumWithUnitByImg(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t num_img_w, uint32_t *num_img_addr, uint32_t unit_img_addr, uint8_t data)
{
	uint8_t i,count=1;
	uint32_t divisor=10;

	while(1)
	{
		if(data/divisor > 0)
		{
			count++;
			divisor = divisor*10;
		}
		else
		{
			divisor = divisor/10;
			break;
		}
	}

	LCD_Fill(x, y, w, h, BLACK);

	for(i=0;i<count;i++)
	{
		LCD_ShowImage(x+i*num_img_w, y, num_img_addr[data/divisor]);
		data = data%divisor;
		divisor = divisor/10;
	}
	LCD_ShowImage(x+i*num_img_w, y, unit_img_addr);
}

void PPGScreenStopTimer(void)
{
	k_timer_stop(&mainmenu_timer);
	k_timer_stop(&ppg_status_timer);
}

static void PPGStatusTimerOutCallBack(struct k_timer *timer_id)
{
	if(screen_id == SCREEN_ID_HR)
	{
		switch(g_ppg_status)
		{
		case PPG_STATUS_PREPARE:
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_HR;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			break;
			
		case PPG_STATUS_MEASURING:
			if(get_hr_ok_flag)
			{
				g_ppg_status = PPG_STATUS_MEASURE_OK;
			}
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_HR;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			break;
			
		case PPG_STATUS_MEASURE_FAIL:
			if(ppg_retry_left > 0)
			{
				g_ppg_status = PPG_STATUS_NOTIFY;
				scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_HR;
				scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			}
			else
			{
				g_ppg_status = PPG_STATUS_MAX;
				EntryIdleScr();
			}
			break;

		case PPG_STATUS_MEASURE_OK:
			g_ppg_status = PPG_STATUS_MAX;
			EntryIdleScr();
			break;

		case PPG_STATUS_NOTIFY:
			g_ppg_status = PPG_STATUS_PREPARE;
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_HR;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			break;
		}
	}
	else if(screen_id == SCREEN_ID_SPO2)
	{
		switch(g_ppg_status)
		{
		case PPG_STATUS_PREPARE:
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_SPO2;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			break;
			
		case PPG_STATUS_MEASURING:
			if(get_spo2_ok_flag)
			{
				g_ppg_status = PPG_STATUS_MEASURE_OK;
			}
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_SPO2;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			break;
			
		case PPG_STATUS_MEASURE_FAIL:
			if(ppg_retry_left > 0)
			{
				g_ppg_status = PPG_STATUS_NOTIFY;
				scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_SPO2;
				scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			}
			else
			{
				g_ppg_status = PPG_STATUS_MAX;
				EntryIdleScr();
			}
			break;

		case PPG_STATUS_MEASURE_OK:
			g_ppg_status = PPG_STATUS_MAX;
			EntryIdleScr();
			break;

		case PPG_STATUS_NOTIFY:
			g_ppg_status = PPG_STATUS_PREPARE;
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_SPO2;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			break;
		}
	}
	else if(screen_id == SCREEN_ID_BP)
	{
		switch(g_ppg_status)
		{
		case PPG_STATUS_PREPARE:
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_BP;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			break;
			
		case PPG_STATUS_MEASURING:
			if(get_bpt_ok_flag)
			{
				g_ppg_status = PPG_STATUS_MEASURE_OK;
			}
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_BP;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			break;
			
		case PPG_STATUS_MEASURE_FAIL:
			if(ppg_retry_left > 0)
			{
				g_ppg_status = PPG_STATUS_NOTIFY;
				scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_BP;
				scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			}
			else
			{
				g_ppg_status = PPG_STATUS_MAX;
				EntryIdleScr();
			}
			break;

		case PPG_STATUS_MEASURE_OK:
			g_ppg_status = PPG_STATUS_MAX;
			EntryIdleScr();
			break;

		case PPG_STATUS_NOTIFY:
			g_ppg_status = PPG_STATUS_PREPARE;
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_BP;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
			break;
		}
	}
}

void BPUpdateStatus(void)
{
	uint8_t i,count1=1,count2=1;
	uint16_t x,y,w,h;
	uint32_t img_anima[3] = {IMG_ID_BPT_ANI_1,IMG_ID_BPT_ANI_2,IMG_ID_BPT_ANI_3};
	bpt_data bpt = {0};
	uint16_t str_id;
	uint32_t divisor1=10,divisor2=10;
	uint32_t img_num[10] = {IMG_ID_FONT_70_NUM_0,IMG_ID_FONT_70_NUM_1,IMG_ID_FONT_70_NUM_2,IMG_ID_FONT_70_NUM_3,IMG_ID_FONT_70_NUM_4,
							IMG_ID_FONT_70_NUM_5,IMG_ID_FONT_70_NUM_6,IMG_ID_FONT_70_NUM_7,IMG_ID_FONT_70_NUM_8,IMG_ID_FONT_70_NUM_9};
	uint32_t img_small_num[10] = {IMG_ID_FONT_35_NUM_0,IMG_ID_FONT_35_NUM_1,IMG_ID_FONT_35_NUM_2,IMG_ID_FONT_35_NUM_3,IMG_ID_FONT_35_NUM_4,
								  IMG_ID_FONT_35_NUM_5,IMG_ID_FONT_35_NUM_6,IMG_ID_FONT_35_NUM_7,IMG_ID_FONT_35_NUM_8,IMG_ID_FONT_35_NUM_9};

	switch(g_ppg_status)
	{
	case PPG_STATUS_PREPARE:
		LCD_Fill(BP_NOTIFY_X, BP_NOTIFY_Y, BP_NOTIFY_W, BP_NOTIFY_H, BLACK);
		
		LCD_SetFontSize(FONT_SIZE_36);

		LCD_MeasureUniStr(STR_ID_STAY_STILL, &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStrRtoL(BP_NOTIFY_X+(BP_NOTIFY_W+w)/2, BP_NOTIFY_Y, STR_ID_STAY_STILL);
		else
	#endif		
			LCD_ShowUniStr(BP_NOTIFY_X+(BP_NOTIFY_W-w)/2, BP_NOTIFY_Y, STR_ID_STAY_STILL);
		
		MenuStartBpt();
		g_ppg_status = PPG_STATUS_MEASURING;
		break;
		
	case PPG_STATUS_MEASURING:
		img_index++;
		if(img_index >= 3)
			img_index = 0;
		LCD_ShowImage(BP_ICON_X, BP_ICON_Y, img_anima[img_index]);

		LCD_SetFontSize(FONT_SIZE_36);

		if(get_bpt_ok_flag)
		{
			LCD_Fill(BP_NOTIFY_X, BP_NOTIFY_Y, BP_NOTIFY_W, BP_NOTIFY_H, BLACK);

			memcpy(&bpt, &g_bpt, sizeof(bpt_data));

			while(1)
			{
				if(bpt.systolic/divisor1 > 0)
				{
					count1++;
					divisor1 = divisor1*10;
				}
				else
				{
					divisor1 = divisor1/10;
					break;
				}
			}
			
			while(1)
			{
				if(bpt.diastolic/divisor2 > 0)
				{
					count2++;
					divisor2 = divisor2*10;
				}
				else
				{
					divisor2 = divisor2/10;
					break;
				}
			}

			x = BP_STR_X+(BP_STR_W-(count1+count2)*BP_NUM_W-BP_SLASH_W)/2;
			y = HR_STR_Y;
			
			for(i=0;i<(count1+count2+1);i++)
			{
				if(i < count1)
				{
					LCD_ShowImage(x, y, img_num[bpt.systolic/divisor1]);
					x += BP_NUM_W;
					bpt.systolic = bpt.systolic%divisor1;
					divisor1 = divisor1/10;
				}
				else if(i == count1)
				{
					LCD_ShowImage(x, y, IMG_ID_FONT_70_SLASH);
					x += BP_SLASH_W;
				}
				else
				{
					LCD_ShowImage(x, y, img_num[bpt.diastolic/divisor2]);
					x += BP_NUM_W;
					bpt.diastolic = bpt.diastolic%divisor2;
					divisor2 = divisor2/10;
				}
			}

			MenuStopBpt();
			k_timer_start(&ppg_status_timer, K_SECONDS(5), K_NO_WAIT);
		}
		break;
		
	case PPG_STATUS_MEASURE_OK:
		LCD_ShowImage(BP_ICON_X, BP_ICON_Y, IMG_ID_BPT_ANI_3);
		PPGShowBpNumByImg(BP_UP_STR_X, BP_UP_STR_Y, BP_UP_STR_W, BP_UP_STR_H, BP_UP_NUM_W, img_small_num, IMG_ID_FONT_35_SLASH, last_health.bpt_max);
		PPGShowBpNumByImg(BP_DOWN_STR_X, BP_DOWN_STR_Y, BP_DOWN_STR_W, BP_DOWN_STR_H, BP_DOWN_NUM_W, img_small_num, IMG_ID_FONT_35_SLASH, last_health.bpt_min);
		k_timer_start(&ppg_status_timer, K_SECONDS(2), K_NO_WAIT);
		break;
		
	case PPG_STATUS_MEASURE_FAIL:
		MenuStopBpt();

		LCD_Fill(BP_NOTIFY_X, BP_NOTIFY_Y, BP_NOTIFY_W, BP_NOTIFY_H, BLACK);
		LCD_ShowImage(BP_ICON_X, BP_ICON_Y, IMG_ID_BPT_ANI_3);

		LCD_SetFontSize(FONT_SIZE_36);

		ppg_retry_left--;
		if(ppg_retry_left == 0)
		{
			str_id = STR_ID_INCONCLUSIVE_AND_RETRY_LATER;
		}
		else
		{
			str_id = STR_ID_INCONCLUSIVE;
		}
		
		LCD_MeasureUniStr(str_id,&w,&h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStrRtoL(BP_NOTIFY_X+(BP_NOTIFY_W+w)/2, BP_NOTIFY_Y, str_id);
		else
	#endif		
			LCD_ShowUniStr(BP_NOTIFY_X+(BP_NOTIFY_W-w)/2, BP_NOTIFY_Y, str_id);

		k_timer_start(&ppg_status_timer, K_SECONDS(5), K_NO_WAIT);
		break;
		
	case PPG_STATUS_NOTIFY:
		LCD_Fill(BP_NOTIFY_X, BP_NOTIFY_Y, BP_NOTIFY_W, BP_NOTIFY_H, BLACK);
		LCD_ShowImage(BP_ICON_X, BP_ICON_Y, IMG_ID_BPT_ANI_3);

		LCD_SetFontSize(FONT_SIZE_36);

		LCD_MeasureUniStr(STR_ID_STAY_STILL_AND_RETRY, &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStrRtoL(BP_NOTIFY_X+(BP_NOTIFY_W+w)/2, BP_NOTIFY_Y, STR_ID_STAY_STILL_AND_RETRY);
		else
	#endif
			LCD_ShowUniStr(BP_NOTIFY_X+(BP_NOTIFY_W-w)/2, BP_NOTIFY_Y, STR_ID_STAY_STILL_AND_RETRY);

		k_timer_start(&ppg_status_timer, K_SECONDS(5), K_NO_WAIT);
		break;
	}
}

void BPShowStatus(void)
{
	uint16_t w,h;
	uint32_t img_num[10] = {IMG_ID_FONT_35_NUM_0,IMG_ID_FONT_35_NUM_1,IMG_ID_FONT_35_NUM_2,IMG_ID_FONT_35_NUM_3,IMG_ID_FONT_35_NUM_4,
							IMG_ID_FONT_35_NUM_5,IMG_ID_FONT_35_NUM_6,IMG_ID_FONT_35_NUM_7,IMG_ID_FONT_35_NUM_8,IMG_ID_FONT_35_NUM_9};

	LCD_ShowImage(BP_ICON_X, BP_ICON_Y, IMG_ID_BPT_ANI_3);
	LCD_ShowImage(BP_UP_ARRAW_X, BP_UP_ARRAW_Y, IMG_ID_BPT_MAX_REC);
	LCD_ShowImage(BP_DOWN_ARRAW_X, BP_DOWN_ARRAW_Y, IMG_ID_BPT_MIN_REC);

	LCD_SetFontSize(FONT_SIZE_36);

	LCD_MeasureUniStr(STR_ID_BPT, &w, &h);
#ifdef LANGUAGE_AR_ENABLE	
	if(g_language_r2l)
		LCD_ShowUniStrRtoL(BP_NOTIFY_X+(BP_NOTIFY_W+w)/2, BP_NOTIFY_Y, STR_ID_BPT);
	else
#endif		
		LCD_ShowUniStr(BP_NOTIFY_X+(BP_NOTIFY_W-w)/2, BP_NOTIFY_Y, STR_ID_BPT);

	PPGShowBpNumByImg(BP_UP_STR_X, BP_UP_STR_Y, BP_UP_STR_W, BP_UP_STR_H, BP_UP_NUM_W, img_num, IMG_ID_FONT_35_SLASH, last_health.bpt_max);
	PPGShowBpNumByImg(BP_DOWN_STR_X, BP_DOWN_STR_Y, BP_DOWN_STR_W, BP_DOWN_STR_H, BP_DOWN_NUM_W, img_num, IMG_ID_FONT_35_SLASH, last_health.bpt_min);

	k_timer_start(&ppg_status_timer, K_SECONDS(2), K_NO_WAIT);
}

void BPScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_BP].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_BP].status = SCREEN_STATUS_CREATED;

		LCD_Clear(BLACK);
		IdleShowSignal();
		IdleShowNetMode();
		IdleShowBatSoc();
		BPShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		if(scr_msg[SCREEN_ID_BP].para&SCREEN_EVENT_UPDATE_SIG)
		{
			scr_msg[SCREEN_ID_BP].para &= (~SCREEN_EVENT_UPDATE_SIG);
			IdleShowSignal();
		}
		if(scr_msg[SCREEN_ID_BP].para&SCREEN_EVENT_UPDATE_NET_MODE)
		{
			scr_msg[SCREEN_ID_BP].para &= (~SCREEN_EVENT_UPDATE_NET_MODE);	
			IdleShowNetMode();
		}
		if(scr_msg[SCREEN_ID_BP].para&SCREEN_EVENT_UPDATE_BAT)
		{
			scr_msg[SCREEN_ID_BP].para &= (~SCREEN_EVENT_UPDATE_BAT);
			IdleUpdateBatSoc();
		}
		if(scr_msg[SCREEN_ID_BP].para&SCREEN_EVENT_UPDATE_BP)
		{
			scr_msg[SCREEN_ID_BP].para &= (~SCREEN_EVENT_UPDATE_BP);
			BPUpdateStatus();
		}
		break;
	}
	
	scr_msg[SCREEN_ID_BP].act = SCREEN_ACTION_NO;
}

void ExitBPScreen(void)
{
	k_timer_stop(&mainmenu_timer);
	k_timer_stop(&ppg_status_timer);

	img_index = 0;
	
#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif

	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();

	LCD_Set_BL_Mode(LCD_BL_AUTO);
	
	EnterIdleScreen();
}

void EnterBPScreen(void)
{
	if(screen_id == SCREEN_ID_BP)
		return;

	k_timer_stop(&mainmenu_timer);
	k_timer_stop(&ppg_status_timer);
#ifdef CONFIG_TEMP_SUPPORT
	k_timer_stop(&temp_status_timer);
#endif

#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();
#ifdef CONFIG_ECG_SUPPORT
	if(IsInEcgScreen())
		ExitEcgScreen();
#endif

	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_BP;	
	scr_msg[SCREEN_ID_BP].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_BP].status = SCREEN_STATUS_CREATING;

	get_bpt_ok_flag = false;
	img_index = 0;
	g_ppg_status = PPG_STATUS_PREPARE;
	ppg_retry_left = 2;

#ifdef CONFIG_ECG_SUPPORT
	SetLeftKeyUpHandler(EnterEcgScreen);
#elif defined(CONFIG_SYNC_SUPPORT)
	SetLeftKeyUpHandler(EnterSyncDataScreen);
#else
	SetLeftKeyUpHandler(EnterSettings);
#endif
	SetRightKeyUpHandler(ExitBPScreen);

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();

 #ifdef CONFIG_ECG_SUPPORT
 	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterEcgScreen);
 #elif defined(CONFIG_SYNC_SUPPORT)
 	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSyncDataScreen);
 #else
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSettings); 
 #endif
 
 	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSPO2Screen);
#endif
}

void SPO2UpdateStatus(void)
{
	uint8_t i,spo2=g_spo2,count=1;
	uint16_t w,h;
	uint16_t str_id;
	uint32_t divisor=10;
	uint32_t img_anima[3] = {IMG_ID_SPO2_ANI_1, IMG_ID_SPO2_ANI_2, IMG_ID_SPO2_ANI_3};
	uint32_t img_num[10] = {IMG_ID_FONT_70_NUM_0,IMG_ID_FONT_70_NUM_1,IMG_ID_FONT_70_NUM_2,IMG_ID_FONT_70_NUM_3,IMG_ID_FONT_70_NUM_4,
							IMG_ID_FONT_70_NUM_5,IMG_ID_FONT_70_NUM_6,IMG_ID_FONT_70_NUM_7,IMG_ID_FONT_70_NUM_8,IMG_ID_FONT_70_NUM_9};
	uint32_t img_small_num[10] = {IMG_ID_FONT_45_NUM_0,IMG_ID_FONT_45_NUM_1,IMG_ID_FONT_45_NUM_2,IMG_ID_FONT_45_NUM_3,IMG_ID_FONT_45_NUM_4,
								  IMG_ID_FONT_45_NUM_5,IMG_ID_FONT_45_NUM_6,IMG_ID_FONT_45_NUM_7,IMG_ID_FONT_45_NUM_8,IMG_ID_FONT_45_NUM_9};

	switch(g_ppg_status)
	{
	case PPG_STATUS_PREPARE:
		LCD_Fill(SPO2_NOTIFY_X, SPO2_NOTIFY_Y, SPO2_NOTIFY_W, SPO2_NOTIFY_H, BLACK);
		
		LCD_SetFontSize(FONT_SIZE_36);

		LCD_MeasureUniStr(STR_ID_STAY_STILL, &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStrRtoL(SPO2_NOTIFY_X+(SPO2_NOTIFY_W+w)/2, SPO2_NOTIFY_Y, STR_ID_STAY_STILL);
		else
	#endif		
			LCD_ShowUniStr(SPO2_NOTIFY_X+(SPO2_NOTIFY_W-w)/2, SPO2_NOTIFY_Y, STR_ID_STAY_STILL);
		
		MenuStartSpo2();
		g_ppg_status = PPG_STATUS_MEASURING;
		break;
		
	case PPG_STATUS_MEASURING:
		img_index++;
		if(img_index >= 3)
			img_index = 0;
		LCD_ShowImage(SPO2_ICON_X, SPO2_ICON_Y, img_anima[img_index]);

		LCD_SetFontSize(FONT_SIZE_36);

		if(get_spo2_ok_flag)
		{
			LCD_Fill(SPO2_NOTIFY_X, SPO2_NOTIFY_Y, SPO2_NOTIFY_W, SPO2_NOTIFY_H, BLACK);

			while(1)
			{
				if(spo2/divisor > 0)
				{
					count++;
					divisor = divisor*10;
				}
				else
				{
					divisor = divisor/10;
					break;
				}
			}

			for(i=0;i<count;i++)
			{
				LCD_ShowImage(SPO2_STR_X+(SPO2_STR_W-count*SPO2_NUM_W-SPO2_PERC_W)/2+i*SPO2_NUM_W, SPO2_STR_Y, img_num[spo2/divisor]);
				spo2 = spo2%divisor;
				divisor = divisor/10;
			}
			LCD_ShowImage(SPO2_STR_X+(SPO2_STR_W-count*SPO2_NUM_W-SPO2_PERC_W)/2+i*SPO2_NUM_W, SPO2_STR_Y, IMG_ID_FONT_70_PERC);
		
			MenuStopSpo2();
			k_timer_start(&ppg_status_timer, K_SECONDS(5), K_NO_WAIT);
		}
		break;
		
	case PPG_STATUS_MEASURE_OK:
		LCD_ShowImage(SPO2_ICON_X, SPO2_ICON_Y, IMG_ID_SPO2_ANI_3);
		PPGShowNumWithUnitByImg(SPO2_UP_STR_X, SPO2_UP_STR_Y, SPO2_UP_STR_W, SPO2_UP_STR_H, SPO2_UP_NUM_W, img_small_num, IMG_ID_FONT_45_PERC, last_health.spo2_max);
		PPGShowNumWithUnitByImg(SPO2_DOWN_STR_X, SPO2_DOWN_STR_Y, SPO2_DOWN_STR_W, SPO2_DOWN_STR_H, SPO2_DOWN_NUM_W, img_small_num, IMG_ID_FONT_45_PERC, last_health.spo2_min);
		k_timer_start(&ppg_status_timer, K_SECONDS(2), K_NO_WAIT);
		break;
		
	case PPG_STATUS_MEASURE_FAIL:
		MenuStopSpo2();

		LCD_Fill(SPO2_NOTIFY_X, SPO2_NOTIFY_Y, SPO2_NOTIFY_W, SPO2_NOTIFY_H, BLACK);
		LCD_ShowImage(SPO2_ICON_X, SPO2_ICON_Y, IMG_ID_SPO2_ANI_3);

		LCD_SetFontSize(FONT_SIZE_36);

		ppg_retry_left--;
		if(ppg_retry_left == 0)
		{
			str_id = STR_ID_INCONCLUSIVE_AND_RETRY_LATER;
		}
		else
		{
			str_id = STR_ID_INCONCLUSIVE;
		}
		
		LCD_MeasureUniStr(str_id, &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStrRtoL(SPO2_NOTIFY_X+(SPO2_NOTIFY_W+w)/2, SPO2_NOTIFY_Y, str_id);
		else
	#endif		
			LCD_ShowUniStr(SPO2_NOTIFY_X+(SPO2_NOTIFY_W-w)/2, SPO2_NOTIFY_Y, str_id);
		
		k_timer_start(&ppg_status_timer, K_SECONDS(5), K_NO_WAIT);
		break;
		
	case PPG_STATUS_NOTIFY:
		LCD_Fill(SPO2_NOTIFY_X, SPO2_NOTIFY_Y, SPO2_NOTIFY_W, SPO2_NOTIFY_H, BLACK);
		LCD_ShowImage(SPO2_ICON_X, SPO2_ICON_Y, IMG_ID_SPO2_ANI_3);

		LCD_SetFontSize(FONT_SIZE_36);
	
		LCD_MeasureUniStr(STR_ID_STAY_STILL_AND_RETRY, &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStrRtoL(SPO2_NOTIFY_X+(SPO2_NOTIFY_W+w)/2, SPO2_NOTIFY_Y, STR_ID_STAY_STILL_AND_RETRY);
		else
	#endif		
			LCD_ShowUniStr(SPO2_NOTIFY_X+(SPO2_NOTIFY_W-w)/2, SPO2_NOTIFY_Y, STR_ID_STAY_STILL_AND_RETRY);

		k_timer_start(&ppg_status_timer, K_SECONDS(5), K_NO_WAIT);
		break;
	}
}

void SPO2ShowStatus(void)
{
	uint16_t w,h;
	uint32_t img_num[10] = {IMG_ID_FONT_45_NUM_0,IMG_ID_FONT_45_NUM_1,IMG_ID_FONT_45_NUM_2,IMG_ID_FONT_45_NUM_3,IMG_ID_FONT_45_NUM_4,
							IMG_ID_FONT_45_NUM_5,IMG_ID_FONT_45_NUM_6,IMG_ID_FONT_45_NUM_7,IMG_ID_FONT_45_NUM_8,IMG_ID_FONT_45_NUM_9};

	LCD_ShowImage(SPO2_ICON_X, SPO2_ICON_Y, IMG_ID_SPO2_ANI_3);
	LCD_ShowImage(SPO2_UP_ARRAW_X, SPO2_UP_ARRAW_Y, IMG_ID_SPO2_MAX_REC);
	LCD_ShowImage(SPO2_DOWN_ARRAW_X, SPO2_DOWN_ARRAW_Y, IMG_ID_SPO2_MIN_REC);

	LCD_SetFontSize(FONT_SIZE_36);

	LCD_MeasureUniStr(STR_ID_SPO2, &w, &h);
#ifdef LANGUAGE_AR_ENABLE	
	if(g_language_r2l)
		LCD_ShowUniStrRtoL(SPO2_NOTIFY_X+(SPO2_NOTIFY_W+w)/2, SPO2_NOTIFY_Y, STR_ID_SPO2);
	else
#endif		
		LCD_ShowUniStr(SPO2_NOTIFY_X+(SPO2_NOTIFY_W-w)/2, SPO2_NOTIFY_Y, STR_ID_SPO2);

	PPGShowNumWithUnitByImg(SPO2_UP_STR_X, SPO2_UP_STR_Y, SPO2_UP_STR_W, SPO2_UP_STR_H, SPO2_UP_NUM_W, img_num, IMG_ID_FONT_45_PERC, last_health.spo2_max);
	PPGShowNumWithUnitByImg(SPO2_DOWN_STR_X, SPO2_DOWN_STR_Y, SPO2_DOWN_STR_W, SPO2_DOWN_STR_H, SPO2_DOWN_NUM_W, img_num, IMG_ID_FONT_45_PERC, last_health.spo2_min);

	k_timer_start(&ppg_status_timer, K_SECONDS(2), K_NO_WAIT);
}

void SPO2ScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_SPO2].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_SPO2].status = SCREEN_STATUS_CREATED;

		LCD_Clear(BLACK);
		IdleShowSignal();
		IdleShowNetMode();
		IdleShowBatSoc();
		SPO2ShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		if(scr_msg[SCREEN_ID_SPO2].para&SCREEN_EVENT_UPDATE_SIG)
		{
			scr_msg[SCREEN_ID_SPO2].para &= (~SCREEN_EVENT_UPDATE_SIG);
			IdleShowSignal();
		}
		if(scr_msg[SCREEN_ID_SPO2].para&SCREEN_EVENT_UPDATE_NET_MODE)
		{
			scr_msg[SCREEN_ID_SPO2].para &= (~SCREEN_EVENT_UPDATE_NET_MODE);	
			IdleShowNetMode();
		}
		if(scr_msg[SCREEN_ID_SPO2].para&SCREEN_EVENT_UPDATE_BAT)
		{
			scr_msg[SCREEN_ID_SPO2].para &= (~SCREEN_EVENT_UPDATE_BAT);
			IdleUpdateBatSoc();
		}
		if(scr_msg[SCREEN_ID_SPO2].para&SCREEN_EVENT_UPDATE_SPO2)
		{
			scr_msg[SCREEN_ID_SPO2].para &= (~SCREEN_EVENT_UPDATE_SPO2);
			SPO2UpdateStatus();
		}
		break;
	}
	
	scr_msg[SCREEN_ID_SPO2].act = SCREEN_ACTION_NO;
}

void ExitSPO2Screen(void)
{
	k_timer_stop(&mainmenu_timer);
	k_timer_stop(&ppg_status_timer);

	img_index = 0;

#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif

	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();

	LCD_Set_BL_Mode(LCD_BL_AUTO);
	
	EnterIdleScreen();
}

void EnterSPO2Screen(void)
{
	if(screen_id == SCREEN_ID_SPO2)
		return;

	k_timer_stop(&mainmenu_timer);
	k_timer_stop(&ppg_status_timer);
#ifdef CONFIG_TEMP_SUPPORT
	k_timer_stop(&temp_status_timer);
#endif

#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(IsInTempScreen()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();
	
	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_SPO2;	
	scr_msg[SCREEN_ID_SPO2].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_SPO2].status = SCREEN_STATUS_CREATING;

	get_spo2_ok_flag = false;
	img_index = 0;
	g_ppg_status = PPG_STATUS_PREPARE;
	ppg_retry_left = 2;

	SetLeftKeyUpHandler(EnterBPScreen);
	SetRightKeyUpHandler(ExitSPO2Screen);
	
#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();

	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterBPScreen);

  #ifdef CONFIG_TEMP_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterTempScreen);
  #else
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterHRScreen);
  #endif
#endif	
}

void HRUpdateStatus(void)
{
	uint8_t i,hr=g_hr,count=1;
	uint16_t w,h;
	uint16_t str_id;
	uint32_t divisor=10;
	uint32_t img_anima[2] = {IMG_ID_HR_ANI_1, IMG_ID_HR_ANI_2};
	uint32_t img_num[10] = {IMG_ID_FONT_70_NUM_0,IMG_ID_FONT_70_NUM_1,IMG_ID_FONT_70_NUM_2,IMG_ID_FONT_70_NUM_3,IMG_ID_FONT_70_NUM_4,
							IMG_ID_FONT_70_NUM_5,IMG_ID_FONT_70_NUM_6,IMG_ID_FONT_70_NUM_7,IMG_ID_FONT_70_NUM_8,IMG_ID_FONT_70_NUM_9};
	uint32_t img_small_num[10] = {IMG_ID_FONT_45_NUM_0,IMG_ID_FONT_45_NUM_1,IMG_ID_FONT_45_NUM_2,IMG_ID_FONT_45_NUM_3,IMG_ID_FONT_45_NUM_4,
								  IMG_ID_FONT_45_NUM_5,IMG_ID_FONT_45_NUM_6,IMG_ID_FONT_45_NUM_7,IMG_ID_FONT_45_NUM_8,IMG_ID_FONT_45_NUM_9};

	switch(g_ppg_status)
	{
	case PPG_STATUS_PREPARE:
		LCD_Fill(HR_NOTIFY_X, HR_NOTIFY_Y, HR_NOTIFY_W, HR_NOTIFY_H, BLACK);
		
		LCD_SetFontSize(FONT_SIZE_36);

		LCD_MeasureUniStr(STR_ID_STAY_STILL, &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStrRtoL(HR_NOTIFY_X+(HR_NOTIFY_W+w)/2, HR_NOTIFY_Y, STR_ID_STAY_STILL);
		else
	#endif		
			LCD_ShowUniStr(HR_NOTIFY_X+(HR_NOTIFY_W-w)/2, HR_NOTIFY_Y, STR_ID_STAY_STILL);
		
		MenuStartHr();
		g_ppg_status = PPG_STATUS_MEASURING;
		break;
		
	case PPG_STATUS_MEASURING:
		img_index++;
		if(img_index >= 2)
			img_index = 0;
		LCD_ShowImage(HR_ICON_X, HR_ICON_Y, img_anima[img_index]);

		LCD_SetFontSize(FONT_SIZE_36);

		if(get_hr_ok_flag)
		{
			LCD_Fill(HR_NOTIFY_X, HR_NOTIFY_Y, HR_NOTIFY_W, HR_NOTIFY_H, BLACK);

			while(1)
			{
				if(hr/divisor > 0)
				{
					count++;
					divisor = divisor*10;
				}
				else
				{
					divisor = divisor/10;
					break;
				}
			}

			for(i=0;i<count;i++)
			{
				LCD_ShowImage(HR_STR_X+(HR_STR_W-count*HR_NUM_W)/2+i*HR_NUM_W, HR_STR_Y, img_num[hr/divisor]);
				hr = hr%divisor;
				divisor = divisor/10;
			}
			LCD_ShowImage(HR_STR_X+(HR_STR_W-count*HR_NUM_W)/2+i*HR_NUM_W, HR_UNIT_Y, IMG_ID_HR_BPM);

			MenuStopHr();

			k_timer_start(&ppg_status_timer, K_SECONDS(5), K_NO_WAIT);
		}
		break;
		
	case PPG_STATUS_MEASURE_OK:
		LCD_ShowImage(HR_ICON_X, HR_ICON_Y, IMG_ID_HR_ANI_2);
		PPGShowNumByImg(HR_UP_STR_X, HR_UP_STR_Y, HR_UP_STR_W, HR_UP_STR_H, HR_UP_NUM_W, img_small_num, last_health.hr_max);
		PPGShowNumByImg(HR_DOWN_STR_X, HR_DOWN_STR_Y, HR_DOWN_STR_W, HR_DOWN_STR_H, HR_DOWN_NUM_W, img_small_num, last_health.hr_min);
		k_timer_start(&ppg_status_timer, K_SECONDS(2), K_NO_WAIT);
		break;
		
	case PPG_STATUS_MEASURE_FAIL:
		MenuStopHr();
		
		LCD_Fill(HR_NOTIFY_X, HR_NOTIFY_Y, HR_NOTIFY_W, HR_NOTIFY_H, BLACK);
		LCD_ShowImage(HR_ICON_X, HR_ICON_Y, IMG_ID_HR_ANI_2);

		LCD_SetFontSize(FONT_SIZE_36);

		ppg_retry_left--;
		if(ppg_retry_left == 0)
		{
			str_id = STR_ID_INCONCLUSIVE_AND_RETRY_LATER;
		}
		else
		{
			str_id = STR_ID_INCONCLUSIVE;
		}
		
		LCD_MeasureUniStr(str_id, &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStrRtoL(HR_NOTIFY_X+(HR_NOTIFY_W+w)/2, HR_NOTIFY_Y, str_id);
		else
	#endif
			LCD_ShowUniStr(HR_NOTIFY_X+(HR_NOTIFY_W-w)/2, HR_NOTIFY_Y, str_id);
	
		k_timer_start(&ppg_status_timer, K_SECONDS(5), K_NO_WAIT);
		break;
		
	case PPG_STATUS_NOTIFY:
		LCD_Fill(HR_NOTIFY_X, HR_NOTIFY_Y, HR_NOTIFY_W, HR_NOTIFY_H, BLACK);
		LCD_ShowImage(HR_ICON_X, HR_ICON_Y, IMG_ID_HR_ANI_2);

		LCD_SetFontSize(FONT_SIZE_36);

		LCD_MeasureUniStr(STR_ID_STAY_STILL_AND_RETRY, &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStrRtoL(HR_NOTIFY_X+(HR_NOTIFY_W+w)/2, HR_NOTIFY_Y, STR_ID_STAY_STILL_AND_RETRY);
		else
	#endif		
			LCD_ShowUniStr(HR_NOTIFY_X+(HR_NOTIFY_W-w)/2, HR_NOTIFY_Y, STR_ID_STAY_STILL_AND_RETRY);

		k_timer_start(&ppg_status_timer, K_SECONDS(5), K_NO_WAIT);
		break;
	}
}

void HRShowStatus(void)
{
	uint8_t count=1;
	uint16_t w,h;
	uint32_t divisor=10;
	uint32_t img_num[10] = {IMG_ID_FONT_45_NUM_0,IMG_ID_FONT_45_NUM_1,IMG_ID_FONT_45_NUM_2,IMG_ID_FONT_45_NUM_3,IMG_ID_FONT_45_NUM_4,
							IMG_ID_FONT_45_NUM_5,IMG_ID_FONT_45_NUM_6,IMG_ID_FONT_45_NUM_7,IMG_ID_FONT_45_NUM_8,IMG_ID_FONT_45_NUM_9};

	LCD_ShowImage(HR_ICON_X, HR_ICON_Y, IMG_ID_HR_ANI_2);
	LCD_ShowImage(HR_UP_ARRAW_X, HR_UP_ARRAW_Y, IMG_ID_HR_MAX_REC);
	LCD_ShowImage(HR_DOWN_ARRAW_X, HR_DOWN_ARRAW_Y, IMG_ID_HR_MIN_REC);

	LCD_SetFontSize(FONT_SIZE_36);

	LCD_MeasureUniStr(STR_ID_HR,&w,&h);
#ifdef LANGUAGE_AR_ENABLE	
	if(g_language_r2l)
		LCD_ShowUniStrRtoL(HR_NOTIFY_X+(HR_NOTIFY_W+w)/2, HR_NOTIFY_Y, STR_ID_HR);
	else
#endif		
		LCD_ShowUniStr(HR_NOTIFY_X+(HR_NOTIFY_W-w)/2, HR_NOTIFY_Y, STR_ID_HR);

	PPGShowNumByImg(HR_UP_STR_X, HR_UP_STR_Y, HR_UP_STR_W, HR_UP_STR_H, HR_UP_NUM_W, img_num, last_health.hr_max);
	PPGShowNumByImg(HR_DOWN_STR_X, HR_DOWN_STR_Y, HR_DOWN_STR_W, HR_DOWN_STR_H, HR_UP_NUM_W, img_num, last_health.hr_min);

	k_timer_start(&ppg_status_timer, K_SECONDS(2), K_NO_WAIT);
}

void HRScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_HR].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_HR].status = SCREEN_STATUS_CREATED;

		LCD_Clear(BLACK);
		IdleShowSignal();
		IdleShowNetMode();
		IdleShowBatSoc();
		HRShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		if(scr_msg[SCREEN_ID_HR].para&SCREEN_EVENT_UPDATE_SIG)
		{
			scr_msg[SCREEN_ID_HR].para &= (~SCREEN_EVENT_UPDATE_SIG);
			IdleShowSignal();
		}
		if(scr_msg[SCREEN_ID_HR].para&SCREEN_EVENT_UPDATE_NET_MODE)
		{
			scr_msg[SCREEN_ID_HR].para &= (~SCREEN_EVENT_UPDATE_NET_MODE);	
			IdleShowNetMode();
		}
		if(scr_msg[SCREEN_ID_HR].para&SCREEN_EVENT_UPDATE_BAT)
		{
			scr_msg[SCREEN_ID_HR].para &= (~SCREEN_EVENT_UPDATE_BAT);
			IdleUpdateBatSoc();
		}
		if(scr_msg[SCREEN_ID_HR].para&SCREEN_EVENT_UPDATE_HR)
		{
			scr_msg[SCREEN_ID_HR].para &= (~SCREEN_EVENT_UPDATE_HR);
			HRUpdateStatus();
		}
		break;
	}
	
	scr_msg[SCREEN_ID_HR].act = SCREEN_ACTION_NO;
}

void ExitHRScreen(void)
{
	k_timer_stop(&mainmenu_timer);
	k_timer_stop(&ppg_status_timer);

	img_index = 0;
	
#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif

	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();

	LCD_Set_BL_Mode(LCD_BL_AUTO);
	
	EnterIdleScreen();
}

void EnterHRScreen(void)
{
	if(screen_id == SCREEN_ID_HR)
		return;

	k_timer_stop(&mainmenu_timer);
	k_timer_stop(&ppg_status_timer);
#ifdef CONFIG_TEMP_SUPPORT
	k_timer_stop(&temp_status_timer);
#endif

#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(IsInTempScreen()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();
	
	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_HR;	
	scr_msg[SCREEN_ID_HR].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_HR].status = SCREEN_STATUS_CREATING;

	get_hr_ok_flag = false;
	img_index = 0;	
	g_ppg_status = PPG_STATUS_PREPARE;
	ppg_retry_left = 2;

#ifdef CONFIG_TEMP_SUPPORT
	SetLeftKeyUpHandler(EnterTempScreen);
#else
	SetLeftKeyUpHandler(EnterSPO2Screen);
#endif
	SetRightKeyUpHandler(ExitHRScreen);

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();

  #ifdef CONFIG_TEMP_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterTempScreen);
  #else
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSPO2Screen);
  #endif

  #if defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
   #ifdef CONFIG_SLEEP_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSleepScreen);
   #elif defined(CONFIG_STEP_SUPPORT)
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterStepsScreen);
   #endif
  #else
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterIdleScreen);
  #endif
#endif	
}
#endif/*CONFIG_PPG_SUPPORT*/

static uint16_t str_x,str_y,str_w,str_h;
void EnterNotifyScreen(void)
{
	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_NOTIFY;	
	scr_msg[SCREEN_ID_NOTIFY].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_NOTIFY].status = SCREEN_STATUS_CREATING;

	SetLeftKeyUpHandler(ExitNotify);
	SetRightKeyUpHandler(ExitNotify);

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
	register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 0, LCD_WIDTH, 0, LCD_HEIGHT, ExitNotify);
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, ExitNotify);
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, ExitNotify);
#endif	
}

void DisplayPopUp(notify_infor infor)
{
	uint32_t len;

	k_timer_stop(&notify_timer);
	k_timer_stop(&mainmenu_timer);

#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif
#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		PPGStopCheck();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(IsInTempScreen()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
#ifdef CONFIG_SYNC_SUPPORT
	if(SyncIsRunning())
		SyncDataStop();
#endif

	memcpy(&notify_msg, &infor, sizeof(notify_infor));

	len = mmi_ucs2strlen((uint8_t*)notify_msg.text);
	if(len > NOTIFY_TEXT_MAX_LEN)
		len = NOTIFY_TEXT_MAX_LEN;
	
	if(notify_msg.type == NOTIFY_TYPE_POPUP)
	{
		k_timer_start(&notify_timer, K_SECONDS(NOTIFY_TIMER_INTERVAL), K_NO_WAIT);
	}
	
	EnterNotifyScreen();
}

void ExitNotify(void)
{
	if(screen_id == SCREEN_ID_NOTIFY)
	{
		scr_msg[screen_id].act = SCREEN_ACTION_EXIT;
	}
}

void ExitNotifyScreen(void)
{
	if(screen_id == SCREEN_ID_NOTIFY)
	{
	#ifdef CONFIG_ANIMATION_SUPPORT
		AnimaStop();
	#endif
		k_timer_stop(&notify_timer);
		EnterIdleScreen();
	}
}

void NotifyTimerOutCallBack(struct k_timer *timer_id)
{
	if(screen_id == SCREEN_ID_NOTIFY)
	{
		scr_msg[screen_id].act = SCREEN_ACTION_EXIT;
	}
}

void ShowStringsInRect(uint16_t rect_x, uint16_t rect_y, uint16_t rect_w, uint16_t rect_h, uint8_t *strbuf)
{
	uint16_t x,y,w,h;
	uint16_t offset_w=4,offset_h=4;

	LCD_MeasureString(strbuf, &w, &h);

	if(w > (rect_w-2*offset_w))
	{
		uint8_t line_count,line_no,line_max;
		uint16_t line_h=(h+offset_h);
		uint16_t byte_no=0,text_len;

		line_max = (rect_h-2*offset_h)/line_h;
		line_count = w/(rect_w-2*offset_w) + ((w%(rect_w-offset_w) != 0)? 1 : 0);
		if(line_count > line_max)
			line_count = line_max;

		line_no = 0;
		text_len = strlen(strbuf);
		y = ((rect_h-2*offset_h)-line_count*line_h)/2;
		y += (rect_y+offset_h);
		while(line_no < line_count)
		{
			uint8_t tmpbuf[128] = {0};
			uint8_t i=0;

			tmpbuf[i++] = strbuf[byte_no++];
			LCD_MeasureString(tmpbuf, &w, &h);
			while(w < (rect_w-2*offset_w))
			{
				if(byte_no < text_len)
				{
					tmpbuf[i++] = strbuf[byte_no++];
					LCD_MeasureString(tmpbuf, &w, &h);
				}
				else
				{
					break;
				}
			}

			if(byte_no < text_len)
			{
				//first few rows
				i--;
				byte_no--;
				tmpbuf[i] = 0x00;

				x = (rect_x+offset_w);
				LCD_ShowString(x,y,tmpbuf);

				y += line_h;
				line_no++;
			}
			else
			{
				//last row
				x = (rect_x+offset_w);
				LCD_ShowString(x,y,tmpbuf);
				break;
			}
		}
	}
	else
	{
		x = (w > (rect_w-2*offset_w))? 0 : ((rect_w-2*offset_w)-w)/2;
		y = (h > (rect_h-2*offset_h))? 0 : ((rect_h-2*offset_h)-h)/2;
		x += (rect_x+offset_w);
		y += (rect_y+offset_h);
		LCD_ShowString(x,y,strbuf);				
	}
}

void NotifyShowStrings(uint16_t rect_x, uint16_t rect_y, uint16_t rect_w, uint16_t rect_h, uint32_t *anima_img, uint8_t anima_count, uint8_t *strbuf)
{
	LCD_DrawRectangle(rect_x, rect_y, rect_w, rect_h);
	LCD_Fill(rect_x+1, rect_y+1, rect_w-2, rect_h-2, BLACK);
	ShowStringsInRect(rect_x, rect_y, rect_w, rect_h, strbuf);	
}

void NotifyUpdate(void)
{
	uint16_t x,y,w=0,h=0;
	uint16_t offset_w=8,offset_h=8;

	switch(notify_msg.align)
	{
	case NOTIFY_ALIGN_CENTER:
		if(scr_msg[SCREEN_ID_NOTIFY].para&SCREEN_EVENT_UPDATE_POP_IMG)
		{
			scr_msg[SCREEN_ID_NOTIFY].para &= (~SCREEN_EVENT_UPDATE_POP_IMG);

		#ifdef CONFIG_ANIMATION_SUPPORT
			AnimaStop();
		#endif

		#ifdef LCD_EQTAC175T1371_CO5300
			LCD_Fill(notify_msg.x+2, notify_msg.y+2, notify_msg.w-2, (notify_msg.h*2)/3-2, BLACK);
		#else
			LCD_Fill(notify_msg.x+2, notify_msg.y+2, notify_msg.w-4, (notify_msg.h*2)/3-4, BLACK);
		#endif
			
			if(notify_msg.img != NULL && notify_msg.img_count > 0)
			{	
				LCD_MeasureImage(notify_msg.img[0], &w, &h);
			#ifdef CONFIG_ANIMATION_SUPPORT
				AnimaShow(notify_msg.x+(notify_msg.w-w)/2, notify_msg.y+(notify_msg.h-h)/2, notify_msg.img, notify_msg.img_count, 500, true, NULL);
			#else
				LCD_ShowImage(notify_msg.x+(notify_msg.w-w)/2, notify_msg.y+(notify_msg.h-h)/2, notify_msg.img[0]);
			#endif
			}
		}
		
		if(scr_msg[SCREEN_ID_NOTIFY].para&SCREEN_EVENT_UPDATE_POP_STR)
		{
			scr_msg[SCREEN_ID_NOTIFY].para &= (~SCREEN_EVENT_UPDATE_POP_STR);

		#ifdef LCD_EQTAC175T1371_CO5300
			LCD_Fill(notify_msg.x+2, str_y+2, notify_msg.w-2, str_h-2, BLACK);
		#else
			LCD_Fill(notify_msg.x+2, str_y+2, notify_msg.w-4, str_h-4, BLACK);
		#endif
			LCD_MeasureUniString(notify_msg.text, &w, &h);
			if(w > (str_w-2*offset_w))
			{
				uint8_t line_count,line_no,line_max;
				uint16_t line_h=(h+offset_h);
				uint16_t byte_no=0,text_len;

				line_max = (str_h-2*offset_h)/line_h;
				line_count = w/(str_w-2*offset_w) + ((w%(str_w-offset_w) != 0)? 1 : 0);
				if(line_count > line_max)
					line_count = line_max;

				line_no = 0;
				text_len = mmi_ucs2strlen(notify_msg.text);
				y = ((str_h-2*offset_h)-line_count*line_h)/2;
				y += (str_y+offset_h);
				while(line_no < line_count)
				{
					uint16_t tmpbuf[128] = {0};
					uint8_t i=0;

					tmpbuf[i++] = notify_msg.text[byte_no++];
					LCD_MeasureUniString(tmpbuf, &w, &h);
					while(w < (str_w-2*offset_w))
					{
						if(byte_no < text_len)
						{
							tmpbuf[i++] = notify_msg.text[byte_no++];
							LCD_MeasureUniString(tmpbuf, &w, &h);
						}
						else
							break;
					}

					if(w >= (str_w-2*offset_w))
					{
						i--;
						byte_no--;
						tmpbuf[i] = 0x00;

						LCD_MeasureUniString(tmpbuf, &w, &h);
					#ifdef LANGUAGE_AR_ENABLE	
						if(g_language_r2l)
							x = str_x+(str_w+w)/2;
						else
					#endif		
							x = str_x+(str_w-w)/2;
						LCD_SmartShowUniString(x, y, tmpbuf);
						
						y += line_h;
						line_no++;
					}
					else
					{
						LCD_MeasureUniString(tmpbuf, &w, &h);
					#ifdef LANGUAGE_AR_ENABLE	
						if(g_language_r2l)
							x = str_x+(str_w+w)/2;
						else
					#endif		
							x = str_x+(str_w-w)/2;
						LCD_SmartShowUniString(x, y, tmpbuf);
						break;
					}
				}
			}
			else if(w > 0)
			{
			#ifdef LANGUAGE_AR_ENABLE	
				if(g_language_r2l)
					x = str_x+(str_w+w)/2;
				else
			#endif		
					x = str_x+(str_w-w)/2;
				y = (h > (str_h-2*offset_h))? 0 : ((str_h-2*offset_h)-h)/2;
				
				LCD_SmartShowUniString(x, y, notify_msg.text);			
			}
		}
		break;
		
	case NOTIFY_ALIGN_BOUNDARY:
	#ifdef LCD_EQTAC175T1371_CO5300
		LCD_Fill(notify_msg.x+2, notify_msg.y+2, notify_msg.w-2, (notify_msg.h*2)/3-2, BLACK);
	#else
		LCD_Fill(notify_msg.x+2, notify_msg.y+2, notify_msg.w-4, (notify_msg.h*2)/3-4, BLACK);
	#endif

	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStringRtoLInRect(notify_msg.x+notify_msg.w-offset_w, notify_msg.y+offset_h, (notify_msg.w-2*offset_w), (notify_msg.h-2*offset_h), notify_msg.text);
		else
	#endif		
			LCD_ShowUniStringInRect(notify_msg.x+offset_w, notify_msg.y+offset_h, (notify_msg.w-2*offset_w), (notify_msg.h-2*offset_h), notify_msg.text);
		break;
	}
}

void NotifyShow(void)
{
	uint16_t x,y,w=0,h=0;
	uint16_t offset_w=8,offset_h=8;

	if(((notify_msg.img == NULL) || (notify_msg.img_count == 0)) 
		&& ((notify_msg.x != 0)&&(notify_msg.y != 0)&&((notify_msg.w != LCD_WIDTH))&&(notify_msg.h != LCD_HEIGHT)))
	{
		LCD_DrawRectangle(notify_msg.x, notify_msg.y, notify_msg.w, notify_msg.h);
	}

#ifdef LCD_EQTAC175T1371_CO5300
	LCD_Fill(notify_msg.x+2, notify_msg.y+2, notify_msg.w-2, notify_msg.h-2, BLACK);
#else
	LCD_Fill(notify_msg.x+2, notify_msg.y+2, notify_msg.w-4, notify_msg.h-4, BLACK);
#endif

	str_x = notify_msg.x;
	str_y = notify_msg.y;
	str_w = notify_msg.w;
	str_h = notify_msg.h;
		
	if(notify_msg.img != NULL && notify_msg.img_count > 0)
	{	
		LCD_MeasureImage(notify_msg.img[0], &w, &h);
	#ifdef CONFIG_ANIMATION_SUPPORT
		AnimaShow(notify_msg.x+(notify_msg.w-w)/2, notify_msg.y+(notify_msg.h-h)/2, notify_msg.img, notify_msg.img_count, 500, true, NULL);
	#else
		LCD_ShowImage(notify_msg.x+(notify_msg.w-w)/2, notify_msg.y+(notify_msg.h-h)/2, notify_msg.img[0]);
	#endif

		str_y = notify_msg.y+(notify_msg.h*2/3);
		str_h = notify_msg.h*1/3;
	}

	switch(notify_msg.align)
	{
	case NOTIFY_ALIGN_CENTER:
		LCD_MeasureUniString(notify_msg.text, &w, &h);
		if(w > (str_w-2*offset_w))
		{
			uint8_t line_count,line_no,line_max;
			uint16_t line_h=(h+offset_h);
			uint16_t byte_no=0,text_len;

			line_max = (str_h-2*offset_h)/line_h;
			line_count = w/(str_w-2*offset_w) + ((w%(str_w-offset_w) != 0)? 1 : 0);
			if(line_count > line_max)
				line_count = line_max;

			line_no = 0;
			text_len = mmi_ucs2strlen(notify_msg.text);
			y = ((str_h-2*offset_h)-line_count*line_h)/2;
			y += (str_y+offset_h);
			while(line_no < line_count)
			{
				uint16_t tmpbuf[128] = {0};
				uint8_t i=0;

				tmpbuf[i++] = notify_msg.text[byte_no++];
				LCD_MeasureUniString(tmpbuf, &w, &h);
				while(w < (str_w-2*offset_w))
				{
					if(byte_no < text_len)
					{
						tmpbuf[i++] = notify_msg.text[byte_no++];
						LCD_MeasureUniString(tmpbuf, &w, &h);
					}
					else
						break;
				}

				if(w >= (str_w-2*offset_w))
				{
					i--;
					byte_no--;
					tmpbuf[i] = 0x00;

					LCD_MeasureUniString(tmpbuf, &w, &h);
				#ifdef LANGUAGE_AR_ENABLE	
					if(g_language_r2l)
						x = str_x+(str_w+w)/2;
					else
				#endif		
						x = str_x+(str_w-w)/2;
					LCD_SmartShowUniString(x, y, tmpbuf);
					
					y += line_h;
					line_no++;
				}
				else
				{
					LCD_MeasureUniString(tmpbuf, &w, &h);
				#ifdef LANGUAGE_AR_ENABLE	
					if(g_language_r2l)
						x = str_x+(str_w+w)/2;
					else
				#endif		
						x = str_x+(str_w-w)/2;
					LCD_SmartShowUniString(x, y, tmpbuf);
					break;
				}
			}
		}
		else if(w > 0)
		{
		#ifdef LANGUAGE_AR_ENABLE	
			if(g_language_r2l)
				x = str_x+(str_w+w)/2;
			else
		#endif		
				x = str_x+(str_w-w)/2;
			y = (h > (str_h-2*offset_h))? 0 : ((str_h-2*offset_h)-h)/2;
			
			LCD_SmartShowUniString(x, y, notify_msg.text);
		}
		break;
		
	case NOTIFY_ALIGN_BOUNDARY:
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_ShowUniStringRtoLInRect(notify_msg.x+notify_msg.w-offset_w, str_y+offset_h, (notify_msg.w-2*offset_w), (str_h-2*offset_h), notify_msg.text);
		else
	#endif		
			LCD_ShowUniStringInRect(notify_msg.x+offset_w, str_y+offset_h, (notify_msg.w-2*offset_w), (str_h-2*offset_h), notify_msg.text);
		break;
	}
}

void NotifyScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_NOTIFY].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_NOTIFY].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_NOTIFY].status = SCREEN_STATUS_CREATED;
				
		NotifyShow();
		break;
		
	case SCREEN_ACTION_UPDATE:
		NotifyUpdate();
		break;

	case SCREEN_ACTION_EXIT:
		ExitNotifyScreen();
		break;
	}
	
	scr_msg[SCREEN_ID_NOTIFY].act = SCREEN_ACTION_NO;
}

#ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
void DlShowStatus(void)
{
	uint16_t x,y,w,h;
	uint8_t str_title[128] = {0};

	LCD_Clear(BLACK);

	switch(g_dl_data_type)
	{
	case DL_DATA_IMG:
	#ifdef FONTMAKER_UNICODE_FONT
		switch(global_settings.language)
		{
	 #if defined(LANGUAGE_CN_ENABLE)||defined(LANGUAGE_JA_ENABLE)||defined(LANGUAGE_KR_ENABLE)
	   #ifdef LANGUAGE_CN_ENABLE
	  	case LANGUAGE_CN:
	   #endif
	   #ifdef LANGUAGE_JA_ENABLE
	  	case LANGUAGE_JA:
	   #endif
	   #ifdef LANGUAGE_KR_ENABLE
		case LANGUAGE_KR:
	   #endif
			StrSmartCpyByID(str_title, STR_ID_UI_UPGRADE, 16);
			break;
	 #endif/*LANGUAGE_CN_ENABLE||LANGUAGE_JP_ENABLE||LANGUAGE_KR_ENABLE*/
	  
		default:
			StrSmartCpyByID(str_title, STR_ID_UI_UPGRADE, 18);
			break;
		}
	#else
		strcpy(str_title, "UI Upgrade");
	#endif
		break;
	
	case DL_DATA_FONT:
	#ifdef FONTMAKER_UNICODE_FONT
		switch(global_settings.language)
		{
	 #if defined(LANGUAGE_CN_ENABLE)||defined(LANGUAGE_JA_ENABLE)||defined(LANGUAGE_KR_ENABLE)
	   #ifdef LANGUAGE_CN_ENABLE
	  	case LANGUAGE_CN:
	   #endif
	   #ifdef LANGUAGE_JA_ENABLE
	  	case LANGUAGE_JA:
	   #endif
	   #ifdef LANGUAGE_KR_ENABLE
		case LANGUAGE_KR:
	   #endif		
			StrSmartCpyByID(str_title, STR_ID_FONT_UPGRADE, 16);
			break;
	 #endif/*LANGUAGE_CN_ENABLE||LANGUAGE_JP_ENABLE||LANGUAGE_KR_ENABLE*/
	  
		default:
			StrSmartCpyByID(str_title, STR_ID_FONT_UPGRADE, 18);
			break;
		}
	#else
		strcpy(str_title, "FONT Upgrade");
	#endif
		break;
	
	case DL_DATA_STR:
	#ifdef FONTMAKER_UNICODE_FONT
		switch(global_settings.language)
		{
	 #if defined(LANGUAGE_CN_ENABLE)||defined(LANGUAGE_JA_ENABLE)||defined(LANGUAGE_KR_ENABLE)
	   #ifdef LANGUAGE_CN_ENABLE
	  	case LANGUAGE_CN:
	   #endif
	   #ifdef LANGUAGE_JA_ENABLE
	  	case LANGUAGE_JA:
	   #endif
	   #ifdef LANGUAGE_KR_ENABLE
		case LANGUAGE_KR:
	   #endif		
			StrSmartCpyByID(str_title, STR_ID_STR_UPGRADE, 16);
			break;
	 #endif/*LANGUAGE_CN_ENABLE||LANGUAGE_JP_ENABLE||LANGUAGE_KR_ENABLE*/
	  
		default:
			StrSmartCpyByID(str_title, STR_ID_STR_UPGRADE, 18);
			break;
		}
	#else
		strcpy(str_title, "STR Upgrade");
	#endif
		break;
	
	case DL_DATA_PPG:
	#ifdef FONTMAKER_UNICODE_FONT
		switch(global_settings.language)
		{
	 #if defined(LANGUAGE_CN_ENABLE)||defined(LANGUAGE_JA_ENABLE)||defined(LANGUAGE_KR_ENABLE)
	   #ifdef LANGUAGE_CN_ENABLE
	  	case LANGUAGE_CN:
	   #endif
	   #ifdef LANGUAGE_JA_ENABLE
	  	case LANGUAGE_JA:
	   #endif
	   #ifdef LANGUAGE_KR_ENABLE
		case LANGUAGE_KR:
	   #endif
			StrSmartCpyByID(str_title, STR_ID_PPG_ALGORITHM_UPGRADE, 16);
			break;
	 #endif/*LANGUAGE_CN_ENABLE||LANGUAGE_JP_ENABLE||LANGUAGE_KR_ENABLE*/
	  
		default:
			StrSmartCpyByID(str_title, STR_ID_PPG_ALGORITHM_UPGRADE, 18);
			break;
		}
	#else
		strcpy(str_title, "PPG_AG Upgrade");
	#endif
		break;
	}

#ifdef FONTMAKER_UNICODE_FONT
	LCD_SetFontSize(FONT_SIZE_20);
#else	
	LCD_SetFontSize(FONT_SIZE_16);
#endif

#ifdef FONTMAKER_UNICODE_FONT
	LCD_MeasureUniString((uint16_t*)str_title, &w, &h);
	y = 25;
  #ifdef LANGUAGE_AR_ENABLE	
	if(g_language_r2l)
	{
		x = (w > (DL_NOTIFY_RECT_W-2*DL_NOTIFY_OFFSET_W))? (LCD_WIDTH-1) : (LCD_WIDTH+w)/2;
		LCD_ShowUniStringRtoL(x, y, (uint16_t*)str_title);
	}
	else
  #endif		
	{
		x = (w > (DL_NOTIFY_RECT_W-2*DL_NOTIFY_OFFSET_W))? 0 : (LCD_WIDTH-w)/2;
		LCD_ShowUniString(x, y, (uint16_t*)str_title);
	}
#else
	LCD_MeasureString(str_title, &w, &h);
	x = (w > (DL_NOTIFY_RECT_W-2*DL_NOTIFY_OFFSET_W))? 0 : ((DL_NOTIFY_RECT_W-2*DL_NOTIFY_OFFSET_W)-w)/2;
	x += (DL_NOTIFY_RECT_X+DL_NOTIFY_OFFSET_W);
	y = 25;
	LCD_ShowString(x,y,str_title);
#endif

#ifdef FONTMAKER_UNICODE_FONT
  #ifdef LANGUAGE_AR_ENABLE
	if(g_language_r2l)
		LCD_AdaptShowUniStrRtoLInRect(LCD_WIDTH-DL_NOTIFY_STRING_X, DL_NOTIFY_STRING_Y, DL_NOTIFY_STRING_W, DL_NOTIFY_STRING_H, STR_ID_MAKESURE_BAT_SOC, SHOW_ALIGN_BOUNDARY, SHOW_ALIGN_BOUNDARY);
	else
  #endif
		LCD_AdaptShowUniStrInRect(DL_NOTIFY_STRING_X, DL_NOTIFY_STRING_Y, DL_NOTIFY_STRING_W, DL_NOTIFY_STRING_H, STR_ID_MAKESURE_BAT_SOC, SHOW_ALIGN_BOUNDARY, SHOW_ALIGN_BOUNDARY);
#else
	ShowStringsInRect(DL_NOTIFY_STRING_X, 
					  DL_NOTIFY_STRING_Y, 
					  DL_NOTIFY_STRING_W, 
					  DL_NOTIFY_STRING_H, 
					  "Make sure the battery is more than 80% full or the charger is connected.");
#endif
	LCD_DrawRectangle(DL_NOTIFY_YES_X, DL_NOTIFY_YES_Y, DL_NOTIFY_YES_W, DL_NOTIFY_YES_H);
	LCD_MeasureString("SOS(Y)", &w, &h);
	x = DL_NOTIFY_YES_X+(DL_NOTIFY_YES_W-w)/2;
	y = DL_NOTIFY_YES_Y+(DL_NOTIFY_YES_H-h)/2;	
	LCD_ShowString(x,y,"SOS(Y)");

	LCD_DrawRectangle(DL_NOTIFY_NO_X, DL_NOTIFY_NO_Y, DL_NOTIFY_NO_W, DL_NOTIFY_NO_H);
	LCD_MeasureString("PWR(N)", &w, &h);
	x = DL_NOTIFY_NO_X+(DL_NOTIFY_NO_W-w)/2;
	y = DL_NOTIFY_NO_Y+(DL_NOTIFY_NO_H-h)/2;	
	LCD_ShowString(x,y,"PWR(N)");

	SetRightKeyUpHandler(dl_start);
	SetLeftKeyUpHandler(dl_exit);
#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
	register_touch_event_handle(TP_EVENT_SINGLE_CLICK, DL_NOTIFY_YES_X, DL_NOTIFY_YES_X+DL_NOTIFY_YES_W, DL_NOTIFY_YES_Y, DL_NOTIFY_YES_Y+DL_NOTIFY_YES_H, dl_start);
	register_touch_event_handle(TP_EVENT_SINGLE_CLICK, DL_NOTIFY_NO_X, DL_NOTIFY_NO_X+DL_NOTIFY_NO_W, DL_NOTIFY_NO_Y, DL_NOTIFY_NO_Y+DL_NOTIFY_NO_H, dl_exit);
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_exit);
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_prev);
#endif

	k_timer_stop(&mainmenu_timer);
	k_timer_start(&mainmenu_timer, K_SECONDS(5), K_NO_WAIT);
}

void DlUpdateStatus(void)
{
	uint16_t pro_len;
	uint16_t x,y,w,h;
	uint8_t pro_buf[16] = {0};
	uint8_t strbuf[256] = {0};
	static bool flag = false;
	static uint16_t pro_str_x,pro_str_y;
	
#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
#endif

	switch(get_dl_status())
	{
	case DL_STATUS_PREPARE:
		flag = false;
		break;
		
	case DL_STATUS_LINKING:
		{
			LCD_Fill(DL_NOTIFY_RECT_X+1, DL_NOTIFY_STRING_Y, DL_NOTIFY_RECT_W-1, DL_NOTIFY_RECT_H-(DL_NOTIFY_STRING_Y-DL_NOTIFY_RECT_Y)-1, BLACK);
		#ifdef FONTMAKER_UNICODE_FONT
		  #ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowUniStrRtoLInRect(LCD_WIDTH-DL_NOTIFY_STRING_X, DL_NOTIFY_STRING_Y, DL_NOTIFY_STRING_W, DL_NOTIFY_STRING_H, STR_ID_FW_LINKING_TO_SERVER);
			else
		  #endif
				LCD_ShowUniStrInRect(DL_NOTIFY_STRING_X, DL_NOTIFY_STRING_Y, DL_NOTIFY_STRING_W, DL_NOTIFY_STRING_H, STR_ID_FW_LINKING_TO_SERVER);
		#else
			ShowStringsInRect(DL_NOTIFY_STRING_X,
						  DL_NOTIFY_STRING_Y,
						  DL_NOTIFY_STRING_W,
						  DL_NOTIFY_STRING_H,
						  "Linking to server...");
		#endif
			ClearAllKeyHandler();
		}
		break;
		
	case DL_STATUS_DOWNLOADING:
		if(!flag)
		{
			flag = true;
			
			LCD_Fill(DL_NOTIFY_STRING_X, DL_NOTIFY_STRING_Y, DL_NOTIFY_STRING_W, DL_NOTIFY_STRING_H, BLACK);
		#ifdef FONTMAKER_UNICODE_FONT
		  #ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowUniStrRtoLInRect(LCD_WIDTH-DL_NOTIFY_STRING_X, DL_NOTIFY_STRING_Y, DL_NOTIFY_STRING_W, DL_NOTIFY_STRING_H, STR_ID_FW_DOWNLOADING_DATA);
			else
		  #endif
				LCD_ShowUniStrInRect(DL_NOTIFY_STRING_X, DL_NOTIFY_STRING_Y, DL_NOTIFY_STRING_W, DL_NOTIFY_STRING_H, STR_ID_FW_DOWNLOADING_DATA);
		#else
			ShowStringsInRect(DL_NOTIFY_STRING_X, 
							  DL_NOTIFY_STRING_Y,
							  DL_NOTIFY_STRING_W,
							  40,
							  "Upgrading...");
		#endif
			LCD_DrawRectangle(DL_NOTIFY_PRO_X, DL_NOTIFY_PRO_Y, DL_NOTIFY_PRO_W, DL_NOTIFY_PRO_H);

			sprintf(pro_buf, "%d%%", g_dl_progress);
			memset(strbuf, 0x00, sizeof(strbuf));
			mmi_asc_to_ucs2(strbuf, pro_buf);
			LCD_MeasureUniString((uint16_t*)strbuf, &w, &h);
			pro_str_x = DL_NOTIFY_PRO_NUM_X+(DL_NOTIFY_PRO_NUM_W-w)/2;
			pro_str_y = DL_NOTIFY_PRO_NUM_Y+(DL_NOTIFY_PRO_NUM_H-h)/2;
			LCD_ShowUniString(pro_str_x,pro_str_y, (uint16_t*)strbuf);
		}
		else
		{
			pro_len = (g_dl_progress*DL_NOTIFY_PRO_W)/100;
			LCD_Fill(DL_NOTIFY_PRO_X+1, DL_NOTIFY_PRO_Y+1, pro_len, DL_NOTIFY_PRO_H-1, WHITE);

			sprintf(pro_buf, "%d%%", g_dl_progress);
			memset(strbuf, 0x00, sizeof(strbuf));
			mmi_asc_to_ucs2(strbuf, pro_buf);
			LCD_MeasureUniString((uint16_t*)strbuf, &w, &h);
			LCD_ShowUniString(pro_str_x,pro_str_y, (uint16_t*)strbuf);
		}

		ClearAllKeyHandler();
		break;
		
	case DL_STATUS_FINISHED:
		{
			flag = false;
		
			LCD_Fill(DL_NOTIFY_RECT_X+1, DL_NOTIFY_STRING_Y, DL_NOTIFY_RECT_W-1, DL_NOTIFY_RECT_H-(DL_NOTIFY_STRING_Y-DL_NOTIFY_RECT_Y)-1, BLACK);
			strcpy(strbuf, "Upgraded successfully!");
		#ifdef FONTMAKER_UNICODE_FONT
		  #ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowUniStrRtoLInRect(LCD_WIDTH-DL_NOTIFY_STRING_X, DL_NOTIFY_STRING_Y, DL_NOTIFY_STRING_W, DL_NOTIFY_STRING_H, STR_ID_FW_UPGRADE_SUCCESSFUL);
			else
		  #endif
				LCD_ShowUniStrInRect(DL_NOTIFY_STRING_X, DL_NOTIFY_STRING_Y, DL_NOTIFY_STRING_W, DL_NOTIFY_STRING_H, STR_ID_FW_UPGRADE_SUCCESSFUL);
		#else	
			ShowStringsInRect(DL_NOTIFY_STRING_X,
							  DL_NOTIFY_STRING_Y,
							  DL_NOTIFY_STRING_W,
							  DL_NOTIFY_STRING_H,
							  strbuf);
		#endif
		
			LCD_DrawRectangle(DL_NOTIFY_YES_X, DL_NOTIFY_YES_Y, DL_NOTIFY_YES_W, DL_NOTIFY_YES_H);
			mmi_asc_to_ucs2(strbuf, "SOS(Y)");
			LCD_MeasureUniString((uint16_t*)strbuf, &w, &h);
			x = DL_NOTIFY_YES_X+(DL_NOTIFY_YES_W-w)/2;
			y = DL_NOTIFY_YES_Y+(DL_NOTIFY_YES_H-h)/2;	
			LCD_ShowUniString(x,y,(uint16_t*)strbuf);

			LCD_DrawRectangle(DL_NOTIFY_NO_X, DL_NOTIFY_NO_Y, DL_NOTIFY_NO_W, DL_NOTIFY_NO_H);
			mmi_asc_to_ucs2(strbuf, "PWR(N)");
			LCD_MeasureUniString((uint16_t*)strbuf, &w, &h);
			x = DL_NOTIFY_NO_X+(DL_NOTIFY_NO_W-w)/2;
			y = DL_NOTIFY_NO_Y+(DL_NOTIFY_NO_H-h)/2;	
			LCD_ShowUniString(x,y,(uint16_t*)strbuf);

			SetRightKeyUpHandler(dl_reboot_confirm);
			SetLeftKeyUpHandler(dl_exit);
		#ifdef CONFIG_TOUCH_SUPPORT
			register_touch_event_handle(TP_EVENT_SINGLE_CLICK, DL_NOTIFY_YES_X-10, DL_NOTIFY_YES_X+DL_NOTIFY_YES_W+10, DL_NOTIFY_YES_Y-10, DL_NOTIFY_YES_Y+DL_NOTIFY_YES_H+10, dl_reboot_confirm);
			register_touch_event_handle(TP_EVENT_SINGLE_CLICK, DL_NOTIFY_NO_X-10, DL_NOTIFY_NO_X+DL_NOTIFY_NO_W+10, DL_NOTIFY_NO_Y-10, DL_NOTIFY_NO_Y+DL_NOTIFY_NO_H+10, dl_exit);
			register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_exit);
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_prev);
		#endif	

			k_timer_stop(&mainmenu_timer);
			k_timer_start(&mainmenu_timer, K_SECONDS(5), K_NO_WAIT);
			}
		break;
		
	case DL_STATUS_ERROR:
		{
			flag = false;

			LCD_Fill(DL_NOTIFY_RECT_X+1, DL_NOTIFY_STRING_Y, DL_NOTIFY_RECT_W-1, DL_NOTIFY_RECT_H-(DL_NOTIFY_STRING_Y-DL_NOTIFY_RECT_Y)-1, BLACK);
			strcpy(strbuf, "Upgrade failed, please check the network or server.");
		#ifdef FONTMAKER_UNICODE_FONT
		  #ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
				LCD_ShowUniStrRtoLInRect(LCD_WIDTH-DL_NOTIFY_STRING_X, DL_NOTIFY_STRING_Y, DL_NOTIFY_STRING_W, DL_NOTIFY_STRING_H, STR_ID_FW_UPGRADE_FAILED);
			else
		  #endif
				LCD_ShowUniStrInRect(DL_NOTIFY_STRING_X, DL_NOTIFY_STRING_Y, DL_NOTIFY_STRING_W, DL_NOTIFY_STRING_H, STR_ID_FW_UPGRADE_FAILED);
		#else
			ShowStringsInRect(DL_NOTIFY_STRING_X,
							  DL_NOTIFY_STRING_Y,
							  DL_NOTIFY_STRING_W,
							  DL_NOTIFY_STRING_H,
							  strbuf);
		#endif
			LCD_DrawRectangle((LCD_WIDTH-DL_NOTIFY_YES_W)/2, DL_NOTIFY_YES_Y, DL_NOTIFY_YES_W, DL_NOTIFY_YES_H);
			mmi_asc_to_ucs2(strbuf, "SOS(Y)");
			LCD_MeasureUniString((uint16_t*)strbuf, &w, &h);
			x = (LCD_WIDTH-DL_NOTIFY_YES_W)/2+(DL_NOTIFY_YES_W-w)/2;
			y = DL_NOTIFY_YES_Y+(DL_NOTIFY_YES_H-h)/2;	
			LCD_ShowUniString(x,y,(uint16_t*)strbuf);

			SetLeftKeyUpHandler(dl_reboot_confirm);
			SetRightKeyUpHandler(dl_exit);
		#ifdef CONFIG_TOUCH_SUPPORT
			register_touch_event_handle(TP_EVENT_SINGLE_CLICK, (LCD_WIDTH-DL_NOTIFY_YES_W)/2-10, (LCD_WIDTH-DL_NOTIFY_YES_W)/2+DL_NOTIFY_YES_W+10, DL_NOTIFY_YES_Y-10, DL_NOTIFY_YES_Y+DL_NOTIFY_YES_H+10, dl_exit);
			register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_exit);
			register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_prev);	
		#endif	

			k_timer_stop(&mainmenu_timer);
			k_timer_start(&mainmenu_timer, K_SECONDS(5), K_NO_WAIT);
		}
		break;
		
	case DL_STATUS_MAX:
		flag = false;
		break;
	}
}

void DlScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_DL].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_DL].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_DL].status = SCREEN_STATUS_CREATED;

		DlShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		DlUpdateStatus();

		if(scr_msg[SCREEN_ID_DL].para == SCREEN_EVENT_UPDATE_NO)
			scr_msg[SCREEN_ID_DL].act = SCREEN_ACTION_NO;
		break;
	}
}

#ifdef CONFIG_IMG_DATA_UPDATE
void PrevDlImgScreen(void)
{
	EnterSettings();
}

void ExitDlImgScreen(void)
{
	if((strcmp(g_new_font_ver,g_font_ver) != 0) && (strlen(g_new_font_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
		dl_font_start();
	else if((strcmp(g_new_str_ver,g_str_ver) != 0) && (strlen(g_new_str_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
		dl_str_start();
#if defined(CONFIG_PPG_SUPPORT)
	else if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
		dl_ppg_start();
#endif
	else
		EnterPoweroffScreen();
}
#endif

#ifdef CONFIG_FONT_DATA_UPDATE
void PrevDlFontScreen(void)
{
	if((strcmp(g_new_ui_ver,g_ui_ver) != 0) && (strlen(g_new_ui_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
		dl_img_start();
	else
		EnterSettings();
}

void ExitDlFontScreen(void)
{
	if((strcmp(g_new_str_ver,g_str_ver) != 0) && (strlen(g_new_str_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
		dl_str_start();
#if defined(CONFIG_PPG_SUPPORT)
	else if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
		dl_ppg_start();
	else
#endif
	EnterPoweroffScreen();
}
#endif

#ifdef CONFIG_STR_DATA_UPDATE
void PrevDlStrScreen(void)
{
	if((strcmp(g_new_font_ver,g_font_ver) != 0) && (strlen(g_new_font_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
		dl_font_start();
	else if((strcmp(g_new_ui_ver,g_ui_ver) != 0) && (strlen(g_new_ui_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
		dl_img_start();
	else
		EnterSettings();
}

void ExitDlStrScreen(void)
{
#if defined(CONFIG_PPG_SUPPORT)
	if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
		dl_ppg_start();
	else
#endif
	EnterPoweroffScreen();
}
#endif

#if defined(CONFIG_PPG_DATA_UPDATE)&&defined(CONFIG_PPG_SUPPORT)
void PrevDlPpgScreen(void)
{
	if((strcmp(g_new_str_ver,g_str_ver) != 0) && (strlen(g_new_str_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
		dl_str_start();
	else if((strcmp(g_new_font_ver,g_font_ver) != 0) && (strlen(g_new_font_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
		dl_font_start();
	else if((strcmp(g_new_ui_ver,g_ui_ver) != 0) && (strlen(g_new_ui_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
		dl_img_start();
	else
		EnterSettings();
}

void ExitDlPpgScreen(void)
{
	EnterPoweroffScreen();
}
#endif


void EnterDlScreen(void)
{
#ifdef CONFIG_ANIMATION_SUPPORT	
	AnimaStop();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(TempIsWorking())
		MenuStopTemp();
#endif

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_DL;	
	scr_msg[SCREEN_ID_DL].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_DL].status = SCREEN_STATUS_CREATING;

	k_timer_stop(&mainmenu_timer);
}
#endif/*CONFIG_DATA_DOWNLOAD_SUPPORT*/

#ifdef CONFIG_FOTA_DOWNLOAD
void VerCheckShowStatus(void)
{
	uint16_t x,y,w,h;
	uint16_t tmpbuf[128] = {0};
	uint16_t str_notify[LANGUAGE_MAX][49] = {
											#ifndef FW_FOR_CN
												{0x0043,0x0068,0x0065,0x0063,0x006B,0x0069,0x006E,0x0067,0x0020,0x0076,0x0065,0x0072,0x0073,0x0069,0x006F,0x006E,0x0020,0x0069,0x006E,0x0066,0x006F,0x0072,0x006D,0x0061,0x0074,0x0069,0x006F,0x006E,0x0000},//Checking version information
												{0x00DC,0x0062,0x0065,0x0072,0x0070,0x0072,0x00FC,0x0066,0x0065,0x0020,0x0056,0x0065,0x0072,0x0073,0x0069,0x006F,0x006E,0x0073,0x0069,0x006E,0x0066,0x006F,0x0072,0x006D,0x0061,0x0074,0x0069,0x006F,0x006E,0x0065,0x006E,0x0000},//?berprüfe Versionsinformationen
												{0x0056,0x00E9,0x0072,0x0069,0x0066,0x0069,0x0065,0x007A,0x0020,0x006C,0x0065,0x0073,0x0020,0x0069,0x006E,0x0066,0x006F,0x0072,0x006D,0x0061,0x0074,0x0069,0x006F,0x006E,0x0073,0x0020,0x0064,0x0065,0x0020,0x0076,0x0065,0x0072,0x0073,0x0069,0x006F,0x006E,0x0000},//Vérifiez les informations de version
												{0x0043,0x006F,0x006E,0x0074,0x0072,0x006F,0x006C,0x006C,0x006F,0x0020,0x006C,0x0065,0x0020,0x0069,0x006E,0x0066,0x006F,0x0072,0x006D,0x0061,0x007A,0x0069,0x006F,0x006E,0x0069,0x0020,0x0073,0x0075,0x006C,0x006C,0x0061,0x0020,0x0076,0x0065,0x0072,0x0073,0x0069,0x006F,0x006E,0x0065,0x0000},//Controllo le informazioni sulla versione
												{0x0053,0x0065,0x0020,0x0065,0x0073,0x0074,0x00E1,0x0020,0x0072,0x0065,0x0076,0x0069,0x0073,0x0061,0x006E,0x0064,0x006F,0x0020,0x006C,0x0061,0x0020,0x0069,0x006E,0x0066,0x006F,0x0072,0x006D,0x0061,0x0063,0x0069,0x00F3,0x006E,0x0020,0x0064,0x0065,0x0020,0x006C,0x0061,0x0020,0x0076,0x0065,0x0072,0x0073,0x0069,0x00F3,0x006E,0x0000},//Se está revisando la información de la versión
												{0x0041,0x0020,0x0076,0x0065,0x0072,0x0069,0x0066,0x0069,0x0063,0x0061,0x0072,0x0020,0x0061,0x0073,0x0020,0x0069,0x006E,0x0066,0x006F,0x0072,0x006D,0x0061,0x00E7,0x00F5,0x0065,0x0073,0x0020,0x0064,0x0061,0x0020,0x0076,0x0065,0x0072,0x0073,0x00E3,0x006F,0x0000},//A verificar as informa??es da vers?o
											#else
												{0x6B63,0x5728,0x68C0,0x67E5,0x7248,0x672C,0x4FE1,0x606F,0x0000},//正在检查版本信息
												{0x0043,0x0068,0x0065,0x0063,0x006B,0x0069,0x006E,0x0067,0x0020,0x0076,0x0065,0x0072,0x0073,0x0069,0x006F,0x006E,0x0020,0x0069,0x006E,0x0066,0x006F,0x0072,0x006D,0x0061,0x0074,0x0069,0x006F,0x006E,0x0000},//Checking version information
											#endif
											};

	LCD_Clear(BLACK);
	LCD_SetFontBgColor(BLACK);
	
	mmi_ucs2smartcpy((uint8_t*)tmpbuf, (uint8_t*)str_notify[global_settings.language], MENU_NOTIFY_STR_MAX);
	LCD_MeasureUniString(tmpbuf, &w, &h);
	LCD_ShowUniString((LCD_WIDTH-w)/2, (LCD_HEIGHT-h)/2, tmpbuf);

	ClearAllKeyHandler();
#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
#endif
}

void VerCheckUpdateStatus(void)
{
}

void VerCheckScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_VER_CHECK].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_VER_CHECK].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_VER_CHECK].status = SCREEN_STATUS_CREATED;

		VerCheckShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		VerCheckUpdateStatus();

		if(scr_msg[SCREEN_ID_VER_CHECK].para == SCREEN_EVENT_UPDATE_NO)
			scr_msg[SCREEN_ID_VER_CHECK].act = SCREEN_ACTION_NO;
		break;
	}
}

void EnterVerCheckScreen(void)
{
	if(screen_id == SCREEN_ID_VER_CHECK)
		return;
	
	k_timer_stop(&mainmenu_timer);

#ifdef CONFIG_ANIMATION_SUPPORT	
	AnimaStop();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(TempIsWorking())
		MenuStopTemp();
#endif

	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_VER_CHECK;	
	scr_msg[SCREEN_ID_VER_CHECK].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_VER_CHECK].status = SCREEN_STATUS_CREATING;

	VerCheckStart();
}

void FOTAShowStatus(void)
{
	LCD_Clear(BLACK);
#ifdef FONTMAKER_UNICODE_FONT
	LCD_SetFontSize(FONT_SIZE_28);
#else	
	LCD_SetFontSize(FONT_SIZE_16);
#endif
	
	LCD_ShowImage(FOTA_LOGO_X, FOTA_LOGO_Y, IMG_ID_OTA_READR);
	LCD_ShowImage(FOTA_YES_X, FOTA_YES_Y, IMG_ID_OTA_SEL_Y);
	LCD_ShowImage(FOTA_NO_X, FOTA_NO_Y, IMG_ID_OTA_SEL_N);

#ifdef FONTMAKER_UNICODE_FONT
  #ifdef LANGUAGE_AR_ENABLE
	if(g_language_r2l)
		LCD_AdaptShowUniStrRtoLInRect(FOTA_START_STR_X+FOTA_START_STR_W, FOTA_START_STR_Y, FOTA_START_STR_W, FOTA_START_STR_H, STR_ID_FW_UPGRADE_REQUEST, SHOW_ALIGN_CENTER, SHOW_ALIGN_CENTER);
	else
  #endif		
		LCD_AdaptShowUniStrInRect(FOTA_START_STR_X, FOTA_START_STR_Y, FOTA_START_STR_W, FOTA_START_STR_H, STR_ID_FW_UPGRADE_REQUEST, SHOW_ALIGN_CENTER, SHOW_ALIGN_CENTER);
#endif

	SetRightKeyUpHandler(fota_excu);
	SetLeftKeyUpHandler(fota_exit);
#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
	register_touch_event_handle(TP_EVENT_SINGLE_CLICK, FOTA_YES_X-30, FOTA_YES_X+FOTA_YES_W+30, FOTA_YES_Y-30, FOTA_YES_Y+FOTA_YES_H+30, fota_excu);
	register_touch_event_handle(TP_EVENT_SINGLE_CLICK, FOTA_NO_X-30, FOTA_NO_X+FOTA_NO_W+30, FOTA_NO_Y-30, FOTA_NO_Y+FOTA_NO_H+30, fota_exit);
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, fota_exit);	
#endif
}

void FOTAUpdateStatus(void)
{
	uint16_t pro_len;
	uint16_t x,y,w,h;
	uint8_t pro_buf[16] = {0};
	uint16_t tmpbuf[128] = {0};
	static bool flag = false;
	static uint16_t pro_str_x,pro_str_y;

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
#endif	
	switch(get_fota_status())
	{
	case FOTA_STATUS_PREPARE:
		flag = false;
		break;
		
	case FOTA_STATUS_LINKING:
		{
			LCD_Fill(0, FOTA_YES_Y, LCD_WIDTH, FOTA_YES_H, BLACK);
			LCD_Fill(0, FOTA_START_STR_Y, LCD_WIDTH, FOTA_START_STR_H, BLACK);

		#ifdef FONTMAKER_UNICODE_FONT
			LCD_MeasureUniStr(STR_ID_FW_LINKING_TO_SERVER, &w, &h);
		  #ifdef LANGUAGE_AR_ENABLE	
			if(g_language_r2l)
			{
				x = w > FOTA_START_STR_W ? (FOTA_START_STR_X+FOTA_START_STR_W) : (LCD_WIDTH+w)/2;
				LCD_ShowUniStrRtoL(x, FOTA_START_STR_Y+(FOTA_START_STR_H-h)/2, STR_ID_FW_LINKING_TO_SERVER);
			}
			else
		  #endif
			{
				x = w > FOTA_START_STR_W ? FOTA_START_STR_X : (LCD_WIDTH-w)/2;
				LCD_ShowUniStr(x, FOTA_START_STR_Y+(FOTA_START_STR_H-h)/2, STR_ID_FW_LINKING_TO_SERVER);
		  	}
		#endif

			ClearAllKeyHandler();
		}
		break;
		
	case FOTA_STATUS_DOWNLOADING:
		if(!flag)
		{
			flag = true;
			
			LCD_Fill(0, FOTA_START_STR_Y, LCD_WIDTH, FOTA_START_STR_H, BLACK);

			LCD_MeasureUniStr(STR_ID_FW_DOWNLOADING_DATA, &w, &h);
		#ifdef LANGUAGE_AR_ENABLE
			if(g_language_r2l)
			{
				x = w > FOTA_START_STR_W ? (FOTA_START_STR_X+FOTA_START_STR_W) : (LCD_WIDTH+w)/2;
				LCD_ShowUniStrRtoL(x, FOTA_START_STR_Y+(FOTA_START_STR_H-h)/2, STR_ID_FW_DOWNLOADING_DATA);
			}
			else
		#endif
			{
				x = w > FOTA_START_STR_W ? FOTA_START_STR_X : (LCD_WIDTH-w)/2;
				LCD_ShowUniStr(x, FOTA_START_STR_Y+(FOTA_START_STR_H-h)/2, STR_ID_FW_DOWNLOADING_DATA);
			}
					
			LCD_DrawRectangle(FOTA_PROGRESS_X, FOTA_PROGRESS_Y, FOTA_PROGRESS_W, FOTA_PROGRESS_H);
			LCD_Fill(FOTA_PROGRESS_X+1, FOTA_PROGRESS_Y+1, FOTA_PROGRESS_W-1, FOTA_PROGRESS_H-1, BLACK);
			
			sprintf(pro_buf, "%d%%", g_fota_progress);
		#ifdef FONTMAKER_UNICODE_FONT
			LCD_SetFontSize(FONT_SIZE_20);
		#else	
			LCD_SetFontSize(FONT_SIZE_16);
		#endif
			memset(tmpbuf, 0x0000, sizeof(tmpbuf));
			mmi_asc_to_ucs2(tmpbuf, pro_buf);
			LCD_MeasureUniString(tmpbuf, &w, &h);
			pro_str_x = FOTA_PRO_NUM_X+(FOTA_PRO_NUM_W-w)/2;
			pro_str_y = FOTA_PRO_NUM_Y+(FOTA_PRO_NUM_H-h)/2;
			LCD_ShowUniString(pro_str_x,pro_str_y, tmpbuf);
		}
		else
		{
			pro_len = (g_fota_progress*FOTA_PROGRESS_W)/100;
			LCD_Fill(FOTA_PROGRESS_X+1, FOTA_PROGRESS_Y+1, pro_len, FOTA_PROGRESS_H-1, WHITE);
			
			sprintf(pro_buf, "%d%%", g_fota_progress);
			memset(tmpbuf, 0x0000, sizeof(tmpbuf));
			mmi_asc_to_ucs2(tmpbuf, pro_buf);
			LCD_MeasureUniString(tmpbuf, &w, &h);
			LCD_ShowUniString(pro_str_x, pro_str_y, tmpbuf);
		}		

		ClearAllKeyHandler();
		break;
		
	case FOTA_STATUS_FINISHED:
		{
			static bool is_finished = false;
			
			flag = false;

			if(!is_finished)
			{
				is_finished = true;
				
				LCD_Clear(BLACK);
				LCD_ShowImage(FOTA_FINISH_ICON_X, FOTA_FINISH_ICON_Y, IMG_ID_OTA_SUCCESS);
				
			#ifdef FONTMAKER_UNICODE_FONT
				LCD_MeasureUniStr(STR_ID_FW_UPGRADE_SUCCESSFUL, &w, &h);
			  #ifdef LANGUAGE_AR_ENABLE	
				if(g_language_r2l)
				{
					x = w > FOTA_START_STR_W ? (FOTA_START_STR_X+FOTA_START_STR_W) : (LCD_WIDTH+w)/2;
					LCD_ShowUniStrRtoL(x, FOTA_FINISH_STR_Y+(FOTA_FINISH_STR_H-h)/2, STR_ID_FW_UPGRADE_SUCCESSFUL);
				}
				else
			  #endif
				{
					x = w > FOTA_START_STR_W ? FOTA_START_STR_X : (LCD_WIDTH-w)/2;
					LCD_ShowUniStr(x, FOTA_FINISH_STR_Y+(FOTA_FINISH_STR_H-h)/2, STR_ID_FW_UPGRADE_SUCCESSFUL);
			  	}
			#endif
			
				SetLeftKeyUpHandler(fota_reboot_confirm);
				SetRightKeyUpHandler(fota_reboot_confirm);
			#ifdef CONFIG_TOUCH_SUPPORT
				register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 0, LCD_WIDTH, 0, LCD_HEIGHT, fota_reboot_confirm);
			#endif
			}
		}
		break;
		
	case FOTA_STATUS_ERROR:
		{
			flag = false;

			LCD_Clear(BLACK);
			LCD_ShowImage(FOTA_FAIL_ICON_X, FOTA_FAIL_ICON_Y, IMG_ID_OTA_FAIL);
			
		#ifdef FONTMAKER_UNICODE_FONT
			LCD_MeasureUniStr(STR_ID_FW_UPGRADE_FAILED, &w, &h);
		  #ifdef LANGUAGE_AR_ENABLE	
			if(g_language_r2l)
			{
				x = w > FOTA_START_STR_W ? (FOTA_START_STR_X+FOTA_START_STR_W) : (LCD_WIDTH+w)/2;
				LCD_ShowUniStrRtoL(x, FOTA_FAIL_STR_Y+(FOTA_FAIL_STR_H-h)/2, STR_ID_FW_UPGRADE_FAILED);
			}
			else
		  #endif
			{
				x = w > FOTA_START_STR_W ? FOTA_START_STR_X : (LCD_WIDTH-w)/2;
				LCD_ShowUniStr(x, FOTA_FAIL_STR_Y+(FOTA_FAIL_STR_H-h)/2, STR_ID_FW_UPGRADE_FAILED);
		  	}
		#endif	

			SetLeftKeyUpHandler(fota_exit);
			SetRightKeyUpHandler(fota_exit);
		#ifdef CONFIG_TOUCH_SUPPORT
			register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 0, LCD_WIDTH, 0, LCD_HEIGHT, fota_exit);
		#endif	
		}
		break;
		
	case FOTA_STATUS_MAX:
		flag = false;
		break;
	}
}

void FOTAScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_FOTA].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_FOTA].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_FOTA].status = SCREEN_STATUS_CREATED;

		FOTAShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		FOTAUpdateStatus();

		if(scr_msg[SCREEN_ID_FOTA].para == SCREEN_EVENT_UPDATE_NO)
			scr_msg[SCREEN_ID_FOTA].act = SCREEN_ACTION_NO;
		break;
	}
}

void ExitFOTAScreen(void)
{
#ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
	if((strcmp(g_new_ui_ver,g_ui_ver) != 0) && (strlen(g_new_ui_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		dl_img_start();
	}
	else if((strcmp(g_new_font_ver,g_font_ver) != 0) && (strlen(g_new_font_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		dl_font_start();
	}
	else if((strcmp(g_new_str_ver,g_str_ver) != 0) && (strlen(g_new_str_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		dl_str_start();
	}
  #if defined(CONFIG_PPG_DATA_UPDATE)&&defined(CONFIG_PPG_SUPPORT)
	else if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
	{
		dl_ppg_start();
	}
  #endif
	else
#endif
	{
		EntryIdleScr();
	}
}

void EnterFOTAScreen(void)
{
	if(screen_id == SCREEN_ID_FOTA)
		return;

#ifdef CONFIG_ANIMATION_SUPPORT	
	AnimaStop();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(TempIsWorking())
		MenuStopTemp();
#endif

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_FOTA;	
	scr_msg[SCREEN_ID_FOTA].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_FOTA].status = SCREEN_STATUS_CREATING;

	k_timer_stop(&mainmenu_timer);
}
#endif/*CONFIG_FOTA_DOWNLOAD*/

#ifdef CONFIG_IMU_SUPPORT
#ifdef CONFIG_FALL_DETECT_SUPPORT
void FallUpdateStatus(void)
{
}

void FallShowStatus(void)
{
	LCD_Clear(BLACK);

	LCD_ShowImage(FALL_ICON_X, FALL_ICON_Y, IMG_ID_FALL_ICON);
	LCD_ShowImage(FALL_YES_X, FALL_YES_Y, IMG_ID_FALL_YES);
	LCD_ShowImage(FALL_NO_X, FALL_NO_Y, IMG_ID_FALL_NO);

	FallChangrStatus();

	//SetLeftKeyUpHandler(FallAlarmConfirm);
	//SetRightKeyUpHandler(FallAlarmCancel);

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
	register_touch_event_handle(TP_EVENT_SINGLE_CLICK, FALL_YES_X, FALL_YES_X+FALL_YES_W, FALL_YES_Y, FALL_YES_Y+FALL_YES_H, FallAlarmConfirm);
	register_touch_event_handle(TP_EVENT_SINGLE_CLICK, FALL_NO_X, FALL_NO_X+FALL_NO_W, FALL_NO_Y, FALL_NO_Y+FALL_NO_H, FallAlarmCancel);
#endif

}

void FallScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_FALL].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_FALL].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_FALL].status = SCREEN_STATUS_CREATED;

		FallShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		FallUpdateStatus();
		break;
	}
	
	scr_msg[SCREEN_ID_FALL].act = SCREEN_ACTION_NO;
}

void EnterFallScreen(void)
{
	if(screen_id == SCREEN_ID_FALL)
		return;

	k_timer_stop(&notify_timer);
	k_timer_stop(&mainmenu_timer);

#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif
#ifdef NB_SIGNAL_TEST
	if(gps_is_working())
		MenuStopGPS();
#endif	
#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		PPGStopCheck();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(IsInTempScreen()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
#ifdef CONFIG_SYNC_SUPPORT
	if(SyncIsRunning())
		SyncDataStop();
#endif

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_FALL;	
	scr_msg[SCREEN_ID_FALL].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_FALL].status = SCREEN_STATUS_CREATING;

	ClearAllKeyHandler();
}
#endif/*CONFIG_FALL_DETECT_SUPPORT*/

#ifdef CONFIG_SLEEP_SUPPORT
void SleepUpdateStatus(void)
{
	uint16_t total_sleep,deep_sleep,light_sleep;
	uint32_t img_big_num[10] = {IMG_FONT_38_NUM_0_ADDR,IMG_FONT_38_NUM_1_ADDR,IMG_FONT_38_NUM_2_ADDR,IMG_FONT_38_NUM_3_ADDR,IMG_FONT_38_NUM_4_ADDR,
								IMG_FONT_38_NUM_5_ADDR,IMG_FONT_38_NUM_6_ADDR,IMG_FONT_38_NUM_7_ADDR,IMG_FONT_38_NUM_8_ADDR,IMG_FONT_38_NUM_9_ADDR};
	uint32_t img_num[10] = {IMG_FONT_24_NUM_0_ADDR,IMG_FONT_24_NUM_1_ADDR,IMG_FONT_24_NUM_2_ADDR,IMG_FONT_24_NUM_3_ADDR,IMG_FONT_24_NUM_4_ADDR,
							IMG_FONT_24_NUM_5_ADDR,IMG_FONT_24_NUM_6_ADDR,IMG_FONT_24_NUM_7_ADDR,IMG_FONT_24_NUM_8_ADDR,IMG_FONT_24_NUM_9_ADDR};
				

	GetSleepTimeData(&deep_sleep, &light_sleep);
	total_sleep = deep_sleep+light_sleep;

	LCD_ShowImage(SLEEP_TOTAL_STR_HR_X+0*SLEEP_TOTAL_NUM_W, SLEEP_TOTAL_STR_HR_Y, img_big_num[(total_sleep/60)/10]);
	LCD_ShowImage(SLEEP_TOTAL_STR_HR_X+1*SLEEP_TOTAL_NUM_W, SLEEP_TOTAL_STR_HR_Y, img_big_num[(total_sleep/60)%10]);
	LCD_ShowImage(SLEEP_TOTAL_STR_MIN_X+0*SLEEP_TOTAL_NUM_W, SLEEP_TOTAL_STR_MIN_Y, img_big_num[(total_sleep%60)/10]);
	LCD_ShowImage(SLEEP_TOTAL_STR_MIN_X+1*SLEEP_TOTAL_NUM_W, SLEEP_TOTAL_STR_MIN_Y, img_big_num[(total_sleep%60)%10]);

	LCD_ShowImage(SLEEP_DEEP_STR_HR_X+0*SLEEP_DEEP_NUM_W, SLEEP_DEEP_STR_HR_Y, img_num[(deep_sleep/60)/10]);
	LCD_ShowImage(SLEEP_DEEP_STR_HR_X+1*SLEEP_DEEP_NUM_W, SLEEP_DEEP_STR_HR_Y, img_num[(deep_sleep/60)%10]);
	LCD_ShowImage(SLEEP_DEEP_STR_MIN_X+0*SLEEP_DEEP_NUM_W, SLEEP_DEEP_STR_MIN_Y, img_num[(deep_sleep%60)/10]);
	LCD_ShowImage(SLEEP_DEEP_STR_MIN_X+1*SLEEP_DEEP_NUM_W, SLEEP_DEEP_STR_MIN_Y, img_num[(deep_sleep%60)%10]);

	LCD_ShowImage(SLEEP_LIGHT_STR_HR_X+0*SLEEP_LIGHT_NUM_W, SLEEP_LIGHT_STR_HR_Y, img_num[(light_sleep/60)/10]);
	LCD_ShowImage(SLEEP_LIGHT_STR_HR_X+1*SLEEP_LIGHT_NUM_W, SLEEP_LIGHT_STR_HR_Y, img_num[(light_sleep/60)%10]);
	LCD_ShowImage(SLEEP_LIGHT_STR_MIN_X+0*SLEEP_LIGHT_NUM_W, SLEEP_LIGHT_STR_MIN_Y, img_num[(light_sleep%60)/10]);
	LCD_ShowImage(SLEEP_LIGHT_STR_MIN_X+1*SLEEP_LIGHT_NUM_W, SLEEP_LIGHT_STR_MIN_Y, img_num[(light_sleep%60)%10]);
}

void SleepShowStatus(void)
{
	uint8_t strbuf[64] = {0};
	uint16_t total_sleep,deep_sleep,light_sleep;
	uint32_t img_big_num[10] = {IMG_FONT_38_NUM_0_ADDR,IMG_FONT_38_NUM_1_ADDR,IMG_FONT_38_NUM_2_ADDR,IMG_FONT_38_NUM_3_ADDR,IMG_FONT_38_NUM_4_ADDR,
							IMG_FONT_38_NUM_5_ADDR,IMG_FONT_38_NUM_6_ADDR,IMG_FONT_38_NUM_7_ADDR,IMG_FONT_38_NUM_8_ADDR,IMG_FONT_38_NUM_9_ADDR};
	uint32_t img_num[10] = {IMG_FONT_24_NUM_0_ADDR,IMG_FONT_24_NUM_1_ADDR,IMG_FONT_24_NUM_2_ADDR,IMG_FONT_24_NUM_3_ADDR,IMG_FONT_24_NUM_4_ADDR,
							IMG_FONT_24_NUM_5_ADDR,IMG_FONT_24_NUM_6_ADDR,IMG_FONT_24_NUM_7_ADDR,IMG_FONT_24_NUM_8_ADDR,IMG_FONT_24_NUM_9_ADDR};

	
	LCD_ShowImage(SLEEP_TOTAL_ICON_X, SLEEP_TOTAL_ICON_Y, IMG_SLEEP_ANI_3_ADDR);
	LCD_ShowImage(SLEEP_TOTAL_UNIT_HR_X, SLEEP_TOTAL_UNIT_HR_Y, IMG_SLEEP_BIG_H_ADDR);
	LCD_ShowImage(SLEEP_TOTAL_UNIT_MIN_X, SLEEP_TOTAL_UNIT_MIN_Y, IMG_SLEEP_BIG_M_ADDR);
	LCD_ShowImage(SLEEP_SEP_LINE_X, SLEEP_SEP_LINE_Y, IMG_SLEEP_LINE_ADDR);

	LCD_ShowImage(SLEEP_DEEP_ICON_X, SLEEP_DEEP_ICON_Y, IMG_SLEEP_BEGIN_ADDR);
	LCD_ShowImage(SLEEP_LIGHT_ICON_X, SLEEP_LIGHT_ICON_Y, IMG_SLEEP_END_ADDR);

	GetSleepTimeData(&deep_sleep, &light_sleep);
	total_sleep = deep_sleep+light_sleep;

	LCD_ShowImage(SLEEP_TOTAL_STR_HR_X+0*SLEEP_TOTAL_NUM_W, SLEEP_TOTAL_STR_HR_Y, img_big_num[(total_sleep/60)/10]);
	LCD_ShowImage(SLEEP_TOTAL_STR_HR_X+1*SLEEP_TOTAL_NUM_W, SLEEP_TOTAL_STR_HR_Y, img_big_num[(total_sleep/60)%10]);
	LCD_ShowImage(SLEEP_TOTAL_STR_MIN_X+0*SLEEP_TOTAL_NUM_W, SLEEP_TOTAL_STR_MIN_Y, img_big_num[(total_sleep%60)/10]);
	LCD_ShowImage(SLEEP_TOTAL_STR_MIN_X+1*SLEEP_TOTAL_NUM_W, SLEEP_TOTAL_STR_MIN_Y, img_big_num[(total_sleep%60)%10]);

	LCD_ShowImage(SLEEP_DEEP_STR_HR_X+0*SLEEP_DEEP_NUM_W, SLEEP_DEEP_STR_HR_Y, img_num[(deep_sleep/60)/10]);
	LCD_ShowImage(SLEEP_DEEP_STR_HR_X+1*SLEEP_DEEP_NUM_W, SLEEP_DEEP_STR_HR_Y, img_num[(deep_sleep/60)%10]);
	LCD_ShowImage(SLEEP_DEEP_UNIT_HR_X, SLEEP_DEEP_STR_HR_Y, IMG_FONT_24_COLON_ADDR);
	LCD_ShowImage(SLEEP_DEEP_STR_MIN_X+0*SLEEP_DEEP_NUM_W, SLEEP_DEEP_STR_MIN_Y, img_num[(deep_sleep%60)/10]);
	LCD_ShowImage(SLEEP_DEEP_STR_MIN_X+1*SLEEP_DEEP_NUM_W, SLEEP_DEEP_STR_MIN_Y, img_num[(deep_sleep%60)%10]);

	LCD_ShowImage(SLEEP_LIGHT_STR_HR_X+0*SLEEP_LIGHT_NUM_W, SLEEP_LIGHT_STR_HR_Y, img_num[(light_sleep/60)/10]);
	LCD_ShowImage(SLEEP_LIGHT_STR_HR_X+1*SLEEP_LIGHT_NUM_W, SLEEP_LIGHT_STR_HR_Y, img_num[(light_sleep/60)%10]);
	LCD_ShowImage(SLEEP_LIGHT_UNIT_HR_X, SLEEP_DEEP_STR_HR_Y, IMG_FONT_24_COLON_ADDR);
	LCD_ShowImage(SLEEP_LIGHT_STR_MIN_X+0*SLEEP_LIGHT_NUM_W, SLEEP_LIGHT_STR_MIN_Y, img_num[(light_sleep%60)/10]);
	LCD_ShowImage(SLEEP_LIGHT_STR_MIN_X+1*SLEEP_LIGHT_NUM_W, SLEEP_LIGHT_STR_MIN_Y, img_num[(light_sleep%60)%10]);
}

void SleepScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_SLEEP].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_SLEEP].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_SLEEP].status = SCREEN_STATUS_CREATED;

		LCD_Clear(BLACK);
		IdleShowSignal();
		IdleShowNetMode();
		IdleShowBatSoc();
		SleepShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		if(scr_msg[SCREEN_ID_SLEEP].para&SCREEN_EVENT_UPDATE_SIG)
		{
			scr_msg[SCREEN_ID_SLEEP].para &= (~SCREEN_EVENT_UPDATE_SIG);
			IdleShowSignal();
		}
		if(scr_msg[SCREEN_ID_SLEEP].para&SCREEN_EVENT_UPDATE_NET_MODE)
		{
			scr_msg[SCREEN_ID_SLEEP].para &= (~SCREEN_EVENT_UPDATE_NET_MODE);	
			IdleShowNetMode();
		}
		if(scr_msg[SCREEN_ID_SLEEP].para&SCREEN_EVENT_UPDATE_BAT)
		{
			scr_msg[SCREEN_ID_SLEEP].para &= (~SCREEN_EVENT_UPDATE_BAT);
			IdleUpdateBatSoc();
		}
		if(scr_msg[SCREEN_ID_SLEEP].para&SCREEN_EVENT_UPDATE_SLEEP)
		{
			scr_msg[SCREEN_ID_SLEEP].para &= (~SCREEN_EVENT_UPDATE_SLEEP);
			SleepUpdateStatus();
		}
		break;
	}
	
	scr_msg[SCREEN_ID_SLEEP].act = SCREEN_ACTION_NO;
}

void ExitSleepScreen(void)
{
	EnterIdleScreen();
}

void EnterSleepScreen(void)
{
	if(screen_id == SCREEN_ID_SLEEP)
		return;

	k_timer_stop(&mainmenu_timer);
#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(IsInTempScreen()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();
#endif
	LCD_Set_BL_Mode(LCD_BL_AUTO);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_SLEEP;	
	scr_msg[SCREEN_ID_SLEEP].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_SLEEP].status = SCREEN_STATUS_CREATING;

#if defined(CONFIG_PPG_SUPPORT)
	SetLeftKeyUpHandler(EnterHRScreen);
#elif defined(CONFIG_TEMP_SUPPORT)
	SetLeftKeyUpHandler(EnterTempScreen);
#elif defined(CONFIG_SYNC_SUPPORT)
	SetLeftKeyUpHandler(EnterSyncDataScreen);
#else
	SetLeftKeyUpHandler(EnterSettings);
#endif
	SetRightKeyUpHandler(ExitSleepScreen);

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();

 #ifdef CONFIG_PPG_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterHRScreen);
 #elif defined(CONFIG_TEMP_SUPPORT)
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterTempScreen);
 #elif defined(CONFIG_SYNC_SUPPORT)
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSyncDataScreen); 
 #else
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSettings);
 #endif

 #ifdef CONFIG_STEP_SUPPORT
 	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterStepsScreen);
 #else
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterIdleScreen);
 #endif
#endif
}
#endif/*CONFIG_SLEEP_SUPPORT*/

#ifdef CONFIG_STEP_SUPPORT
void StepUpdateStatus(void)
{
	uint8_t i,language,count=1;
	uint16_t steps,calorie,distance;
	uint16_t total_sleep,deep_sleep,light_sleep;
	uint32_t divisor=10;
	uint32_t img_num[10] = {IMG_FONT_38_NUM_0_ADDR,IMG_FONT_38_NUM_1_ADDR,IMG_FONT_38_NUM_2_ADDR,IMG_FONT_38_NUM_3_ADDR,IMG_FONT_38_NUM_4_ADDR,
								IMG_FONT_38_NUM_5_ADDR,IMG_FONT_38_NUM_6_ADDR,IMG_FONT_38_NUM_7_ADDR,IMG_FONT_38_NUM_8_ADDR,IMG_FONT_38_NUM_9_ADDR};
	uint32_t step_unit[2] = {IMG_STEP_UNIT_EN_ICON_ADDR,IMG_STEP_UNIT_CN_ICON_ADDR};
	uint32_t cal_uint[2] = {IMG_STEP_KCAL_EN_ADDR,IMG_STEP_KCAL_CN_ADDR};
	
	switch(global_settings.language)
	{
  #ifdef LANGUAGE_CN_ENABLE
	case LANGUAGE_CN:
		language = 1;
		break;
  #endif

	default:
		language = 0;
		break;
	}

	LCD_Fill(IMU_STEP_STR_X, IMU_STEP_STR_Y, LCD_WIDTH, IMU_NUM_H, BLACK);
	LCD_Fill(IMU_CAL_STR_X, IMU_CAL_STR_X, LCD_WIDTH, IMU_NUM_H, BLACK);

	GetSportData(&steps, &calorie, &distance);

	divisor = 10000;
	for(i=0;i<5;i++)
	{
		LCD_ShowImage(IMU_STEP_STR_X+(IMU_STEP_STR_W-5*IMU_NUM_W)/2+i*IMU_NUM_W, IMU_STEP_STR_Y, img_num[steps/divisor]);
		steps = steps%divisor;
		divisor = divisor/10;
	}
	LCD_ShowImage(IMU_STEP_STR_X+(IMU_STEP_STR_W-5*IMU_NUM_W)/2+5*IMU_NUM_W, IMU_STEP_UNIT_Y, step_unit[language]);

	divisor = 1000;
	for(i=0;i<4;i++)
	{
		LCD_ShowImage(IMU_CAL_STR_X+(IMU_CAL_STR_W-4*IMU_NUM_W)/2+i*IMU_NUM_W, IMU_CAL_STR_Y, img_num[calorie/divisor]);
		calorie = calorie%divisor;
		divisor = divisor/10;
	}
	LCD_ShowImage(IMU_CAL_STR_X+(IMU_CAL_STR_W-4*IMU_NUM_W)/2+i*IMU_NUM_W, IMU_CAL_UNIT_Y, cal_uint[language]);

#ifdef CONFIG_SLEEP_SUPPORT
	GetSleepTimeData(&deep_sleep, &light_sleep);
	total_sleep = deep_sleep+light_sleep;
	LCD_ShowImage(IMU_SLEEP_H_STR_X+0*IMU_NUM_W, IMU_SLEEP_H_STR_Y, img_num[(total_sleep/60)/10]);
	LCD_ShowImage(IMU_SLEEP_H_STR_X+1*IMU_NUM_W, IMU_SLEEP_H_STR_Y, img_num[(total_sleep/60)%10]);
	LCD_ShowImage(IMU_SLEEP_M_STR_X+0*IMU_NUM_W, IMU_SLEEP_M_STR_Y, img_num[(total_sleep%60)/10]);
	LCD_ShowImage(IMU_SLEEP_M_STR_X+1*IMU_NUM_W, IMU_SLEEP_M_STR_Y, img_num[(total_sleep%60)%10]);
#endif	
}

void StepShowStatus(void)
{
	uint8_t i,language,count=1;
	uint16_t steps,calorie,distance;
	uint16_t total_sleep,deep_sleep,light_sleep;
	uint32_t divisor=10;
	uint32_t img_num[10] = {IMG_FONT_38_NUM_0_ADDR,IMG_FONT_38_NUM_1_ADDR,IMG_FONT_38_NUM_2_ADDR,IMG_FONT_38_NUM_3_ADDR,IMG_FONT_38_NUM_4_ADDR,
							IMG_FONT_38_NUM_5_ADDR,IMG_FONT_38_NUM_6_ADDR,IMG_FONT_38_NUM_7_ADDR,IMG_FONT_38_NUM_8_ADDR,IMG_FONT_38_NUM_9_ADDR};
	uint32_t step_unit[2] = {IMG_STEP_UNIT_EN_ICON_ADDR,IMG_STEP_UNIT_CN_ICON_ADDR};
	uint32_t cal_uint[2] = {IMG_STEP_KCAL_EN_ADDR,IMG_STEP_KCAL_CN_ADDR};
	
	switch(global_settings.language)
	{
  #ifdef LANGUAGE_CN_ENABLE
	case LANGUAGE_CN:
		language = 1;
		break;
  #endif

	default:
		language = 0;
		break;
	}

	LCD_ShowImage(IMU_STEP_ICON_X, IMU_STEP_ICON_Y, IMG_STEP_STEP_ICON_ADDR);
	LCD_ShowImage(IMU_CAL_ICON_X, IMU_CAL_ICON_Y, IMG_STEP_CAL_ICON_ADDR);

	GetSportData(&steps, &calorie, &distance);

	divisor = 10000;
	for(i=0;i<5;i++)
	{
		LCD_ShowImage(IMU_STEP_STR_X+(IMU_STEP_STR_W-5*IMU_NUM_W)/2+i*IMU_NUM_W, IMU_STEP_STR_Y, img_num[steps/divisor]);
		steps = steps%divisor;
		divisor = divisor/10;
	}
	LCD_ShowImage(IMU_STEP_STR_X+(IMU_STEP_STR_W-5*IMU_NUM_W)/2+5*IMU_NUM_W, IMU_STEP_UNIT_Y, step_unit[language]);

	divisor = 1000;
	for(i=0;i<4;i++)
	{
		LCD_ShowImage(IMU_CAL_STR_X+(IMU_CAL_STR_W-4*IMU_NUM_W)/2+i*IMU_NUM_W, IMU_CAL_STR_Y, img_num[calorie/divisor]);
		calorie = calorie%divisor;
		divisor = divisor/10;
	}
	LCD_ShowImage(IMU_CAL_STR_X+(IMU_CAL_STR_W-4*IMU_NUM_W)/2+i*IMU_NUM_W, IMU_CAL_UNIT_Y, cal_uint[language]);

#ifdef CONFIG_SLEEP_SUPPORT
	GetSleepTimeData(&deep_sleep, &light_sleep);
	total_sleep = deep_sleep+light_sleep;

	LCD_ShowImage(IMU_SLEEP_ICON_X, IMU_SLEEP_ICON_Y, IMG_STEP_SLEEP_ICON_ADDR);
	LCD_ShowImage(IMU_SLEEP_H_UNIT_X, IMU_SLEEP_H_UNIT_Y, IMG_SLEEP_H_ADDR);
	LCD_ShowImage(IMU_SLEEP_M_UNIT_X, IMU_SLEEP_M_UNIT_Y, IMG_SLEEP_M_ADDR);
	
	LCD_ShowImage(IMU_SLEEP_H_STR_X+0*IMU_NUM_W, IMU_SLEEP_H_STR_Y, img_num[(total_sleep/60)/10]);
	LCD_ShowImage(IMU_SLEEP_H_STR_X+1*IMU_NUM_W, IMU_SLEEP_H_STR_Y, img_num[(total_sleep/60)%10]);
	LCD_ShowImage(IMU_SLEEP_M_STR_X+0*IMU_NUM_W, IMU_SLEEP_M_STR_Y, img_num[(total_sleep%60)/10]);
	LCD_ShowImage(IMU_SLEEP_M_STR_X+1*IMU_NUM_W, IMU_SLEEP_M_STR_Y, img_num[(total_sleep%60)%10]);
#endif	
}

void StepsScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_STEPS].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_STEPS].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_STEPS].status = SCREEN_STATUS_CREATED;

		LCD_Clear(BLACK);
		IdleShowSignal();
		IdleShowNetMode();
		IdleShowBatSoc();
		StepShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		if(scr_msg[SCREEN_ID_STEPS].para&SCREEN_EVENT_UPDATE_SIG)
		{
			scr_msg[SCREEN_ID_STEPS].para &= (~SCREEN_EVENT_UPDATE_SIG);
			IdleShowSignal();
		}
		if(scr_msg[SCREEN_ID_STEPS].para&SCREEN_EVENT_UPDATE_NET_MODE)
		{
			scr_msg[SCREEN_ID_STEPS].para &= (~SCREEN_EVENT_UPDATE_NET_MODE);	
			IdleShowNetMode();
		}
		if(scr_msg[SCREEN_ID_STEPS].para&SCREEN_EVENT_UPDATE_BAT)
		{
			scr_msg[SCREEN_ID_STEPS].para &= (~SCREEN_EVENT_UPDATE_BAT);
			IdleUpdateBatSoc();
		}
		if(scr_msg[SCREEN_ID_STEPS].para&SCREEN_EVENT_UPDATE_SPORT)
		{
			scr_msg[SCREEN_ID_STEPS].para &= (~SCREEN_EVENT_UPDATE_SPORT);
			StepUpdateStatus();
		}
		break;
	}
	
	scr_msg[SCREEN_ID_STEPS].act = SCREEN_ACTION_NO;
}

void ExitStepsScreen(void)
{
	EnterIdleScreen();
}

void EnterStepsScreen(void)
{
	if(screen_id == SCREEN_ID_STEPS)
		return;

	k_timer_stop(&mainmenu_timer);
#ifdef CONFIG_ANIMATION_SUPPORT	
	AnimaStop();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(IsInTempScreen()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();
#endif
	LCD_Set_BL_Mode(LCD_BL_AUTO);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_STEPS;	
	scr_msg[SCREEN_ID_STEPS].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_STEPS].status = SCREEN_STATUS_CREATING;

#ifdef CONFIG_SLEEP_SUPPORT
	SetLeftKeyUpHandler(EnterSleepScreen);
#elif defined(CONFIG_PPG_SUPPORT)
	SetLeftKeyUpHandler(EnterHRScreen);
#elif defined(CONFIG_TEMP_SUPPORT)
	SetLeftKeyUpHandler(EnterTempScreen);
#elif defined(CONFIG_SYNC_SUPPORT)
	SetLeftKeyUpHandler(EnterSyncDataScreen);
#else
	SetLeftKeyUpHandler(EnterSettings);
#endif
	SetRightKeyUpHandler(ExitStepsScreen);
	
#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();

  #ifdef CONFIG_SLEEP_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSleepScreen);
  #elif defined(CONFIG_PPG_SUPPORT)
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterHRScreen);
  #elif defined(CONFIG_TEMP_SUPPORT)
  	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterTempScreen);
  #elif defined(CONFIG_SYNC_SUPPORT)
  	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSyncDataScreen);
  #else
  	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSettings);
  #endif
  
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterIdleScreen);
#endif
}
#endif/*CONFIG_STEP_SUPPORT*/
#endif/*CONFIG_IMU_SUPPORT*/

void EntryIdleScr(void)
{
	entry_idle_flag = true;
}

void EnterIdleScreen(void)
{
	if(screen_id == SCREEN_ID_IDLE)
		return;

	k_timer_stop(&notify_timer);
	k_timer_stop(&mainmenu_timer);
#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif
#ifdef NB_SIGNAL_TEST
	MenuStopNB();
	if(gps_is_working())
		MenuStopGPS();
#endif	
#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		PPGStopCheck();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(IsInTempScreen()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
#ifdef CONFIG_SYNC_SUPPORT
	if(SyncIsRunning())
		SyncDataStop();
#endif
#ifdef CONFIG_ECG_SUPPORT
	if(IsInEcgScreen())
		ExitEcgScreen();
#endif

	LCD_Set_BL_Mode(LCD_BL_AUTO);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_IDLE;
	scr_msg[SCREEN_ID_IDLE].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_IDLE].status = SCREEN_STATUS_CREATING;

	ClearAllKeyHandler();
#ifdef NB_SIGNAL_TEST
	SetLeftKeyUpHandler(EnterNBTestScreen);
#else
 #if 0	//xb add 2026-02-06
 	SetLeftKeyUpHandler(EnterMainMenu);
 #else	
  #if defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
   #ifdef CONFIG_STEP_SUPPORT
	SetLeftKeyUpHandler(EnterStepsScreen);
   #elif defined(CONFIG_SLEEP_SUPPORT)
	SetLeftKeyUpHandler(EnterSleepScreen);
   #endif
  #elif defined(CONFIG_PPG_SUPPORT)
	SetLeftKeyUpHandler(EnterHRScreen);
  #elif defined(CONFIG_TEMP_SUPPORT)
	SetLeftKeyUpHandler(EnterTempScreen);
  #elif defined(CONFIG_SYNC_SUPPORT)
	SetLeftKeyUpHandler(EnterSyncDataScreen);
  #else
	SetLeftKeyUpHandler(EnterSettings);
  #endif
 #endif 
#endif

	SetRightKeyLongPressHandler(SOSTrigger);
	SetRightKeyUpHandler(EnterIdleScreen);

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();

 #ifdef NB_SIGNAL_TEST
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterNBTestScreen);
 	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterPoweroffScreen);
 #else
  #if 0	//xb add 2026-02-06
  	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterMainMenu);
  #else
  #if defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
   #ifdef CONFIG_STEP_SUPPORT
   	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterStepsScreen);
   #elif defined(CONFIG_SLEEP_SUPPORT)
   	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSleepScreen);
   #endif
  #elif defined(CONFIG_PPG_SUPPORT)
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterHRScreen);
  #elif defined(CONFIG_TEMP_SUPPORT)
  	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterTempScreen);
  #elif defined(CONFIG_SYNC_SUPPORT)
  	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSyncDataScreen);
  #else
  	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSettings);
  #endif
  #endif
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterPoweroffScreen);
 #endif
#endif	
}

#ifdef CONFIG_ALARM_SUPPORT
void AlarmScreenProcess(void)
{
	uint16_t rect_x,rect_y,rect_w=180,rect_h=80;
	uint16_t x,y,w,h;
	uint8_t notify[20] = "Alarm Notify!";

	switch(scr_msg[SCREEN_ID_ALARM].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_ALARM].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_ALARM].status = SCREEN_STATUS_CREATED;
				
		rect_x = (LCD_WIDTH-rect_w)/2;
		rect_y = (LCD_HEIGHT-rect_h)/2;
		
		LCD_DrawRectangle(rect_x, rect_y, rect_w, rect_h);
		LCD_Fill(rect_x+1, rect_y+1, rect_w-2, rect_h-2, BLACK);

	#ifdef FONTMAKER_UNICODE_FONT
		LCD_SetFontSize(FONT_SIZE_28);
	#else
		LCD_SetFontSize(FONT_SIZE_24);
	#endif
		LCD_MeasureString(notify,&w,&h);
		x = (w > rect_w)? 0 : (rect_w-w)/2;
		y = (h > rect_h)? 0 : (rect_h-h)/2;
		x += rect_x;
		y += rect_y;
		LCD_ShowString(x,y,notify);
		break;
		
	case SCREEN_ACTION_UPDATE:
		break;
	}
	
	scr_msg[SCREEN_ID_ALARM].act = SCREEN_ACTION_NO;
}

void EnterAlarmScreen(void)
{
	if(screen_id == SCREEN_ID_ALARM)
		return;

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_ALARM;	
	scr_msg[SCREEN_ID_ALARM].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_ALARM].status = SCREEN_STATUS_CREATING;	

	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	SetLeftKeyUpHandler(AlarmRemindStop);
	SetRightKeyUpHandler(AlarmRemindStop);
	
#ifdef CONFIG_TOUCH_SUPPORT
	register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 0, LCD_WIDTH, 0, LCD_HEIGHT, AlarmRemindStop);
	register_touch_event_handle(TP_EVENT_MOVING_UP, 0, LCD_WIDTH, 0, LCD_HEIGHT, AlarmRemindStop);
	register_touch_event_handle(TP_EVENT_MOVING_DOWN, 0, LCD_WIDTH, 0, LCD_HEIGHT, AlarmRemindStop);
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, AlarmRemindStop);
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, AlarmRemindStop);
#endif	
}

void FindDeviceScreenProcess(void)
{
	uint16_t rect_x,rect_y,rect_w=180,rect_h=80;
	uint16_t x,y,w,h;
	uint8_t notify[128] = "Find Device!";

	switch(scr_msg[SCREEN_ID_FIND_DEVICE].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_FIND_DEVICE].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_FIND_DEVICE].status = SCREEN_STATUS_CREATED;
				
		rect_x = (LCD_WIDTH-rect_w)/2;
		rect_y = (LCD_HEIGHT-rect_h)/2;
		
		LCD_DrawRectangle(rect_x, rect_y, rect_w, rect_h);
		LCD_Fill(rect_x+1, rect_y+1, rect_w-2, rect_h-2, BLACK);

	#ifdef FONTMAKER_UNICODE_FONT
		LCD_SetFontSize(FONT_SIZE_28);
	#else	
		LCD_SetFontSize(FONT_SIZE_24);
	#endif
		LCD_MeasureString(notify,&w,&h);
		x = (w > rect_w)? 0 : (rect_w-w)/2;
		y = (h > rect_h)? 0 : (rect_h-h)/2;
		x += rect_x;
		y += rect_y;
		LCD_ShowString(x,y,notify);
		break;
		
	case SCREEN_ACTION_UPDATE:
		break;
	}
	
	scr_msg[SCREEN_ID_FIND_DEVICE].act = SCREEN_ACTION_NO;
}

void EnterFindDeviceScreen(void)
{
	if(screen_id == SCREEN_ID_FIND_DEVICE)
		return;

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_FIND_DEVICE;	
	scr_msg[SCREEN_ID_FIND_DEVICE].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_FIND_DEVICE].status = SCREEN_STATUS_CREATING;

	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	SetLeftKeyUpHandler(FindDeviceStop);
	SetRightKeyUpHandler(FindDeviceStop);
	
#ifdef CONFIG_TOUCH_SUPPORT
	register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 0, LCD_WIDTH, 0, LCD_HEIGHT, FindDeviceStop);
	register_touch_event_handle(TP_EVENT_MOVING_UP, 0, LCD_WIDTH, 0, LCD_HEIGHT, FindDeviceStop);
	register_touch_event_handle(TP_EVENT_MOVING_DOWN, 0, LCD_WIDTH, 0, LCD_HEIGHT, FindDeviceStop);
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, FindDeviceStop);
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, FindDeviceStop);
#endif	
}
#endif

#ifdef NB_SIGNAL_TEST
#ifdef CONFIG_WIFI_SUPPORT
void TestWifiUpdateInfor(void)
{
	uint8_t tmpbuf[512] = {0};
	
	LCD_Fill((LCD_WIDTH-300)/2, 80, 300, 300, BLACK);

#ifdef FONTMAKER_UNICODE_FONT
	LCD_SetFontSize(FONT_SIZE_36);
	mmi_asc_to_ucs2(tmpbuf, wifi_test_info);
	LCD_ShowUniStringInRect((LCD_WIDTH-300)/2, 80, 300, 300, (uint16_t*)tmpbuf);
#else	
	LCD_SetFontSize(FONT_SIZE_16);
	LCD_ShowStringInRect((LCD_WIDTH-300)/2, 80, 300, 300, wifi_test_info);
#endif
}

void TestWifiShowInfor(void)
{
	uint16_t x,y,w,h;
	uint8_t strbuf[512] = {0};
	
	LCD_Clear(BLACK);
	
#ifdef FONTMAKER_UNICODE_FONT
	LCD_SetFontSize(FONT_SIZE_36);
	mmi_asc_to_ucs2(strbuf, "WIFI TESTING");
	LCD_MeasureUniString((uint16_t*)strbuf, &w, &h);
	LCD_ShowUniString((LCD_WIDTH-w)/2, 20, (uint16_t*)strbuf);
	mmi_asc_to_ucs2(strbuf, "Wifi Starting...");
	LCD_ShowUniStringInRect((LCD_WIDTH-300)/2, 80, 300, 300, (uint16_t*)strbuf);
#else	
	LCD_SetFontSize(FONT_SIZE_16);
	strcpy(strbuf, "WIFI TESTING");
	LCD_MeasureString(strbuf, &w, &h);
	LCD_ShowString((LCD_WIDTH-w)/2, 20, strbuf);
	LCD_ShowStringInRect((LCD_WIDTH-300)/2, 80, 300, 300, "Wifi Starting...");
#endif
}

void TestWifiScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_WIFI_TEST].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_WIFI_TEST].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_WIFI_TEST].status = SCREEN_STATUS_CREATED;

		TestWifiShowInfor();
		break;
		
	case SCREEN_ACTION_UPDATE:
		TestWifiUpdateInfor();
		break;
	}
	
	scr_msg[SCREEN_ID_WIFI_TEST].act = SCREEN_ACTION_NO;
}

void ExitWifiTestScreen(void)
{
	k_timer_stop(&mainmenu_timer);
	
	if(wifi_is_working())
		MenuStopWifi();

	LCD_Set_BL_Mode(LCD_BL_AUTO);
	
	EnterIdleScreen();
}

void EnterWifiTestScreen(void)
{
	if(screen_id == SCREEN_ID_WIFI_TEST)
		return;

	k_timer_stop(&mainmenu_timer);
	k_timer_start(&mainmenu_timer, K_SECONDS(3), K_NO_WAIT);
#ifdef CONFIG_PPG_SUPPORT
	PPGStopCheck();
#endif
	if(gps_is_working())
		MenuStopGPS();
	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_WIFI_TEST;	
	scr_msg[SCREEN_ID_WIFI_TEST].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_WIFI_TEST].status = SCREEN_STATUS_CREATING;

	SetLeftKeyUpHandler(EnterSettings);
	SetRightKeyUpHandler(ExitWifiTestScreen);
#ifdef CONFIG_TOUCH_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSettings);
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterGPSTestScreen);
#endif	
}
#endif

void TestGPSUpdateInfor(void)
{
	uint8_t tmpbuf[512] = {0};
	
	LCD_Fill((LCD_WIDTH-300)/2, 80, 300, 300, BLACK);

#ifdef FONTMAKER_UNICODE_FONT
	LCD_SetFontSize(FONT_SIZE_36);
	mmi_asc_to_ucs2(tmpbuf, gps_test_info);
	LCD_ShowUniStringInRect((LCD_WIDTH-300)/2, 80, 300, 300, (uint16_t*)tmpbuf);
#else	
	LCD_SetFontSize(FONT_SIZE_16);
	LCD_ShowStringInRect((LCD_WIDTH-300)/2, 80, 300, 300, gps_test_info);
#endif
}

void TestGPSShowInfor(void)
{
	uint16_t x,y,w,h;
	uint8_t strbuf[512] = {0};
	
	LCD_Clear(BLACK);
	
#ifdef FONTMAKER_UNICODE_FONT
	LCD_SetFontSize(FONT_SIZE_36);
	mmi_asc_to_ucs2(strbuf, "GPS TESTING");
	LCD_MeasureUniString((uint16_t*)strbuf, &w, &h);
	LCD_ShowUniString((LCD_WIDTH-w)/2, 20, (uint16_t*)strbuf);
	mmi_asc_to_ucs2(strbuf, "GPS Starting...");
	LCD_ShowUniStringInRect((LCD_WIDTH-300)/2, 80, 300, 300, (uint16_t*)strbuf);
#else	
	LCD_SetFontSize(FONT_SIZE_16);
	strcpy(strbuf, "GPS TESTING");
	LCD_MeasureString(strbuf, &w, &h);
	LCD_ShowString((LCD_WIDTH-w)/2, 20, strbuf);
	LCD_ShowStringInRect((LCD_WIDTH-300)/2, 80, 300, 300, "GPS Starting...");
#endif
}

void TestGPSScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_GPS_TEST].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_GPS_TEST].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_GPS_TEST].status = SCREEN_STATUS_CREATED;

		TestGPSShowInfor();
		break;
		
	case SCREEN_ACTION_UPDATE:
		TestGPSUpdateInfor();
		break;
	}
	
	scr_msg[SCREEN_ID_GPS_TEST].act = SCREEN_ACTION_NO;
}

void ExitGPSTestScreen(void)
{
	k_timer_stop(&mainmenu_timer);
	
	if(gps_is_working())
		MenuStopGPS();

	LCD_Set_BL_Mode(LCD_BL_AUTO);
	
	EnterIdleScreen();
}

void EnterGPSTestScreen(void)
{
	if(screen_id == SCREEN_ID_GPS_TEST)
		return;

	k_timer_stop(&mainmenu_timer);
	k_timer_start(&mainmenu_timer, K_SECONDS(3), K_NO_WAIT);
#ifdef CONFIG_PPG_SUPPORT
	PPGStopCheck();
#endif
#ifdef CONFIG_WIFI_SUPPORT
	if(wifi_is_working())
		MenuStopWifi();
#endif
	MenuStopNB();
	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_GPS_TEST; 
	scr_msg[SCREEN_ID_GPS_TEST].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_GPS_TEST].status = SCREEN_STATUS_CREATING;

#ifdef CONFIG_WIFI_SUPPORT
	SetLeftKeyUpHandler(EnterWifiTestScreen);
#else
	SetLeftKeyUpHandler(EnterSettings);
#endif
	SetRightKeyUpHandler(ExitGPSTestScreen);

#ifdef CONFIG_TOUCH_SUPPORT
  #ifdef CONFIG_WIFI_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterWifiTestScreen);
  #else
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSettings);
  #endif
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterNBTestScreen);
#endif	
}

void TestNBUpdateINfor(void)
{
	uint8_t tmpbuf[512] = {0};

	if(screen_id == SCREEN_ID_NB_TEST)
	{
		LCD_Fill((LCD_WIDTH-300)/2, 80, 300, 300, BLACK);
	#ifdef FONTMAKER_UNICODE_FONT
		mmi_asc_to_ucs2(tmpbuf, nb_test_info);
		LCD_ShowUniStringInRect((LCD_WIDTH-300)/2, 80, 300, 300, (uint16_t*)tmpbuf);	
	#else
		LCD_ShowStringInRect((LCD_WIDTH-300)/2, 80, 300, 300, nb_test_info);
	#endif
	}
}

void TestNBShowInfor(void)
{
	uint16_t x,y,w,h;
	uint8_t strbuf[512] = {0};
	
	LCD_Clear(BLACK);
#ifdef FONTMAKER_UNICODE_FONT
	LCD_SetFontSize(FONT_SIZE_36);
	mmi_asc_to_ucs2(strbuf, "NB-IoT TESTING");
	LCD_MeasureUniString((uint16_t*)strbuf, &w, &h);
	LCD_ShowUniString((LCD_WIDTH-w)/2, 20, (uint16_t*)strbuf);
	mmi_asc_to_ucs2(strbuf, nb_test_info);
	LCD_ShowUniStringInRect((LCD_WIDTH-300)/2, 80, 300, 300, (uint16_t*)strbuf);	
#else	
	LCD_SetFontSize(FONT_SIZE_16);
	strcpy(strbuf, "NB-IoT TESTING");
	LCD_MeasureString(strbuf, &w, &h);
	LCD_ShowString((LCD_WIDTH-w)/2, 20, strbuf);
	LCD_ShowStringInRect((LCD_WIDTH-300)/2, 80, 300, 300, nb_test_info);
#endif
}

void TestNBScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_NB_TEST].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_NB_TEST].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_NB_TEST].status = SCREEN_STATUS_CREATED;

		TestNBShowInfor();
		break;
		
	case SCREEN_ACTION_UPDATE:
		TestNBUpdateINfor();
		break;
	}
	
	scr_msg[SCREEN_ID_NB_TEST].act = SCREEN_ACTION_NO;
}

void ExitNBTestScreen(void)
{
	k_timer_stop(&mainmenu_timer);
	
	LCD_Set_BL_Mode(LCD_BL_AUTO);

	EnterIdleScreen();
}

void EnterNBTestScreen(void)
{
	if(screen_id == SCREEN_ID_NB_TEST)
		return;

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_NB_TEST;	
	scr_msg[SCREEN_ID_NB_TEST].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_NB_TEST].status = SCREEN_STATUS_CREATING;

	k_timer_stop(&mainmenu_timer);
	k_timer_start(&mainmenu_timer, K_SECONDS(5), K_NO_WAIT);
	
	if(gps_is_working())
		MenuStopGPS();

	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	SetLeftKeyUpHandler(EnterGPSTestScreen);
	SetRightKeyUpHandler(ExitNBTestScreen);
#ifdef CONFIG_TOUCH_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterGPSTestScreen);
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterIdleScreen);
#endif
}
#endif

#ifdef CONFIG_QRCODE_SUPPORT
void DeviceShowQRStatus(void)
{
	uint8_t buf[512] = {0};
	uint16_t w,h;
	uint16_t title[5] = {0x8BBE,0x5907,0x4FE1,0x606F,0x0000};//设备信息
	
	LCD_Clear(BLACK);
	LCD_SetFontSize(FONT_SIZE_20);
	LCD_MeasureUniString(title, &w, &h);
	LCD_ShowUniString((LCD_WIDTH-w)/2, 10, title);

	//IMEI
	strcpy(buf, "IMEI:");
	strcat(buf, g_imei);
	strcat(buf, ",");
	//IMSI
	strcat(buf, "IMSI:");
	strcat(buf, g_imsi);
	strcat(buf, ",");
	//ICCID
	strcat(buf, "ICCID:");
	strcat(buf, g_iccid);
	strcat(buf, ",");
	//nrf9160 FW ver
	strcat(buf, "MCU:");
	strcat(buf, g_fw_version);
	strcat(buf, ",");
#ifdef CONFIG_WIFI_SUPPORT	
	//WIFI FW ver
	strcat(buf, "WIFI:");
	strcat(buf, g_wifi_ver);
	strcat(buf, ",");
	//WIFI Mac
	strcat(buf, "WIFI MAC:");
	strcat(buf, g_wifi_mac_addr);
	strcat(buf, ",");
#else
	//WIFI FW ver
	strcat(buf, "WIFI:");
	strcat(buf, "NO");
	strcat(buf, ",");
	//WIFI Mac
	strcat(buf, "WIFI MAC:");
	strcat(buf, "NO");
	strcat(buf, ",");
#endif
	//BLE FW ver
	strcat(buf, "BLE:");
	strcat(buf, &g_ble_app_ver[15]);
	strcat(buf, ",");
	//BLE Mac
	strcat(buf, "BLE MAC:");
	strcat(buf, g_ble_mac_addr);
	strcat(buf, ",");
#ifdef CONFIG_PPG_SUPPORT	
	//PPG ver
	strcat(buf, "PPG:");
	strcat(buf, g_ppg_ver);
	strcat(buf, ",");
#else
	//PPG ver
	strcat(buf, "PPG:");
	strcat(buf, "NO");
	strcat(buf, ",");
#endif
	//Modem ver
	strcat(buf, "MODEM:");
	strcat(buf, &g_modem[12]);

	show_QR_code(strlen(buf), buf);
}

void DeviceScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_DEVICE_INFOR].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_DEVICE_INFOR].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_DEVICE_INFOR].status = SCREEN_STATUS_CREATED;

		DeviceShowQRStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		DeviceShowQRStatus();
		break;
	}
	
	scr_msg[SCREEN_ID_DEVICE_INFOR].act = SCREEN_ACTION_NO;
}

void ExitDeviceScreen(void)
{
	LCD_Set_BL_Mode(LCD_BL_AUTO);
	EnterIdleScreen();
}

void EnterDeviceScreen(void)
{
	if(screen_id == SCREEN_ID_DEVICE_INFOR)
		return;

	k_timer_stop(&mainmenu_timer);

#ifdef CONFIG_ANIMATION_SUPPORT	
	AnimaStop();
#endif
#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		PPGStopCheck();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(IsInTempScreen()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
#ifdef CONFIG_SYNC_SUPPORT
	if(SyncIsRunning())
		SyncDataStop();
#endif

	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_DEVICE_INFOR;	
	scr_msg[SCREEN_ID_DEVICE_INFOR].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_DEVICE_INFOR].status = SCREEN_STATUS_CREATING;

	SetLeftKeyUpHandler(ExitDeviceScreen);
	SetRightKeyUpHandler(ExitDeviceScreen);
#ifdef CONFIG_TOUCH_SUPPORT
	register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 0, LCD_WIDTH, 0, LCD_HEIGHT, ExitDeviceScreen);
	register_touch_event_handle(TP_EVENT_MOVING_UP, 0, LCD_WIDTH, 0, LCD_HEIGHT, ExitDeviceScreen);
	register_touch_event_handle(TP_EVENT_MOVING_DOWN, 0, LCD_WIDTH, 0, LCD_HEIGHT, ExitDeviceScreen);
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, ExitDeviceScreen);
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, ExitDeviceScreen);
#endif	
}

#ifdef CONFIG_FACTORY_TEST_SUPPORT
void FTSmtResultsShowQRStatus(void)
{
	uint8_t sper[] = {'\r','\n',0x00};
	uint8_t buf[512] = {0};
	uint16_t w,h;
	uint16_t title[8] = {0x0053,0x004D,0x0054,0x6D4B,0x8BD5,0x7ED3,0x679C,0x0000};//SMT测试结果
	
	LCD_Clear(BLACK);
	LCD_SetFontSize(FONT_SIZE_20);
	LCD_MeasureUniString(title, &w, &h);
	LCD_ShowUniString((LCD_WIDTH-w)/2, 10, title);

	//IMEI
	strcpy(buf, "IMEI:");
	strcat(buf, g_imei);
	strcat(buf, sper);
	
	//Current Test
	strcat(buf, "CUR:");
	switch(ft_smt_results.cur_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//KEY Test
	strcat(buf, "KEY:");
	switch(ft_smt_results.key_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//LCD Test
	strcat(buf, "LCD:");
	switch(ft_smt_results.lcd_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//Touch Test
	strcat(buf, "TOUCH:");
	switch(ft_smt_results.touch_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//Temp Test
	strcat(buf, "TEMP:");
	switch(ft_smt_results.temp_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//IMU Test
	strcat(buf, "IMU:");
	switch(ft_smt_results.imu_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//Flash Test
	strcat(buf, "FLASH:");
	switch(ft_smt_results.flash_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//SIM Test
	strcat(buf, "SIM:");
	switch(ft_smt_results.sim_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//BLE Test
	strcat(buf, "BLE:");
	switch(ft_smt_results.ble_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
#ifdef CONFIG_PPG_SUPPORT	
	//PPG Test
	strcat(buf, "PPG:");
	switch(ft_smt_results.ppg_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
#endif

	//Charging Test
	strcat(buf, "CHRGE:");
	switch(ft_smt_results.pmu_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//Vibration Test
	strcat(buf, "VIB:");
	switch(ft_smt_results.vib_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
#ifdef CONFIG_WIFI_SUPPORT	
	//WIFI Test
	strcat(buf, "WIFI:");
	switch(ft_smt_results.wifi_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
#endif

	//Network Test
	strcat(buf, "NET:");
	switch(ft_smt_results.net_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//GPS Test
	strcat(buf, "GPS:");
	switch(ft_smt_results.gps_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);

	show_QR_code(strlen(buf), buf);
}

void FTSmtResultsScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_FT_SMT_RESULT_INFOR].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_FT_SMT_RESULT_INFOR].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_FT_SMT_RESULT_INFOR].status = SCREEN_STATUS_CREATED;

		FTSmtResultsShowQRStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		FTSmtResultsShowQRStatus();
		break;
	}
	
	scr_msg[SCREEN_ID_FT_SMT_RESULT_INFOR].act = SCREEN_ACTION_NO;
}

void ExitFTSmtResultsScreen(void)
{
	LCD_Set_BL_Mode(LCD_BL_AUTO);
	EnterIdleScreen();
}

void EnterFTSmtResultsScreen(void)
{
	if(screen_id == SCREEN_ID_FT_SMT_RESULT_INFOR)
		return;

	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_FT_SMT_RESULT_INFOR;	
	scr_msg[SCREEN_ID_FT_SMT_RESULT_INFOR].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_FT_SMT_RESULT_INFOR].status = SCREEN_STATUS_CREATING;

	ClearAllKeyHandler();
	SetLeftKeyUpHandler(EnterFTAssemResultsScreen);
	SetRightKeyUpHandler(ExitFTSmtResultsScreen);
#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterFTAssemResultsScreen);
	if(!FactorySmtTestFinished())
		register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterFactorySmtTest);
	else
		register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterFactoryAssemTest);
#endif
}

void FTAssemResultsShowQRStatus(void)
{
	uint8_t sper[] = {'\r','\n',0x00};
	uint8_t buf[512] = {0};
	uint16_t w,h;
	uint16_t title[7] = {0x7EC4,0x88C5,0x6D4B,0x8BD5,0x7ED3,0x679C,0x0000};//测试结果
	
	LCD_Clear(BLACK);
	LCD_SetFontSize(FONT_SIZE_20);
	LCD_MeasureUniString(title, &w, &h);
	LCD_ShowUniString((LCD_WIDTH-w)/2, 10, title);

	//IMEI
	strcpy(buf, "IMEI:");
	strcat(buf, g_imei);
	strcat(buf, sper);
	
	//KEY Test
	strcat(buf, "KEY:");
	switch(ft_assem_results.key_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//LCD Test
	strcat(buf, "LCD:");
	switch(ft_assem_results.lcd_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//Touch Test
	strcat(buf, "TOUCH:");
	switch(ft_assem_results.touch_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//Temp Test
	strcat(buf, "TEMP:");
	switch(ft_assem_results.temp_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//IMU Test
	strcat(buf, "IMU:");
	switch(ft_assem_results.imu_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//Flash Test
	strcat(buf, "FLASH:");
	switch(ft_assem_results.flash_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//SIM Test
	strcat(buf, "SIM:");
	switch(ft_assem_results.sim_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//BLE Test
	strcat(buf, "BLE:");
	switch(ft_assem_results.ble_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
#ifdef CONFIG_PPG_SUPPORT	
	//PPG Test
	strcat(buf, "PPG:");
	switch(ft_assem_results.ppg_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
#endif

	//Charging Test
	strcat(buf, "CHRGE:");
	switch(ft_assem_results.pmu_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//Vibration Test
	strcat(buf, "VIB:");
	switch(ft_assem_results.vib_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
#ifdef CONFIG_WIFI_SUPPORT	
	//WIFI Test
	strcat(buf, "WIFI:");
	switch(ft_assem_results.wifi_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
#endif

	//Network Test
	strcat(buf, "NET:");
	switch(ft_assem_results.net_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);
	
	//GPS Test
	strcat(buf, "GPS:");
	switch(ft_assem_results.gps_ret)
	{
	case 0:
		strcat(buf, "TBT");
		break;
	case 1:
		strcat(buf, "PASS");
		break;
	case 2:
		strcat(buf, "FAIL");
		break;
	}
	strcat(buf, sper);

	show_QR_code(strlen(buf), buf);
}

void FTAssemResultsScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_FT_ASSEM_RESULT_INFOR].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_FT_ASSEM_RESULT_INFOR].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_FT_ASSEM_RESULT_INFOR].status = SCREEN_STATUS_CREATED;

		FTAssemResultsShowQRStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		FTAssemResultsShowQRStatus();
		break;
	}
	
	scr_msg[SCREEN_ID_FT_ASSEM_RESULT_INFOR].act = SCREEN_ACTION_NO;
}

void ExitFTAssemResultsScreen(void)
{
	LCD_Set_BL_Mode(LCD_BL_AUTO);
	EnterIdleScreen();
}

void EnterFTAssemResultsScreen(void)
{
	if(screen_id == SCREEN_ID_FT_ASSEM_RESULT_INFOR)
		return;

	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_FT_ASSEM_RESULT_INFOR;	
	scr_msg[SCREEN_ID_FT_ASSEM_RESULT_INFOR].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_FT_ASSEM_RESULT_INFOR].status = SCREEN_STATUS_CREATING;

	ClearAllKeyHandler();
	SetLeftKeyUpHandler(EnterFTAgingTest);
	SetRightKeyUpHandler(ExitFTAssemResultsScreen);
#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterFTAgingTest);
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterFTSmtResultsScreen);
#endif	
}

#endif/*CONFIG_FACTORY_TEST_SUPPORT*/
#endif/*CONFIG_QRCODE_SUPPORT*/

void SOSUpdateStatus(void)
{
	switch(sos_state)
	{
	case SOS_STATUS_IDLE:
		break;
		
	case SOS_STATUS_SENDING:
		break;
	
	case SOS_STATUS_SENT:
	#ifdef CONFIG_ANIMATION_SUPPORT	
		AnimaStop();
	#endif

		LCD_ShowImage(SOS_ICON_X, SOS_ICON_Y, IMG_ID_SOS_ANI_3);
		sos_state = SOS_STATUS_RECEIVED;
		break;
	
	case SOS_STATUS_RECEIVED:
		break;
	
	case SOS_STATUS_CANCEL:
		break;
	}
}

void SOSShowStatus(void)
{
	uint32_t img_anima[4] = {IMG_ID_SOS_ANI_0,IMG_ID_SOS_ANI_1,IMG_ID_SOS_ANI_2,IMG_ID_SOS_ANI_3};
	
	LCD_Clear(BLACK);

#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaShow(SOS_ICON_X, SOS_ICON_Y, img_anima, ARRAY_SIZE(img_anima), 500, true, NULL);
#endif

	SOSChangrStatus();
}

void SOSScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_SOS].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_SOS].act = SCREEN_ACTION_NO;
		scr_msg[SCREEN_ID_SOS].status = SCREEN_STATUS_CREATED;

		SOSShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		SOSUpdateStatus();

		if(scr_msg[SCREEN_ID_SOS].para == SCREEN_EVENT_UPDATE_NO)
			scr_msg[SCREEN_ID_SOS].act = SCREEN_ACTION_NO;
		break;
	}
}

void EnterSOSScreen(void)
{
	if(screen_id == SCREEN_ID_SOS)
		return;

	k_timer_stop(&notify_timer);
	k_timer_stop(&mainmenu_timer);

#ifdef CONFIG_ANIMATION_SUPPORT
	AnimaStop();
#endif
#ifdef NB_SIGNAL_TEST
	if(gps_is_working())
		MenuStopGPS();
#endif	
#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		PPGStopCheck();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(IsInTempScreen()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
#ifdef CONFIG_SYNC_SUPPORT
	if(SyncIsRunning())
		SyncDataStop();
#endif

	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_SOS;	
	scr_msg[SCREEN_ID_SOS].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_SOS].status = SCREEN_STATUS_CREATING;

	ClearAllKeyHandler();
#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
#endif
	
}

#if 0	//xb add 2026-03-19
void MainMenuUpdateStatus(void)
{
	uint16_t i,x,y,w,h;
	uint32_t img_pg_dot[2] = {IMG_ID_PAGE2_1, IMG_ID_PAGE2_2};
	
	for(i=0;i<MAIN_MENU_MAX_PER_PG;i++)
	{
		LCD_ShowImage(MAIN_MENU_ICON_X+(i%MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_ICON_W+MAIN_MENU_ICON_OFFSET_W), MAIN_MENU_ICON_Y+(i/MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_ICON_H+MAIN_MENU_ICON_OFFSET_H), main_menu.icon[i+main_menu.index]);

		LCD_SetFontSize(FONT_SIZE_28);
		LCD_SetFontColor(WHITE);

		LCD_Fill(MAIN_MENU_STR_X+(i%MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_STR_W+MAIN_MENU_STR_OFFSET_W),
					MAIN_MENU_STR_Y+(i/MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_STR_H+MAIN_MENU_STR_OFFSET_H),
					MAIN_MENU_STR_W,
					MAIN_MENU_STR_H,
					BLACK);
		LCD_MeasureUniStr(main_menu.name[i+main_menu.index], &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_AdaptShowUniStrRtoLInRect(MAIN_MENU_STR_X+(i%MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_STR_W+MAIN_MENU_STR_OFFSET_W)+w,
											MAIN_MENU_STR_Y+(i/MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_STR_H+MAIN_MENU_STR_OFFSET_H),
											MAIN_MENU_STR_W,
											MAIN_MENU_STR_H,
											main_menu.name[i+main_menu.index],
											SHOW_ALIGN_CENTER,
											SHOW_ALIGN_BOUNDARY);
		else
	#endif
			LCD_AdaptShowUniStrInRect(MAIN_MENU_STR_X+(i%MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_STR_W+MAIN_MENU_STR_OFFSET_W),
										MAIN_MENU_STR_Y+(i/MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_STR_H+MAIN_MENU_STR_OFFSET_H),
										MAIN_MENU_STR_W,
										MAIN_MENU_STR_H,
										main_menu.name[i+main_menu.index],
										SHOW_ALIGN_CENTER,
										SHOW_ALIGN_BOUNDARY);

	#ifdef CONFIG_TOUCH_SUPPORT
		register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
									MAIN_MENU_ICON_X+(i%MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_ICON_W+MAIN_MENU_ICON_OFFSET_W), 
									MAIN_MENU_ICON_X+(i%MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_ICON_W+MAIN_MENU_ICON_OFFSET_W)+MAIN_MENU_ICON_W, 
									MAIN_MENU_ICON_Y+(i/MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_ICON_H+MAIN_MENU_ICON_OFFSET_H), 
									MAIN_MENU_ICON_Y+(i/MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_ICON_H+MAIN_MENU_ICON_OFFSET_H)+MAIN_MENU_ICON_H, 
									main_menu.sel_handler[i+main_menu.index]);
	#endif
	}

#ifdef LANGUAGE_AR_ENABLE
	if(g_language_r2l)
		LCD_ShowImage(LCD_WIDTH-MAIN_MENU_PG_DOT_X-MAIN_MENU_PG_DOT_W, MAIN_MENU_PG_DOT_Y, img_pg_dot[main_menu.index/MAIN_MENU_MAX_PER_PG]);
	else
#endif		
		LCD_ShowImage(MAIN_MENU_PG_DOT_X, MAIN_MENU_PG_DOT_Y, img_pg_dot[main_menu.index/MAIN_MENU_MAX_PER_PG]);

#ifdef CONFIG_TOUCH_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_UP, 0, LCD_WIDTH, 0, LCD_HEIGHT, main_menu.pg_handler[0]);
	register_touch_event_handle(TP_EVENT_MOVING_DOWN, 0, LCD_WIDTH, 0, LCD_HEIGHT, main_menu.pg_handler[1]);
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, main_menu.pg_handler[2]);
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, main_menu.pg_handler[3]);
#endif		
}

void MainMenuShowStatus(void)
{
	uint16_t i,x,y,w,h;
	uint32_t img_pg_dot[2] = {IMG_ID_PAGE2_1, IMG_ID_PAGE2_2};
	
	for(i=0;i<MAIN_MENU_MAX_PER_PG;i++)
	{
		LCD_ShowImage(MAIN_MENU_ICON_X+(i%MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_ICON_W+MAIN_MENU_ICON_OFFSET_W), MAIN_MENU_ICON_Y+(i/MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_ICON_H+MAIN_MENU_ICON_OFFSET_H), main_menu.icon[i+main_menu.index]);

		LCD_SetFontSize(FONT_SIZE_28);
		LCD_SetFontColor(WHITE);

		LCD_MeasureUniStr(main_menu.name[i+main_menu.index], &w, &h);
	#ifdef LANGUAGE_AR_ENABLE	
		if(g_language_r2l)
			LCD_AdaptShowUniStrRtoLInRect(MAIN_MENU_STR_X+(i%MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_STR_W+MAIN_MENU_STR_OFFSET_W)+w,
											MAIN_MENU_STR_Y+(i/MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_STR_H+MAIN_MENU_STR_OFFSET_H),
											MAIN_MENU_STR_W,
											MAIN_MENU_STR_H,
											main_menu.name[i+main_menu.index],
											SHOW_ALIGN_CENTER,
											SHOW_ALIGN_BOUNDARY);
		else
	#endif
			LCD_AdaptShowUniStrInRect(MAIN_MENU_STR_X+(i%MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_STR_W+MAIN_MENU_STR_OFFSET_W),
										MAIN_MENU_STR_Y+(i/MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_STR_H+MAIN_MENU_STR_OFFSET_H),
										MAIN_MENU_STR_W,
										MAIN_MENU_STR_H,
										main_menu.name[i+main_menu.index],
										SHOW_ALIGN_CENTER,
										SHOW_ALIGN_BOUNDARY);

	#ifdef CONFIG_TOUCH_SUPPORT
		register_touch_event_handle(TP_EVENT_SINGLE_CLICK, 
									MAIN_MENU_ICON_X+(i%MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_ICON_W+MAIN_MENU_ICON_OFFSET_W), 
									MAIN_MENU_ICON_X+(i%MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_ICON_W+MAIN_MENU_ICON_OFFSET_W)+MAIN_MENU_ICON_W, 
									MAIN_MENU_ICON_Y+(i/MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_ICON_H+MAIN_MENU_ICON_OFFSET_H), 
									MAIN_MENU_ICON_Y+(i/MAIN_MENU_MAX_PER_ROW)*(MAIN_MENU_ICON_H+MAIN_MENU_ICON_OFFSET_H)+MAIN_MENU_ICON_H, 
									main_menu.sel_handler[i+main_menu.index]);
	#endif
	}

#ifdef LANGUAGE_AR_ENABLE
	if(g_language_r2l)
		LCD_ShowImage(LCD_WIDTH-MAIN_MENU_PG_DOT_X-MAIN_MENU_PG_DOT_W, MAIN_MENU_PG_DOT_Y, img_pg_dot[main_menu.index/MAIN_MENU_MAX_PER_PG]);
	else
#endif		
		LCD_ShowImage(MAIN_MENU_PG_DOT_X, MAIN_MENU_PG_DOT_Y, img_pg_dot[main_menu.index/MAIN_MENU_MAX_PER_PG]);

#ifdef CONFIG_TOUCH_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_UP, 0, LCD_WIDTH, 0, LCD_HEIGHT, main_menu.pg_handler[0]);
	register_touch_event_handle(TP_EVENT_MOVING_DOWN, 0, LCD_WIDTH, 0, LCD_HEIGHT, main_menu.pg_handler[1]);
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, main_menu.pg_handler[2]);
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, main_menu.pg_handler[3]);
#endif		
}

void MainMenuScreenProcess(void)
{
	switch(scr_msg[SCREEN_ID_MAIN_MENU].act)
	{
	case SCREEN_ACTION_ENTER:
		scr_msg[SCREEN_ID_MAIN_MENU].status = SCREEN_STATUS_CREATED;
		
		LCD_Clear(BLACK);
		IdleShowSignal();
		IdleShowNetMode();
		IdleShowBatSoc();
	#ifdef CONFIG_BLE_SUPPORT	
		//IdleShowBleStatus();
	#endif
		MainMenuShowStatus();
		break;
		
	case SCREEN_ACTION_UPDATE:
		MainMenuUpdateStatus();
		break;
	}
	
	scr_msg[SCREEN_ID_MAIN_MENU].act = SCREEN_ACTION_NO;
}

void ExitMainMenuScreen(void)
{
	EntryIdleScr();
}

void EnterMainMenuScreen(void)
{
	if(screen_id == SCREEN_ID_MAIN_MENU)
		return;

	k_timer_stop(&mainmenu_timer);
#ifdef CONFIG_ANIMATION_SUPPORT	
	AnimaStop();
#endif
#ifdef CONFIG_TEMP_SUPPORT
	if(TempIsWorking()&&!TempIsWorkingTiming())
		MenuStopTemp();
#endif
#ifdef CONFIG_PPG_SUPPORT
	if(IsInPPGScreen()&&!PPGIsWorkingTiming())
		MenuStopPPG();
#endif
#ifdef CONFIG_WIFI_SUPPORT
	if(IsInWifiScreen()&&wifi_is_working())
		MenuStopWifi();
#endif

	LCD_Set_BL_Mode(LCD_BL_AUTO);
	
	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_MAIN_MENU;	
	scr_msg[SCREEN_ID_MAIN_MENU].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_MAIN_MENU].status = SCREEN_STATUS_CREATING;

#ifdef NB_SIGNAL_TEST
	SetLeftKeyUpHandler(EnterPoweroffScreen);
#else
 #ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
 	if((strlen(g_ui_ver) == 0) 
		|| (strlen(g_font_ver) == 0)
		|| (strlen(g_str_ver) == 0)
		|| (strlen(g_ppg_algo_ver) == 0)
		)
	{
		SPIFlash_Read_DataVer(g_ui_ver, g_font_ver, g_str_ver, g_ppg_algo_ver);
	}
 
  	if((strcmp(g_new_ui_ver,g_ui_ver) != 0) && (strlen(g_new_ui_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		SetLeftKeyUpHandler(dl_img_start);
	}
	else if((strcmp(g_new_font_ver,g_font_ver) != 0) && (strlen(g_new_font_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		SetLeftKeyUpHandler(dl_font_start);
	}
	else if((strcmp(g_new_str_ver,g_str_ver) != 0) && (strlen(g_new_str_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		SetLeftKeyUpHandler(dl_str_start);
	}
  #if defined(CONFIG_PPG_DATA_UPDATE)&&defined(CONFIG_PPG_SUPPORT)
	else if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
	{
		SetLeftKeyUpHandler(dl_ppg_start);
	}
  #endif
  	else
 #endif/*CONFIG_DATA_DOWNLOAD_SUPPORT*/
  	{
  		SetLeftKeyUpHandler(EnterPoweroffScreen);
  	}
#endif/*NB_SIGNAL_TEST*/
	SetRightKeyUpHandler(ExitSettingsScreen);

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();

 #ifdef NB_SIGNAL_TEST
	register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterPoweroffScreen);
 #else
  #ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
	if((strcmp(g_new_ui_ver,g_ui_ver) != 0) && (strlen(g_new_ui_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_img_start);
	}
	else if((strcmp(g_new_font_ver,g_font_ver) != 0) && (strlen(g_new_font_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_font_start);
	}
	else if((strcmp(g_new_str_ver,g_str_ver) != 0) && (strlen(g_new_str_ver) > 0) && (strcmp(g_new_fw_ver, g_fw_version) == 0))
	{
		register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_str_start);
	}
   #if defined(CONFIG_PPG_DATA_UPDATE)&&defined(CONFIG_PPG_SUPPORT)
	else if((strcmp(g_new_ppg_ver,g_ppg_algo_ver) != 0) && (strlen(g_new_ppg_ver) > 0))
	{
		register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, dl_ppg_start);
	}
   #endif
	else
  #endif/*CONFIG_DATA_DOWNLOAD_SUPPORT*/
	{
		register_touch_event_handle(TP_EVENT_MOVING_LEFT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterPoweroffScreen);
 	}
 #endif/*NB_SIGNAL_TEST*/

 #ifdef NB_SIGNAL_TEST
  #ifdef CONFIG_WIFI_SUPPORT
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterWifiTestScreen);
  #else
 	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterGPSTestScreen);
  #endif
 #else
  #ifdef CONFIG_SYNC_SUPPORT
  	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSyncDataScreen);
  #elif defined(CONFIG_PPG_SUPPORT)
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterBPScreen);
  #elif defined(CONFIG_TEMP_SUPPORT)
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterTempScreen);
  #elif defined(CONFIG_IMU_SUPPORT)&&(defined(CONFIG_STEP_SUPPORT)||defined(CONFIG_SLEEP_SUPPORT))
   #ifdef CONFIG_SLEEP_SUPPORT
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterSleepScreen);
   #elif defined(CONFIG_STEP_SUPPORT)
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterStepsScreen);
   #endif
  #else
	register_touch_event_handle(TP_EVENT_MOVING_RIGHT, 0, LCD_WIDTH, 0, LCD_HEIGHT, EnterIdleScreen);
  #endif/*CONFIG_SYNC_SUPPORT*/
 #endif/*NB_SIGNAL_TEST*/
#endif
}
#endif

void GoBackHistoryScreen(void)
{
	SCREEN_ID_ENUM scr_id;
	
	scr_id = screen_id;
	scr_msg[scr_id].act = SCREEN_ACTION_NO;
	scr_msg[scr_id].status = SCREEN_STATUS_NO;

	screen_id = history_screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_ENTER;
	scr_msg[history_screen_id].status = SCREEN_STATUS_CREATING;	
}

void ScreenMsgProcess(void)
{
	if(entry_idle_flag)
	{
		EnterIdleScreen();
		entry_idle_flag = false;
	}
	
	if(scr_msg[screen_id].act != SCREEN_ACTION_NO)
	{
		if(scr_msg[screen_id].status != SCREEN_STATUS_CREATED)
			scr_msg[screen_id].act = SCREEN_ACTION_ENTER;

		switch(screen_id)
		{
		case SCREEN_ID_IDLE:
			IdleScreenProcess();
			break;
	#if 0	//xb add 2026-03-19		
		case SCREEN_ID_MAIN_MENU:
			MainMenuScreenProcess();
			break;
	#endif
	#ifdef CONFIG_ALARM_SUPPORT	
		case SCREEN_ID_ALARM:
			AlarmScreenProcess();
			break;
		case SCREEN_ID_FIND_DEVICE:
			FindDeviceScreenProcess();
			break;
	#endif		
	#ifdef CONFIG_PPG_SUPPORT	
		case SCREEN_ID_HR:
			HRScreenProcess();
			break;
		case SCREEN_ID_SPO2:
			SPO2ScreenProcess();
			break;
		case SCREEN_ID_ECG:
			EcgScreenProcess();
			break;
		case SCREEN_ID_BP:
			BPScreenProcess();
			break;
	#endif		
		case SCREEN_ID_SOS:
			SOSScreenProcess();
			break;
	#ifdef CONFIG_IMU_SUPPORT
	  #ifdef CONFIG_STEP_SUPPORT
		case SCREEN_ID_STEPS:
			StepsScreenProcess();
			break;
	  #endif
	  #ifdef CONFIG_SLEEP_SUPPORT
		case SCREEN_ID_SLEEP:
			SleepScreenProcess();
			break;
	  #endif
	  #ifdef CONFIG_FALL_DETECT_SUPPORT
		case SCREEN_ID_FALL:
			FallScreenProcess();
			break;
	  #endif
	#endif
	#if 0
		case SCREEN_ID_WRIST:
			WristScreenProcess();
			break;
	#endif
		case SCREEN_ID_SETTINGS:
			SettingsScreenProcess();
			break;
	#ifdef NB_SIGNAL_TEST
	  #ifdef CONFIG_WIFI_SUPPORT
		case SCREEN_ID_WIFI_TEST:
			TestWifiScreenProcess();
			break;
	  #endif
		case SCREEN_ID_GPS_TEST:
			TestGPSScreenProcess();
			break;
		case SCREEN_ID_NB_TEST:
			TestNBScreenProcess();
			break;
	#endif
		case SCREEN_ID_POWEROFF:
			PowerOffScreenProcess();
			break;
		case SCREEN_ID_NOTIFY:
			NotifyScreenProcess();
			break;
	#ifdef CONFIG_SYNC_SUPPORT
		case SCREEN_ID_SYNC:
			SyncScreenProcess();
			break;
	#endif
	#ifdef CONFIG_FOTA_DOWNLOAD
		case SCREEN_ID_VER_CHECK:
			VerCheckScreenProcess();
			break;
		case SCREEN_ID_FOTA:
			FOTAScreenProcess();
			break;
	#endif
	#ifdef CONFIG_DATA_DOWNLOAD_SUPPORT
		case SCREEN_ID_DL:
			DlScreenProcess();
			break;
	#endif
	#ifdef CONFIG_TEMP_SUPPORT
		case SCREEN_ID_TEMP:
			TempScreenProcess();
			break;
	#endif
	#ifdef CONFIG_QRCODE_SUPPORT
		case SCREEN_ID_DEVICE_INFOR:
			DeviceScreenProcess();
			break;
	  #ifdef CONFIG_FACTORY_TEST_SUPPORT		
		case SCREEN_ID_FT_SMT_RESULT_INFOR:
			FTSmtResultsScreenProcess();
			break;
		case SCREEN_ID_FT_ASSEM_RESULT_INFOR:
			FTAssemResultsScreenProcess();
			break;
	  #endif
	#endif
		}
	}
}
