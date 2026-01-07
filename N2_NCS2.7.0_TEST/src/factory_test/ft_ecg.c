/****************************************Copyright (c)************************************************
** File Name:			    ft_ppg.c
** Descriptions:			Factory test ecg module source file
** Created By:				xie biao
** Created Date:			2026-01-05
** Modified Date:      		2026-01-05 
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#ifdef CONFIG_TOUCH_SUPPORT
#include "CST816.h"
#endif
#include "ecg.h"
#include "screen.h"
#include "settings.h"
#include "key.h"
#include "ft_main.h"
#include "ft_ecg.h"
#include "logger.h"

#define FT_ECG_TITLE_W				150
#define FT_ECG_TITLE_H				40
#define FT_ECG_TITLE_X				((LCD_WIDTH-FT_ECG_TITLE_W)/2)
#define FT_ECG_TITLE_Y				20

#define FT_ECG_MENU_STR_W			150
#define FT_ECG_MENU_STR_H			30
#define FT_ECG_MENU_STR_X			((LCD_WIDTH-FT_ECG_MENU_STR_W)/2)
#define FT_ECG_MENU_STR_Y			80
#define FT_ECG_MENU_STR_OFFSET_Y	5

#define FT_ECG_SLE1_STR_W			70
#define FT_ECG_SLE1_STR_H			30
#define FT_ECG_SLE1_STR_X			40
#define FT_ECG_SLE1_STR_Y			170
#define FT_ECG_SLE2_STR_W			70
#define FT_ECG_SLE2_STR_H			30
#define FT_ECG_SLE2_STR_X			130
#define FT_ECG_SLE2_STR_Y			170

#define FT_ECG_RET_STR_W			120
#define FT_ECG_RET_STR_H			60
#define FT_ECG_RET_STR_X			((LCD_WIDTH-FT_ECG_RET_STR_W)/2)
#define FT_ECG_RET_STR_Y			((LCD_HEIGHT-FT_ECG_RET_STR_H)/2)

#define FT_ECG_NOTIFY_W				240
#define FT_ECG_NOTIFY_H				40
#define FT_ECG_NOTIFY_X				((LCD_WIDTH-FT_ECG_NOTIFY_W)/2)
#define FT_ECG_NOTIFY_Y				100
				
#define FT_ECG_TEST_TIMEROUT		2*60
			
static bool ft_ecg_check_ok = false;
static bool ft_ecg_checking = false;
static bool update_show_flag = false;
static uint8_t ft_ecg_scaned_count = 0;
static uint8_t ft_ecg_infor[256] = {0};

static void ECGTestTimerOutCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(ecg_test_timer, ECGTestTimerOutCallBack, NULL);

static void FTMenuECGDumpProc(void){}

const ft_menu_t FT_MENU_ECG = 
{
	FT_ECG,
	0,
	0,
	{
		{
			{0x0000},
			FTMenuECGDumpProc,
		},
	},
	{
		//page proc func
		FTMenuECGDumpProc,
		FTMenuECGDumpProc,
		FTMenuECGDumpProc,
		FTMenuECGDumpProc,
	},
};

static void FTMenuECGSle1Hander(void)
{
	ft_ecg_checking = false;
	k_timer_stop(&ecg_test_timer);
	FTStopECG();
	
	switch(g_ft_status)
	{
	case FT_STATUS_SMT:
		FT_SMT_MENU_MAIN.item[ft_main_menu_index+1].sel_handler();
		break;
		
	case FT_STATUS_ASSEM:
		FT_ASSEM_MENU_MAIN.item[ft_main_menu_index+1].sel_handler();
		break;
	}
}

static void FTMenuECGSle2Hander(void)
{
	ExitFTMenuECG();
}

static void FTMenuECGStopTest(void)
{
	ft_ecg_checking = false;
	FTStopECG();
	k_timer_stop(&ecg_test_timer);
	scr_msg[SCREEN_ID_FACTORY_TEST].act = SCREEN_ACTION_UPDATE;
}

static void FTMenuECGStartTest(void)
{
	ft_ecg_checking = true;
	ft_ecg_check_ok = false;
	FTStartECG();
	k_timer_start(&ecg_test_timer, K_SECONDS(FT_ECG_TEST_TIMEROUT), K_NO_WAIT);
	scr_msg[SCREEN_ID_FACTORY_TEST].act = SCREEN_ACTION_UPDATE;
}

static void ECGTestTimerOutCallBack(struct k_timer *timer_id)
{
	FTMenuECGStopTest();
}

static void FTMenuECGUpdate(void)
{
	uint16_t x,y,w,h;
	uint16_t ret_str[2][5] = {
								{0x0046,0x0041,0x0049,0x004C,0x0000},//FAIL
								{0x0050,0x0041,0x0053,0x0053,0x0000},//PASS
							  };

	if(ft_ecg_checking)
	{
		uint8_t tmpbuf[512] = {0};
		
		if(!update_show_flag)
		{
			update_show_flag = true;
			LCD_Fill(FT_ECG_NOTIFY_X, FT_ECG_NOTIFY_Y, FT_ECG_NOTIFY_W, FT_ECG_NOTIFY_H, BLACK);
			LCD_SetFontSize(FONT_SIZE_20);
		}

		LCD_Fill((LCD_WIDTH-180)/2, 60, 180, 100, BLACK);
		mmi_asc_to_ucs2(tmpbuf, ecg_test_info);
		LCD_ShowUniStringInRect((LCD_WIDTH-180)/2, 60, 180, 100, (uint16_t*)tmpbuf);
	}
	else
	{
		update_show_flag = false;

		LCD_SetFontSize(FONT_SIZE_28);

		//pass or fail
		LCD_SetFontSize(FONT_SIZE_52);
		LCD_SetFontColor(BRRED);
		LCD_SetFontBgColor(GREEN);
		LCD_MeasureUniString(ret_str[ft_ecg_check_ok], &w, &h);
		LCD_ShowUniString(FT_ECG_RET_STR_X+(FT_ECG_RET_STR_W-w)/2, FT_ECG_RET_STR_Y+(FT_ECG_RET_STR_H-h)/2, ret_str[ft_ecg_check_ok]);
		LCD_ReSetFontBgColor();
		LCD_ReSetFontColor();

		if(ft_ecg_check_ok)
			ft_menu_checked[ft_main_menu_index] = true;
		else
			ft_menu_checked[ft_main_menu_index] = false;

		switch(g_ft_status)
		{
		case FT_STATUS_SMT:
			if(ft_ecg_check_ok)
				ft_smt_results.ecg_ret = 1;
			else
				ft_smt_results.ecg_ret = 2;
			
			SaveFactoryTestResults(FT_STATUS_SMT, &ft_smt_results);
			break;
			
		case FT_STATUS_ASSEM:
			if(ft_ecg_check_ok)
				ft_assem_results.ecg_ret = 1;
			else
				ft_assem_results.ecg_ret = 2;
			
			SaveFactoryTestResults(FT_STATUS_ASSEM, &ft_assem_results);
			break;
		}
	}
}

static void FTMenuECGShow(void)
{
	uint8_t i;
	uint16_t x,y,w,h;
	uint16_t title_str[8] = {0x0045,0x0043,0x0047,0x6D4B,0x8BD5,0x0000};//ECG测试
	uint16_t sle_str[2][5] = {
								{0x4E0B,0x4E00,0x9879,0x0000},//下一项
								{0x9000,0x51FA,0x0000},//退出
							  };
	uint16_t notify_str[9] = {0x6B63,0x5728,0x542F,0x52A8,0x4F20,0x611F,0x5668,0x2026,0x0000};//正在启动传感器…

#ifdef CONFIG_TOUCH_SUPPORT
	clear_all_touch_event_handle();
#endif

	LCD_Clear(BLACK);
	LCD_Set_BL_Mode(LCD_BL_ALWAYS_ON);

	LCD_SetFontSize(FONT_SIZE_28);
	LCD_MeasureUniString(title_str, &w, &h);
	LCD_ShowUniString(FT_ECG_TITLE_X+(FT_ECG_TITLE_W-w)/2, FT_ECG_TITLE_Y, title_str);
	LCD_MeasureUniString(notify_str, &w, &h);
	LCD_ShowUniString(FT_ECG_NOTIFY_X+(FT_ECG_NOTIFY_W-w)/2, FT_ECG_NOTIFY_Y+(FT_ECG_NOTIFY_H-h)/2, notify_str);

	LCD_SetFontSize(FONT_SIZE_28);
	LCD_MeasureUniString(sle_str[0], &w, &h);
	x = FT_ECG_SLE1_STR_X+(FT_ECG_SLE1_STR_W-w)/2;
	y = FT_ECG_SLE1_STR_Y+(FT_ECG_SLE1_STR_H-h)/2;
	LCD_DrawRectangle(FT_ECG_SLE1_STR_X, FT_ECG_SLE1_STR_Y, FT_ECG_SLE1_STR_W, FT_ECG_SLE1_STR_H);
	LCD_ShowUniString(x, y, sle_str[0]);
	LCD_MeasureUniString(sle_str[1], &w, &h);
	x = FT_ECG_SLE2_STR_X+(FT_ECG_SLE2_STR_W-w)/2;
	y = FT_ECG_SLE2_STR_Y+(FT_ECG_SLE2_STR_H-h)/2;
	LCD_DrawRectangle(FT_ECG_SLE2_STR_X, FT_ECG_SLE2_STR_Y, FT_ECG_SLE2_STR_W, FT_ECG_SLE2_STR_H);
	LCD_ShowUniString(x, y, sle_str[1]);

	ClearAllKeyHandler();
	SetLeftKeyUpHandler(FTMenuECGSle1Hander);
	SetRightKeyUpHandler(FTMenuECGSle2Hander);
	
#ifdef CONFIG_TOUCH_SUPPORT
	register_touch_event_handle(TP_EVENT_SINGLE_CLICK, FT_ECG_SLE1_STR_X, FT_ECG_SLE1_STR_X+FT_ECG_SLE1_STR_W, FT_ECG_SLE1_STR_Y, FT_ECG_SLE1_STR_Y+FT_ECG_SLE1_STR_H, FTMenuECGSle1Hander);
	register_touch_event_handle(TP_EVENT_SINGLE_CLICK, FT_ECG_SLE2_STR_X, FT_ECG_SLE2_STR_X+FT_ECG_SLE2_STR_W, FT_ECG_SLE2_STR_Y, FT_ECG_SLE2_STR_Y+FT_ECG_SLE2_STR_H, FTMenuECGSle2Hander);
#endif		
}

void FTMenuECGProcess(void)
{
	if(scr_msg[SCREEN_ID_FACTORY_TEST].act != SCREEN_ACTION_NO)
	{
		if(scr_msg[SCREEN_ID_FACTORY_TEST].status != SCREEN_STATUS_CREATED)
			scr_msg[SCREEN_ID_FACTORY_TEST].act = SCREEN_ACTION_ENTER;

		switch(scr_msg[SCREEN_ID_FACTORY_TEST].act)
		{
		case SCREEN_ACTION_ENTER:
			scr_msg[SCREEN_ID_FACTORY_TEST].status = SCREEN_STATUS_CREATED;
			FTMenuECGShow();
			break;
			
		case SCREEN_ACTION_UPDATE:
			FTMenuECGUpdate();
			break;
		}
	
		scr_msg[SCREEN_ID_FACTORY_TEST].act = SCREEN_ACTION_NO;
	}
}

bool IsFTECGTesting(void)
{
	if((screen_id == SCREEN_ID_FACTORY_TEST)&&(ft_menu.id == FT_ECG))
		return true;
	else
		return false;
}

void FTECGStatusUpdate(uint8_t hr, uint8_t spo2)
{
	static uint8_t count = 0;

	if((screen_id == SCREEN_ID_FACTORY_TEST)&&(ft_menu.id == FT_ECG))
	{
		if(spo2 > 0)
		{
			count++;
			if(count > 2)
			{
				count = 0;
				ft_ecg_checking = false;
				ft_ecg_check_ok = true;
				ft_menu_checked[ft_main_menu_index] = true;
				FTMenuECGStopTest();
			}
		}

		scr_msg[SCREEN_ID_FACTORY_TEST].act = SCREEN_ACTION_UPDATE;
	}
}

void ExitFTMenuECG(void)
{
	ft_ecg_checking = false;
	ft_ecg_check_ok = false;
	k_timer_stop(&ecg_test_timer);
	FTStopECG();
	ReturnFTMainMenu();
}

void EnterFTMenuECG(void)
{
	ft_ecg_check_ok = false;
	update_show_flag = false;
	memcpy(&ft_menu, &FT_MENU_ECG, sizeof(ft_menu_t));
	
	history_screen_id = screen_id;
	scr_msg[history_screen_id].act = SCREEN_ACTION_NO;
	scr_msg[history_screen_id].status = SCREEN_STATUS_NO;

	screen_id = SCREEN_ID_FACTORY_TEST; 
	scr_msg[SCREEN_ID_FACTORY_TEST].act = SCREEN_ACTION_ENTER;
	scr_msg[SCREEN_ID_FACTORY_TEST].status = SCREEN_STATUS_CREATING;

	FTMenuECGStartTest();
}
