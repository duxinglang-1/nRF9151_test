/****************************************Copyright (c)************************************************
** File Name:			    mainmenu.c
** Descriptions:			main menu process source file
** Created By:				xie biao
** Created Date:			2026-02-06
** Modified Date:
** Version:			    	V1.0
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include "screen.h"
#include "mainmenu.h"
#include "lcd.h"
#include "logger.h"


static bool mainmenu_redraw_flag = false;

static uint8_t main_menu_index_bk = 0;

main_menu_t main_menu = {0};

static void MainMenu1Proc(void);
static void MainMenu2Proc(void);
static void MainMenu3Proc(void);
static void MainMenu4Proc(void);
static void MainMenu5Proc(void);
static void MainMenu6Proc(void);
static void MainMenu7Proc(void);
static void MainMenu8Proc(void);
static void MainMenu9Proc(void);
static void MainMenu10Proc(void);
static void MainMenu11Proc(void);
static void MainMenu12Proc(void);
static void MainMenu13Proc(void);
static void MainMenu14Proc(void);
static void MainMenuPgUpProc(void);
static void MainMenuPgDownProc(void);
static void MainMenuPgLeftProc(void);
static void MainMenuPgRightProc(void);
static void MainMenuDumpProc(void);
static void MainMenuMsgPorcess(void);


const main_menu_t MAIN_MENU_DATA = 
{
	MAIN_MENU_RESPIRATORY,
	0,
	12,
	{
		IMG_ID_MENU_RESPIRATORY_ICON,
		IMG_ID_MENU_FAT_ICON,
		IMG_ID_MENU_PRESURE_ICON,
		IMG_ID_MENU_SYNC_ICON,
		IMG_ID_MENU_PWROFF_ICON,
		IMG_ID_MENU_SETTINGS_ICON,
		IMG_ID_MENU_STEP_ICON,
		IMG_ID_MENU_HR_ICON,
		IMG_ID_MENU_TEMP_C_ICON,
		IMG_ID_MENU_SPO2_ICON,
		IMG_ID_MENU_BPT_ICON,
		IMG_ID_MENU_ECG_ICON,
	},
	{
		STR_ID_LANGUAGES,
		STR_ID_SCR_BRIGHT,
		STR_ID_TEMP_DSP,
		STR_ID_DEVICE_INFO,
		STR_ID_CAREMATE_QR,
		STR_ID_FACTORY_DEFAULT,
		STR_ID_OTA,
		STR_ID_LANGUAGES,
		STR_ID_LANGUAGES,
		STR_ID_LANGUAGES,
		STR_ID_LANGUAGES,
		STR_ID_LANGUAGES,
	},
	{
		//select proc func
		MainMenu1Proc,
		MainMenu2Proc,
		MainMenu3Proc,
		MainMenu4Proc,
		MainMenu5Proc,
		MainMenu6Proc,
		MainMenu7Proc,
		MainMenu8Proc,
		MainMenu9Proc,
		MainMenu10Proc,
		MainMenu11Proc,
		MainMenu12Proc,
	},
	{	
		//page proc func
		MainMenuPgUpProc,
		MainMenuPgDownProc,
		MainMenuDumpProc,
		MainMenuDumpProc,
	},
};

void MainMenu1Proc(void)
{
	main_menu_index_bk = main_menu.index;

}

void MainMenu2Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenu3Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenu4Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenu5Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenu6Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenu7Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenu8Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenu9Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenu10Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenu11Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenu12Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenu13Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenu14Proc(void)
{
	main_menu_index_bk = main_menu.index;
}

void MainMenuPgUpProc(void)
{
}

void MainMenuPgDownProc(void)
{
}

void MainMenuPgLeftProc(void)
{
}

void MainMenuPgRightProc(void)
{
}


void MainMenuMsgPorcess(void)
{
}

void EnterMainMenu(void)
{
	memcpy(&main_menu, &MAIN_MENU_DATA, sizeof(main_menu_t));
	main_menu.index = main_menu_index_bk;
	
	EnterMainMenuScreen();
}
