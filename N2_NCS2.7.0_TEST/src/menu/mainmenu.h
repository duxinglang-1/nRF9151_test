/****************************************Copyright (c)************************************************
** File Name:			    mainmenu.h
** Descriptions:			main menu process head file
** Created By:				xie biao
** Created Date:			2026-02-06
** Modified Date:
** Version:			    	V1.0
******************************************************************************************************/
#ifndef __MAINMENU_H__
#define __MAINMENU_H__

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include "strdef.h"
#include "imgdef.h"

#if 0	//xb add 2026-03-19

#define MAIN_MENU_MAX_COUNT	15
#define MAIN_MENU_NAME_MAX	25
#define MAIN_MENU_NAME_STR_MAX	15

#define MAIN_MENU_MAX_PER_PG		6
#define MAIN_MENU_MAX_PER_ROW		3

#define MAIN_MENU_BG_X			0
#define MAIN_MENU_BG_Y			70
#define MAIN_MENU_BG_W			LCD_WIDTH
#define MAIN_MENU_BG_H			(LCD_HEIGHT-MAIN_MENU_BG_Y)

#define MAIN_MENU_ICON_W			90
#define MAIN_MENU_ICON_H			90
#define MAIN_MENU_ICON_X			30
#define MAIN_MENU_ICON_Y			90
#define MAIN_MENU_ICON_OFFSET_W		30
#define MAIN_MENU_ICON_OFFSET_H		84
#define MAIN_MENU_STR_W				114
#define MAIN_MENU_STR_H				52
#define MAIN_MENU_STR_X				18
#define MAIN_MENU_STR_Y				190
#define MAIN_MENU_STR_OFFSET_W		6
#define MAIN_MENU_STR_OFFSET_H		130
#define MAIN_MENU_PG_DOT_W			10
#define MAIN_MENU_PG_DOT_H			28
#define MAIN_MENU_PG_DOT_X			370
#define MAIN_MENU_PG_DOT_Y			(LCD_HEIGHT-MAIN_MENU_PG_DOT_H)/2

typedef void(*main_menu_handler)(void);

typedef enum
{
	MAIN_MENU_STEPS,
	MAIN_MENU_HR,
	MAIN_MENU_TEMP,
	MAIN_MENU_SPO2,
	MAIN_MENU_BPT,
	MAIN_MENU_ECG,
	MAIN_MENU_RESPIRATORY,
	MAIN_MENU_BODY_FAT,
	MAIN_MENU_EMOTION,
	MAIN_MENU_SYNCH,
	MAIN_MENU_SHUTDOWN,
	MAIN_MENU_SETTINGS,
	MAIN_MENU_FENCE,
 	MAIN_MENU_MAX
}MAIN_MENU_ID;

typedef struct
{
	MAIN_MENU_ID id;
	uint8_t index;
	uint8_t count;
	uint16_t icon[MAIN_MENU_MAX_COUNT];
	uint16_t name[MAIN_MENU_MAX_COUNT];
	main_menu_handler sel_handler[MAIN_MENU_MAX_COUNT];
	main_menu_handler pg_handler[4];
}main_menu_t;

extern uint8_t g_main_menu_index;
extern main_menu_t main_menu;

extern void EnterMainMenu(void);

#endif
#endif/*__MAINMENU_H__*/
