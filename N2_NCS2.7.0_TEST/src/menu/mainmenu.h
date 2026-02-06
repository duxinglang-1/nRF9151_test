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

#define MAIN_MENU_MAX_COUNT	15
#define MAIN_MENU_NAME_MAX	25
#define MAIN_MENU_NAME_STR_MAX	15

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

extern void EnterMainMenu(void);
#endif/*__MAINMENU_H__*/
