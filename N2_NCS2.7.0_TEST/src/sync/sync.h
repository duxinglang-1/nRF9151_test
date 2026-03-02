/****************************************Copyright (c)************************************************
** File Name:			    sync.h
** Descriptions:			data synchronism process head file
** Created By:				xie biao
** Created Date:			2021-12-16
** Modified Date:      		2021-12-16 
** Version:			    	V1.0
******************************************************************************************************/
#ifndef __SYNC_H__
#define __SYNC_H__

#include <zephyr/types.h>
#include "lcd.h"

#ifdef __cplusplus
extern "C" {
#endif

//synchronism data
#define SYNC_ICON_W					220
#define SYNC_ICON_H					254
#define SYNC_ICON_X					((LCD_WIDTH-SYNC_ICON_W)/2)
#define SYNC_ICON_Y					79
#define SYNC_RUNNING_ANI_W			132
#define SYNC_RUNNING_ANI_H			22
#define SYNC_RUNNING_ANI_X			((LCD_WIDTH-SYNC_RUNNING_ANI_W)/2)
#define SYNC_RUNNING_ANI_Y			378

typedef enum
{
	SYNC_STATUS_IDLE,
	SYNC_STATUS_LINKING,	
	SYNC_STATUS_SENT,
	SYNC_STATUS_FAIL,
	SYNC_STATUS_MAX
}SYNC_STATUS;

extern SYNC_STATUS sync_state;

extern void SyncDataStart(void);
extern void SyncDataStop(void);

#ifdef __cplusplus
}
#endif

#endif/*__SYNC_H__*/
