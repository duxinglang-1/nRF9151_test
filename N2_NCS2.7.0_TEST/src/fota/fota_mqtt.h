/****************************************Copyright (c)************************************************
** File Name:			    fota_mqtt.h
** Descriptions:			fota by mqtt protocal process head file
** Created By:				xie biao
** Created Date:			2021-03-29
** Modified Date:      		2021-03-29 
** Version:			    	V1.0
******************************************************************************************************/
#ifdef CONFIG_FOTA_DOWNLOAD
#ifndef __FOTA_MQTT_H__
#define __FOTA_MQTT_H__

#include "lcd.h"

#define FOTA_LOGO_W				120
#define FOTA_LOGO_H				120
#define FOTA_LOGO_X				((LCD_WIDTH-FOTA_LOGO_W)/2)
#define FOTA_LOGO_Y				92
#define FOTA_YES_W				70
#define FOTA_YES_H				70
#define FOTA_YES_X				240
#define FOTA_YES_Y				330
#define FOTA_NO_W				70
#define FOTA_NO_H				70
#define FOTA_NO_X				80
#define FOTA_NO_Y				330
#define FOTA_START_STR_W		350
#define FOTA_START_STR_H		80
#define FOTA_START_STR_X		((LCD_WIDTH-FOTA_START_STR_W)/2)
#define FOTA_START_STR_Y		232
#define FOTA_PROGRESS_W			310
#define FOTA_PROGRESS_H			16
#define FOTA_PROGRESS_X			((LCD_WIDTH-FOTA_PROGRESS_W)/2)
#define FOTA_PROGRESS_Y			308
#define FOTA_PRO_NUM_W			120
#define FOTA_PRO_NUM_H			50
#define FOTA_PRO_NUM_X			((LCD_WIDTH-FOTA_PRO_NUM_W)/2)
#define FOTA_PRO_NUM_Y			356
#define FOTA_FINISH_ICON_W		120
#define FOTA_FINISH_ICON_H		120
#define FOTA_FINISH_ICON_X		((LCD_WIDTH-FOTA_FINISH_ICON_W)/2)
#define FOTA_FINISH_ICON_Y		132
#define FOTA_FINISH_STR_W		350
#define FOTA_FINISH_STR_H		36
#define FOTA_FINISH_STR_X		((LCD_WIDTH-FOTA_FINISH_STR_W)/2)
#define FOTA_FINISH_STR_Y		292
#define FOTA_FAIL_ICON_W		120
#define FOTA_FAIL_ICON_H		120
#define FOTA_FAIL_ICON_X		((LCD_WIDTH-FOTA_FAIL_ICON_W)/2)
#define FOTA_FAIL_ICON_Y		132
#define FOTA_FAIL_STR_W			350
#define FOTA_FAIL_STR_H			36
#define FOTA_FAIL_STR_X			((LCD_WIDTH-FOTA_FAIL_STR_W)/2)
#define FOTA_FAIL_STR_Y			292


typedef enum
{
	FOTA_STATUS_PREPARE,
	FOTA_STATUS_LINKING,
	FOTA_STATUS_DOWNLOADING,
	FOTA_STATUS_FINISHED,
	FOTA_STATUS_ERROR,
	FOTA_STATUS_MAX
}FOTA_STATUS_ENUM;

extern uint8_t g_fota_progress;

extern void fota_work_init(struct k_work_q *work_q);
extern void fota_init(void);
extern void fota_start(void);
extern void fota_excu(void);
extern void fota_start_confirm(void);
extern void fota_reboot_confirm(void);
extern void fota_exit(void);
extern bool fota_is_running(void);
extern void VerCheckStart(void);
extern FOTA_STATUS_ENUM get_fota_status(void);

#endif/*__FOTA_MQTT_H__*/
#endif/*CONFIG_FOTA_DOWNLOAD*/