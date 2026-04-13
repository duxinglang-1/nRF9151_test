/****************************************Copyright (c)************************************************
** File Name:			    ecg.h
** Descriptions:			ecg function main head file
** Created By:				xie biao
** Created Date:			2024-04-11
** Modified Date:      		2024-04-11
** Version:			    	V1.0
******************************************************************************************************/
#ifndef __ECG_H__
#define __ECG_H__

#define COM_ECG_SET_OPEN		"OPEN:"
#define COM_ECG_SET_CLOSE		"CLOSE:"
#define COM_ECG_GET_INFOR		"INFOR:"
#define COM_ECG_GET_DATA		"ECG_DATA:"
#define COM_ECG_LEAD_STATUS		"LEAD_STATUS:"
#define COM_ECG_LEAD_OFF		"LEAD_OFF:"
#define COM_ECG_LEAD_ON		    "LEAD_ON:"
#define COM_ECG_LEAD_TIME_OUT	"LEAD_TIME_OUT:"

//#define ECG_ADS1292
#define ECG_MAX86176

#define ECG_CHECK_MENU				30
#define ECG_CHECK_TIMELY			2

typedef enum
{
	ECG_STATUS_PREPARE,
	ECG_STATUS_MEASURING,
	ECG_STATUS_MEASURE_FAIL,
	ECG_STATUS_MEASURE_OK,
	ECG_STATUS_NOTIFY,
	ECG_STATUS_MAX
}ECG_WORK_STATUS;

//sensor trigger type
typedef enum
{
	ECG_TRIGGER_BY_MENU		=	0x01,
	ECG_TRIGGER_BY_APP		=	0x02,
	ECG_TRIGGER_BY_HOURLY	=	0x04,
	ECG_TRIGGER_BY_FT		=	0x08,
}ECG_TRIGGER_SOUCE;

//ECG lead status
typedef enum
{
	ECG_LEAD_STATUS_UNKNOWN = 0,
	ECG_LEAD_STATUS_OFF,
	ECG_LEAD_STATUS_ON,
	ECG_LEAD_STATUS_TIMEOUT,
}ECG_LEAD_STATUS;

#ifdef CONFIG_FACTORY_TEST_SUPPORT
extern uint8_t ecg_test_info[256];
#endif

extern bool IsInEcgScreen(void);
extern void StartECG(ECG_TRIGGER_SOUCE trigger_type);
extern void MenuStartECG(void);
extern void MenuStopECG(void);

#endif/*__ECG_H__*/