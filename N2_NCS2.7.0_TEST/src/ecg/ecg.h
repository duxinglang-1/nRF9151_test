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

//#define ECG_ADS1292
#define ECG_MAX86176

#define ECG_CHECK_MENU				60
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

#ifdef CONFIG_FACTORY_TEST_SUPPORT
extern uint8_t ecg_test_info[256];
#endif

#endif/*__ECG_H__*/