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

#include <stdbool.h>
#include <stdint.h>
#include "settings.h"

#define COM_ECG_SET_OPEN		"OPEN:"
#define COM_ECG_SET_CLOSE		"CLOSE:"
#define COM_ECG_GET_INFOR		"INFOR:"
#define COM_ECG_GET_DATA		"ECG_DATA:"
#define COM_ECG_LEAD_STATUS		"LEAD_STATUS:"
#define COM_ECG_LEAD_OFF		"LEAD_OFF:"
#define COM_ECG_LEAD_ON		    "LEAD_ON:"
#define COM_ECG_LEAD_TIME_OUT	"LEAD_TIME_OUT:"
#define COM_ECG_WEAR_STATUS		"WEAR_STATUS:"
#define COM_ECG_WEAR_LEFT		"WEAR_LEFT:"
#define COM_ECG_WEAR_RIGHT		"WEAR_RIGHT:"
#define COM_ECG_HR_DATA			"ECG_HR:"
#define COM_ECG_HRV_DATA		"ECG_HRV:"

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

//ECG display phase
typedef enum
{
	ECG_DISPLAY_PREPARE,
	ECG_DISPLAY_WAVE,
}ECG_DISPLAY_PHASE;

#ifdef CONFIG_FACTORY_TEST_SUPPORT
extern uint8_t ecg_test_info[256];
#endif

extern ECG_DISPLAY_PHASE g_ecg_display_phase;
extern ECG_LEAD_STATUS g_ecg_lead_status;
extern bool g_ecg_lead_on_ready;
extern bool g_ecg_lead_off_timeout;

// UART ??????????????????????????????????????? g_ecg_lead_status/????????/???????
// ??? uart_rece_cache ?????????? ECG ???????? k_sleep ??? LEAD_OFF ?????????
extern void EcgPeekLeadStatus(const uint8_t *data, uint32_t data_len);

extern bool IsInEcgScreen(void);
extern void StartECG(ECG_TRIGGER_SOUCE trigger_type);
extern void MenuStartECG(WEAR_WAY wear_way);
extern void MenuStopECG(void);
extern void ExitEcgScreen(void);

typedef struct {
    uint16_t hr;
    uint16_t hrv;
} ecg_health_data_t;

extern ecg_health_data_t g_ecg_health_data;


#endif/*__ECG_H__*/