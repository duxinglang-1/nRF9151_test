/****************************************Copyright (c)************************************************
** File Name:			    audio.h
** Descriptions:			audio process head file
** Created By:				xie biao
** Created Date:			2021-03-04
** Modified Date:      		2021-05-08 
** Version:			    	V1.1
******************************************************************************************************/
#ifndef __AUDIO_H__
#define __AUDIO_H__

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>

#define COM_AUDIO_GET_INFOR		"INFOR:"
#define COM_AUDIO_PLAY			"PLAY:"
#define COM_AUDIO_STOP			"STOP"
#define COM_AUDIO_PAUSE			"PAUSE"
#define COM_AUDIO_RESUME		"RESUME"
#define COM_AUDIO_NEXT			"NEXT"
#define COM_AUDIO_PRE			"PRE"
#define COM_AUDIO_VOL_INC		"VOL_INC"
#define COM_AUDIO_VOL_DEC		"VOL_DEC"

extern void audio_init(void);
extern void AudioMsgProcess(void);
extern void SOSStopAlarm(void);
extern void SOSPlayAlarm(void);
extern void FallStopAlarm(void);
extern void FallPlayAlarmCn(void);
extern void FallPlayAlarmEn(void);

#endif/*__AUDIO_H__*/
