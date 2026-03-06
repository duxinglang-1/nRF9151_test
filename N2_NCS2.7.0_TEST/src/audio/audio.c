/****************************************Copyright (c)************************************************
** File Name:			    audio.c
** Descriptions:			audio process source file
** Created By:				xie biao
** Created Date:			2021-03-04
** Modified Date:      		2021-05-08 
** Version:			    	V1.1
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include "audio.h"
#include "uart.h"
#include "logger.h"

//播放120报警声
void audio_play_alarm(void)
{
	uint8_t sound_id = 0x33;
	uint8_t buffer[64] = {0};
	uint32_t len;
	
	strcpy(buffer, COM_AUDIO_PLAY);
	len = strlen(COM_AUDIO_PLAY);
	memcpy(&buffer[len], &sound_id, sizeof(sound_id));
	CopcsSendData(UART_DATA_AUIOD, buffer, len+sizeof(sound_id));	
}

//播放中文语音提示
void audio_play_chn_voice(void)
{
	uint8_t sound_id = 0x31;
	uint8_t buffer[64] = {0};
	uint32_t len;
	
	strcpy(buffer, COM_AUDIO_PLAY);
	len = strlen(COM_AUDIO_PLAY);
	memcpy(&buffer[len], &sound_id, sizeof(sound_id));
	CopcsSendData(UART_DATA_AUIOD, buffer, len+sizeof(sound_id));
}

//播放英文语音提示
void audio_play_en_voice(void)
{
	uint8_t sound_id = 0x32;
	uint8_t buffer[64] = {0};
	uint32_t len;
	
	strcpy(buffer, COM_AUDIO_PLAY);
	len = strlen(COM_AUDIO_PLAY);
	memcpy(&buffer[len], &sound_id, sizeof(sound_id));
	CopcsSendData(UART_DATA_AUIOD, buffer, len+sizeof(sound_id));
}

//启动/停止循环播放
void audio_loop(void)
{
	CopcsSendData(UART_DATA_AUIOD, CPM_AUDIO_REPEAT, strlen(CPM_AUDIO_REPEAT));
}

//停止播放
void audio_stop(void)
{
	CopcsSendData(UART_DATA_AUIOD, COM_AUDIO_STOP, strlen(COM_AUDIO_STOP));
}

//SOS停止播放报警
void SOSStopAlarm(void)
{
	CopcsSendData(UART_DATA_AUIOD, COM_AUDIO_STOP, strlen(COM_AUDIO_STOP));
}

//SOS播放报警
void SOSPlayAlarm(void)
{
	uint8_t sound_id = 0x30;
	uint8_t buffer[64] = {0};
	uint32_t len;

	strcpy(buffer, COM_AUDIO_PLAY);
	len = strlen(COM_AUDIO_PLAY);
	memcpy(&buffer[len], &sound_id, sizeof(sound_id));
	CopcsSendData(UART_DATA_AUIOD, buffer, len+sizeof(sound_id));
}

//摔倒停止播放报警
void FallStopAlarm(void)
{
	CopcsSendData(UART_DATA_AUIOD, COM_AUDIO_STOP, strlen(COM_AUDIO_STOP));
}

//摔倒播放中文报警
void FallPlayAlarmCn(void)
{
	uint8_t sound_id = 1;
	uint8_t buffer[64] = {0};
	uint32_t len;
	
	strcpy(buffer, COM_AUDIO_PLAY);
	len = strlen(COM_AUDIO_PLAY);
	memcpy(&buffer[len], &sound_id, sizeof(sound_id));
	CopcsSendData(UART_DATA_AUIOD, buffer, len+sizeof(sound_id));
}

//摔倒播放英文报警
void FallPlayAlarmEn(void)
{
	uint8_t sound_id = 2;
	uint8_t buffer[64] = {0};
	uint32_t len;
	
	strcpy(buffer, COM_AUDIO_PLAY);
	len = strlen(COM_AUDIO_PLAY);
	memcpy(&buffer[len], &sound_id, sizeof(sound_id));
	CopcsSendData(UART_DATA_AUIOD, buffer, len+sizeof(sound_id));
}

void UartAudioEventHandle(uint8_t *data, uint32_t data_len)
{
	uint8_t *ptr;
	
	if(data == NULL || data_len == 0)
		return;

	ptr = strstr(data, AUDIO_DATA_HEAD);
	if(ptr != NULL)
	{
		uint8_t *ptr1,*ptr2;

		ptr += strlen(AUDIO_DATA_HEAD);
		if((ptr1 = strstr(ptr, COM_AUDIO_GET_INFOR)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_PLAY)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_COMPLETED)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_STOP)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_PAUSE)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_RESUME)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_NEXT)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_PRE)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_VOL_INC)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_AUDIO_VOL_DEC)) != NULL)
		{
		}
	}
}

//io口初始化 
void audio_init(void)
{
	//Delay_ms(100);
	
	CopcsSendData(UART_DATA_AUIOD, COM_AUDIO_GET_INFOR, strlen(COM_AUDIO_GET_INFOR));
}

void AudioMsgProcess(void)
{
}
