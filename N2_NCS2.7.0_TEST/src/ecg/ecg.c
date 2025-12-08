/****************************************Copyright (c)************************************************
** File Name:			    ecg.c
** Descriptions:			ecg function main source file
** Created By:				xie biao
** Created Date:			2024-04-11
** Modified Date:      		2024-04-11
** Version:			    	V1.0
******************************************************************************************************/
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include "logger.h"
#include "uart.h"
#include "ecg.h"
#ifdef ECG_ADS1292
#include "ads1292.h"
#endif

void ECGKeyPressed(void)
{
#ifdef ECG_ADS1292
	ADS1x9x_Disable_Start();
	ADS1x9x_Enable_Start();
	ADS1x9x_Disable_Start();			// Disable START (SET START to high)

	ECG_CS_LOW();						// CS = 0
	k_sleep(K_MSEC(1));
	ECG_CS_HIGH();		// CS = 1
	k_sleep(K_MSEC(5));
	ECG_CS_LOW();			// CS =0

	k_sleep(K_MSEC(5));
	Start_Read_Data_Continuous();		//RDATAC command
	k_sleep(K_MSEC(5));
	ADS1x9x_Interrupt_Enable();
	ADS1x9x_Enable_Start(); 			// Enable START (SET START to high)

	ECG_Recoder_state.state = ECG_STATE_RECORDING;
#endif
}

void ECGkeyReleased(void)
{
#ifdef ECG_ADS1292
	ECG_Recoder_state.state = ECG_STATE_IDLE;
	ADS1x9x_Interrupt_Disable();		// Disable DRDY interrupt
	Stop_Read_Data_Continuous();		// SDATAC command
#endif
}

void ECG_Start(void)
{
#ifdef ECG_ADS1292
	
#endif
}

void ECG_Stop(void)
{
#ifdef ECG_ADS1292
	ECG_Recoder_state.state = ECG_STATE_IDLE;
	ADS1x9x_Interrupt_Disable();		// Disable DRDY interrupt
	Stop_Read_Data_Continuous();		// SDATAC command
#endif
}

void ECGDataProcess(uint8_t *data, uint32_t data_len)
{
}

void UartECGEventHandle(uint8_t *data, uint32_t data_len)
{
	uint8_t *ptr;
	static uint32_t page_num=0,flash_partial=0;

	if(data == NULL || data_len == 0)
		return;

	ptr = strstr(data, ECG_DATA_HEAD);
	if(ptr != NULL)
	{
		uint8_t *ptr1,*ptr2;

		ptr += strlen(ECG_DATA_HEAD);
		if((ptr1 = strstr(ptr, COM_ECG_SET_OPEN)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_ECG_SET_CLOSE)) != NULL)
		{
		}
		else if((ptr1 = strstr(ptr, COM_ECG_GET_DATA)) != NULL)
		{
			ptr1 += strlen(COM_ECG_GET_DATA);
			ECGDataProcess(ptr1, data_len-(ptr1-data));
		}
		else if((ptr1 = strstr(ptr, COM_ECG_GET_INFOR)) != NULL)
		{
		}
	}
}

void ECG_init(void)
{
#ifdef ECG_ADS1292
	ADS1x9x_Init();
#endif
}

void ECGMsgProcess(void)
{
#ifdef ECG_ADS1292
	ADS1x9x_Msg_Process();
#endif
}
