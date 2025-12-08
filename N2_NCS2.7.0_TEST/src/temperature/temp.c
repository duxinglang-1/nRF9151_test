/****************************************Copyright (c)************************************************
** File Name:			    temp.c
** Descriptions:			temperature message process source file
** Created By:				xie biao
** Created Date:			2021-12-24
** Modified Date:      		
** Version:			    	V1.0
******************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include "external_flash.h"
#include "lcd.h"
#include "screen.h"
#include "uart.h"
#include "temp.h"
#include "inner_flash.h"
#include "logger.h"
#ifdef CONFIG_PPG_SUPPORT
#include "max32674.h"
#endif
#ifdef CONFIG_CRC_SUPPORT
#include "crc_check.h"
#endif

//#define TEMP_DEBUG

static bool temp_check_ok = false;
static bool temp_start_flag = false;
static bool temp_test_flag = false;
static bool temp_stop_flag = false;
static bool temp_redraw_data_flag = false;
static bool temp_power_flag = false;
static bool menu_start_temp = false;
static bool ft_start_temp = false;

static uint32_t measure_count = 0;
static float t_sensor = 0.0;		//传感器温度值
static float t_body = 0.0; 			//显示的温度值
static float t_predict = 0.0;		//预测的人体温度值
static float t_temp80 = 0.0;		//预测的人体温度值

static uint8_t rec2buf[TEMP_REC2_DATA_SIZE] = {0};
static uint8_t databuf[TEMP_REC2_DATA_SIZE] = {0};

#ifdef CONFIG_CRC_SUPPORT
static CRC_8 crc_8_CUSTOM = {0x31,0xff,0x00,false,false};
#endif

bool get_temp_ok_flag = false;

#ifndef CONFIG_PPG_SUPPORT
sys_date_timer_t g_health_check_time = {0};
#endif
TEMP_WORK_STATUS g_temp_status = TEMP_STATUS_PREPARE;

uint8_t g_temp_trigger = 0;
float g_temp_skin = 0.0;
float g_temp_body = 0.0;
float g_temp_menu = 0.0;
float g_temp_hourly = 0.0;

static void temp_auto_stop_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(temp_stop_timer, temp_auto_stop_timerout, NULL);
static void temp_menu_stop_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(temp_menu_stop_timer, temp_menu_stop_timerout, NULL);

static void temp_auto_stop_timerout(struct k_timer *timer_id)
{
	if((g_temp_trigger&TEMP_TRIGGER_BY_MENU) == 0)
		temp_stop_flag = true;
}

static void temp_menu_stop_timerout(struct k_timer *timer_id)
{
	temp_stop_flag = true;
	
	if(screen_id == SCREEN_ID_TEMP)
	{
		g_temp_status = TEMP_STATUS_MEASURE_FAIL;
		
		scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_TEMP;
		scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
	}
}

void ClearAllTempRecData(void)
{
	g_temp_skin = 0.0;
	g_temp_body = 0.0;
	g_temp_menu = 0.0;
	memset(&rec2buf, 0xff, sizeof(rec2buf));

	SpiFlash_Write(rec2buf, TEMP_REC2_DATA_ADDR, TEMP_REC2_DATA_SIZE);
}

void SetCurDayTempRecData(sys_date_timer_t time_stamp, float data)
{
	uint16_t i,deca_temp = round(data*10.0);
	temp_rec2_nod *p_temp,tmp_temp = {0};

	tmp_temp.year = time_stamp.year;
	tmp_temp.month = time_stamp.month;
	tmp_temp.day = time_stamp.day;
	tmp_temp.hour = time_stamp.hour;
	tmp_temp.min = time_stamp.minute;
	tmp_temp.deca_temp = deca_temp;

	memset(&databuf, 0x00, sizeof(databuf));
	memset(&rec2buf, 0x00, sizeof(rec2buf));
	
	SpiFlash_Read(rec2buf, TEMP_REC2_DATA_ADDR, TEMP_REC2_DATA_SIZE);
	p_temp = (temp_rec2_nod*)rec2buf;
	if((p_temp->year == 0xffff || p_temp->year == 0x0000)
		||(p_temp->month == 0xff || p_temp->month == 0x00)
		||(p_temp->day == 0xff || p_temp->day == 0x00)
		||(p_temp->hour == 0xff || p_temp->min == 0xff)
		||((p_temp->year == tmp_temp.year)
			&&(p_temp->month == tmp_temp.month)
			&&(p_temp->day == tmp_temp.day)
			&&(p_temp->hour == tmp_temp.hour)
			&&(p_temp->min == tmp_temp.min))
		)
	{
		//直接覆盖写在第一条
		memcpy(p_temp, &tmp_temp, sizeof(temp_rec2_nod));
		SpiFlash_Write(rec2buf, TEMP_REC2_DATA_ADDR, TEMP_REC2_DATA_SIZE);
	}
	else if((tmp_temp.year < p_temp->year)
			||((tmp_temp.year == p_temp->year)&&(tmp_temp.month < p_temp->month))
			||((tmp_temp.year == p_temp->year)&&(tmp_temp.month == p_temp->month)&&(tmp_temp.day < p_temp->day))
			||((tmp_temp.year == p_temp->year)&&(tmp_temp.month == p_temp->month)&&(tmp_temp.day == p_temp->day)&&(tmp_temp.hour < p_temp->hour))
			||((tmp_temp.year == p_temp->year)&&(tmp_temp.month == p_temp->month)&&(tmp_temp.day == p_temp->day)&&(tmp_temp.hour == p_temp->hour)&&(tmp_temp.min < p_temp->min))
			)
	{
		//插入新的数据,旧的数据往后挪，丢掉最后一个
		memcpy(&databuf[0*sizeof(temp_rec2_nod)], &tmp_temp, sizeof(temp_rec2_nod));
		memcpy(&databuf[1*sizeof(temp_rec2_nod)], &rec2buf[0*sizeof(temp_rec2_nod)], TEMP_REC2_DATA_SIZE-sizeof(temp_rec2_nod));
		SpiFlash_Write(databuf, TEMP_REC2_DATA_ADDR, TEMP_REC2_DATA_SIZE);
	}
	else
	{
		//寻找合适的插入位置
		for(i=0;i<TEMP_REC2_DATA_SIZE/sizeof(temp_rec2_nod);i++)
		{
			p_temp = rec2buf+i*sizeof(temp_rec2_nod);
			if((p_temp->year == 0xffff || p_temp->year == 0x0000)
				||(p_temp->month == 0xff || p_temp->month == 0x00)
				||(p_temp->day == 0xff || p_temp->day == 0x00)
				||(p_temp->hour == 0xff || p_temp->min == 0xff)
				||((p_temp->year == tmp_temp.year)
					&&(p_temp->month == tmp_temp.month)
					&&(p_temp->day == tmp_temp.day)
					&&(p_temp->hour == tmp_temp.hour)
					&&(p_temp->min == tmp_temp.min))
				)
			{
				//直接覆盖写
				memcpy(p_temp, &tmp_temp, sizeof(temp_rec2_nod));
				SpiFlash_Write(rec2buf, TEMP_REC2_DATA_ADDR, TEMP_REC2_DATA_SIZE);
				return;
			}
			else if((tmp_temp.year > p_temp->year)
					||((tmp_temp.year == p_temp->year)&&(tmp_temp.month > p_temp->month))
					||((tmp_temp.year == p_temp->year)&&(tmp_temp.month == p_temp->month)&&(tmp_temp.day > p_temp->day))
					||((tmp_temp.year == p_temp->year)&&(tmp_temp.month == p_temp->month)&&(tmp_temp.day == p_temp->day)&&(tmp_temp.hour > p_temp->hour))
					||((tmp_temp.year == p_temp->year)&&(tmp_temp.month == p_temp->month)&&(tmp_temp.day == p_temp->day)&&(tmp_temp.hour == p_temp->hour)&&(tmp_temp.min > p_temp->min))
					)
			{
				if(i < (TEMP_REC2_DATA_SIZE/sizeof(temp_rec2_nod)-1))
				{
					p_temp++;
					if((tmp_temp.year < p_temp->year)
						||((tmp_temp.year == p_temp->year)&&(tmp_temp.month < p_temp->month))
						||((tmp_temp.year == p_temp->year)&&(tmp_temp.month == p_temp->month)&&(tmp_temp.day < p_temp->day))
						||((tmp_temp.year == p_temp->year)&&(tmp_temp.month == p_temp->month)&&(tmp_temp.day == p_temp->day)&&(tmp_temp.hour < p_temp->hour))
						||((tmp_temp.year == p_temp->year)&&(tmp_temp.month == p_temp->month)&&(tmp_temp.day == p_temp->day)&&(tmp_temp.hour == p_temp->hour)&&(tmp_temp.min < p_temp->min))
						)
					{
						break;
					}
				}
			}
		}

		if(i < (TEMP_REC2_DATA_SIZE/sizeof(temp_rec2_nod)-1))
		{
			//找到位置，插入新数据，老数据整体往后挪，丢掉最后一个
			memcpy(&databuf[0*sizeof(temp_rec2_nod)], &rec2buf[0*sizeof(temp_rec2_nod)], (i+1)*sizeof(temp_rec2_nod));
			memcpy(&databuf[(i+1)*sizeof(temp_rec2_nod)], &tmp_temp, sizeof(temp_rec2_nod));
			memcpy(&databuf[(i+2)*sizeof(temp_rec2_nod)], &rec2buf[(i+1)*sizeof(temp_rec2_nod)], TEMP_REC2_DATA_SIZE-(i+1)*sizeof(temp_rec2_nod));
			SpiFlash_Write(databuf, TEMP_REC2_DATA_ADDR, TEMP_REC2_DATA_SIZE);
		}
		else
		{
			//未找到位置，直接接在末尾，老数据整体往前移，丢掉最前一个
			memcpy(&databuf[0*sizeof(temp_rec2_nod)], &rec2buf[1*sizeof(temp_rec2_nod)], TEMP_REC2_DATA_SIZE-sizeof(temp_rec2_nod));
			memcpy(&databuf[TEMP_REC2_DATA_SIZE-sizeof(temp_rec2_nod)], &tmp_temp, sizeof(temp_rec2_nod));
			SpiFlash_Write(databuf, TEMP_REC2_DATA_ADDR, TEMP_REC2_DATA_SIZE);
		}
	}
}

void GetCurDayTempRecData(uint8_t *databuf)
{
	uint16_t i,j=0;
	temp_rec2_nod *p_temp;
	
	if(databuf == NULL)
		return;

	memset(&rec2buf, 0x00, sizeof(rec2buf));
	SpiFlash_Read(rec2buf, TEMP_REC2_DATA_ADDR, TEMP_REC2_DATA_SIZE);
	p_temp = (temp_rec2_nod*)rec2buf;
	for(i=0;i<TEMP_REC2_DATA_SIZE/sizeof(temp_rec2_nod);i++)
	{
		if((p_temp->year == 0xffff || p_temp->year == 0x0000)
			||(p_temp->month == 0xff || p_temp->month == 0x00)
			||(p_temp->day == 0xff || p_temp->day == 0x00)
			||(p_temp->hour == 0xff || p_temp->min == 0xff)
			)
		{
			break;
		}
		else if((p_temp->year < date_time.year)
			||((p_temp->year == date_time.year)&&(p_temp->month < date_time.month))
			||((p_temp->year == date_time.year)&&(p_temp->month == date_time.month)&&(p_temp->day < date_time.day))
			)
		{
			p_temp++;
			continue;
		}
		else if((p_temp->year > date_time.year)
				||((p_temp->year == date_time.year)&&(p_temp->month > date_time.month))
				||((p_temp->year == date_time.year)&&(p_temp->month == date_time.month)&&(p_temp->day > date_time.day))
				)
		{
			break;
		}
		else
		{
			memcpy(&databuf[j*sizeof(temp_rec2_nod)], p_temp, sizeof(temp_rec2_nod));
			j++;
		}

		p_temp++;
	}
}

void GetGivenDayTempRecData(sys_date_timer_t date, uint8_t *databuf)
{
	uint16_t i,j=0;
	temp_rec2_nod *p_temp;

	if(!CheckSystemDateTimeIsValid(date))
		return;
	if(databuf == NULL)
		return;

	memset(&rec2buf, 0x00, sizeof(rec2buf));
	SpiFlash_Read(rec2buf, TEMP_REC2_DATA_ADDR, TEMP_REC2_DATA_SIZE);
	p_temp = (temp_rec2_nod*)rec2buf;
	for(i=0;i<TEMP_REC2_DATA_SIZE/sizeof(hr_rec2_nod);i++)
	{
		if((p_temp->year == 0xffff || p_temp->year == 0x0000)
			||(p_temp->month == 0xff || p_temp->month == 0x00)
			||(p_temp->day == 0xff || p_temp->day == 0x00)
			||(p_temp->hour == 0xff || p_temp->min == 0xff)
			)
		{
			break;
		}
		else if((p_temp->year < date.year)
			||((p_temp->year == date.year)&&(p_temp->month < date.month))
			||((p_temp->year == date.year)&&(p_temp->month == date.month)&&(p_temp->day < date.day))
			)
		{
			p_temp++;
			continue;
		}
		else if((p_temp->year > date.year)
				||((p_temp->year == date.year)&&(p_temp->month > date.month))
				||((p_temp->year == date.year)&&(p_temp->month == date.month)&&(p_temp->day > date.day))
				)
		{
			break;
		}
		else
		{
			memcpy(&databuf[j*sizeof(temp_rec2_nod)], p_temp, sizeof(temp_rec2_nod));
			j++;
		}

		p_temp++;
	}
}

void GetGivenTimeTempRecData(sys_date_timer_t date, uint16_t *temp)
{
	uint16_t i;
	temp_rec2_nod *p_temp;

	if(!CheckSystemDateTimeIsValid(date))
		return;	
	if(temp == NULL)
		return;

	memset(&rec2buf, 0x00, sizeof(rec2buf));
	SpiFlash_Read(rec2buf, TEMP_REC2_DATA_ADDR, TEMP_REC2_DATA_SIZE);
	p_temp = (temp_rec2_nod*)rec2buf;
	for(i=0;i<TEMP_REC2_DATA_SIZE/sizeof(temp_rec2_nod);i++)
	{
		if((p_temp->year == 0xffff || p_temp->year == 0x0000)
			||(p_temp->month == 0xff || p_temp->month == 0x00)
			||(p_temp->day == 0xff || p_temp->day == 0x00)
			||(p_temp->hour == 0xff || p_temp->min == 0xff)
			)
		{
			break;
		}
		
		if((p_temp->year == date.year)
			&&(p_temp->month == date.month)
			&&(p_temp->day == date.day)
			&&(p_temp->hour == date.hour)
			&&(p_temp->min == date.minute)
			)
		{
			*temp = p_temp->deca_temp;
			break;
		}

		p_temp++;
	}
}

void UpdateLastTempData(sys_date_timer_t time_stamp, float data)
{
	uint16_t dec_temp = round(data*10.0);

	memcpy(&last_health.temp_rec.timestamp, &time_stamp, sizeof(sys_date_timer_t));
	last_health.temp_rec.deca_temp = dec_temp;
	if(dec_temp > last_health.deca_temp_max)
	{
		if(last_health.deca_temp_min == 0)
		{
			if(last_health.deca_temp_max > 0)
				last_health.deca_temp_min = last_health.deca_temp_max;
			else
				last_health.deca_temp_min = dec_temp;
		}
		last_health.deca_temp_max = dec_temp;
	}
	else if(dec_temp < last_health.deca_temp_min)
	{
		last_health.deca_temp_min = dec_temp;
	}

	save_cur_health_to_record(&last_health);
}

bool IsInTempScreen(void)
{
	if(screen_id == SCREEN_ID_TEMP)
		return true;
	else
		return false;
}

bool TempIsWorkingTiming(void)
{
	if((g_temp_trigger&TEMP_TRIGGER_BY_HOURLY) != 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool TempIsWorking(void)
{
	if(temp_power_flag == false)
		return false;
	else
		return true;
}

void TempStop(void)
{
	temp_stop_flag = true;
}

void TempRedrawHourlyData(void)
{
	if(screen_id == SCREEN_ID_IDLE)
	{
		if((g_temp_hourly >= (TEMP_MIN/10.0))&&(g_temp_hourly <= (TEMP_MAX/10.0)))
		{
			scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_TEMP;
			scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
		}
	}
}

void TempRedrawData(void)
{
	if(screen_id == SCREEN_ID_TEMP)
	{
		scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_TEMP;
		scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
	}
}

void StartTemp(TEMP_TRIGGER_SOUCE trigger_type)
{
	notify_infor infor = {0};

	infor.x = 0;
	infor.y = 0;
	infor.w = LCD_WIDTH;
	infor.h = LCD_HEIGHT;
	infor.align = NOTIFY_ALIGN_CENTER;
	infor.type = NOTIFY_TYPE_POPUP;


	if(1
	#ifdef CONFIG_FACTORY_TEST_SUPPORT
		&& !IsFTTempTesting()
	#endif
		)
	{
		StartSCC();
	}

	switch(trigger_type)
	{
	case TEMP_TRIGGER_BY_HOURLY:
		g_temp_hourly = 0.0;
	case TEMP_TRIGGER_BY_APP:
	case TEMP_TRIGGER_BY_FT:
		if(!is_wearing())
		{
			return;
		}
		break;
		
	case TEMP_TRIGGER_BY_MENU:
		if(!is_wearing())
		{
			infor.img[0] = IMG_WRIST_OFF_ICON_ADDR;
			infor.img_count = 1;

			DisplayPopUp(infor);
			return;
		}

		g_temp_menu = 0.0;
		break;
	}

	g_temp_trigger |= trigger_type;
	g_temp_skin = 0.0;
	g_temp_body = 0.0;
	get_temp_ok_flag = false;

	temp_start_flag = true;
}

void MenuStartTemp(void)
{
	menu_start_temp = true;
}

void MenuStopTemp(void)
{
	temp_stop_flag = true;
}

#ifdef CONFIG_FACTORY_TEST_SUPPORT
void FTStartTemp(void)
{
	ft_start_temp = true;
}

void FTStopTemp(void)
{
	temp_stop_flag = true;
}
#endif

void TempDataProcess(uint8_t *data, uint32_t len)
{
	bool flag=false;
	uint8_t crc=0;
	uint8_t databuf[10] = {0};
	uint16_t trans_temp = 0;
	float skin_temp, body_temp;

	if(!CheckSCC()
	#ifdef CONFIG_FACTORY_TEST_SUPPORT
		&& !IsFTTempTesting()
		&& !IsFTTempAging()
	#endif
		)
	{
		notify_infor infor = {0};

		infor.x = 0;
		infor.y = 0;
		infor.w = LCD_WIDTH;
		infor.h = LCD_HEIGHT;
		infor.align = NOTIFY_ALIGN_CENTER;
		infor.type = NOTIFY_TYPE_POPUP;

		if((g_temp_trigger&TEMP_TRIGGER_BY_MENU) == TEMP_TRIGGER_BY_MENU)
		{
			infor.img[0] = IMG_WRIST_OFF_ICON_ADDR;
			infor.img_count = 1;
			DisplayPopUp(infor);
		}
		
		temp_stop_flag = true;
		return;
	}
	
#ifdef TEMP_DEBUG
	LOGD("temp:%02x,%02x,%02x", data[0],data[1],data[2]);
#endif

#ifdef CONFIG_CRC_SUPPORT
	crc = crc8_cal(data, 2, crc_8_CUSTOM);
#ifdef TEMP_DEBUG
	LOGD("crc:%02x", crc);
#endif
	if(crc != data[2])
		return;
#endif

	trans_temp = data[0]*0x100 + data[1];
	t_sensor = 175.0*(float)trans_temp/65535.0-45.0;
	if(t_sensor > 99.9)
		t_sensor = 0.0;
	skin_temp = t_sensor;
	body_temp = 0;

#ifdef TEMP_DEBUG
	LOGD("count:%d, real temp:%d.%d", measure_count, (int16_t)(t_sensor*10)/10, (int16_t)(t_sensor*10)%10);
#endif

	if(t_sensor > 28)			//如果上一次测温大于32，那么开始计数
	{
		measure_count = measure_count+1;
	}
	else if(measure_count == 2000)
	{
		measure_count = 2000;
	}
	else
	{
		measure_count = 0;
	}

	if(measure_count == 0)
	{
		t_body = t_sensor; 
		t_predict = 0;
	}
	else if((measure_count > 0)&&(measure_count <20))
	{
		t_body = t_sensor;
	}
	else if(measure_count == 20)
	{
		t_body = t_sensor;
		t_temp80 = t_sensor;
		if((t_sensor > 36)&&(t_sensor <= 41))
			t_predict = 36.9 + (t_sensor-36)*4.1/5;
		else if((t_sensor > 28)&&(t_sensor <= 36))
			t_predict = 36.1 + (t_sensor-28)*0.8/8; 
		else
			t_predict = t_sensor;
	}
	else if((measure_count > 20)&&(measure_count <= 25))
	{
		t_body = ((measure_count-20)*0.8*(t_predict-t_temp80))/5 + t_temp80;
	}

	else if((measure_count > 25)&&(measure_count <= 30))
	{
		t_body = t_predict - ((30-measure_count)*0.2*(t_predict-t_temp80))/5;
	}
	else
	{
		if((t_sensor > 36)&&(t_sensor <= 41))
			t_body = 36.9 + (t_sensor-36)*4.1/5;
		else if((t_sensor > 28)&&(t_sensor <= 36))
			t_body = 36.1 + (t_sensor-28)*0.8/8;
		else
			t_body = t_sensor;

	#ifdef CONFIG_FACTORY_TEST_SUPPORT
		if(!IsFTTempAging())
	#endif

		flag = true;
	}

	body_temp = t_body;

#ifdef TEMP_DEBUG
	LOGD("flag:%d, t_temp80:%d.%d, t_body:%d.%d", flag, (int16_t)(t_temp80*10)/10, (int16_t)(t_temp80*10)%10, (int16_t)(t_body*10)/10, (int16_t)(t_body*10)%10);
#endif

	if(skin_temp > 0.0)
	{
		g_temp_skin = skin_temp;
		if(body_temp >= TEMP_MIN/10.0)
		{
			g_temp_body = body_temp;
			if(flag)
			{
				temp_stop_flag = true;
				get_temp_ok_flag = true;
			}
		}

		temp_redraw_data_flag = true;

	#ifdef CONFIG_FACTORY_TEST_SUPPORT
		if(IsFTTempTesting())
			FTTempStatusUpdate();
	#endif
	}
}

void UartTempEventHandle(uint8_t *data, uint32_t data_len)
{
	uint8_t *ptr;
	
	if(data == NULL || data_len == 0)
		return;

	ptr = strstr(data, TEMP_DATA_HEAD);
	if(ptr != NULL)
	{
		uint8_t *ptr1,*ptr2;

		ptr += strlen(TEMP_DATA_HEAD);
		if((ptr1 = strstr(ptr, COM_TEMP_GET_INFOR)) != NULL)
		{
			uint8_t buffer[3] = {0};
			
			ptr1 += strlen(COM_TEMP_GET_INFOR);
			memcpy(buffer, ptr1, data_len-(ptr1-data));
		#ifdef TEMP_DEBUG
			LOGD("sensor id:%d", buffer[0]*0x100 + buffer[1]);
		#endif

			temp_check_ok = true;
		}
		else if((ptr1 = strstr(ptr, COM_TEMP_GET_DATA)) != NULL)
		{
			ptr1 += strlen(COM_TEMP_GET_DATA);
			
			TempDataProcess(ptr1, data_len-(ptr1-data));
		}
	}
}

void temp_init(void)
{
	//Display the last record within 7 days.
	get_cur_health_from_record(&last_health);
	DateIncrease(&last_health.temp_rec.timestamp, 7);
	if(DateCompare(last_health.temp_rec.timestamp, date_time) > 0)
	{
		g_temp_body = (float)(last_health.temp_rec.deca_temp/10.0);
	}
	
	CopcsSendData(UART_DATA_TEMP, COM_TEMP_GET_INFOR, strlen(COM_TEMP_GET_INFOR));
}

void TempMsgProcess(void)
{
	if(menu_start_temp)
	{
		StartTemp(TEMP_TRIGGER_BY_MENU);
		menu_start_temp = false;
	}

	if(ft_start_temp)
	{
		StartTemp(TEMP_TRIGGER_BY_FT);
		ft_start_temp = false;
	}
	
	if(temp_start_flag)
	{
		temp_start_flag = false;
		if(temp_power_flag)
			return;
		
		CopcsSendData(UART_DATA_TEMP, COM_TEMP_GET_DATA, strlen(COM_TEMP_GET_DATA));
		temp_power_flag = true;
	
		if((g_temp_trigger&TEMP_TRIGGER_BY_HOURLY) == TEMP_TRIGGER_BY_HOURLY)
		{
			k_timer_start(&temp_stop_timer, K_MSEC(TEMP_CHECK_TIMELY*60*1000), K_NO_WAIT);
		}
	#ifndef UI_STYLE_HEALTH_BAR	
		else if((g_temp_trigger&TEMP_TRIGGER_BY_MENU) == TEMP_TRIGGER_BY_MENU)
		{
			k_timer_start(&temp_menu_stop_timer, K_SECONDS(TEMP_CHECK_MENU), K_NO_WAIT);
		}
	#endif
	}

	if(temp_stop_flag)
	{
		temp_stop_flag = false;
		if(!temp_power_flag)
			return;
		
		CopcsSendData(UART_DATA_TEMP, COM_TEMP_SET_CLOSE, strlen(COM_TEMP_SET_CLOSE));
	
		temp_power_flag = false;
		k_timer_stop(&temp_stop_timer);
		k_timer_stop(&temp_menu_stop_timer);

	#ifdef CONFIG_BLE_SUPPORT
		if((g_temp_trigger&TEMP_TRIGGER_BY_APP) != 0)
		{
			uint8_t data[2] = {0};
			uint16_t deca_temp = 0;
			
			g_temp_trigger = g_temp_trigger&(~TEMP_TRIGGER_BY_APP);

			if(get_temp_ok_flag)
				deca_temp = round(g_temp_body*10.0);
			
			data[0] = deca_temp>>8;
			data[1] = (uint8_t)(deca_temp&0x00ff);
			MCU_send_app_get_temp_data(data);
		}
	#endif	
		if((g_temp_trigger&TEMP_TRIGGER_BY_MENU) != 0)
		{
			g_temp_trigger = g_temp_trigger&(~TEMP_TRIGGER_BY_MENU);

			if(get_temp_ok_flag)
			{
				g_temp_menu = g_temp_body;
				UpdateLastTempData(date_time, g_temp_menu);

			#ifdef CONFIG_BLE_SUPPORT
				if(g_ble_connected)
				{
					uint8_t data[2] = {0};
					uint16_t deca_temp = 0;

					deca_temp = round(g_temp_body*10.0);
					data[0] = deca_temp>>8;
					data[1] = (uint8_t)(deca_temp&0x00ff);
					MCU_send_app_get_temp_data(data);
				}
			#endif
			
				SyncSendHealthData();
				g_temp_menu = 0;
			}
		}
		if((g_temp_trigger&TEMP_TRIGGER_BY_HOURLY) != 0)
		{
			g_temp_trigger = g_temp_trigger&(~TEMP_TRIGGER_BY_HOURLY);
			if(!ppg_skin_contacted_flag)
			{
			#ifdef CONFIG_PPG_SUPPORT
				bpt_data tmp_bpt = {254,254};

				g_hr_hourly = 0;
				g_spo2_hourly = 0;
				memset(&g_bpt_hourly, 0x00, sizeof(bpt_data));
				
				SetCurDayHrRecData(g_health_check_time, 254);
				SetCurDaySpo2RecData(g_health_check_time, 254);
				SetCurDayBptRecData(g_health_check_time, tmp_bpt);
			#endif
				SetCurDayTempRecData(g_health_check_time, 254.0);
			}
			else
			{
				g_temp_hourly = g_temp_body;
				
				SetCurDayTempRecData(g_health_check_time, g_temp_body);
			#ifdef CONFIG_PPG_SUPPORT
				StartPPG(PPG_DATA_HR, TRIGGER_BY_HOURLY);
			#endif
			}
		}
	#ifdef CONFIG_FACTORY_TEST_SUPPORT	
		if((g_temp_trigger&TEMP_TRIGGER_BY_FT) != 0)
		{
			g_temp_trigger = g_temp_trigger&(~TEMP_TRIGGER_BY_FT);
			return;
		}
	#endif
	}

	if(temp_redraw_data_flag)
	{
		TempRedrawData();
		temp_redraw_data_flag = false;
	}
}

