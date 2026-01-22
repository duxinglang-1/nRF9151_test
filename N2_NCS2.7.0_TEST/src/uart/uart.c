/*
* Copyright (c) 2019 Nordic Semiconductor ASA
*
* SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
*/
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <stdio.h>
#include <string.h>
#include "logger.h"
#include "transfer_cache.h"
#include "datetime.h"
#include "Settings.h"
#include "Uart.h"
#ifdef CONFIG_BLE_SUPPORT
#include "ble.h"
#endif
#ifdef CONFIG_TOUCH_SUPPORT
#include "CST816.h"
#endif
#include "gps.h"
#include "max20353.h"
#ifdef CONFIG_PPG_SUPPORT
#include "max32674.h"
#endif
#include "screen.h"
#include "inner_flash.h"
#ifdef CONFIG_WIFI_SUPPORT
#include "esp8266.h"
#endif

//#define UART_DEBUG

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)
#define COPCS_DEV DT_NODELABEL(uart0)
#else
#error "uart0 devicetree node is disabled"
#define COPCS_DEV	""
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio0), okay)
#define COPCS_PORT DT_NODELABEL(gpio0)
#else
#error "gpio0 devicetree node is disabled"
#define COPCS_PORT	""
#endif

#define COPCS_INT_PIN		7
#define COPCS_WAKE_PIN		4

#define BUF_MAXSIZE	4096

#define PACKET_HEAD	0xAB
#define PACKET_END	0x88

#define BLE_WORK_MODE_ID		0xFF10			//5340工作状态正常
#define HEART_RATE_ID			0xFF31			//心率
#define BLOOD_OXYGEN_ID			0xFF32			//血氧
#define BLOOD_PRESSURE_ID		0xFF33			//血压
#define TEMPERATURE_ID			0xFF62			//体温
#define	ONE_KEY_MEASURE_ID		0xFF34			//一键测量
#define	PULL_REFRESH_ID			0xFF35			//下拉刷新
#define	SLEEP_DETAILS_ID		0xFF36			//睡眠详情
#define	FIND_DEVICE_ID			0xFF37			//查找手环
#define SMART_NOTIFY_ID			0xFF38			//智能提醒
#define	ALARM_SETTING_ID		0xFF39			//闹钟设置
#define USER_INFOR_ID			0xFF40			//用户信息
#define	SEDENTARY_ID			0xFF41			//久坐提醒
#define	SHAKE_SCREEN_ID			0xFF42			//抬手亮屏
#define	MEASURE_HOURLY_ID		0xFF43			//整点测量设置
#define	SHAKE_PHOTO_ID			0xFF44			//摇一摇拍照
#define	LANGUAGE_SETTING_ID		0xFF45			//中英日文切换
#define	TIME_24_SETTING_ID		0xFF46			//12/24小时设置
#define	FIND_PHONE_ID			0xFF47			//查找手机回复
#define	WEATHER_INFOR_ID		0xFF48			//天气信息下发
#define	TIME_SYNC_ID			0xFF49			//时间同步
#define	TARGET_STEPS_ID			0xFF50			//目标步数
#define	BATTERY_LEVEL_ID		0xFF51			//电池电量
#define	FIRMWARE_INFOR_ID		0xFF52			//固件版本号
#define	FACTORY_RESET_ID		0xFF53			//清除手环数据
#define	ECG_ID					0xFF54			//心电
#define	LOCATION_ID				0xFF55			//获取定位信息
#define	DATE_FORMAT_ID			0xFF56			//年月日格式设置
#define NOTIFY_CONTENT_ID		0xFF57			//智能提醒内容
#define CHECK_WHITELIST_ID		0xFF58			//判断手机ID是否在手环白名单
#define INSERT_WHITELIST_ID`	0xFF59			//将手机ID插入白名单
#define DEVICE_SEND_128_RAND_ID	0xFF60			//手环发送随机的128位随机数
#define PHONE_SEND_128_AES_ID	0xFF61			//手机发送AES 128 CBC加密数据给手环

#define	BLE_CONNECT_ID			0xFFB0			//BLE断连提醒
#define	CTP_NOTIFY_ID			0xFFB1			//CTP触屏消息
#define GET_BLE_VER_ID			0xFFB2			//获取BLE版本号
#define GET_BLE_MAC_ADDR_ID		0xFFB3			//获取BLE MAC地址
#define GET_BLE_STATUS_ID		0xFFB4			//获取BLE当前工作状态	0:关闭 1:休眠 2:广播 3:连接
#define SET_BEL_WORK_MODE_ID	0xFFB5			//设置BLE工作模式		0:关闭 1:打开 2:唤醒 3:休眠


#ifdef CONFIG_PM_DEVICE
bool uart_sleep_flag = false;
bool uart_wake_flag = false;
bool uart_is_waked = true;
#define UART_WAKE_HOLD_TIME_SEC		(5)
#endif
static bool reply_cur_data_flag = false;
static bool uart_send_data_flag = false;
static bool uart_rece_data_flag = false;
static bool uart_rece_frame_flag = false;

static CacheInfo uart_send_cache = {0};
static CacheInfo uart_rece_cache = {0};

static uint32_t uart_rece_len=0;
static uint32_t uart_send_len=0;
static uint8_t uart_rx_buf[BUF_MAXSIZE]={0};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static struct device *uart_copcs = NULL;
static struct device *gpio_copcs = NULL;
static struct gpio_callback gpio_cb;

static struct uart_data_t
{
	void  *fifo_reserved;
	uint8_t    data[BUF_MAXSIZE];
	uint16_t   len;
};

static void UartSendDataCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_send_data_timer, UartSendDataCallBack, NULL);
static void UartReceDataCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_rece_data_timer, UartReceDataCallBack, NULL);
static void UartReceFrameCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_rece_frame_timer, UartReceFrameCallBack, NULL);
#ifdef CONFIG_PM_DEVICE
static void UartSleepInCallBack(struct k_timer *timer_id);
K_TIMER_DEFINE(uart_sleep_in_timer, UartSleepInCallBack, NULL);
#endif


void MCU_wakeup_copcs(void)
{
	gpio_pin_set(gpio_copcs, COPCS_WAKE_PIN, 0);
	k_sleep(K_MSEC(5));
	gpio_pin_set(gpio_copcs, COPCS_WAKE_PIN, 1);
}

static void uart_send_data_handle(struct device *dev, uint8_t *buffer, uint32_t datalen)
{
	MCU_wakeup_copcs();

#ifdef CONFIG_PM_DEVICE
	uart_sleep_out(dev);
#endif

	uart_fifo_fill(dev, buffer, datalen);
	uart_irq_tx_enable(dev); 	
}

static void uart_receive_data_handle(struct device *dev, uint8_t *data, uint32_t datalen)
{
	uint32_t data_len = 0;

#ifdef UART_DEBUG
	LOGD("len:%d, data:%s", datalen, data);
#endif

#ifdef CONFIG_PPG_SUPPORT
	if(strncmp(data, PPG_DATA_HEAD, strlen(PPG_DATA_HEAD)) == 0)
	{
		UartPPGEventHandle(data, datalen);
	}
#endif

#ifdef CONFIG_ECG_SUPPORT
	if(strncmp(data, ECG_DATA_HEAD, strlen(ECG_DATA_HEAD)) == 0)
	{
		UartECGEventHandle(data, datalen);
	}
#endif

#ifdef CONFIG_TEMP_SUPPORT
	if(strncmp(data, TEMP_DATA_HEAD, strlen(TEMP_DATA_HEAD)) == 0)
	{
		UartTempEventHandle(data, datalen);
	}
#endif

#ifdef CONFIG_WIFI_SUPPORT	
	if(strncmp(data, WIFI_DATA_HEAD, strlen(WIFI_DATA_HEAD)) == 0)
	{
		UartWifiEventHandle(data, datalen);
	}
#endif

#ifdef CONFIG_AUDIO_SUPPORT
	if(strncmp(data, AUDIO_DATA_HEAD, strlen(AUDIO_DATA_HEAD)) == 0)
	{
		UartAudioEventHandle(data, datalen);
	}
#endif

#ifdef CONFIG_BLE_SUPPORT
	if(strncmp(data, BLE_DATA_HEAD, strlen(BLE_DATA_HEAD)) == 0)
	{
		UartBleEventHandle(data, datalen);
	}
#endif
}

void UartSendData(void)
{
	uint8_t data_type,*p_data;
	uint32_t data_len;
	int ret;

	ret = get_data_from_cache(&uart_send_cache, &p_data, &data_len, &data_type);
	if(ret)
	{
	#ifdef UART_DEBUG
		LOGD("begin");
	#endif
		uart_send_data_handle(uart_copcs, p_data, data_len);
		delete_data_from_cache(&uart_send_cache);
		k_timer_start(&uart_send_data_timer, K_MSEC(20), K_NO_WAIT);
	}
}

void UartSendDataStart(void)
{
	k_timer_start(&uart_send_data_timer, K_MSEC(20), K_NO_WAIT);
}

bool SendCacheIsEmpty(void)
{
	if(cache_is_empty(&uart_send_cache))
		return true;
	else
		return false;
}

void CopcsSendData(UART_DATA_TYPE type, uint8_t *data, uint32_t datalen)
{
	int ret;
	uint8_t head_len, *ptr;

	ptr = k_malloc(datalen+UART_DATA_HEAD_MAX_LEN);
	if(ptr != NULL)
	{
		memset(ptr, 0x00, datalen+UART_DATA_HEAD_MAX_LEN);
		
		switch(type)
		{
		case UART_DATA_PPG:
			strcpy(ptr, PPG_DATA_HEAD);
			head_len = strlen(PPG_DATA_HEAD);
			break;
		case UART_DATA_ECG:
			strcpy(ptr, ECG_DATA_HEAD);
			head_len = strlen(ECG_DATA_HEAD);
			break;
		case UART_DATA_TEMP:
			strcpy(ptr, TEMP_DATA_HEAD);
			head_len = strlen(TEMP_DATA_HEAD);
			break;
		case UART_DATA_WIFI:
			strcpy(ptr, WIFI_DATA_HEAD);
			head_len = strlen(WIFI_DATA_HEAD);
			break;			
		case UART_DATA_AUIOD:
			strcpy(ptr, AUDIO_DATA_HEAD);
			head_len = strlen(AUDIO_DATA_HEAD);
			break;
		case UART_DATA_BLE:
			strcpy(ptr, BLE_DATA_HEAD);
			head_len = strlen(BLE_DATA_HEAD);
			break;
		}

		memcpy(ptr+head_len, data, datalen);
		ret = add_data_into_cache(&uart_send_cache, ptr, datalen+head_len, DATA_TRANSFER);
		if(ret)
			UartSendDataStart();

		k_free(ptr);
	}
}

void UartReceData(void)
{
	uint8_t data_type,*p_data;
	uint32_t data_len;
	int ret;

	ret = get_data_from_cache(&uart_rece_cache, &p_data, &data_len, &data_type);
	if(ret)
	{
		uart_receive_data_handle(uart_copcs, p_data, data_len);
		delete_data_from_cache(&uart_rece_cache);
		k_timer_start(&uart_rece_data_timer, K_MSEC(20), K_NO_WAIT);
	}
}

void ReceDataStart(void)
{
	k_timer_start(&uart_rece_data_timer, K_MSEC(20), K_NO_WAIT);
}

bool ReceCacheIsEmpty(void)
{
	if(cache_is_empty(&uart_rece_cache))
		return true;
	else
		return false;
}

void UartReceFrameData(uint8_t *data, uint32_t datalen)
{
	int ret;

	ret = add_data_into_cache(&uart_rece_cache, data, datalen, DATA_TRANSFER);
	ReceDataStart();
}

static void uart_cb(struct device *x)
{
	uint8_t tmpbyte = 0;
	uint32_t len=0;

	uart_irq_update(x);

	if(uart_irq_rx_ready(x)) 
	{
		if(uart_rece_len >= BUF_MAXSIZE)
			uart_rece_len = 0;

		while((len = uart_fifo_read(x, &uart_rx_buf[uart_rece_len], BUF_MAXSIZE-uart_rece_len)) > 0)
		{
			uart_rece_len += len;
			k_timer_start(&uart_rece_frame_timer, K_MSEC(10), K_NO_WAIT);
		}
	}
	
	if(uart_irq_tx_ready(x))
	{
		struct uart_data_t *buf;
		uint16_t written = 0;

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		/* Nothing in the FIFO, nothing to send */
		if(!buf)
		{
			uart_irq_tx_disable(x);
			return;
		}

		while(buf->len > written)
		{
			written += uart_fifo_fill(x, &buf->data[written], buf->len - written);
		}

		while (!uart_irq_tx_complete(x))
		{
			/* Wait for the last byte to get
			* shifted out of the module
			*/
		}

		if (k_fifo_is_empty(&fifo_uart_tx_data))
		{
			uart_irq_tx_disable(x);
		}

		k_free(buf);
	}
}

#ifdef CONFIG_PM_DEVICE
void uart_sleep_out(struct device *dev)
{
	if(k_timer_remaining_get(&uart_sleep_in_timer) > 0)
		k_timer_stop(&uart_sleep_in_timer);
	k_timer_start(&uart_sleep_in_timer, K_SECONDS(UART_WAKE_HOLD_TIME_SEC), K_NO_WAIT);

	if(uart_is_waked)
		return;
	
	pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
	uart_is_waked = true;

#ifdef UART_DEBUG
	LOGD("uart set active success!");
#endif
}

void uart_sleep_in(struct device *dev)
{	
	if(!uart_is_waked)
		return;
	
	pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);
	uart_is_waked = false;

#ifdef UART_DEBUG
	LOGD("uart set low power success!");
#endif
}

static void copcs_interrupt_event(struct device *interrupt, struct gpio_callback *cb, uint32_t pins)
{
	uart_wake_flag = true;
}

static void UartSleepInCallBack(struct k_timer *timer_id)
{
#ifdef UART_DEBUG
	LOGD("begin");
#endif
	uart_sleep_flag = true;
}
#endif

static void UartSendDataCallBack(struct k_timer *timer)
{
	uart_send_data_flag = true;
}

static void UartReceDataCallBack(struct k_timer *timer_id)
{
	uart_rece_data_flag = true;
}

static void UartReceFrameCallBack(struct k_timer *timer_id)
{
	//uart_rece_frame_flag = true;
	UartReceFrameData(uart_rx_buf, uart_rece_len);
	uart_rece_len = 0;
}

void UartWifiOff(void)
{
	if(k_timer_remaining_get(&uart_send_data_timer) > 0)
		k_timer_stop(&uart_send_data_timer);
	delete_all_from_cache(&uart_send_cache);
	
#ifdef CONFIG_PM_DEVICE
	uart_sleep_in(uart_copcs);
#endif
}

void uart_init(void)
{
	gpio_flags_t flag = GPIO_INPUT|GPIO_PULL_UP;

#ifdef UART_DEBUG
	LOGD("begin");
#endif

	uart_copcs = DEVICE_DT_GET(COPCS_DEV);
	if(!uart_copcs)
	{
	#ifdef UART_DEBUG
		LOGD("Could not get uart!");
	#endif
		return;
	}

	uart_irq_callback_set(uart_copcs, uart_cb);
	uart_irq_rx_enable(uart_copcs);

	gpio_copcs = DEVICE_DT_GET(COPCS_PORT);
	if(!gpio_copcs)
	{
	#ifdef UART_DEBUG
		LOGD("Could not get gpio!");
	#endif
		return;
	}	
	gpio_pin_configure(gpio_copcs, COPCS_WAKE_PIN, GPIO_OUTPUT);
	gpio_pin_set(gpio_copcs, COPCS_WAKE_PIN, 1);

#ifdef CONFIG_PM_DEVICE
	gpio_pin_configure(gpio_copcs, COPCS_INT_PIN, flag);
	gpio_pin_interrupt_configure(gpio_copcs, COPCS_INT_PIN, GPIO_INT_DISABLE);
	gpio_init_callback(&gpio_cb, copcs_interrupt_event, BIT(COPCS_INT_PIN));
	gpio_add_callback(gpio_copcs, &gpio_cb);
	gpio_pin_interrupt_configure(gpio_copcs, COPCS_INT_PIN, GPIO_INT_ENABLE|GPIO_INT_EDGE_FALLING);	
	k_timer_start(&uart_sleep_in_timer, K_SECONDS(UART_WAKE_HOLD_TIME_SEC), K_NO_WAIT);
#endif
}

void UartMsgProc(void)
{
#ifdef CONFIG_PM_DEVICE
	if(uart_wake_flag)
	{
		uart_wake_flag = false;
		uart_sleep_out(uart_copcs);
	}

	if(uart_sleep_flag)
	{
		uart_sleep_flag = false;
		uart_sleep_in(uart_copcs);
	}
#endif/*CONFIG_PM_DEVICE*/

	if(uart_send_data_flag)
	{
		UartSendData();
		uart_send_data_flag = false;
	}

	if(uart_rece_data_flag)
	{
		UartReceData();
		uart_rece_data_flag = false;
	}
	
	if(uart_rece_frame_flag)
	{
		UartReceFrameData(uart_rx_buf, uart_rece_len);
		memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
		uart_rece_len = 0;
		uart_rece_frame_flag = false;
	}
}

void UartSetBaudRate(uint32_t baudrate)
{
	struct uart_config uart_cfg;

	uart_config_get(uart_copcs, &uart_cfg);
	uart_cfg.baudrate = baudrate;
	uart_configure(uart_copcs, &uart_cfg);
}

void test_uart_ble(void)
{
#ifdef UART_DEBUG
	LOGD("begin");
#endif

	while(1)
	{
		CopcsSendData(UART_DATA_BLE, "Hello World!\n", strlen("Hello World!\n"));
		k_sleep(K_MSEC(1000));
	}
}
