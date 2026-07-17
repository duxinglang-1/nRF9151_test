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
#include "external_flash.h"
#include "lcd.h"
#include "screen.h"
#include "settings.h"
#include "logger.h"
#include "uart.h"
#include "ecg.h"
#ifdef CONFIG_BLE_SUPPORT
#include "Ble.h"
#endif

uint8_t g_ecg_trigger = 0;

ECG_WORK_STATUS g_ecg_status = ECG_STATUS_PREPARE;

// ECG health data
ecg_health_data_t g_ecg_health_data = {0};

static bool ecg_start_flag = false;
static bool ecg_test_flag = false;
static bool ecg_stop_flag = false;
static bool menu_start_ecg = false;
static bool ft_start_ecg = false;
static bool app_start_ecg = false;

WEAR_WAY ecg_wear_option = WEAR_WAY_LEFT;

// ECG lead status
ECG_LEAD_STATUS g_ecg_lead_status = ECG_LEAD_STATUS_UNKNOWN;

// ECG display phase
ECG_DISPLAY_PHASE g_ecg_display_phase = ECG_DISPLAY_PREPARE;
bool g_ecg_lead_on_ready = false;
static int64_t ecg_lead_on_timestamp = 0;

#ifdef CONFIG_FACTORY_TEST_SUPPORT
uint8_t ecg_test_info[256] = {0};
#endif

static void ecg_lead_on_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(ecg_lead_on_timer, ecg_lead_on_timerout, NULL);
static void ecg_lead_off_timerout(struct k_timer *timer_id);
K_TIMER_DEFINE(ecg_lead_off_timer, ecg_lead_off_timerout, NULL);

static void ecg_lead_on_timerout(struct k_timer *timer_id) {
  // 2 seconds of continuous LEAD_ON detected
  g_ecg_lead_on_ready = true;
  LOGD("ECG Lead ON detected for 2s (timer), ready to show waveform");

  // Notify screen to update lead status display
  if (screen_id == SCREEN_ID_ECG) {
    scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_ECG_LEAD;
    scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
  }
}

bool g_ecg_lead_off_timeout = false;

static void ecg_lead_off_timerout(struct k_timer *timer_id) {
  // Lead has been off for 1 second during waveform display
  g_ecg_lead_off_timeout = true;
  LOGD("ECG Lead OFF for 1s, returning to prepare screen");

  // Notify screen to handle lead off timeout
  if (screen_id == SCREEN_ID_ECG) {
    scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_ECG_LEAD;
    scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
  }
}

bool IsInEcgScreen(void) {
  if (screen_id == SCREEN_ID_ECG)
    return true;
  else
    return false;
}

void StartECG(ECG_TRIGGER_SOUCE trigger_type) {
  notify_infor infor = {0};

  infor.x = 0;
  infor.y = 0;
  infor.w = LCD_WIDTH;
  infor.h = LCD_HEIGHT;
  infor.align = NOTIFY_ALIGN_CENTER;
  infor.type = NOTIFY_TYPE_POPUP;

  // if(1
  // #ifdef CONFIG_FACTORY_TEST_SUPPORT
  //	&& !IsFTECGTesting()
  // #endif
  //	)
  //{
  //	StartSCC();
  // }

  switch (trigger_type) {
  case ECG_TRIGGER_BY_HOURLY:
  case ECG_TRIGGER_BY_APP:
  case ECG_TRIGGER_BY_FT:
    if (!is_wearing()) {
      return;
    }
    break;

  case ECG_TRIGGER_BY_MENU:
    if (!is_wearing()) {
      infor.img[0] = IMG_ID_WRIST_OFF_ICON;
      infor.img_count = 1;

      DisplayPopUp(infor);
      return;
    }

    break;
  }

  g_ecg_trigger |= trigger_type;

  ecg_start_flag = true;
}

void MenuStartECG(WEAR_WAY wear_way) { 
  uint8_t buf[32] = {0};
  if(wear_way ==WEAR_WAY_LEFT){
     uint8_t buffer[64] = {0};
      strcpy(buffer, COM_ECG_WEAR_STATUS);
      strcat(buffer, COM_ECG_WEAR_LEFT);
      CopcsSendData(UART_DATA_ECG, buffer, strlen(buffer));
  }
  else if(wear_way ==WEAR_WAY_RIGHT){
     uint8_t buffer[64] = {0};
      strcpy(buffer, COM_ECG_WEAR_STATUS);
      strcat(buffer, COM_ECG_WEAR_RIGHT);
      CopcsSendData(UART_DATA_ECG, buffer, strlen(buffer));
  }
  menu_start_ecg = true; 
}

void MenuStopECG(void) { ecg_stop_flag = true; }

void APPStartECG(void) { app_start_ecg = true; }

#ifdef CONFIG_FACTORY_TEST_SUPPORT
void FTStartECG(void) { ft_start_ecg = true; }

void FTStopECG(void) { ecg_stop_flag = true; }
#endif

void ECGDataProcess(uint8_t *data, uint32_t data_len) {
  EcgDisplayProcessData(data, data_len);
}

void UartECGEventHandle(uint8_t *data, uint32_t data_len) {
    uint8_t *ptr;

    if (data == NULL || data_len == 0)
        return;

    ptr = strstr(data, ECG_DATA_HEAD);
    if (ptr != NULL) {
        uint8_t *ptr1, *ptr2;

        ptr += strlen(ECG_DATA_HEAD);
        if ((ptr1 = strstr(ptr, COM_ECG_SET_OPEN)) != NULL) {
        } else if ((ptr1 = strstr(ptr, COM_ECG_SET_CLOSE)) != NULL) {
        } else if ((ptr1 = strstr(ptr, COM_ECG_GET_DATA)) != NULL) {
            ptr1 += strlen(COM_ECG_GET_DATA);
            ECGDataProcess(ptr1, data_len - (ptr1 - data));
        } else if ((ptr1 = strstr(ptr, COM_ECG_GET_INFOR)) != NULL) {
            ptr1 += strlen(COM_ECG_GET_INFOR);
        } else if ((ptr1 = strstr(ptr, COM_ECG_LEAD_STATUS)) != NULL) {
            ECG_LEAD_STATUS new_status = ECG_LEAD_STATUS_UNKNOWN;

            // Parse lead status
            if (strstr(ptr, COM_ECG_LEAD_OFF) != NULL) {
                new_status = ECG_LEAD_STATUS_OFF;
                LOGD("ECG Lead Status: OFF");
            } else if (strstr(ptr, COM_ECG_LEAD_ON) != NULL) {
                new_status = ECG_LEAD_STATUS_ON;
                LOGD("ECG Lead Status: ON");
            } else if (strstr(ptr, COM_ECG_LEAD_TIME_OUT) != NULL) {
                new_status = ECG_LEAD_STATUS_TIMEOUT;
                //	LOGD("ECG Lead Status: TIMEOUT");
            }

            // Handle lead on timing for 2 seconds continuous detection
            // Use a timer instead of relying on repeated LEAD_ON messages from UART,
            // because the sensor may only send LEAD_ON once when status changes.
            if (new_status == ECG_LEAD_STATUS_ON &&
                g_ecg_lead_status != ECG_LEAD_STATUS_ON) {
                // First LEAD_ON detected, start 2-second timer
                ecg_lead_on_timestamp = k_uptime_get();
                g_ecg_lead_on_ready = false;
                k_timer_start(&ecg_lead_on_timer, K_MSEC(2000), K_NO_WAIT);
                // Lead recovered, stop the lead-off timer
                k_timer_stop(&ecg_lead_off_timer);
                g_ecg_lead_off_timeout = false;
            } else if (new_status != ECG_LEAD_STATUS_ON) {
                // Lead lost, stop lead-on timer and reset
                k_timer_stop(&ecg_lead_on_timer);
                ecg_lead_on_timestamp = 0;
                g_ecg_lead_on_ready = false;
                // If currently in waveform display phase, start 1-second lead-off timer
                if (g_ecg_display_phase == ECG_DISPLAY_WAVE) {
                    // 立即冻结波形绘制并丢弃积压的脱落过渡期数据，
                    // 避免脱落后1秒内繼续绘制乱波造成花屏
                    EcgHaltDisplay();
                    ClearUartReceCache();
                    g_ecg_lead_off_timeout = false;
                    k_timer_start(&ecg_lead_off_timer, K_MSEC(1000), K_NO_WAIT);
                }
            }
            // Note: ecg_lead_on_timerout() sets g_ecg_lead_on_ready after 2s.
            // Note: ecg_lead_off_timerout() sets g_ecg_lead_off_timeout after 1s.

            g_ecg_lead_status = new_status;

            // Notify screen to update lead status display
            if (screen_id == SCREEN_ID_ECG) {
                scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_ECG_LEAD;
                scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
            }
        } else if ((ptr1 = strstr(ptr, COM_ECG_HR_DATA)) != NULL) {
            // Parse ECG Heart Rate data
            // Format expected: ECG_HR:XX
            ptr1 += strlen(COM_ECG_HR_DATA);
            int hr = atoi((char*)ptr1);
            if (hr > 0 && hr < 300) {
                g_ecg_health_data.hr = (uint16_t)hr;
                LOGD("ECG Heart Rate: %d bpm", g_ecg_health_data.hr);
            }
        } else if ((ptr1 = strstr(ptr, COM_ECG_HRV_DATA)) != NULL) {
            // Parse ECG HRV data
            // Format expected: ECG_HRV:XX
            ptr1 += strlen(COM_ECG_HRV_DATA);
            int hrv = atoi((char*)ptr1);
            if (hrv >= 0 && hrv < 1000) {
                g_ecg_health_data.hrv = (uint16_t)hrv;
                LOGD("ECG HRV: %d ms", g_ecg_health_data.hrv);
            }
        }
    }
}

// 导联脱落快速通道：由 UART 10ms 帧定时器回调（ISR 上下文）直接调用
// 在 uart_rx_buf 原始数据进入 uart_rece_cache FIFO 之前就预览 LEAD_STATUS
// 避免主循环被 EcgDisplayProcessData 的 k_sleep 占用时 LEAD_OFF 滞后几秒
// 幂等设计：new_status 与 g_ecg_lead_status 相同则直接返回，不与 ECGDataProcess 的正式处理重复触发定时器
void EcgPeekLeadStatus(const uint8_t *data, uint32_t data_len)
{
  if (data == NULL || data_len == 0) {
    return;
  }
  // 必须包含 "LEAD_STATUS:" 前缀，过滤纯 ECG 采样帧
  if (strstr((const char *)data, COM_ECG_LEAD_STATUS) == NULL) {
    return;
  }

  ECG_LEAD_STATUS new_status = ECG_LEAD_STATUS_UNKNOWN;
  if (strstr((const char *)data, COM_ECG_LEAD_OFF) != NULL) {
    new_status = ECG_LEAD_STATUS_OFF;
  } else if (strstr((const char *)data, COM_ECG_LEAD_ON) != NULL) {
    new_status = ECG_LEAD_STATUS_ON;
  } else if (strstr((const char *)data, COM_ECG_LEAD_TIME_OUT) != NULL) {
    new_status = ECG_LEAD_STATUS_TIMEOUT;
  } else {
    return;
  }

  // 幂等：状态未变化不处理（ECGDataProcess 后续消费同一数据时也会短路）
  if (new_status == g_ecg_lead_status) {
    return;
  }

  // 镜像 ECGDataProcess 中 COM_ECG_LEAD_STATUS 分支的定时器逻辑
  if (new_status == ECG_LEAD_STATUS_ON &&
      g_ecg_lead_status != ECG_LEAD_STATUS_ON) {
    ecg_lead_on_timestamp = k_uptime_get();
    g_ecg_lead_on_ready = false;
    k_timer_start(&ecg_lead_on_timer, K_MSEC(2000), K_NO_WAIT);
    k_timer_stop(&ecg_lead_off_timer);
    g_ecg_lead_off_timeout = false;
  } else if (new_status != ECG_LEAD_STATUS_ON) {
    k_timer_stop(&ecg_lead_on_timer);
    ecg_lead_on_timestamp = 0;
    g_ecg_lead_on_ready = false;
    if (g_ecg_display_phase == ECG_DISPLAY_WAVE) {
      // 立即冻结波形绘制，避免脱落后1秒内继续绘制乱波造成花屏。
      // 此处处于ISR上下文（k_timer回调），不能调用含有k_free的ClearUartReceCache，
      // 积压包的丢弃由主循环UartECGEventHandle路径完成。
      EcgHaltDisplay();
      g_ecg_lead_off_timeout = false;
      k_timer_start(&ecg_lead_off_timer, K_MSEC(1000), K_NO_WAIT);
    }
  }

  g_ecg_lead_status = new_status;

  // 通知屏幕刷新导联状态（screen.c 的 EcgDisplayProcessData k_sleep 后门闩会读 g_ecg_lead_status 立即 return）
  if (screen_id == SCREEN_ID_ECG) {
    scr_msg[screen_id].para |= SCREEN_EVENT_UPDATE_ECG_LEAD;
    scr_msg[screen_id].act = SCREEN_ACTION_UPDATE;
  }
}

void ECG_init(void) {}

void ECGMsgProcess(void) {
  if (menu_start_ecg) {
    StartECG(ECG_TRIGGER_BY_MENU);
    menu_start_ecg = false;
  }

  if (app_start_ecg) {
    StartECG(ECG_TRIGGER_BY_APP);
    app_start_ecg = false;
  }

  if (ft_start_ecg) {
    StartECG(ECG_TRIGGER_BY_FT);
    ft_start_ecg = false;
  }

  if (ecg_start_flag) {
    ecg_start_flag = false;

    CopcsSendData(UART_DATA_ECG, COM_ECG_SET_OPEN, strlen(COM_ECG_SET_OPEN));
  }

  if (ecg_stop_flag) {
    CopcsSendData(UART_DATA_ECG, COM_ECG_SET_CLOSE, strlen(COM_ECG_SET_CLOSE));
    ecg_stop_flag = false;
    // 清空UART接收缓存，丢弃积压的ECG数据，避免停止后继续绘制
    ClearUartReceCache();
  }
}
