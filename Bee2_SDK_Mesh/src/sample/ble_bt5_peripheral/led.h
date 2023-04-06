/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     led.h
  * @brief
  * @details
  * @author   bill
  * @date     2019-3-10
  * @version  v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _LED_H_
#define _LED_H_

#include <stdint.h>

#define CHANNEL_LINK            3
#define CHANNEL_ON_OFF          0

void led_init(void);
void led_lighten(uint8_t channel, uint16_t lightness);
void led_blink(uint8_t channel, uint32_t period_ms, uint8_t duty);

#endif



