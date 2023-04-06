/*
 * rtk_uart.h
 *
 *  Created on: 2021/3/19
 *      Author: mi
 */
#ifndef RTK_UART_H_
#define RTK_UART_H_

#include "gatt_dfu/mible_dfu_main.h"

#define TX_PIN              P3_2
#define RX_PIN              P3_3
#define WAKE_PIN_NUM        P4_3
#define WAKE_PIN            GPIO_GetPin(WAKE_PIN_NUM)
#define WAKE_PIN_IRQn       GPIO31_IRQn
#define WAKE_PIN_Handler    GPIO31_Handler

void rtk_mcu_wakeup(void);
bool rtk_mcu_is_idle(void);
void rtk_mcu_status(mible_dfu_state_t state, mible_dfu_param_t *param);
void rtk_mcu_perip_power_off(void);
void rtk_mcu_perip_power_on(void);

#endif //RTK_UART_H_
