/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     io_management.c
* @brief    This file configures the peripherals.
* @details
* @author   astor zhang
* @date     2019-07-19
* @version  v1.0
*********************************************************************************************************
*/


#include "rtl876x_gpio.h"
#include "board.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "rtl876x_nvic.h"
#include "app_task.h"
#include "io_management.h"
#include "mijia_mesh_app.h"
#include "gap_scheduler.h"
#include "light_cwrgb_app.h"
#include "trace.h"
#include "common/mible_beacon.h"
#include "light_sw_timer.h"
#include "single_fire_switch_config.h"
#include "mesh_api.h"

plt_timer_t xTimerStopBeacon;
plt_timer_t xTimerProv;
uint8_t lpn_req_cnt;

void lpn_button_board_init(void)
{
    Pad_Config(LPN_BUTTON, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    Pad_Config(P4_1, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_HIGH);
    Pinmux_Config(LPN_BUTTON, DWGPIO);
    Pinmux_Config(P4_1, DWGPIO);

    GPIO_InitTypeDef GPIO_struct_init;
    GPIO_StructInit(&GPIO_struct_init);

    GPIO_struct_init.GPIO_Pin = GPIO_LPN_BUTTON;
    GPIO_struct_init.GPIO_Mode = GPIO_Mode_IN;
    GPIO_struct_init.GPIO_DebounceTime = 20;
    GPIO_struct_init.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
    GPIO_struct_init.GPIO_ITCmd = ENABLE;
    GPIO_struct_init.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
    GPIO_struct_init.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
    GPIO_Init(&GPIO_struct_init);
    keystatus = GPIO_ReadInputDataBit(GPIO_LPN_BUTTON);
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = GPIO_LPN_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    GPIO_MaskINTConfig(GPIO_LPN_BUTTON, DISABLE);
    GPIO_INTConfig(GPIO_LPN_BUTTON, ENABLE);

    GPIO_StructInit(&GPIO_struct_init);
    GPIO_struct_init.GPIO_Pin = GPIO_GetPin(P4_1);
    GPIO_struct_init.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_struct_init.GPIO_ITCmd = DISABLE;
    GPIO_Init(&GPIO_struct_init);
    GPIO_SetBits(GPIO_GetPin(P4_1));
}

void gpio_driver_init(void)
{
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
}

/**
  * @brief  GPIO interrupt handler function.
  * @param  No parameter.
  * @return void
  */
void GPIO_LPN_Handler(void)
{

    GPIO_INTConfig(GPIO_LPN_BUTTON, DISABLE);
    GPIO_MaskINTConfig(GPIO_LPN_BUTTON, ENABLE);

    keystatus = GPIO_ReadInputDataBit(GPIO_LPN_BUTTON);
    T_IO_MSG button_msg;
    if (keystatus == 0)
    {
        GPIO->INTPOLARITY |= GPIO_LPN_BUTTON;
    }
    else
    {
        GPIO->INTPOLARITY &= ~GPIO_LPN_BUTTON;
        button_msg.type = IO_MSG_TYPE_GPIO;
        button_msg.subtype = MSG_BUTTON_SHORT_PRESS;
        app_send_msg_to_apptask(&button_msg);
    }
    GPIO_ClearINTPendingBit(GPIO_LPN_BUTTON);
    GPIO_MaskINTConfig(GPIO_LPN_BUTTON, DISABLE);
    GPIO_INTConfig(GPIO_LPN_BUTTON, ENABLE);
}

void pTimerSUCallback(void *pTimer)
{
    T_IO_MSG switch_msg;
    switch_msg.type = IO_MSG_TYPE_TIMER;
    switch_msg.subtype = MIOT_UNPROV_ADV_TIMEOUT;
    app_send_msg_to_apptask(&switch_msg);
}

bool revert = true;
void pTimerProvCallback(void *pTimer)
{
    if (revert)
    {
        GPIO_SetBits(GPIO_GetPin(P4_1));
        revert = false;
    }
    else
    {
        GPIO_ResetBits(GPIO_GetPin(P4_1));
        revert = true;
    }
}

void sw_timer_init(void)
{
    xTimerStopBeacon = plt_timer_create("xTimerStopBeacon", MI_PROV_TIMEOUT, false, 0,
                                        pTimerSUCallback);
    if (NULL == xTimerStopBeacon)
    {
        APP_PRINT_ERROR0("xTimerStopBeacon create fail.");
    }

    xTimerProv = plt_timer_create("xTimerProv", MI_LED_FLASH_INTERVAL, true, 0, pTimerProvCallback);
    if (NULL == xTimerProv)
    {
        APP_PRINT_ERROR0("xTimerProv create fail.");
    }
}

void start_beacon_timer(void)
{
    plt_timer_start(xTimerStopBeacon, 0);
}

void stop_beacon_timer(void)
{
    plt_timer_stop(xTimerStopBeacon, 0);
}

void prov_led_flash(void)
{
    plt_timer_start(xTimerProv, 0);
}

void restart_beacon_timer(void)
{
    plt_timer_change_period(xTimerStopBeacon, MI_PROV_TIMEOUT, 0);
}

void prov_led_turnout(void)
{
    plt_timer_stop(xTimerProv, 0);
    GPIO_SetBits(GPIO_GetPin(P4_1));
    revert = true;
}

void prov_led_connected(void)
{
    plt_timer_stop(xTimerProv, 0);
    GPIO_ResetBits(GPIO_GetPin(P4_1));
    revert = false;
}

void single_adv_timeout_handle(void)
{
    mibeacon_adv_stop();
    gap_sched_scan(false);
    prov_led_turnout();
    allowEnterDlps = true;
}
