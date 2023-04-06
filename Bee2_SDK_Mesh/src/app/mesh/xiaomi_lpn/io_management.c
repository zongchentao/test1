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
#include "os_timer.h"
#include "io_management.h"
#include "mijia_mesh_app.h"
#include "gap_scheduler.h"
#include "light_cwrgb_app.h"
#include "trace.h"
#include "demo_lpn_export.h"
#include "common/mible_beacon.h"

bool is_long_press = false;
void *xTimerStopBeacon;
void *xTimerLpnReq;
void *xTimerLongPress;
uint8_t lpn_req_cnt;

void lpn_button_board_init(void)
{
    Pad_Config(LPN_BUTTON, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    Pinmux_Config(LPN_BUTTON, DWGPIO);

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

//    NVIC_InitTypeDef nvic_init_struct;
//    nvic_init_struct.NVIC_IRQChannel         = System_IRQn;
//    nvic_init_struct.NVIC_IRQChannelCmd      = (FunctionalState)ENABLE;
//    nvic_init_struct.NVIC_IRQChannelPriority = 3;
//    NVIC_Init(&nvic_init_struct); //Enable SYSTEM_ON Interrupt

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
        is_long_press = false;
        GPIO->INTPOLARITY |= GPIO_LPN_BUTTON;
        os_timer_start(&xTimerLongPress);
    }
    else
    {
        GPIO->INTPOLARITY &= ~GPIO_LPN_BUTTON;
        if (!is_long_press)
        {
            os_timer_stop(&xTimerLongPress);
            button_msg.type = IO_MSG_TYPE_GPIO;
            button_msg.subtype = MSG_BUTTON_SHORT_PRESS;
            app_send_msg_to_apptask(&button_msg);
        }
        is_long_press = false;
    }
    GPIO_ClearINTPendingBit(GPIO_LPN_BUTTON);
    GPIO_MaskINTConfig(GPIO_LPN_BUTTON, DISABLE);
    GPIO_INTConfig(GPIO_LPN_BUTTON, ENABLE);
}

void pTimerSUCallback(void *pTimer)
{
    mibeacon_adv_stop();
    gap_sched_scan(false);
    allowEnterDlps = true;
}

void pTimerLpnReqCallback(void *pTimer)
{
    lpn_req_cnt++;
    if ((lpn_req_cnt % 10) != 0)
    {
        os_timer_restart(&xTimerLpnReq, 1000);
    }
    demo_lpn_req(0, 0, 20, 200);
}

void pTimerLonngPressCallback(void *pTimer)
{
    T_IO_MSG button_msg;
    is_long_press = true;
    button_msg.type = IO_MSG_TYPE_GPIO;
    button_msg.subtype = MSG_BUTTON_LONG_PRESS;
    app_send_msg_to_apptask(&button_msg);
}

void sw_timer_init(void)
{
    uint8_t ret;
    ret = os_timer_create(&xTimerStopBeacon, "xTimerStopBeacon", 0, 25000, false, pTimerSUCallback);
    if (!ret)
    {
        APP_PRINT_ERROR0("xTimerStopBeacon create fail.");
    }

    ret = os_timer_create(&xTimerLpnReq, "xTimerLpnReq", 0, 3000, false, pTimerLpnReqCallback);
    if (!ret)
    {
        APP_PRINT_ERROR0("xTimerLpnReq create fail.");
    }

    ret = os_timer_create(&xTimerLongPress, "xTimerLongPress", 0, 2000, false,
                          pTimerLonngPressCallback);
    if (!ret)
    {
        APP_PRINT_ERROR0("xTimerLongPress create fail.");
    }

}

void start_beacon_timer(void)
{
    os_timer_start(&xTimerStopBeacon);
}

void start_lpn_req(void)
{
    lpn_req_cnt = 0;
    os_timer_start(&xTimerLpnReq);
}

void stop_lpn_req(void)
{
    os_timer_stop(&xTimerLpnReq);
    lpn_req_cnt = 0;
}
