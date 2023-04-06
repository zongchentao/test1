/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     relay_module_gpio_app.c
* @brief    This file configures the gpio for simulation of switch with relay modules.
* @details
* @author   astor zhang
* @date     2019-06-14
* @version  v1.0
*********************************************************************************************************
*/


#include "rtl876x_gpio.h"
#include "board.h"
#include "relay_module_gpio_app.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "rtl876x_nvic.h"
#include "app_task.h"
#include "trace.h"

uint8_t revert_cnt = 0;

void relay_gpio_board_init(void)
{

    Pad_Config(RELAY_PIN_1, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pinmux_Config(RELAY_PIN_1, DWGPIO);
    Pad_Config(RELAY_PIN_2, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);
    Pinmux_Config(RELAY_PIN_2, DWGPIO);
    Pad_Config(DEMO_BUTTON, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);
    Pinmux_Config(DEMO_BUTTON, DWGPIO);

    GPIO_InitTypeDef GPIO_struct_init;
    GPIO_StructInit(&GPIO_struct_init);

    GPIO_struct_init.GPIO_Pin = GPIO_GetPin(RELAY_PIN_1) | GPIO_GetPin(RELAY_PIN_2);
    GPIO_struct_init.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(&GPIO_struct_init);

    GPIO_StructInit(&GPIO_struct_init);
    GPIO_struct_init.GPIO_Pin = GPIO_DEMO_BUTTON;
    GPIO_struct_init.GPIO_Mode = GPIO_Mode_IN;
    GPIO_struct_init.GPIO_DebounceTime = 10;
    GPIO_struct_init.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
    GPIO_struct_init.GPIO_ITCmd = ENABLE;
    GPIO_struct_init.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
    GPIO_struct_init.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
    GPIO_Init(&GPIO_struct_init);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = GPIO_DEMO_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    GPIO_MaskINTConfig(GPIO_DEMO_BUTTON, DISABLE);
    GPIO_INTConfig(GPIO_DEMO_BUTTON, ENABLE);

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
void GPIO_DEMO_Handler(void)
{
    GPIO_INTConfig(GPIO_DEMO_BUTTON, DISABLE);
    GPIO_MaskINTConfig(GPIO_DEMO_BUTTON, ENABLE);

    T_IO_MSG int_gpio_msg;
    uint8_t keystatus = GPIO_ReadInputDataBit(GPIO_DEMO_BUTTON);
    int_gpio_msg.type = IO_MSG_TYPE_GPIO;
    if (keystatus)
    {
    }
    else
    {
        if (revert_cnt % 2 == 0)
        {
            revert_cnt++;
            int_gpio_msg.subtype = 0;
        }
        else
        {
            revert_cnt++;
            int_gpio_msg.subtype = 1;
        }
        if (false == app_send_msg_to_apptask(&int_gpio_msg))
        {
            APP_PRINT_ERROR0("[io_gpio] GPIO_Input_Handler: Send int_gpio_msg failed!");
            //Add user code here!
            GPIO_ClearINTPendingBit(GPIO_DEMO_BUTTON);
            return;
        }
    }

    GPIO_ClearINTPendingBit(GPIO_DEMO_BUTTON);
    GPIO_MaskINTConfig(GPIO_DEMO_BUTTON, DISABLE);
    GPIO_INTConfig(GPIO_DEMO_BUTTON, ENABLE);
}
