/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     demo_button.c
* @brief    This file configures the gpio for simulation of demo_button.
* @details
* @author   astor zhang
* @date     2019-07-02
* @version  v1.0
*********************************************************************************************************
*/


#include "rtl876x_gpio.h"
#include "board.h"
#include "demo_button.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "rtl876x_nvic.h"
#include "app_task.h"
#include "sensor_report_app.h"

void demo_button_board_init(void)
{
    Pad_Config(DEMO_BUTTON, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);
    Pinmux_Config(DEMO_BUTTON, DWGPIO);

    GPIO_InitTypeDef GPIO_struct_init;
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

    uint8_t keystatus = GPIO_ReadInputDataBit(GPIO_DEMO_BUTTON);
    if (keystatus)
    {
    }
    else
    {
        handle_demo_data();
    }
    GPIO_ClearINTPendingBit(GPIO_DEMO_BUTTON);
    GPIO_MaskINTConfig(GPIO_DEMO_BUTTON, DISABLE);
    GPIO_INTConfig(GPIO_DEMO_BUTTON, ENABLE);
}
