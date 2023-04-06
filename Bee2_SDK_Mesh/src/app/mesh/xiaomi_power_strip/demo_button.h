/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file     demo_button.h
* @brief    Head file for gpio initialization of demo_button.
* @details  Init interface
* @author   astor zhang
* @date     2019-07-02
* @version  v1.0
* *************************************************************************************
*/

#ifndef _DEMO_BUTTON_H_
#define _DEMO_BUTTON_H_

#define DEMO_BUTTON P4_2
#define GPIO_DEMO_BUTTON  GPIO_GetPin(DEMO_BUTTON)

#define GPIO_DEMO_IRQn   GPIO30_IRQn
#define GPIO_DEMO_Handler      GPIO30_Handler

/**
 * @brief initialize demo_button gpio pinmux
 */
void demo_button_board_init(void);

/**
 * @brief initialize demo_button gpio driver
 */
void gpio_driver_init(void);

#endif /** _DEMO_BUTTON_H_ */
