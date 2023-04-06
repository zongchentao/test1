/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file     relay_module_gpio_app.h
* @brief    Head file for gpio initialization of relay module.
* @details  Init interface
* @author   astor zhang
* @date     2019-06-14
* @version  v1.0
* *************************************************************************************
*/

#ifndef _RELAY_MODULE_GPIO_APP_H_
#define _RELAY_MODULE_GPIO_APP_H_

#define RELAY_PIN_1 P2_3
#define RELAY_PIN_2 P2_4
#define DEMO_BUTTON P4_2
#define GPIO_DEMO_BUTTON  GPIO_GetPin(DEMO_BUTTON)

#define GPIO_DEMO_IRQn   GPIO30_IRQn
#define GPIO_DEMO_Handler      GPIO30_Handler

/**
 * @brief initialize relay module gpio pinmux
 */
void relay_gpio_board_init(void);

/**
 * @brief initialize relay module gpio driver
 */
void gpio_driver_init(void);

#endif /** _RELAY_MODULE_GPIO_APP_H_ */
