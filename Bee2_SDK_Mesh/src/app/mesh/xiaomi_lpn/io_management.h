/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     io_management.h
* @brief    Head file of peripherals.
* @details
* @author   astor zhang
* @date     2019-07-19
* @version  v1.0
*********************************************************************************************************
*/

#include "mible_api.h"

/**
 * @brief init sw timer
 */
void sw_timer_init(void);

/**
 * @brief start timer to control beacon
 */
void start_beacon_timer(void);

/**
 * @brief button configuration
 */
void lpn_button_board_init(void);

/**
 * @brief gpio driver init
 */
void gpio_driver_init(void);

/**
 * @brief interrupt handler of button
 */
void GPIO_LPN_Handler(void);

/**
 * @brief start to send lpn req
 */
void start_lpn_req(void);

/**
 * @brief stop lpn req
 */
void stop_lpn_req(void);

typedef enum
{
    MSG_BUTTON_SHORT_PRESS,
    MSG_BUTTON_LONG_PRESS
} T_BUTTON_PRESS_EVENT;

extern uint8_t keystatus;
