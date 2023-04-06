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
 * @brief stop timer of control beacon
 */
void stop_beacon_timer(void);

void restart_beacon_timer(void);

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

void prov_led_flash(void);

void prov_led_turnout(void);

void prov_led_connected(void);

void single_adv_timeout_handle(void);

typedef enum
{
    MSG_BUTTON_SHORT_PRESS,
} T_BUTTON_PRESS_EVENT;

extern uint8_t keystatus;
