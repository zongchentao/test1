/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      switch_ctl_app.h
* @brief     Smart mesh demo switch ctl application
* @details
* @author    astor zhang
* @date      2019-6-20
* @version   v1.0
* *********************************************************************************************************
*/

#ifndef _SWITCH_CTL_APP_H
#define _SWITCH_CTL_APP_H

#include "platform_types.h"

BEGIN_DECLS

/**
 * @addtogroup SWITCH_CTL_APP
 * @{
 */

/**
 * @defgroup Switch_Ctl_App exported functions
 * @brief
 * @{
 */

/**
* @brief initialize sw timer
*/
void sw_timer_init(void);

/**
 * @brief initialize ctl switch server models
 */
void switch_ctl_server_models_init(void);

/**
 * @brief switch ctl subscribe
 */
void switch_ctl_server_models_sub(void);

/**
 * @brief revert switch state when button pressed
 */
void handle_demo_switch_revert(uint8_t num, bool re_pub);

/** @} */
/** @} */

END_DECLS

#endif /* _SWITCH_CTL_APP_H */

