/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      single_fire_switch_ctl_server_app.h
* @brief     Smart mesh demo single fire switch ctl application
* @details
* @author    astor
* @date      2019-10-17
* @version   v1.0
* *********************************************************************************************************
*/

#ifndef _LIGHT_CTL_SERVER_APP_H
#define _LIGHT_CTL_SERVER_APP_H

#include "platform_types.h"
#include "generic_on_off.h"
BEGIN_DECLS

/**
 * @addtogroup CTL_SERVER_APP
 * @{
 */
extern generic_on_off_t switch_state;
/**
 * @defgroup Ctl_Server_Exported_Functions CTL Server Exported Functions
 * @brief
 * @{
 */
/**
 * @brief initialize ctl light server models
 */
void ctl_server_models_init(void);

/**
 * @brief ctl subscribe
 */
void ctl_server_models_sub(void);

void switch_scan_timeout_handle(void);
void switch_beacon_timeout_handle(void);
bool switch_timer_start(void);
void switch_timer_stop(void);
void switch_periodic_pub_stop(void);
/** @} */
/** @} */

END_DECLS

#endif /* _CTL_SERVER_APP_H */

