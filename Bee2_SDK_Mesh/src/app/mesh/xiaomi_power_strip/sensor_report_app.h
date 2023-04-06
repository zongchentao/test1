/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      sensor_report_app.h
* @brief     Smart mesh power strip demo application
* @details
* @author    astor zhang
* @date      2019-07-01
* @version   v1.0
* *********************************************************************************************************
*/

#ifndef _SENSOR_REPORT_APP_H
#define _SENSOR_REPORT_APP_H

#include "platform_types.h"

BEGIN_DECLS

/**
 * @addtogroup SENSOR_REPORT_APP
 * @{
 */

/**
 * @defgroup Sensor_Server_Exported_Functions Light CTL Server Exported Functions
 * @brief
 * @{
 */
/**
 * @brief initialize sensor server models
 */
void sensor_server_models_init(void);

/**
 * @brief sensor model subscribe
 */
void sensor_server_models_sub(void);

/**
 * @brief change data for demo
 */
void handle_demo_data(void);
/** @} */
/** @} */

END_DECLS

#endif /* _SENSOR_REPORT_APP_H */

