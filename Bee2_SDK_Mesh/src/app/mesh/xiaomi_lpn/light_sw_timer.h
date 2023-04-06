/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      light_sw_timer.h
* @brief     light software timer
* @details
* @author    hector_huang
* @date      2019-05-14
* @version   v1.0
* *********************************************************************************************************
*/
#ifndef _LIGHT_SW_TIMER_H_
#define _LIGHT_SW_TIMER_H_

#include "platform_types.h"
#include "app_msg.h"

BEGIN_DECLS

/* timeout type */
typedef enum
{
    MIOT_INDICATE_TIMEOUT,
    MIOT_NET_PARAM_REQ_TIMEOUT,
} light_sw_timer_type_t;

/**
 * @brief handle software timeout
 * @param[in] pmsg: io message
 */
void light_sw_timer_handle_timeout(const T_IO_MSG *pmsg);

END_DECLS

#endif /* _LIGHT_SW_TIMER_H_ */
