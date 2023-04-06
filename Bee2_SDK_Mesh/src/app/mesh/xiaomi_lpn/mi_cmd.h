/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file     mi_cmd.h
* @brief    Head file for mi cmd.
* @details  User command interfaces.
* @author   hector_huang
* @date     2019-1-11
* @version  v1.0
* *************************************************************************************
*/
#ifndef _MI_CMD_H_
#define _MI_CMD_H_

#include "platform_types.h"
#include "data_uart.h"
#include "user_cmd_parse.h"

BEGIN_DECLS

/**
 * @brief user command table
 */
extern const user_cmd_table_entry_t mi_cmd_table[];

END_DECLS

#endif

