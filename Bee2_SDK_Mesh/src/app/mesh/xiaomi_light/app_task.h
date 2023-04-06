/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2013 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */
#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <stdint.h>
#include <stdbool.h>
#include "app_msg.h"

/**
 * @brief initialize app task
 * @return void
 */
void app_task_init(void);

/**
 * @brief send message to app task.
 * @param[in] p_msg:  message need to send
 * @return The status of the message queue peek.
 * @retval true  Message send successfully.
 * @retval false Message send failed.
 */
bool app_send_msg_to_apptask(T_IO_MSG *p_msg);

#endif

