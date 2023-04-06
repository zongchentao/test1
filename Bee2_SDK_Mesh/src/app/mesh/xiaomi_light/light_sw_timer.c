/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      light_sw_timer.c
* @brief     light software timer
* @details
* @author    hector_huang
* @date      2019-05-14
* @version   v1.0
* *********************************************************************************************************
*/
#include "light_sw_timer.h"
#include "miot_model_server_app.h"

void light_sw_timer_handle_timeout(const T_IO_MSG *pmsg)
{
    switch (pmsg->subtype)
    {
    case MIOT_INDICATE_TIMEOUT:
        miot_server_handle_indicate_timeout();
        break;
    case MIOT_NET_PARAM_REQ_TIMEOUT:
        miot_server_handle_net_param_req_timeout();
        break;
    default:
        break;
    }
}
