/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     demo_lpn_export.c
* @brief    Exported functions of lpn demo
* @details
* @author   astor zhang
* @date     2019-07-19
* @version  v1.0
*********************************************************************************************************
*/


#include "demo_lpn_export.h"
#include "mesh_api.h"
#include "lpn_app.h"

bool demo_lpn_init(uint8_t fn_num)
{
    return lpn_init(fn_num, lpn_cb);
}

bool demo_lpn_req(uint8_t fn_index, uint16_t net_key_index, uint32_t poll_interval,
                  uint32_t poll_timeout)
{
    lpn_req_params_t req_params = {50, 100, {1, 0, 0}};
    req_params.poll_interval = poll_interval;
    req_params.poll_timeout = poll_timeout;
    mesh_node.frnd_rx_delay = 30;
//    mesh_node.frnd_rx_widen = 0;
    return lpn_req(fn_index, net_key_index,
                   &req_params);
}

void demo_lpn_sub(uint8_t fn_index, uint16_t sub_addr, bool add_rm)
{
    uint16_t addr = sub_addr;
    frnd_sub_list_add_rm(fn_index, &addr, 1, add_rm);
}

void demo_lpn_clear(uint8_t fn_index)
{
    lpn_clear(fn_index);
}
