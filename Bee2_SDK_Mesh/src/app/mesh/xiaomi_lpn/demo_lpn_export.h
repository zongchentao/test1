/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     demo_lpn_export.h
* @brief    Head file of exported functions of lpn demo
* @details
* @author   astor zhang
* @date     2019-07-19
* @version  v1.0
*********************************************************************************************************
*/

#include "mesh_api.h"

/**
 * @brief init lpn feature
 * @param[in] fn_num: number of fn, 1 available
 */
bool demo_lpn_init(uint8_t fn_num);

/**
 * @brief send lpn request
 * @param[in] fn_index: friend node index in list
 * @param[in] net_key_index: Netkey index in list
 * @param[in] poll_interval: interval of friend poll operation
 * @param[in] poll_timeout: lpn lost time
 */
bool demo_lpn_req(uint8_t fn_index, uint16_t net_key_index, uint32_t poll_interval,
                  uint32_t poll_timeout);

/**
 * @brief lpn subscribe
 * @param[in] fn_index: friend node index in list
 * @param[in] sub_addr: subscribe address
 * @param[in] add_rm: add/remove subscription
 */
void demo_lpn_sub(uint8_t fn_index, uint16_t sub_addr, bool add_rm);

/**
 * @brief send lpn request
 * @param[in] fn_index: friend node index in list
 */
void demo_lpn_clear(uint8_t fn_index);
