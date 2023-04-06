/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      miot_model_server_app.h
* @brief     xiaomi vendor model server application
* @details
* @author    hector_huang
* @date      2019-04-16
* @version   v1.0
* *********************************************************************************************************
*/
#ifndef _MIOT_MODEL_SERVER_APP_H_
#define _MIOT_MODEL_SERVER_APP_H_

#include "mesh_api.h"

BEGIN_DECLS

/* export miot server model for external use */
extern mesh_model_info_t miot_server;

/**
 * @brief miot server indication
 * @param[in] dst: dst address
 * @param[in] app_key_index: app key index
 * @param[in] pdata: indication data
 * @param[in] len: indication data length
 */
void miot_server_indication(uint16_t dst, uint16_t app_key_index, const uint8_t *pdata,
                            uint8_t len);

/**
 * @brief handle miot server indication timeout message
 */
void miot_server_handle_indicate_timeout(void);

/**
 * @brief start network parameter request
 * @param[in] dst: dst address
 * @param[in] app_key_index: app key index
 */
void miot_server_net_param_req_start(uint16_t dst, uint16_t app_key_index);

/**
 * @brief stop network parameter request
 */
void miot_server_net_param_req_stop(void);

/**
 * @brief handle network parameter request timeout
 */
void miot_server_handle_net_param_req_timeout(void);

/**
 * @brief miot server model data process
 * @param[in] pmodel_info: miot model server handler
 * @param[in] type: data type
 * @param[in] pargs: data need to process
 * @return process status
 */
int32_t miot_server_data(const mesh_model_info_p pmodel_info, uint32_t type,
                         void *pargs);


END_DECLS

#endif /* _MIOT_MODEL_SERVER_APP_H_ */
