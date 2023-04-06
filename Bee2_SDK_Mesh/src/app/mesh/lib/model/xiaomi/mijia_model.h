/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      mijia_model.h
* @brief     mijia vendor model
* @details
* @author    astor zhang
* @date      2019-07-04
* @version   v1.0
* *********************************************************************************************************
*/
#ifndef _MIJIA_MODEL_H_
#define _MIJIA_MODEL_H_

#include "mesh_api.h"

BEGIN_DECLS

/**
 * @addtogroup MIJIA_MODEL
 * @{
 */

/**
 * @defgroup MIJIA_MODEL_ID Model ID
 * @brief Mesh model id
 * @{
 */
#define MESH_MODEL_MIJIA_SERVER                0x0002038F
#define MESH_MODEL_MIJIA_CLIENT                0x0003038F
/** @} */


/**
 * @defgroup MIOT_SERVER_API Server API
 * @brief Functions declaration
 * @{
 */

/**
 * @brief register mijia server
 * @param[in] element_index: element index that model registered to
 * @param[in] pmodel_info: pointer to mijia server model context
 * @retval TRUE: register success
 * @retval FALSE: register failed
 */
bool mijia_server_reg(uint8_t element_index, mesh_model_info_p pmodel_info);

/**
 * @brief return mijia data status
 * @param[in] pmodel_info: pointer to mijia server model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] pdata: mijia status data
 * @param[in] len: mijia status data length
 * @param[in] delay_time: message response delay time
 * @return send status
 */
mesh_msg_send_cause_t mijia_status(const mesh_model_info_p pmodel_info, uint16_t dst,
                                  uint16_t app_key_index, const uint8_t *pdata, uint8_t len,
                                  uint32_t delay_time);

/**
 * @brief publish mijia data
 * @param[in] pmodel_info: pointer to mijia server model context
 * @param[in] pdata: mijia status data
 * @param[in] len: mijia status data length
 * @return send status
 */
mesh_msg_send_cause_t mijia_publish(const mesh_model_info_p pmodel_info, const uint8_t *pdata,
                                   uint8_t len);

/**
 * @brief indication mijia data
 * @param[in] pmodel_info: pointer to mijia server model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] pdata: data need to publish
 * @param[in] data_len: data length
 * @return publish status
 */
mesh_msg_send_cause_t mijia_indication(const mesh_model_info_p pmodel_info, uint16_t dst,
                                      uint16_t app_key_index,
                                      const uint8_t *pdata, uint8_t len);

/** @} */

/**
 * @defgroup MIOT_CLIENT_API Client API
 * @brief Functions declaration
 * @{
 */

/**
 * @brief register mijia client
 * @param[in] element_index: element index that model registered to
 * @param[in] pmodel_info: pointer to mijia client model context
 * @retval TRUE: register success
 * @retval FALSE: register failed
 */
bool mijia_client_reg(uint8_t element_index, mesh_model_info_p pmodel_info);

/**
 * @brief get mijia status
 * @param[in] pmodel_info: pointer to mijia client model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] pdata: mijia get data
 * @param[in] len: mijia get data length
 * @return send status
 */
mesh_msg_send_cause_t mijia_get(const mesh_model_info_p pmodel_info, uint16_t dst,
                               uint16_t app_key_index, const uint8_t *pdata, uint8_t len);

/**
 * @brief set mijia data
 * @param[in] pmodel_info: pointer to mijia client model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] pdata: mijia set data
 * @param[in] len: mijia set data length
 * @param[in] ack: acknowledge flag
 * @return send status
 */
mesh_msg_send_cause_t mijia_set(const mesh_model_info_p pmodel_info, uint16_t dst,
                               uint16_t app_key_index, const uint8_t *pdata, uint8_t len,
                               bool ack);
/**
 * @brief ack mijia indication
 * @param[in] pmodel_info: pointer to mijia client model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] pdata: mijia indication ack data
 * @param[in] len: mijia indication ack data length
 * @return send status
 */
mesh_msg_send_cause_t mijia_indication_ack(const mesh_model_info_p pmodel_info, uint16_t dst,
                                          uint16_t app_key_index, const uint8_t *pdata, uint8_t len);

/** @} */
/** @} */

END_DECLS

#endif /* _MIJIA_MODEL_H */
