/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      miot_model.h
* @brief     xiaomi vendor model
* @details
* @author    hector_huang
* @date      2019-04-15
* @version   v1.0
* *********************************************************************************************************
*/
#ifndef _MIOT_MODEL_H_
#define _MIOT_MODEL_H_

#include "mesh_api.h"

BEGIN_DECLS

/**
 * @addtogroup MIOT_MODEL
 * @{
 */

/**
 * @defgroup MIOT_ACCESS_ACCESS_OPCODE Access Opcode
 * @brief Mesh message access opcode
 * @{
 */
#define MESH_MSG_MIOT_GET                          0xC18F03
#define MESH_MSG_MIOT_SET                          0xC38F03
#define MESH_MSG_MIOT_SET_UNACK                    0xC48F03
#define MESH_MSG_MIOT_STATUS                       0xC58F03
#define MESH_MSG_MIOT_INDICATION                   0xCE8F03
#define MESH_MSG_MIOT_INDICATION_ACK               0xCF8F03
#define MESH_MSG_MIOT_MESH_CONFIG                  0xFF8F03

#define MIOT_MESH_CONFIG_TYPE_SUB_STATUS           1
#define MIOT_MESH_CONFIG_TYPE_DEV_DISCOVER_REQ     2
#define MIOT_MESH_CONFIG_TYPE_DEV_DISCOVER_RSP     3
#define MIOT_MESH_CONFIG_TYPE_NET_PARAM_REQ        4
#define MIOT_MESH_CONFIG_TYPE_NET_PARAM_RSP        5

/** @} */

/**
 * @defgroup MIOT_MODEL_ID Model ID
 * @brief Mesh model id
 * @{
 */
#define MESH_MODEL_MIOT_SERVER                 0x0000038F
#define MESH_MODEL_MIOT_CLIENT                 0x0001038F
/** @} */

/**
 * @defgroup MIOT_MESH_MSG Mesh Msg
 * @brief Mesh message types used by models
 * @{
 */
#define MIOT_PARAM_MAX_LEN                     6

typedef struct
{
    uint8_t opcode[ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_GET)];
    uint8_t parameters[MIOT_PARAM_MAX_LEN];
} _PACKED_ miot_get_t;

typedef struct
{
    uint8_t opcode[ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_SET)];
    uint8_t parameters[MIOT_PARAM_MAX_LEN];
} _PACKED_ miot_set_t;

typedef struct
{
    uint8_t opcode[ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_STATUS)];
    uint8_t parameters[MIOT_PARAM_MAX_LEN];
} _PACKED_ miot_status_t;

typedef struct
{
    uint8_t opcode[ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_INDICATION)];
    uint8_t parameters[MIOT_PARAM_MAX_LEN];
} _PACKED_ miot_indication_t;

typedef struct
{
    uint8_t opcode[ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_INDICATION_ACK)];
    uint8_t parameters[MIOT_PARAM_MAX_LEN];
} _PACKED_ miot_indication_ack_t;

typedef struct
{
    uint8_t opcode[ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_MESH_CONFIG)];
    uint8_t type;
    uint16_t sub_addr;
} _PACKED_ miot_sub_status_t;

typedef struct
{
    uint8_t opcode[ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_MESH_CONFIG)];
    uint8_t type;
    uint8_t tid;
} _PACKED_ miot_dev_discover_req_t;

typedef struct
{
    uint8_t opcode[ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_MESH_CONFIG)];
    uint8_t type;
} _PACKED_ miot_dev_discover_rsp_t;

typedef struct
{
    uint8_t opcode[ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_MESH_CONFIG)];
    uint8_t type;
    uint8_t retry_cnt;
} _PACKED_ miot_net_param_req_t;

typedef struct
{
    uint8_t opcode[ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_MESH_CONFIG)];
    uint8_t type;
    uint8_t pub_interval;
} _PACKED_ miot_net_param_rsp_t;

/** @} */

/**
 * @defgroup MIOT_SERVER_DATA Server Data
 * @brief Data types and structure used by data process callback
 * @{
 */
#define MIOT_SERVER_GET                                        0 //!< @ref miot_server_get_t
#define MIOT_SERVER_SET                                        1 //!< @ref miot_server_set_t
#define MIOT_SERVER_SET_UNACK                                  2 //!< @ref miot_server_set_t
#define MIOT_SERVER_SUB_STATUS                                 3 //!< @ref miot_servers_sub_status_t
#define MIOT_SERVER_DEV_DISCOVER_RSP                           4 //!< @ref miot_server_dev_discover_rsp_t
#define MIOT_SERVER_NET_PARAM_REQ                              5 //!< @ref miot_server_net_param_req_t
#define MIOT_SERVER_NET_PARAM_RSP                              6 //!< @ref miot_server_net_param_rsp_t
#define MIOT_SERVER_INDICATION_ACK                             7 //!< @ref miot_server_indication_ack_t

typedef struct
{
    uint16_t src;
    uint16_t dst;
    uint16_t app_key_index;
    uint8_t param_len;
    uint8_t parameters[MIOT_PARAM_MAX_LEN];
} miot_server_get_t;

typedef struct
{
    uint16_t src;
    uint16_t dst;
    uint16_t app_key_index;
    uint8_t param_len;
    uint8_t parameters[MIOT_PARAM_MAX_LEN];
} miot_server_set_t;

typedef struct
{
    uint16_t src;
    uint16_t dst;
    uint16_t primary_grp_addr;
} miot_server_sub_status_t;

typedef struct
{
    uint16_t src;
    uint16_t dst;
} miot_server_dev_discover_rsp_t;

typedef struct
{
    uint16_t src;
    uint16_t dst;
    uint8_t retry_cnt;
} miot_server_net_param_req_t;

typedef struct
{
    uint16_t src;
    uint16_t dst;
    uint8_t pub_interval;
} miot_server_net_param_rsp_t;

typedef struct
{
    uint8_t param_len;
    uint8_t parameters[MIOT_PARAM_MAX_LEN];
} miot_server_indication_ack_t;
/** @} */

/**
 * @defgroup MIOT_CLIENT_DATA Client Data
 * @brief Data types and structure used by data process callback
 * @{
 */
#define MIOT_CLIENT_STATUS                     0 //!< @ref miot_client_status_t
#define MIOT_CLIENT_INDICATION                 1 //!< @ref miot_client_indication_t

typedef struct
{
    uint8_t param_len;
    uint8_t parameters[MIOT_PARAM_MAX_LEN];
} miot_client_status_t;

typedef struct
{
    uint16_t src;
    uint16_t app_key_index;
    uint8_t param_len;
    uint8_t parameters[MIOT_PARAM_MAX_LEN];
} miot_client_indication_t;
/** @} */

/**
 * @defgroup MIOT_SERVER_API Server API
 * @brief Functions declaration
 * @{
 */

/**
 * @brief register miot server
 * @param[in] element_index: element index that model registered to
 * @param[in] pmodel_info: pointer to miot server model context
 * @retval TRUE: register success
 * @retval FALSE: register failed
 */
bool miot_server_reg(uint8_t element_index, mesh_model_info_p pmodel_info);

/**
 * @brief return miot data status
 * @param[in] pmodel_info: pointer to miot server model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] pdata: miot status data
 * @param[in] len: miot status data length
 * @param[in] delay_time: message response delay time
 * @return send status
 */
mesh_msg_send_cause_t miot_status(const mesh_model_info_p pmodel_info, uint16_t dst,
                                  uint16_t app_key_index, const uint8_t *pdata, uint8_t len,
                                  uint32_t delay_time);

/**
 * @brief publish miot data
 * @param[in] pmodel_info: pointer to miot server model context
 * @param[in] pdata: miot status data
 * @param[in] len: miot status data length
 * @return send status
 */
mesh_msg_send_cause_t miot_publish(const mesh_model_info_p pmodel_info, const uint8_t *pdata,
                                   uint8_t len);

/**
 * @brief indication miot data
 * @param[in] pmodel_info: pointer to miot server model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] pdata: data need to publish
 * @param[in] data_len: data length
 * @return publish status
 */
mesh_msg_send_cause_t miot_indication(const mesh_model_info_p pmodel_info, uint16_t dst,
                                      uint16_t app_key_index,
                                      const uint8_t *pdata, uint8_t len);

/**
 * @brief miot subscribe status
 * @param[in] pmodel_info: pointer to miot server model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] sub_addr: subscribe address
 * @param[in] delay_time: message response delay time
 * @return send status
 */
mesh_msg_send_cause_t miot_sub_status(const mesh_model_info_p pmodel_info, uint16_t dst,
                                      uint16_t app_key_index, uint16_t sub_addr, uint32_t delay_time);

/**
 * @brief miot device discover request
 * @param[in] pmodel_info: pointer to miot server model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] tid: tid
 * @param[in] delay_time: message response delay time
 * @return send status
 */
mesh_msg_send_cause_t miot_dev_discover_req(const mesh_model_info_p pmodel_info, uint16_t dst,
                                            uint16_t app_key_index, uint8_t tid, uint32_t delay_time);

/**
 * @brief miot request
 * @param[in] pmodel_info: pointer to miot server model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] pub_interval: periodic publish interval
 * @param[in] delay_time: message response delay time
 * @return send status
 */
mesh_msg_send_cause_t miot_dev_discover_rsp(const mesh_model_info_p pmodel_info, uint16_t dst,
                                            uint16_t app_key_index, uint32_t delay_time);


/**
 * @brief miot network information request
 * @param[in] pmodel_info: pointer to miot server model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] retry_cnt: retry count
 * @param[in] delay_time: message response delay time
 * @return send status
 */
mesh_msg_send_cause_t miot_net_param_req(const mesh_model_info_p pmodel_info, uint16_t dst,
                                         uint16_t app_key_index, uint8_t retry_cnt, uint32_t delay_time);

/**
 * @brief miot network information request
 * @param[in] pmodel_info: pointer to miot server model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] pub_interval: periodic publish interval
 * @param[in] delay_time: message response delay time
 * @return send status
 */
mesh_msg_send_cause_t miot_net_param_rsp(const mesh_model_info_p pmodel_info, uint16_t dst,
                                         uint16_t app_key_index, uint8_t pub_interval, uint32_t delay_time);




/** @} */

/**
 * @defgroup MIOT_CLIENT_API Client API
 * @brief Functions declaration
 * @{
 */

/**
 * @brief register miot client
 * @param[in] element_index: element index that model registered to
 * @param[in] pmodel_info: pointer to miot client model context
 * @retval TRUE: register success
 * @retval FALSE: register failed
 */
bool miot_client_reg(uint8_t element_index, mesh_model_info_p pmodel_info);

/**
 * @brief get miot status
 * @param[in] pmodel_info: pointer to miot client model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] pdata: miot get data
 * @param[in] len: miot get data length
 * @return send status
 */
mesh_msg_send_cause_t miot_get(const mesh_model_info_p pmodel_info, uint16_t dst,
                               uint16_t app_key_index, const uint8_t *pdata, uint8_t len);

/**
 * @brief set miot data
 * @param[in] pmodel_info: pointer to miot client model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] pdata: miot set data
 * @param[in] len: miot set data length
 * @param[in] ack: acknowledge flag
 * @return send status
 */
mesh_msg_send_cause_t miot_set(const mesh_model_info_p pmodel_info, uint16_t dst,
                               uint16_t app_key_index, const uint8_t *pdata, uint8_t len,
                               bool ack);
/**
 * @brief ack miot indication
 * @param[in] pmodel_info: pointer to miot client model context
 * @param[in] dst: remote address
 * @param[in] app_key_index: mesh message used app key index
 * @param[in] pdata: miot indication ack data
 * @param[in] len: miot indication ack data length
 * @return send status
 */
mesh_msg_send_cause_t miot_indication_ack(const mesh_model_info_p pmodel_info, uint16_t dst,
                                          uint16_t app_key_index, const uint8_t *pdata, uint8_t len);

/** @} */
/** @} */

END_DECLS

#endif /* _MIOT_MODEL_H */
