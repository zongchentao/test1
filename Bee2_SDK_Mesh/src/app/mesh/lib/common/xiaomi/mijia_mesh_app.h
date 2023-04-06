/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file     mijia_mesh_app.h
* @brief    Head file for mijia mesh application.
* @details  Data types and external functions declaration.
* @author   hector_huang
* @date     2019-1-8
* @version  v1.0
* *************************************************************************************
*/
#ifndef _MIJIA_MESH_APP_H_
#define _MIJIA_MESH_APP_H_

#include "platform_types.h"
#include "app_msg.h"

/* mi inner message type */
typedef enum
{
    MI_BT_STATUS_UPDATE,
    MI_BEACON_TIMEOUT,
    MI_UDB_PERIOD_TIMEOUT,
    MI_SCHD_TIMEOUT,
    MI_PUB_TIMEOUT,
    MI_PUB_SINGLE_TIMEOUT,
} mi_inner_msg_type_t;

/* mi inner message data */
typedef struct
{
    mi_inner_msg_type_t type;
    uint16_t sub_type;
    union
    {
        uint32_t parm;
        void *pbuf;
    };
} mi_inner_msg_t;

/* mi bond capability */
typedef enum
{
    MI_BOND_CAP_USER_CHOOSE,
    MI_BOND_CAP_SCAN_AND_CONFIRM,
    MI_BOND_CAP_SCAN_AND_CONNECT,
    MI_BOND_CAP_COMBO
} mi_bond_cap_t;

/* mi io capability */
typedef enum
{
    MI_IO_CAP_INPUT_NUM,
    MI_IO_CAP_INPUT_ALPHA,
    MI_IO_CAP_READ_NFC_TAG,
    MI_IO_CAP_READ_QR_CODE,
    MI_IO_CAP_OUTPUT_NUM,
    MI_IO_CAP_OUTPUT_ALPHA,
    MI_IO_CAP_GENERATE_NFC_TAG,
    MI_IO_CAP_GENERATE_QR_CODE,
} mi_io_cap_t;

/* mi mesh state */
typedef enum
{
    MI_MESH_STATE_UNAUTH,
    MI_MESH_STATE_UNPROV,
    MI_MESH_STATE_UNCONFIG,
    MI_MESH_STATE_AVIAL,
} mi_mesh_state_t;

/**
 * @brief run mi mesh wrapper
 */
void mi_mesh_run(void);

/**
 * @brief handle gap message
 * @param[in] pmsg: message need to handle
 * @return void
 */
void mi_handle_gap_msg(T_IO_MSG *pmsg);

/**
 * @brief initialize mi mesh event type and queue
 * @param[in] event_mi: event used in mi app
 * @param[in] event_queue: event queue used to receive and send message to app task
 */
void mi_mesh_start(uint8_t event_mi, void *event_queue);

/**
 * @brief send mi inner message to app task
 * @param[in] pmsg: message need to send
 * @return send status
 * @retval TRUE: send success
 * @retval FALSE: send failed
 */
bool mi_inner_msg_send(mi_inner_msg_t *pmsg);

/**
 * @brief handle message from app task
 * @param[in] event: event type
 */
void mi_inner_msg_handle(uint8_t event);

/**
 * @brief notify node unprovisioned
 */
void mi_unprov(void);

/**
 * @brief configure node default parameters
 */
void mi_default_config(void);

/**
 * @brief suspend rtc
 */
void mi_gatts_suspend(void);

/**
 * @brief resume rtc
 */
void mi_gatts_resume(void);

/**
 * @brief startup delay
 */
void mi_startup_delay(void);

#endif /* _MIJIA_BLE_APP_H_ */
