/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      miot_model_client.c
* @brief     xiaomi vendor model client
* @details
* @author    hector_huang
* @date      2019-04-15
* @version   v1.0
* *********************************************************************************************************
*/
#include "miot_model.h"

static mesh_msg_send_cause_t miot_client_send(const mesh_model_info_p pmodel_info,
                                              uint16_t dst, uint16_t app_key_index,
                                              uint8_t *pmsg, uint16_t msg_len)
{
    mesh_msg_t mesh_msg;
    mesh_msg.pmodel_info = pmodel_info;
    access_cfg(&mesh_msg);
    mesh_msg.pbuffer = pmsg;
    mesh_msg.msg_len = msg_len;
    mesh_msg.dst = dst;
    mesh_msg.app_key_index = app_key_index;
    return access_send(&mesh_msg);
}

mesh_msg_send_cause_t miot_get(const mesh_model_info_p pmodel_info, uint16_t dst,
                               uint16_t app_key_index, const uint8_t *pdata, uint8_t len)
{
    if (len > MIOT_PARAM_MAX_LEN)
    {
        return MESH_MSG_SEND_CAUSE_PAYLOAD_SIZE_EXCEED;
    }

    miot_get_t msg;
    ACCESS_OPCODE_BYTE(msg.opcode, MESH_MSG_MIOT_GET);
    uint16_t msg_len = ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_GET) + len;
    memcpy(msg.parameters, pdata, len);

    return miot_client_send(pmodel_info, dst, app_key_index, (uint8_t *)&msg, msg_len);
}

mesh_msg_send_cause_t miot_set(const mesh_model_info_p pmodel_info, uint16_t dst,
                               uint16_t app_key_index, const uint8_t *pdata, uint8_t len,
                               bool ack)
{
    if (len > MIOT_PARAM_MAX_LEN)
    {
        return MESH_MSG_SEND_CAUSE_PAYLOAD_SIZE_EXCEED;
    }

    miot_set_t msg;
    if (ack)
    {
        ACCESS_OPCODE_BYTE(msg.opcode, MESH_MSG_MIOT_SET);
    }
    else
    {
        ACCESS_OPCODE_BYTE(msg.opcode, MESH_MSG_MIOT_SET_UNACK);
    }

    uint16_t msg_len = ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_SET) + len;
    memcpy(msg.parameters, pdata, len);

    return miot_client_send(pmodel_info, dst, app_key_index, (uint8_t *)&msg, msg_len);
}

mesh_msg_send_cause_t miot_indication_ack(const mesh_model_info_p pmodel_info, uint16_t dst,
                                          uint16_t app_key_index, const uint8_t *pdata, uint8_t len)
{
    if (len > MIOT_PARAM_MAX_LEN)
    {
        return MESH_MSG_SEND_CAUSE_PAYLOAD_SIZE_EXCEED;
    }

    miot_get_t msg;
    ACCESS_OPCODE_BYTE(msg.opcode, MESH_MSG_MIOT_INDICATION_ACK);
    uint16_t msg_len = ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_INDICATION_ACK) + len;
    memcpy(msg.parameters, pdata, len);

    return miot_client_send(pmodel_info, dst, app_key_index, (uint8_t *)&msg, msg_len);
}

static bool miot_client_receive(mesh_msg_p pmesh_msg)
{
    bool ret = TRUE;
    uint8_t *pbuffer = pmesh_msg->pbuffer + pmesh_msg->msg_offset;
    mesh_model_info_p pmodel_info = pmesh_msg->pmodel_info;

    switch (pmesh_msg->access_opcode)
    {
    case MESH_MSG_MIOT_STATUS:
        {
            miot_client_status_t status_data;
            status_data.param_len = pmesh_msg->msg_len - ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_STATUS);
            if (status_data.param_len > MIOT_PARAM_MAX_LEN)
            {
                status_data.param_len = MIOT_PARAM_MAX_LEN;
            }
            memcpy(status_data.parameters, pbuffer + MEMBER_OFFSET(miot_status_t, parameters),
                   status_data.param_len);
            if (NULL != pmodel_info->model_data_cb)
            {
                pmodel_info->model_data_cb(pmodel_info, MIOT_CLIENT_STATUS, &status_data);
            }
        }
        break;
    case MESH_MSG_MIOT_INDICATION:
        {
            miot_client_indication_t indication_data;
            indication_data.src = pmesh_msg->src;
            indication_data.app_key_index = pmesh_msg->app_key_index;
            indication_data.param_len = pmesh_msg->msg_len - ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_INDICATION);
            if (indication_data.param_len > MIOT_PARAM_MAX_LEN)
            {
                indication_data.param_len = MIOT_PARAM_MAX_LEN;
            }
            memcpy(indication_data.parameters, pbuffer + MEMBER_OFFSET(miot_indication_t, parameters),
                   indication_data.param_len);
            if (NULL != pmodel_info->model_data_cb)
            {
                pmodel_info->model_data_cb(pmodel_info, MIOT_CLIENT_INDICATION, &indication_data);
            }
        }
        break;
    default:
        ret = FALSE;
        break;
    }
    return ret;
}

bool miot_client_reg(uint8_t element_index, mesh_model_info_p pmodel_info)
{
    if (NULL == pmodel_info)
    {
        return FALSE;
    }

    pmodel_info->model_id = MESH_MODEL_MIOT_CLIENT;
    if (NULL == pmodel_info->model_receive)
    {
        pmodel_info->model_receive = miot_client_receive;
        if (NULL == pmodel_info->model_data_cb)
        {
            printw("miot_client_reg: missing data process callback!");
        }
    }
    return mesh_model_reg(element_index, pmodel_info);
}

