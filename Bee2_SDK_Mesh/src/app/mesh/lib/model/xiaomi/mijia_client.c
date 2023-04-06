/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      mijia_model_client.c
* @brief     mijia vendor model client
* @details
* @author    astor zhang
* @date      2019-07-04
* @version   v1.0
* *********************************************************************************************************
*/
#include "mijia_model.h"

static mesh_msg_send_cause_t mijia_client_send(const mesh_model_info_p pmodel_info,
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

mesh_msg_send_cause_t mijia_get(const mesh_model_info_p pmodel_info, uint16_t dst,
                               uint16_t app_key_index, const uint8_t *pdata, uint8_t len)
{
    return MESH_MSG_SEND_CAUSE_SUCCESS;
}

mesh_msg_send_cause_t mijia_set(const mesh_model_info_p pmodel_info, uint16_t dst,
                               uint16_t app_key_index, const uint8_t *pdata, uint8_t len,
                               bool ack)
{
    return MESH_MSG_SEND_CAUSE_SUCCESS;
}

mesh_msg_send_cause_t mijia_indication_ack(const mesh_model_info_p pmodel_info, uint16_t dst,
                                          uint16_t app_key_index, const uint8_t *pdata, uint8_t len)
{
    return MESH_MSG_SEND_CAUSE_SUCCESS;
}

static bool mijia_client_receive(mesh_msg_p pmesh_msg)
{
    bool ret = TRUE;

    switch (pmesh_msg->access_opcode)
    {
    default:
        ret = FALSE;
        break;
    }
    return ret;
}

bool mijia_client_reg(uint8_t element_index, mesh_model_info_p pmodel_info)
{
    if (NULL == pmodel_info)
    {
        return FALSE;
    }

    pmodel_info->model_id = MESH_MODEL_MIJIA_CLIENT;
    if (NULL == pmodel_info->model_receive)
    {
        pmodel_info->model_receive = mijia_client_receive;
        if (NULL == pmodel_info->model_data_cb)
        {
            printw("mijia_client_reg: missing data process callback!");
        }
    }
    return mesh_model_reg(element_index, pmodel_info);
}

