/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      mijia_model_server.c
* @brief     mijia vendor model server
* @details
* @author    astor zhang
* @date      2019-07-04
* @version   v1.0
* *********************************************************************************************************
*/
#include "mijia_model.h"
#if MODEL_ENABLE_DELAY_MSG_RSP
#include "delay_msg_rsp.h"
#endif


mesh_msg_send_cause_t mijia_status(const mesh_model_info_p pmodel_info, uint16_t dst,
                                  uint16_t app_key_index, const uint8_t *pdata, uint8_t len,
                                  uint32_t delay_time)
{
    return MESH_MSG_SEND_CAUSE_SUCCESS;
}

mesh_msg_send_cause_t mijia_publish(const mesh_model_info_p pmodel_info, const uint8_t *pdata,
                                   uint8_t len)
{
    mesh_msg_send_cause_t ret = MESH_MSG_SEND_CAUSE_INVALID_DST;
    if (mesh_model_pub_check(pmodel_info))
    {
        ret = mijia_status(pmodel_info, 0, 0, pdata, len, 0);
    }

    return ret;
}

mesh_msg_send_cause_t mijia_indication(const mesh_model_info_p pmodel_info, uint16_t dst,
                                      uint16_t app_key_index,
                                      const uint8_t *pdata, uint8_t len)
{
    return MESH_MSG_SEND_CAUSE_SUCCESS;
}

/**
 * @brief default mijia receive function
 * @param[in] pmesh_msg: received mesh message
 * @return process result
 */
static bool mijia_server_receive(mesh_msg_p pmesh_msg)
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

bool mijia_server_reg(uint8_t element_index, mesh_model_info_p pmodel_info)
{
    if (NULL == pmodel_info)
    {
        return FALSE;
    }

    pmodel_info->model_id = MESH_MODEL_MIJIA_SERVER;
    if (NULL == pmodel_info->model_receive)
    {
        pmodel_info->model_receive = mijia_server_receive;
        if (NULL == pmodel_info->model_data_cb)
        {
            printw("mijia_server_reg: missing model data process callback!");
        }
    }

    return mesh_model_reg(element_index, pmodel_info);
}

