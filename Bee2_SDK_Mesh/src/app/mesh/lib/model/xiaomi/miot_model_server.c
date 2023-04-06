/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      miot_model_server.c
* @brief     xiaomi vendor model server
* @details
* @author    hector_huang
* @date      2019-04-15
* @version   v1.0
* *********************************************************************************************************
*/
#include "miot_model.h"
#if MODEL_ENABLE_DELAY_MSG_RSP
#include "delay_msg_rsp.h"
#endif


mesh_msg_send_cause_t miot_status(const mesh_model_info_p pmodel_info, uint16_t dst,
                                  uint16_t app_key_index, const uint8_t *pdata, uint8_t len,
                                  uint32_t delay_time)
{
    if (len > MIOT_PARAM_MAX_LEN)
    {
        return MESH_MSG_SEND_CAUSE_PAYLOAD_SIZE_EXCEED;
    }
    miot_status_t msg;
    uint16_t msg_len = ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_STATUS) + len;
    ACCESS_OPCODE_BYTE(msg.opcode, MESH_MSG_MIOT_STATUS);
    memcpy(msg.parameters, pdata, len);

    mesh_msg_t mesh_msg;
    mesh_msg.pmodel_info = pmodel_info;
    access_cfg(&mesh_msg);
    mesh_msg.pbuffer = (uint8_t *)&msg;
    mesh_msg.msg_len = msg_len;
    if (0 != dst)
    {
        mesh_msg.dst = dst;
        mesh_msg.app_key_index = app_key_index;
    }
    mesh_msg.delay_time = delay_time;
    return access_send(&mesh_msg);
}

mesh_msg_send_cause_t miot_sub_status(const mesh_model_info_p pmodel_info, uint16_t dst,
                                      uint16_t app_key_index, uint16_t sub_addr, uint32_t delay_time)
{
    miot_sub_status_t msg;
    ACCESS_OPCODE_BYTE(msg.opcode, MESH_MSG_MIOT_MESH_CONFIG);
    msg.type = MIOT_MESH_CONFIG_TYPE_SUB_STATUS;
    msg.sub_addr = sub_addr;

    mesh_msg_t mesh_msg;
    mesh_msg.pmodel_info = pmodel_info;
    access_cfg(&mesh_msg);
    mesh_msg.pbuffer = (uint8_t *)&msg;
    mesh_msg.msg_len = sizeof(msg);
    mesh_msg.dst = dst;
    mesh_msg.app_key_index = app_key_index;
    mesh_msg.delay_time = delay_time;
    return access_send(&mesh_msg);
}

mesh_msg_send_cause_t miot_dev_discover_req(const mesh_model_info_p pmodel_info, uint16_t dst,
                                            uint16_t app_key_index, uint8_t tid, uint32_t delay_time)
{
    miot_dev_discover_req_t msg;
    ACCESS_OPCODE_BYTE(msg.opcode, MESH_MSG_MIOT_MESH_CONFIG);
    msg.type = MIOT_MESH_CONFIG_TYPE_DEV_DISCOVER_REQ;
    msg.tid = tid;

    mesh_msg_t mesh_msg;
    mesh_msg.pmodel_info = pmodel_info;
    access_cfg(&mesh_msg);
    mesh_msg.pbuffer = (uint8_t *)&msg;
    mesh_msg.msg_len = sizeof(msg);
    mesh_msg.dst = dst;
    mesh_msg.app_key_index = app_key_index;
    mesh_msg.delay_time = delay_time;
    return access_send(&mesh_msg);
}

mesh_msg_send_cause_t miot_dev_discover_rsp(const mesh_model_info_p pmodel_info, uint16_t dst,
                                            uint16_t app_key_index, uint32_t delay_time)
{
    miot_dev_discover_rsp_t msg;
    ACCESS_OPCODE_BYTE(msg.opcode, MESH_MSG_MIOT_MESH_CONFIG);
    msg.type = MIOT_MESH_CONFIG_TYPE_DEV_DISCOVER_RSP;

    mesh_msg_t mesh_msg;
    mesh_msg.pmodel_info = pmodel_info;
    access_cfg(&mesh_msg);
    mesh_msg.pbuffer = (uint8_t *)&msg;
    mesh_msg.msg_len = sizeof(msg);
    mesh_msg.dst = dst;
    mesh_msg.app_key_index = app_key_index;
    mesh_msg.delay_time = delay_time;
    return access_send(&mesh_msg);
}

mesh_msg_send_cause_t miot_net_param_req(const mesh_model_info_p pmodel_info, uint16_t dst,
                                         uint16_t app_key_index, uint8_t retry_cnt, uint32_t delay_time)
{
    miot_net_param_req_t msg;
    ACCESS_OPCODE_BYTE(msg.opcode, MESH_MSG_MIOT_MESH_CONFIG);
    msg.type = MIOT_MESH_CONFIG_TYPE_NET_PARAM_REQ;
    msg.retry_cnt = retry_cnt;

    mesh_msg_t mesh_msg;
    mesh_msg.pmodel_info = pmodel_info;
    access_cfg(&mesh_msg);
    mesh_msg.pbuffer = (uint8_t *)&msg;
    mesh_msg.msg_len = sizeof(msg);
    mesh_msg.dst = dst;
    mesh_msg.app_key_index = app_key_index;
    mesh_msg.delay_time = delay_time;
    return access_send(&mesh_msg);
}

mesh_msg_send_cause_t miot_net_param_rsp(const mesh_model_info_p pmodel_info, uint16_t dst,
                                         uint16_t app_key_index, uint8_t pub_interval, uint32_t delay_time)
{
    miot_net_param_rsp_t msg;
    ACCESS_OPCODE_BYTE(msg.opcode, MESH_MSG_MIOT_MESH_CONFIG);
    msg.type = MIOT_MESH_CONFIG_TYPE_NET_PARAM_RSP;
    msg.pub_interval = pub_interval;

    mesh_msg_t mesh_msg;
    mesh_msg.pmodel_info = pmodel_info;
    access_cfg(&mesh_msg);
    mesh_msg.pbuffer = (uint8_t *)&msg;
    mesh_msg.msg_len = sizeof(msg);
    mesh_msg.dst = dst;
    mesh_msg.app_key_index = app_key_index;
    mesh_msg.delay_time = delay_time;
    return access_send(&mesh_msg);
}

mesh_msg_send_cause_t miot_publish(const mesh_model_info_p pmodel_info, const uint8_t *pdata,
                                   uint8_t len)
{
    mesh_msg_send_cause_t ret = MESH_MSG_SEND_CAUSE_INVALID_DST;
    if (mesh_model_pub_check(pmodel_info))
    {
        ret = miot_status(pmodel_info, 0, 0, pdata, len, 0);
    }

    return ret;
}

mesh_msg_send_cause_t miot_indication(const mesh_model_info_p pmodel_info, uint16_t dst,
                                      uint16_t app_key_index,
                                      const uint8_t *pdata, uint8_t len)
{
    if (len > MIOT_PARAM_MAX_LEN)
    {
        return MESH_MSG_SEND_CAUSE_PAYLOAD_SIZE_EXCEED;
    }
    miot_status_t msg;
    uint16_t msg_len = ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_INDICATION) + len;
    ACCESS_OPCODE_BYTE(msg.opcode, MESH_MSG_MIOT_INDICATION);
    memcpy(msg.parameters, pdata, len);

    mesh_msg_t mesh_msg;
    mesh_msg.pmodel_info = pmodel_info;
    access_cfg(&mesh_msg);
    mesh_msg.pbuffer = (uint8_t *)&msg;
    mesh_msg.msg_len = msg_len;
    mesh_msg.dst = dst;
    mesh_msg.app_key_index = app_key_index;
    return access_send(&mesh_msg);
}

/**
 * @brief default miot receive function
 * @param[in] pmesh_msg: received mesh message
 * @return process result
 */
static bool miot_server_receive(mesh_msg_p pmesh_msg)
{
    bool ret = TRUE;
    uint8_t *pbuffer = pmesh_msg->pbuffer + pmesh_msg->msg_offset;
    mesh_model_info_p pmodel_info = pmesh_msg->pmodel_info;

    switch (pmesh_msg->access_opcode)
    {
    case MESH_MSG_MIOT_GET:
        {
            miot_server_get_t get_data;
            get_data.src = pmesh_msg->src;
            get_data.dst = pmesh_msg->dst;
            get_data.app_key_index = pmesh_msg->app_key_index;
            get_data.param_len = pmesh_msg->msg_len - ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_GET);
            if (get_data.param_len > MIOT_PARAM_MAX_LEN)
            {
                get_data.param_len = MIOT_PARAM_MAX_LEN;
            }
            memcpy(get_data.parameters, pbuffer + MEMBER_OFFSET(miot_get_t, parameters), get_data.param_len);
            if (NULL != pmodel_info->model_data_cb)
            {
                pmodel_info->model_data_cb(pmodel_info, MIOT_SERVER_GET, &get_data);
            }
        }
        break;
    case MESH_MSG_MIOT_SET:
    case MESH_MSG_MIOT_SET_UNACK:
        {
            miot_server_set_t set_data;
            set_data.src = pmesh_msg->src;
            set_data.dst = pmesh_msg->dst;
            set_data.app_key_index = pmesh_msg->app_key_index;
            set_data.param_len = pmesh_msg->msg_len - ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_SET);
            if (set_data.param_len > MIOT_PARAM_MAX_LEN)
            {
                set_data.param_len = MIOT_PARAM_MAX_LEN;
            }
            memcpy(set_data.parameters, pbuffer + MEMBER_OFFSET(miot_set_t, parameters), set_data.param_len);
            if (NULL != pmodel_info->model_data_cb)
            {
                if (pmesh_msg->access_opcode == MESH_MSG_MIOT_SET)
                {
                    pmodel_info->model_data_cb(pmodel_info, MIOT_SERVER_SET, &set_data);
                }
                else
                {
                    pmodel_info->model_data_cb(pmodel_info, MIOT_SERVER_SET_UNACK, &set_data);
                }
            }
        }
        break;
    case MESH_MSG_MIOT_INDICATION_ACK:
        {
            miot_server_indication_ack_t indication_ack_data;
            indication_ack_data.param_len = pmesh_msg->msg_len - ACCESS_OPCODE_SIZE(
                                                MESH_MSG_MIOT_INDICATION_ACK);
            if (indication_ack_data.param_len > MIOT_PARAM_MAX_LEN)
            {
                indication_ack_data.param_len = MIOT_PARAM_MAX_LEN;
            }
            memcpy(indication_ack_data.parameters, pbuffer + MEMBER_OFFSET(miot_indication_ack_t, parameters),
                   indication_ack_data.param_len);
            if (NULL != pmodel_info->model_data_cb)
            {
                pmodel_info->model_data_cb(pmodel_info, MIOT_SERVER_INDICATION_ACK, &indication_ack_data);
            }
        }
        break;
    case MESH_MSG_MIOT_MESH_CONFIG:
        {
            uint8_t type = *(pbuffer + ACCESS_OPCODE_SIZE(MESH_MSG_MIOT_MESH_CONFIG));
            switch (type)
            {
            case MIOT_MESH_CONFIG_TYPE_SUB_STATUS:
                {
                    miot_sub_status_t *pdata = (miot_sub_status_t *)pbuffer;
                    miot_server_sub_status_t status_data;
                    status_data.src = pmesh_msg->src;
                    status_data.dst = pmesh_msg->dst;
                    status_data.primary_grp_addr = pdata->sub_addr;
                    if (NULL != pmodel_info->model_data_cb)
                    {
                        pmodel_info->model_data_cb(pmodel_info, MIOT_SERVER_SUB_STATUS, &status_data);
                    }
                }
                break;
            case MIOT_MESH_CONFIG_TYPE_DEV_DISCOVER_REQ:
                {
                    /* send response */
                    uint32_t delay_rsp_time = 0;
#if MODEL_ENABLE_DELAY_MSG_RSP
                    delay_rsp_time = delay_msg_get_rsp_delay(pmesh_msg->dst);
#endif
                    miot_dev_discover_rsp(pmodel_info, pmesh_msg->src, pmesh_msg->app_key_index, delay_rsp_time);
                }
                break;
            case MIOT_MESH_CONFIG_TYPE_DEV_DISCOVER_RSP:
                {
                    //miot_dev_discover_rsp_t *pdata = (miot_dev_discover_rsp_t *)pbuffer;
                    miot_server_dev_discover_rsp_t rsp_data;
                    rsp_data.src = pmesh_msg->src;
                    rsp_data.dst = pmesh_msg->dst;
                    if (NULL != pmodel_info->model_data_cb)
                    {
                        pmodel_info->model_data_cb(pmodel_info, MIOT_SERVER_DEV_DISCOVER_RSP, &rsp_data);
                    }
                }
                break;
            case MIOT_MESH_CONFIG_TYPE_NET_PARAM_REQ:
                {
                    miot_net_param_req_t *pdata = (miot_net_param_req_t *)pbuffer;
                    miot_server_net_param_req_t req_data;
                    req_data.src = pmesh_msg->src;
                    req_data.dst = pmesh_msg->dst;
                    req_data.retry_cnt = pdata->retry_cnt;
                    if (NULL != pmodel_info->model_data_cb)
                    {
                        pmodel_info->model_data_cb(pmodel_info, MIOT_SERVER_NET_PARAM_REQ, &req_data);
                    }
                }
                break;
            case MIOT_MESH_CONFIG_TYPE_NET_PARAM_RSP:
                {
                    miot_net_param_rsp_t *pdata = (miot_net_param_rsp_t *)pbuffer;
                    miot_server_net_param_rsp_t rsp_data;
                    rsp_data.src = pmesh_msg->src;
                    rsp_data.dst = pmesh_msg->dst;
                    rsp_data.pub_interval = pdata->pub_interval;
                    if (NULL != pmodel_info->model_data_cb)
                    {
                        pmodel_info->model_data_cb(pmodel_info, MIOT_SERVER_NET_PARAM_RSP, &rsp_data);
                    }
                }
                break;
            default:
                break;
            }
        }
        break;
    default:
        ret = FALSE;
        break;
    }
    return ret;
}

bool miot_server_reg(uint8_t element_index, mesh_model_info_p pmodel_info)
{
    if (NULL == pmodel_info)
    {
        return FALSE;
    }

    pmodel_info->model_id = MESH_MODEL_MIOT_SERVER;
    if (NULL == pmodel_info->model_receive)
    {
        pmodel_info->model_receive = miot_server_receive;
        if (NULL == pmodel_info->model_data_cb)
        {
            printw("miot_server_reg: missing model data process callback!");
        }
    }

    return mesh_model_reg(element_index, pmodel_info);
}

