/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      miot_model_server_app.c
* @brief     xiaomi vendor model server application
* @details
* @author    hector_huang
* @date      2019-04-16
* @version   v1.0
* *********************************************************************************************************
*/
#include <stdlib.h>
#include "miot_model_server_app.h"
#include "miot_model.h"
#include "app_msg.h"
#include "light_sw_timer.h"
#include "app_task.h"
#if MODEL_ENABLE_DELAY_MSG_RSP
#include "delay_msg_rsp.h"
#endif
#include "mijia_mesh_config.h"
#include "mijia_mesh_publish.h"

/* miot server model */
mesh_model_info_t miot_server;

/* miot indication period */
#define MIOT_INDICATE_PERIOD             300 /* unit is ms */

/* indication timer */
static plt_timer_t miot_indicate_timer;

/* indication count */
static uint8_t miot_indicate_count;
#define MIOT_INDICATE_MAX_COUNT          20

/* indication data */
typedef struct
{
    uint16_t dst;
    uint16_t app_key_index;
    uint8_t data[MIOT_PARAM_MAX_LEN];
    uint8_t len;
} miot_server_indicate_data_t;

static miot_server_indicate_data_t miot_indicate_data;

/* periodic publish parameters */
typedef struct
{
    uint16_t dst;
    uint16_t app_key_index;
} miot_net_param_req_data_t;

#define MIOT_NET_PARAM_REQ_INTERVAL_SLOW             14400000
#define MIOT_NET_PARAM_REQ_INTERVAL_FAST             3600000
#define MIOT_NET_PARAM_REQ_INIT_TIME_MIN             1000
#define MIOT_NET_PARAM_REQ_INIT_TIME_MAX             1800000
#define MIOT_NET_PARAM_RETRY_CNT_MAX                 0xFF
static plt_timer_t miot_net_param_req_timer;
static uint8_t miot_net_param_retry_cnt;
static miot_net_param_req_data_t miot_net_param_req_data;

int32_t miot_server_data(const mesh_model_info_p pmodel_info, uint32_t type,
                         void *pargs)
{
    switch (type)
    {
    case MIOT_SERVER_GET:
        {
            miot_server_get_t *pdata = pargs;
            printi("miot_server_get:");
            dprinti(pdata->parameters, pdata->param_len);

            /* @note use actual data to replace sample data */
            uint8_t status[MIOT_PARAM_MAX_LEN] = {pdata->parameters[0], pdata->parameters[1], 1, 2, 3, 4};
            uint32_t delay_rsp_time = 0;
#if MODEL_ENABLE_DELAY_MSG_RSP
            delay_rsp_time = delay_msg_get_rsp_delay(pdata->dst);
#endif
            miot_status(pmodel_info, pdata->src, pdata->app_key_index, status, 6, delay_rsp_time);
        }
        break;
    case MIOT_SERVER_SET:
    case MIOT_SERVER_SET_UNACK:
        {
            miot_server_set_t *pdata = pargs;
            if (MIOT_SERVER_SET == type)
            {
                printi("miot_server_set:");
                dprinti(pdata->parameters, pdata->param_len);
                /* @note use actual data to replace sample data */
                uint8_t status[MIOT_PARAM_MAX_LEN] = {pdata->parameters[0], pdata->parameters[1], 1, 2, 3, 4};
                uint32_t delay_rsp_time = 0;
#if MODEL_ENABLE_DELAY_MSG_RSP
                delay_rsp_time = delay_msg_get_rsp_delay(pdata->dst);
#endif
                miot_status(pmodel_info, pdata->src, pdata->app_key_index, status, 6, delay_rsp_time);
            }
            else
            {
                printi("miot_server_set_unack:");
                dprinti(pdata->parameters, pdata->param_len);
            }
        }
        break;
    case MIOT_SERVER_INDICATION_ACK:
        {
            miot_server_indication_ack_t *pdata = pargs;
            printi("miot_server_indication_ack:");
            dprinti(pdata->parameters, pdata->param_len);

            /* stop indication send */
            if (NULL != miot_indicate_timer)
            {
                if (plt_timer_is_active(miot_indicate_timer))
                {
                    plt_timer_stop(miot_indicate_timer, 0);
                }
            }
            miot_indicate_count = 0;
        }
        break;
#if MI_PUB_NEW_STRATEGY
    case MIOT_SERVER_NET_PARAM_RSP:
        {
            miot_server_net_param_rsp_t *pdata = pargs;
            printi("miot_server_net_param_rsp: interval %d", pdata->pub_interval);
            mi_publish_interval_set(pdata->pub_interval);
            miot_net_param_retry_cnt = 0;
            plt_timer_change_period(miot_net_param_req_timer, MIOT_NET_PARAM_REQ_INTERVAL_SLOW, 0);
        }
        break;
#endif
    default:
        break;
    }

    return 0;
}

static void miot_server_indicate_timeout(void *pargs)
{
    T_IO_MSG miot_indicate_timeout_msg;
    miot_indicate_timeout_msg.type = IO_MSG_TYPE_TIMER;
    miot_indicate_timeout_msg.subtype  = MIOT_INDICATE_TIMEOUT;
    app_send_msg_to_apptask(&miot_indicate_timeout_msg);
}

void miot_server_handle_indicate_timeout(void)
{
    miot_indicate_count ++;
    if (miot_indicate_count >= MIOT_INDICATE_MAX_COUNT)
    {
        plt_timer_stop(miot_indicate_timer, 0);
    }

    miot_indication(&miot_server, miot_indicate_data.dst, miot_indicate_data.app_key_index,
                    miot_indicate_data.data, miot_indicate_data.len);
}

void miot_server_indication(uint16_t dst, uint16_t app_key_index, const uint8_t *pdata, uint8_t len)
{
    if (len > MIOT_PARAM_MAX_LEN)
    {
        printe("inidication length(%d) exceed maximum allowed(%d)", len, MIOT_PARAM_MAX_LEN);
    }

    miot_indicate_data.dst = dst;
    miot_indicate_data.app_key_index = app_key_index;
    memcpy(miot_indicate_data.data, pdata, len);
    miot_indicate_data.len = len;

    if (NULL == miot_indicate_timer)
    {
        miot_indicate_timer = plt_timer_create("miot_indicate", MIOT_INDICATE_PERIOD, TRUE, 0,
                                               miot_server_indicate_timeout);
        if (NULL != miot_indicate_timer)
        {
            plt_timer_start(miot_indicate_timer, 0);
        }
    }
    else
    {
        if (!plt_timer_is_active(miot_indicate_timer))
        {
            plt_timer_start(miot_indicate_timer, 0);
        }
    }

    miot_indicate_count = 0;
}

static void miot_server_net_param_req_timeout(void *pargs)
{
    T_IO_MSG miot_net_param_req_timeout_msg;
    miot_net_param_req_timeout_msg.type = IO_MSG_TYPE_TIMER;
    miot_net_param_req_timeout_msg.subtype  = MIOT_NET_PARAM_REQ_TIMEOUT;
    app_send_msg_to_apptask(&miot_net_param_req_timeout_msg);
}

void miot_server_handle_net_param_req_timeout(void)
{
    miot_net_param_req(&miot_server, miot_net_param_req_data.dst, miot_net_param_req_data.app_key_index,
                       miot_net_param_retry_cnt, 0);

    if (miot_net_param_retry_cnt < MIOT_NET_PARAM_RETRY_CNT_MAX)
    {
        miot_net_param_retry_cnt ++;
    }

    plt_timer_change_period(miot_net_param_req_timer, MIOT_NET_PARAM_REQ_INTERVAL_FAST, 0);
}


void miot_server_net_param_req_start(uint16_t dst, uint16_t app_key_index)
{
    miot_net_param_req_data.dst = dst;
    miot_net_param_req_data.app_key_index = app_key_index;
    miot_net_param_retry_cnt = 0;

    /* calculate random delay time */
    int delay = rand();
    uint32_t real_delay = delay;
    /* random delay 1-1800s */
    real_delay %= MIOT_NET_PARAM_REQ_INIT_TIME_MAX;
    if (real_delay < MIOT_NET_PARAM_REQ_INIT_TIME_MIN)
    {
        real_delay += MIOT_NET_PARAM_REQ_INIT_TIME_MIN;
    }

    /* start publish interval request timer */
    if (NULL == miot_net_param_req_timer)
    {
        /* request after random delay */
        miot_net_param_req_timer = plt_timer_create("pub_req", real_delay, FALSE, 0,
                                                    miot_server_net_param_req_timeout);
        plt_timer_start(miot_net_param_req_timer, 0);
    }
    else
    {
        plt_timer_change_period(miot_net_param_req_timer, real_delay, 0);
    }
}

void miot_server_net_param_req_stop(void)
{
    if (NULL != miot_net_param_req_timer)
    {
        plt_timer_delete(miot_net_param_req_timer, 0);
        miot_net_param_req_timer = NULL;
    }
    memset(&miot_net_param_req_data, 0, sizeof(miot_net_param_req_data_t));
    miot_net_param_retry_cnt = 0;
}
