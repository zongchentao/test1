/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file     mijia_mesh_app.c
* @brief    Source file for mijia ble application.
* @details  Data types and external functions declaration.
* @author   hector_huang
* @date     2019-1-8
* @version  v1.0
* *************************************************************************************
*/
#include <stdlib.h>
#include <os_task.h>
#include "mem_config.h"
#include "mesh_api.h"
#include "data_uart.h"
#include "mijia_mesh_app.h"
#include "mible_api.h"
#define MI_LOG_MODULE_NAME "MI_APP"
#include "mible_log.h"
#include "common/mible_beacon.h"
#include "mesh_auth/mible_mesh_auth.h"
#include "gap_scheduler.h"
#include "mijia_mesh_config.h"
#include "light_effect_app.h"
#include "rtl876x_nvic.h"
#include "rtl876x_rcc.h"
#include "rtl876x_rtc.h"

#include "gap_vendor.h"
#include "otp_config.h"

#include "mijia_mesh_publish.h"
#include "miot_model_server_app.h"


/* external functions */
extern mible_status_t mible_gap_adv_send(void);
extern mible_status_t mible_record_init(void);
extern bool prov_cb(prov_cb_type_t cb_type, prov_cb_data_t cb_data);
extern void driver_rtc_init(void);
extern T_GAP_CAUSE le_vendor_set_priority(T_GAP_VENDOR_PRIORITY_PARAM *p_priority_param);

/* app message parameters */
static uint8_t mi_event;
static plt_os_queue_handle_t mi_event_queue_handle;
static plt_os_queue_handle_t mi_queue_handle;

/* RTC parameters */
#define RTC_PRESCALER_VALUE     (32-1)//f = 1000Hz
/* RTC has four comparators.0~3 */
#define RTC_COMP_INDEX          1
#define RTC_INT_CMP_NUM         RTC_INT_CMP1
#define RTC_COMP_VALUE          (10)



/* unprovisioned device becaon timer handle */
#if MI_ENABLE_STOP_UDB
static void *mi_udb_period_timer;
#endif

/* mible state, avoid multiple initialize */
static uint8_t mible_start = FALSE;

/* beacon parameters */
static mible_addr_t dev_mac;

#if !MI_PUB_NEW_STRATEGY
static bool mi_model_publish_start(void)
{
    mesh_element_t *pelement = (mesh_element_t *)mesh_node.element_queue.pfirst;
    while (NULL != pelement)
    {
        mesh_model_t *pmodel = (mesh_model_t *)pelement->model_queue.pfirst;
        while (NULL != pmodel)
        {
            for (uint16_t index = 0; index < mesh_node.app_key_num; index++)
            {
                if (plt_bit_pool_get(pmodel->app_key_binding, index) &&
                    mesh_node.app_key_list[index].key_state != MESH_KEY_STATE_INVALID)
                {
                    mi_model_publish_set(pmodel, MI_PUB_ADDRESS, app_key_index_to_global(index));
                    mesh_model_pub_start(pmodel);
                    break;
                }
            }
            pmodel = pmodel->pnext;
        }
        pelement = pelement->pnext;
    }

    return TRUE;
}
#endif

/**
 * @brief add app keys to node
 * @param[in] pkey: app key list
 */
static void mi_appkey_add(const appkey_list_t *pkey)
{
    uint16_t net_key_index = 0;
    uint16_t app_key_index = 0;
    for (uint8_t i = 0; i < pkey->size; ++i)
    {
        net_key_index = net_key_index_from_global(pkey->head[i].net_idx);

        app_key_index = app_key_add(net_key_index, pkey->head[i].app_idx, pkey->head[i].appkey);
        mesh_flash_store(MESH_FLASH_PARAMS_APP_KEY, &app_key_index);
    }
}

/**
 * @brief model app key bind
 * @param[in] pbind: model bind information
 */
static void mi_appkey_bind(const model_bind_list_t *pbind)
{
    mesh_element_p pelement;
    uint32_t model_id;
    uint16_t app_key_index;
    mesh_model_p pmodel;
    for (uint8_t i = 0; i < pbind->size; ++i)
    {
        pelement = mesh_element_get(pbind->head[i].elem_idx);
        if (NULL == pelement)
        {
            MI_LOG_ERROR("mi_appkey_bind: invalid element index(%d)", pbind->head[i].elem_idx);
            continue;
        }

        if (0 == pbind->head[i].vendor)
        {
            /* sig model */
            model_id = pbind->head[i].model;
            model_id = MESH_MODEL_TRANSFORM(model_id);
        }
        else if (COMPANY_ID == pbind->head[i].vendor)
        {
            /* vendor model */
            model_id = pbind->head[i].model;
            model_id <<= 16;
            model_id |= pbind->head[i].vendor;
        }
        else
        {
            MI_LOG_ERROR("mi_appkey_bind: invalid model id(0x%04x-0x%04x-%d)", pbind->head[i].model,
                         pbind->head[i].vendor, pbind->head[i].elem_idx);
            continue;
        }
        MI_LOG_DEBUG("mi_appkey_bind: bind model(0x%08x-%d)", model_id, pbind->head[i].elem_idx);
        pmodel = mesh_model_get_by_model_id(pelement, model_id);
        if (NULL == pmodel)
        {
            MI_LOG_ERROR("mi_appkey_bind: invalid model(0x%08x-%d)", model_id, pbind->head[i].elem_idx);
            continue;
        }

        app_key_index = app_key_index_from_global(pbind->head[i].appkey_idx);
        if (mesh_node.app_key_num == app_key_index)
        {
            MI_LOG_ERROR("mi_app_bind: invalid appkey index(%d-%d)", pbind->head[i].appkey_idx,
                         pbind->head[i].elem_idx);
            continue;
        }

        if (!plt_bit_pool_get(pmodel->app_key_binding, app_key_index))
        {
            plt_bit_pool_set(pmodel->app_key_binding, app_key_index, TRUE);
            mesh_flash_store(MESH_FLASH_PARAMS_MODEL_APP_KEY_BINDING, pmodel);
        }

        /* set model publish parameters */
#if !MI_PUB_NEW_STRATEGY
        mi_model_publish_set(pmodel, MI_PUB_ADDRESS, pbind->head[i].appkey_idx);
//      mesh_flash_store(MESH_FLASH_PARAMS_MODEL_PUB_PARAMS, pmodel);
        /* start publish */
        mesh_model_pub_start(pmodel);
#endif
    }
}

/**
 * @brief switch beacon when provision successed
 */
static void beacon_switch(void)
{
    /** switch beacon & service adv */
    if (mesh_node.features.snb)
    {
        beacon_start();
    }
    else if (mesh_node.features.udb)
    {
        beacon_stop();
    }

    if (mesh_node.features.proxy == 1)
    {
        mesh_service_adv_start();
    }
    else if (mesh_node.features.prov == 1)
    {
        mesh_service_adv_stop();
    }
}

void mi_inner_msg_handle(uint8_t event)
{
    if (event != mi_event)
    {
        MI_LOG_ERROR("mi_inner_msg_handle: fail, event(%d) is not mesh event(%d)!", event, mi_event);
        return;
    }

    mi_inner_msg_t inner_msg, *pmsg;
    if (FALSE == plt_os_queue_receive(mi_queue_handle, &inner_msg, 0))
    {
        MI_LOG_ERROR("mi_inner_msg_handle: fail to receive msg!");
        return;
    }

    pmsg = &inner_msg;
    switch (pmsg->type)
    {
    case MI_BEACON_TIMEOUT:
        mible_gap_adv_send();
        break;
    case MI_UDB_PERIOD_TIMEOUT:
        if (mesh_node.node_state == UNPROV_DEVICE)
        {
            mibeacon_adv_stop();
        }
        break;
    case MI_SCHD_TIMEOUT:
        mi_schd_process();
        break;
#if MI_PUB_NEW_STRATEGY
    case MI_PUB_TIMEOUT:
        mi_process_publish_timeout();
        break;
    case MI_PUB_SINGLE_TIMEOUT:
        mi_process_publish_single_timeout(pmsg->pbuf);
        break;
#endif
    default:
        MI_LOG_WARNING("mi_inner_msg_handle: fail, mi queue message unknown type: %d!", pmsg->type);
        break;
    }
}

bool mi_inner_msg_send(mi_inner_msg_t *pmsg)
{
    static uint32_t error_num = 0;

    if (!plt_os_queue_send(mi_queue_handle, pmsg, 0))
    {
        if (++error_num % 20 == 0)
        {
            MI_LOG_ERROR("failed to send msg to mi msg queue");
        }
        return FALSE;
    }

    /* send event to notify app task */
    if (!plt_os_queue_send(mi_event_queue_handle, &mi_event, 0))
    {
        if (++error_num % 20 == 0)
        {
            MI_LOG_ERROR("failed to send msg to mi event queue");
        }
        return FALSE;
    }

    return TRUE;
}

#if MI_ENABLE_STOP_UDB
/**
 * @brief stop udb timeout callback
 * @param[in] timer: timer handle
 */
static void mi_udb_monitor_timeout_cb(plt_timer_t timer)
{
    mi_inner_msg_t msg;
    msg.type = MI_UDB_PERIOD_TIMEOUT;
    mi_inner_msg_send(&msg);
}

/**
 * @biref start udb monitor timer
 * @param[in] interval: monitor time, unit is ms
 */
static void mi_udb_monitor_start(uint32_t interval)
{
    if (NULL == mi_udb_period_timer)
    {
        mible_status_t status = mible_timer_create(&mi_udb_period_timer, mi_udb_monitor_timeout_cb,
                                                   MIBLE_TIMER_SINGLE_SHOT);

        if (MI_SUCCESS != status)
        {
            MI_LOG_ERROR("mi_udb_monitor_start: fail, timer is not created");
            return ;
        }
    }

    mible_timer_start(mi_udb_period_timer, interval, NULL);
}

/**
 * @brief stop udb monitor timer
 */
static void mi_udb_monitor_stop(void)
{
    if (NULL != mi_udb_period_timer)
    {
        mible_timer_delete(mi_udb_period_timer);
        mi_udb_period_timer = NULL;
    }
}
#endif

void mi_mesh_start(uint8_t event_mi, void *event_queue)
{
    mi_event = event_mi;
    mi_event_queue_handle = event_queue;
    mi_queue_handle = plt_os_queue_create(MI_INNER_MSG_NUM, sizeof(mi_inner_msg_t));
}

void mi_handle_gap_msg(T_IO_MSG *pmsg)
{
    T_LE_GAP_MSG gap_msg;
    memcpy(&gap_msg, &pmsg->u.param, sizeof(pmsg->u.param));

    switch (pmsg->subtype)
    {
    case GAP_MSG_LE_DEV_STATE_CHANGE:
        mi_mesh_run();
        break;
    case GAP_MSG_LE_CONN_STATE_CHANGE:
        {
            T_GAP_CONN_STATE conn_state = (T_GAP_CONN_STATE)gap_msg.msg_data.gap_conn_state_change.new_state;
            uint8_t conn_id = gap_msg.msg_data.gap_conn_state_change.conn_id;
            MI_LOG_DEBUG("mible_handle_gap_msg: conn_id = %d, new_state = %d, cause = %d",
                         conn_id, conn_state, gap_msg.msg_data.gap_conn_state_change.disc_cause);
            if (conn_id >= GAP_SCHED_LE_LINK_NUM)
            {
                MI_LOG_WARNING("mible_handle_gap_msg: exceed the maximum supported link num %d",
                               GAP_SCHED_LE_LINK_NUM);
                break;
            }

            mible_gap_evt_param_t param;
            memset(&param, 0, sizeof(param));
            param.conn_handle = conn_id;
            if (conn_state == GAP_CONN_STATE_CONNECTED)
            {
                mi_gatts_resume();
                T_GAP_CONN_INFO conn_info;
                le_get_conn_info(conn_id, &conn_info);
                memcpy(param.connect.peer_addr, conn_info.remote_bd, GAP_BD_ADDR_LEN);
                param.connect.type = (mible_addr_type_t)conn_info.remote_bd_type;
                param.connect.role = conn_info.role == GAP_LINK_ROLE_MASTER ? MIBLE_GAP_PERIPHERAL :
                                     MIBLE_GAP_CENTRAL;
                le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &param.connect.conn_param.min_conn_interval, conn_id);
                le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &param.connect.conn_param.max_conn_interval, conn_id);
                le_get_conn_param(GAP_PARAM_CONN_LATENCY, &param.connect.conn_param.slave_latency, conn_id);
                le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &param.connect.conn_param.conn_sup_timeout, conn_id);

                mible_gap_event_callback(MIBLE_GAP_EVT_CONNECTED, &param);
            }
            else if (conn_state == GAP_CONN_STATE_DISCONNECTED)
            {
                uint16_t disc_cause = gap_msg.msg_data.gap_conn_state_change.disc_cause;
                if (disc_cause == (HCI_ERR | HCI_ERR_CONN_TIMEOUT))
                {
                    param.disconnect.reason = CONNECTION_TIMEOUT;
                }
                else if (disc_cause == (HCI_ERR | HCI_ERR_REMOTE_USER_TERMINATE))
                {
                    param.disconnect.reason = REMOTE_USER_TERMINATED;
                }
                else if (disc_cause == (HCI_ERR | HCI_ERR_LOCAL_HOST_TERMINATE))
                {
                    param.disconnect.reason = LOCAL_HOST_TERMINATED;
                }

                mible_gap_event_callback(MIBLE_GAP_EVT_DISCONNECT, &param);
            }
        }
        break;
    case GAP_MSG_LE_CONN_PARAM_UPDATE:
        if (GAP_CONN_PARAM_UPDATE_STATUS_SUCCESS == gap_msg.msg_data.gap_conn_param_update.status)
        {
            uint8_t conn_id = gap_msg.msg_data.gap_conn_param_update.conn_id;
            mible_gap_evt_param_t param;
            memset(&param, 0, sizeof(param));
            param.conn_handle = conn_id;

            le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &param.update_conn.conn_param.min_conn_interval,
                              conn_id);
            le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &param.update_conn.conn_param.max_conn_interval,
                              conn_id);
            le_get_conn_param(GAP_PARAM_CONN_LATENCY, &param.update_conn.conn_param.slave_latency, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &param.update_conn.conn_param.conn_sup_timeout, conn_id);
            mible_gap_event_callback(MIBLE_GAP_EVT_CONN_PARAM_UPDATED, &param);
        }
        break;
    default:
        break;
    }
}

/**
  * @brief  Initialize rtc peripheral.
  * @param   No parameter.
  * @return  void
  */
void driver_rtc_init(void)
{
    RTC_DeInit();
    RTC_SetPrescaler(RTC_PRESCALER_VALUE);

    RTC_SetComp(RTC_COMP_INDEX, RTC_COMP_VALUE);
    RTC_MaskINTConfig(RTC_INT_CMP_NUM, DISABLE);
    RTC_CompINTConfig(RTC_INT_CMP_NUM, ENABLE);

    /* Config RTC interrupt */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    RTC_RunCmd(ENABLE);
}

void RTC_Handler(void)
{
    if (RTC_GetINTStatus(RTC_INT_CMP_NUM) == SET)
    {
        /* Notes: DBG_DIRECT function is only used for debugging demonstrations, not for application projects.*/
        mi_inner_msg_t msg;
        msg.type = MI_SCHD_TIMEOUT;
        mi_inner_msg_send(&msg);
        //DBG_DIRECT("[main]RTC_Handler: RTC counter current value = %d", RTC_GetCounter());
        RTC_SetComp(RTC_COMP_INDEX, (RTC_GetCounter() + RTC_COMP_VALUE) & 0xffffff);
        RTC_ClearCompINT(RTC_COMP_INDEX);
    }
}

/**
 * @brief initialize mi gatt process task
 */
void mi_gatts_init(void)
{
    driver_rtc_init();
}

void mi_gatts_suspend(void)
{
    RTC_RunCmd(DISABLE);
}

void mi_gatts_resume(void)
{
    RTC_RunCmd(ENABLE);
}

/**
 * @brief mi scheduler event handler
 * @param[in] p_event: event need to process
 */
static void mi_schd_event_handler(schd_evt_t *p_event)
{
    switch (p_event->id)
    {
    case SCHD_EVT_REG_SUCCESS:
        break;
    case SCHD_EVT_REG_FAILED:
        break;
    case SCHD_EVT_ADMIN_LOGIN_SUCCESS:
        break;
    case SCHD_EVT_ADMIN_LOGIN_FAILED:
        break;
    case SCHD_EVT_SHARE_LOGIN_SUCCESS:
        break;
    case SCHD_EVT_SHARE_LOGIN_FAILED:
        break;
    case SCHD_EVT_TIMEOUT:
        break;
    case SCHD_EVT_KEY_NOT_FOUND:
        break;
    case SCHD_EVT_KEY_FOUND:
        break;
    case SCHD_EVT_KEY_DEL_FAIL:
        break;
    case SCHD_EVT_KEY_DEL_SUCC:
        break;
    case SCHD_EVT_MESH_REG_SUCCESS:
        {
            provisionin_data_t *pprov_data = p_event->data.mesh_config.p_prov_data;
            /* clear the flash first */
            mesh_node_clear();

            /* store device key */
            mesh_node.dev_key_list[0].used = 1;
            mesh_node.dev_key_list[0].element_num = mesh_node.element_queue.count;
            mesh_node.dev_key_list[0].unicast_addr = pprov_data->address;
            memcpy(mesh_node.dev_key_list[0].dev_key, p_event->data.mesh_config.p_devkey, MESH_COMMON_KEY_SIZE);
            uint16_t dev_key_index = 0;
            mesh_flash_store(MESH_FLASH_PARAMS_DEV_KEY, &dev_key_index);

            /* store netkey */
            uint16_t net_key_index = 0;
            if (pprov_data->flags & 0x01)
            {
                uint8_t key[MESH_COMMON_KEY_SIZE] = {0};
                net_key_update(net_key_index, pprov_data->net_idx, key);
                net_key_update(net_key_index, pprov_data->net_idx, pprov_data->netkey);
                net_key_refresh(net_key_index);
            }
            else
            {
                net_key_update(net_key_index, pprov_data->net_idx, pprov_data->netkey);
            }
            mesh_flash_store(MESH_FLASH_PARAMS_NET_KEY, &net_key_index);
            iv_index_set(pprov_data->iv);
            mesh_flash_store(MESH_FLASH_PARAMS_IV_INDEX, NULL);
            mesh_node.iv_update_flag = (pprov_data->flags & 0x02) >> 1;
            mesh_node.iv_timer_count = MESH_IV_INDEX_48W; //!< should stay (MESH_IV_INDEX_48W)
            mesh_seq_clear();
            mesh_node.unicast_addr = pprov_data->address;
            mesh_node.node_state = PROV_NODE;
            /* store the node state finally */
            mesh_flash_store(MESH_FLASH_PARAMS_NODE_INFO, NULL);

            /* switch beacon */
            beacon_switch();

            /* bind keys */
            mi_appkey_add(p_event->data.mesh_config.p_appkey_list);
            mi_appkey_bind(p_event->data.mesh_config.p_bind_list);

            /* set mesh beacon avaliable */
            mibeacon_adv_data_set(0, MI_MESH_STATE_AVIAL, 0, NULL);

            /* light effect */
            //light_prov_complete();
#if MI_ENABLE_STOP_UDB
            mi_udb_monitor_stop();
#endif
            /* change beacon interval */
            mibeacon_adv_start(MI_BEACON_INTERVAL_PROVED);

#if MI_PUB_NEW_STRATEGY
            mi_publish_start();
            miot_server_net_param_req_start(MI_PUB_ADDRESS, 0);
#endif
            prov_cb_data_t droppable_data;
            droppable_data.pb_generic_cb_type = PB_GENERIC_CB_MSG;
            prov_cb(PROV_CB_TYPE_COMPLETE, droppable_data);
        }
        break;
    case SCHD_EVT_MESH_REG_FAILED:
        mibeacon_adv_data_set(0, MI_MESH_STATE_UNAUTH, 0, NULL);
        break;
    case SCHD_EVT_OOB_REQUEST:
        break;
    default:
        break;
    }
}

void mi_unprov(void)
{
    /* clear auth info */
    if (mible_start)
    {
        mi_scheduler_start(SYS_KEY_DELETE);
    }
    /* set mesh beacon unavaliable */
    mibeacon_adv_data_set(0, MI_MESH_STATE_UNAUTH, 0, NULL);

    /* change beacon interval */
    mibeacon_adv_start(MI_BEACON_INTERVAL_UNPROV);
#if MI_ENABLE_STOP_UDB
    mi_udb_monitor_start(MI_STOP_UDB_PERIOD);
#endif

#if MI_PUB_NEW_STRATEGY
    mi_publish_stop();
    miot_server_net_param_req_stop();
#endif
}

void mi_mesh_run(void)
{
    if (!mible_start)
    {
        T_GAP_VENDOR_PRIORITY_PARAM pri_param;
        memset(&pri_param, 0, sizeof(T_GAP_VENDOR_PRIORITY_PARAM));
        pri_param.set_priority_mode = GAP_VENDOR_SET_PRIORITY;
        pri_param.link_priority_mode = GAP_VENDOR_SET_ALL_LINK_PRIORITY;
        pri_param.link_priority_level = GAP_VENDOR_PRIORITY_LEVEL_10;
        pri_param.scan_priority.set_priority_flag = true;
        pri_param.scan_priority.priority_level = GAP_VENDOR_PRIORITY_LEVEL_0;
        pri_param.adv_priority.set_priority_flag = true;
        pri_param.adv_priority.priority_level = GAP_VENDOR_PRIORITY_LEVEL_0;
        pri_param.initiate_priority.set_priority_flag = false;
        pri_param.initiate_priority.priority_level = GAP_VENDOR_RESERVED_PRIORITY;
        le_vendor_set_priority(&pri_param);

        mible_start = TRUE;
        mible_record_init();
        mi_scheduler_init(MI_SCHEDULER_INTERVAL, mi_schd_event_handler, NULL);

        mi_gatts_init();
        /* get device mac */
        mible_gap_address_get(dev_mac);

#if MI_PUB_NEW_STRATEGY
        mi_publish_init();
#endif

        if (mesh_node.node_state == UNPROV_DEVICE)
        {
            mibeacon_adv_data_set(0, MI_MESH_STATE_UNAUTH, 0, NULL);
            /* delete key if unprov node */
            /* TODO: do not do this on every startup sequence */
            mi_scheduler_start(SYS_KEY_DELETE);
            mibeacon_adv_start(MI_BEACON_INTERVAL_UNPROV);
#if MI_ENABLE_STOP_UDB
            mi_udb_monitor_start(MI_STOP_UDB_PERIOD);
#endif
        }
        else
        {
            mibeacon_adv_data_set(0, MI_MESH_STATE_AVIAL, 0, NULL);
            mi_scheduler_start(SYS_KEY_RESTORE);
            mibeacon_adv_start(MI_BEACON_INTERVAL_PROVED);

#if MI_PUB_NEW_STRATEGY
            mi_publish_start();
            miot_server_net_param_req_start(MI_PUB_ADDRESS, 0);
#else
            mi_model_publish_start();
#endif
        }
    }
}

void mi_default_config(void)
{
    mesh_node.ttl = MI_DEFAULT_TTL;
    mesh_node.net_trans_count = MI_NET_RETRANS_COUNT;
    mesh_node.net_trans_steps = MI_NET_RETRANS_INTERVAL_STEPS;
    mesh_node.relay_retrans_count = MI_RELAY_RETRANS_COUNT;
    mesh_node.relay_retrans_steps = MI_RELAY_RETRANS_INTERVAL_STEPS;
    mesh_node.nmc_size = MI_NMC_SIZE;
    mesh_node.seq_siv = MI_IV_UPDATE_TRIGGER_SEQUENCE_NUM;
    mesh_node.trans_retrans_base = MI_TIME_RECV_SEG_ACK;
    mesh_node.trans_retrans_seg_factor = 0;
    mesh_node.trans_ack_base = MI_TIME_SEND_SEG_ACK;
    mesh_node.trans_ack_seg_factor = 0;
    mesh_node.relay_parallel_max = MI_GAP_SCHED_RELAY_PARALLEL_MAX_NUM;
    uint8_t task_num = MI_GAP_SCHED_TASK_NUM;
    uint16_t scan_window = MI_GAP_SCHED_INTERWAVE_SCAN_WINDOW;
    uint16_t scan_interval = MI_GAP_SCHED_INTERWAVE_SCAN_INTERVAL;
    gap_sched_params_set(GAP_SCHED_PARAMS_INTERWAVE_SCAN_WINDOW, &scan_window, sizeof(scan_window));
    gap_sched_params_set(GAP_SCHED_PARAMS_INTERWAVE_SCAN_INTERVAL, &scan_interval,
                         sizeof(scan_interval));
    scan_window = MI_GAP_SCHED_SCAN_WINDOW;
    scan_interval = MI_GAP_SCHED_SCAN_INTERVAL;
    gap_sched_params_set(GAP_SCHED_PARAMS_SCAN_WINDOW, &scan_window, sizeof(scan_window));
    gap_sched_params_set(GAP_SCHED_PARAMS_SCAN_INTERVAL, &scan_interval,
                         sizeof(scan_interval));
    gap_sched_params_set(GAP_SCHED_PARAMS_TASK_NUM, &task_num, sizeof(task_num));
}

void rtk_gap_adv_timeout(void)
{
    mi_inner_msg_t msg;
    msg.type = MI_BEACON_TIMEOUT;
    mi_inner_msg_send(&msg);
}

void mi_startup_delay(void)
{
    int delay = rand();
    uint32_t real_delay = delay;
    real_delay %= 2000;
    DBG_DIRECT("startup delay: %d ms", real_delay);
    plt_delay_ms(real_delay);
#if (ROM_WATCH_DOG_ENABLE == 1)
    WDG_Restart();
#endif
}
