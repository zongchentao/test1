/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      single_fire_switch_ctl_server_app.c
* @brief     Smart mesh demo single fire switch ctl application
* @details
* @author    astor
* @date      2019-10-17
* @version   v1.0
* *********************************************************************************************************
*/

#include "mesh_api.h"
#include "single_fire_switch_ctl_server_app.h"
#include "generic_on_off.h"
#include "light_lightness.h"
#include "light_ctl.h"
#include "light_cwrgb_app.h"
#include "light_storage_app.h"
#include "light_controller_app.h"
#include "generic_transition_time.h"
#include "app_task.h"
#include "light_sw_timer.h"
#include "miot_model.h"
#include "miot_model_server_app.h"
#include "mijia_model.h"
#include "common/mible_beacon.h"
#include "mesh_auth/mible_mesh_auth.h"
#include "mijia_mesh_app.h"
#include "mible_api.h"
#include "single_fire_switch_config.h"

#include "mijia_mesh_config.h"
#include "mijia_mesh_publish.h"

/* external functions */
extern mible_status_t mible_gap_adv_send(void);

/* switch subscribe address */
static const uint16_t ctl_sub_addr[] = {0xFE01};

/* models */
mesh_model_info_t generic_on_off_server;
static mesh_model_info_t mijia_server;

/* state */
generic_on_off_t switch_state = GENERIC_OFF;

static plt_timer_t switch_scan_timer = NULL;
static plt_timer_t switch_beacon_timer = NULL;

#define SWITCH_SCAN_STATE_START           0
#define SWITCH_SCAN_STATE_STOP            1

static uint8_t switch_scan_state = SWITCH_SCAN_STATE_START;
static uint8_t mi_beacon_times = 0;

/**
 * @brief generic on/off server data callback
 * @param[in] pmodel_info: generic on/off server model handler
 * @param[in] type: data type
 * @param[in, out] pargs: data need to process
 */
static int32_t generic_on_off_server_data(const mesh_model_info_p pmodel_info, uint32_t type,
                                          void *pargs)
{
    switch (type)
    {
    case GENERIC_ON_OFF_SERVER_GET:
        {
            generic_on_off_server_get_t *pdata = pargs;
            pdata->on_off = switch_state;
        }
        break;
    case GENERIC_ON_OFF_SERVER_GET_DEFAULT_TRANSITION_TIME:
        {
        }
        break;
    case GENERIC_ON_OFF_SERVER_SET:
        {
            generic_on_off_server_set_t *pdata = pargs;
            if (pdata->total_time.num_steps == pdata->remaining_time.num_steps)
            {
                if (GENERIC_ON == pdata->on_off)
                {
                    printi("received message turn on");
                    switch_state = GENERIC_ON;
                }
                else
                {
                    printi("received message turn off");
                    switch_state = GENERIC_OFF;
                }
            }
#if MI_PUB_NEW_STRATEGY
            mi_publish_single_start(pmodel_info);
#endif
        }
        break;
    default:
        break;
    }

    return 0;
}

void switch_scan_timeout_handle(void)
{
    switch (switch_scan_state)
    {
    case SWITCH_SCAN_STATE_START:
        gap_sched_scan(TRUE);
        plt_timer_change_period(switch_scan_timer, SWITCH_SCAN_PERIOD, 0);
        switch_scan_state = SWITCH_SCAN_STATE_STOP;

        mibeacon_adv_data_set(0, MI_MESH_STATE_AVIAL, 0, NULL);
        mibeacon_adv_start(SWITCH_BEACON_INTERVAL);
        mibeacon_adv_stop();
        mible_gap_adv_send();
        mi_beacon_times = 1;
        plt_timer_start(switch_beacon_timer, 0);
        break;
    case SWITCH_SCAN_STATE_STOP:
        gap_sched_scan(FALSE);
        plt_timer_change_period(switch_scan_timer, SWITCH_SCAN_INTERVAL, 0);
        switch_scan_state = SWITCH_SCAN_STATE_START;
        break;
    default:
        break;
    }
}

static void switch_scan_timeout(void *pargs)
{
    T_IO_MSG switch_msg;
    switch_msg.type = IO_MSG_TYPE_TIMER;
    switch_msg.subtype = MIOT_SWITCH_SCAN_TIMEOUT;
    app_send_msg_to_apptask(&switch_msg);
}

void switch_beacon_timeout_handle(void)
{
    /*
    if (0 == mi_beacon_times)
    {
        mibeacon_adv_data_set(0, MI_MESH_STATE_AVIAL, 0, NULL);
        mibeacon_adv_start(SWITCH_BEACON_INTERVAL);
        mibeacon_adv_stop();
    }
    */

    if (mi_beacon_times < SWITCH_BEACON_TIMES)
    {
        mible_gap_adv_send();
        mi_beacon_times ++;
    }
    else
    {
        plt_timer_stop(switch_beacon_timer, 0);
        mi_beacon_times = 0;
    }
}

static void switch_beacon_timeout(void *pargs)
{
    T_IO_MSG switch_msg;
    switch_msg.type = IO_MSG_TYPE_TIMER;
    switch_msg.subtype = MIOT_SWITCH_BEACON_TIMEOUT;
    app_send_msg_to_apptask(&switch_msg);
}

bool switch_timer_start(void)
{
    gap_sched_scan(FALSE);
    if (NULL == switch_scan_timer)
    {
        switch_scan_timer = plt_timer_create("scan", SWITCH_SCAN_INTERVAL, FALSE, 0,
                                             switch_scan_timeout);
        if (NULL == switch_scan_timer)
        {
            printe("create scan timer failed!");
            return FALSE;
        }

        switch_beacon_timer = plt_timer_create("beacon", SWITCH_BEACON_INTERVAL, TRUE, 0,
                                               switch_beacon_timeout);
        if (NULL == switch_beacon_timer)
        {
            printe("create beacon timer failed!");
            plt_timer_delete(switch_scan_timer, 0);
            switch_scan_timer = NULL;
            return FALSE;
        }
        plt_timer_start(switch_scan_timer, 0);
    }
    else
    {
        if (plt_timer_is_active(switch_beacon_timer))
        {
            plt_timer_stop(switch_beacon_timer, 0);
        }
        plt_timer_change_period(switch_scan_timer, SWITCH_SCAN_INTERVAL, 0);
    }
    switch_scan_state = SWITCH_SCAN_STATE_START;
    mi_beacon_times = 0;

    return TRUE;
}

void switch_timer_stop(void)
{
    if (NULL != switch_scan_timer)
    {
        plt_timer_delete(switch_scan_timer, 0);
        switch_scan_timer = NULL;
    }
    if (NULL != switch_beacon_timer)
    {
        plt_timer_delete(switch_beacon_timer, 0);
        switch_beacon_timer = NULL;
    }
    switch_scan_state = SWITCH_SCAN_STATE_START;
    mi_beacon_times = 0;
}

void switch_periodic_pub_stop(void)
{
    mesh_element_t *pelement = (mesh_element_t *)mesh_node.element_queue.pfirst;
    while (NULL != pelement)
    {
        mesh_model_t *pmodel = (mesh_model_t *)pelement->model_queue.pfirst;
        while (NULL != pmodel)
        {
            if (NULL != pmodel->pub_timer)
            {
                plt_timer_delete(pmodel->pub_timer, 0);
                pmodel->pub_timer = NULL;
            }
            pmodel = pmodel->pnext;
        }
        pelement = pelement->pnext;
    }
}

void ctl_server_models_sub(void)
{
    for (uint8_t i = 0; i < sizeof(ctl_sub_addr) / sizeof(uint16_t); ++i)
    {
        mesh_model_sub(generic_on_off_server.pmodel, ctl_sub_addr[i]);
    }
}


static int32_t server_data(const mesh_model_info_p pmodel_info, uint32_t type,
                           void *pargs)
{
    return 0;
}

void ctl_server_models_init(void)
{
    /* binding models */

    /* register generic on/off models */
    miot_server.pmodel_bound = &generic_on_off_server;
    mijia_server.pmodel_bound = &miot_server;

    generic_on_off_server.model_data_cb = generic_on_off_server_data;
    generic_on_off_server_reg(0, &generic_on_off_server);

    /* register miot model */
    miot_server.model_data_cb = server_data;
    miot_server_reg(0, &miot_server);

    mijia_server.model_data_cb = server_data;
    mijia_server_reg(0, &mijia_server);
}

