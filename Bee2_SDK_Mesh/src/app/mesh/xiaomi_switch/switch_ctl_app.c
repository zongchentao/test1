/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      switch_ctl_app.c
* @brief     Smart mesh demo switch ctl application
* @details
* @author    astor zhang
* @date      2019-6-14
* @version   v1.0
* *********************************************************************************************************
*/

#include "mesh_api.h"
#include "light_ctl_server_app.h"
#include "generic_on_off.h"
#include "light_lightness.h"
#include "light_ctl.h"
#include "light_cwrgb_app.h"
#include "light_storage_app.h"
#include "light_controller_app.h"
#include "generic_transition_time.h"
#include "rtl876x_gpio.h"
#include "switch_ctl_app.h"
#include "os_timer.h"
#include "miot_model.h"
#include "miot_model_server_app.h"
#include "mijia_model.h"
#include "configuration.h"

#include "mijia_mesh_config.h"
#include "mijia_mesh_publish.h"

/* switch subscribe address */
//TODO: change the address for light to that for switch
static const uint16_t switch_ctl_sub_addr[] = {0xFE01};
void *xTimerPubDelay;

/* ctl switch models */
static mesh_model_info_t generic_on_off_server_1st;
static mesh_model_info_t generic_on_off_server_2nd;
static mesh_model_info_t mijia_server;

static generic_on_off_t switch_1st_on_off = GENERIC_OFF;
static generic_on_off_t switch_2nd_on_off = GENERIC_OFF;

static uint8_t current_target = 0;
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
            pdata->on_off = GENERIC_OFF;
            if ((light_get_red()->lightness) &&
                pmodel_info == &generic_on_off_server_1st)
            {
                pdata->on_off = GENERIC_ON;
            }
            else if ((light_get_green()->lightness) &&
                     pmodel_info == &generic_on_off_server_2nd)
            {
                pdata->on_off = GENERIC_ON;
            }
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
                    if (pmodel_info == &generic_on_off_server_1st)
                    {
                        light_lighten_red(8192);
                        GPIO_ResetBits(GPIO_GetPin(P2_3));
                        data_uart_debug("turn on switch 1\r\n");
                        switch_1st_on_off = GENERIC_ON;
                        //TODO: turn on switch channel 1 here

                    }
                    else if (pmodel_info == &generic_on_off_server_2nd)
                    {
                        light_lighten_green(8192);
                        GPIO_ResetBits(GPIO_GetPin(P2_4));
                        data_uart_debug("turn on switch 2\r\n");
                        switch_2nd_on_off = GENERIC_ON;
                        //TODO: turn on switch channel 2 here
                    }
                }
                else
                {
                    if (pmodel_info == &generic_on_off_server_1st)
                    {
                        light_lighten_red(0);
                        GPIO_SetBits(GPIO_GetPin(P2_3));
                        data_uart_debug("turn off switch 1\r\n");
                        switch_1st_on_off = GENERIC_OFF;
                        //TODO: turn off switch channel 1 here
                    }
                    else if (pmodel_info == &generic_on_off_server_2nd)
                    {
                        light_lighten_green(0);
                        GPIO_SetBits(GPIO_GetPin(P2_4));
                        data_uart_debug("turn off switch 2\r\n");
                        switch_2nd_on_off = GENERIC_OFF;
                        //TODO: turn off switch channel 2 here
                    }
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

/**
 * @brief mijia server data callback
 * @param[in] pmodel_info: light lightness server model handler
 * @param[in] type: data type
 * @param[in, out] pargs: data need to process
 */
static int32_t mijia_server_data(const mesh_model_info_p pmodel_info, uint32_t type,
                                 void *pargs)
{
    return 0;
}

static bool cfg_server_receive_peek(mesh_msg_p pmesh_msg)
{
    bool ret = cfg_server_receive(pmesh_msg);
    if (ret)
    {
        switch (pmesh_msg->access_opcode)
        {
        case MESH_MSG_CFG_MODEL_SUB_OVERWRITE:
            {
                cfg_model_sub_add_t *pmsg = (cfg_model_sub_add_t *)(pmesh_msg->pbuffer + pmesh_msg->msg_offset);
                uint16_t primary_sub_addr = 0;
                mesh_model_t *pmodel = NULL;
                uint16_t sub_addr = 0;
                if (pmsg->element_addr == mesh_node.unicast_addr)
                {
                    pmodel = generic_on_off_server_2nd.pmodel;
                    sub_addr = pmsg->addr + 1;
                    primary_sub_addr = pmsg->addr;
                }
                else
                {
                    pmodel = generic_on_off_server_1st.pmodel;
                    sub_addr = pmsg->addr - 1;
                    primary_sub_addr = pmsg->addr - 1;
                }

                /* send vendor message */
                for (uint8_t i = 0; i < 2; ++i)
                {
                    miot_sub_status(&miot_server, pmesh_msg->src, 0, primary_sub_addr, 0);
                }

                /* update subscribe list */
                mesh_model_sub(pmodel, sub_addr);
                mesh_flash_store(MESH_FLASH_PARAMS_MODEL_SUBSCRIBE_ADDR, pmodel);
                switch_ctl_server_models_sub();
            }
            break;
        }
    }

    return ret;
}

void switch_ctl_server_models_sub(void)
{
    for (uint8_t i = 0; i < sizeof(switch_ctl_sub_addr) / sizeof(uint16_t); ++i)
    {
        mesh_model_sub(generic_on_off_server_1st.pmodel, switch_ctl_sub_addr[i]);
        mesh_model_sub(generic_on_off_server_2nd.pmodel, switch_ctl_sub_addr[i]);
    }
}

void switch_ctl_server_models_init(void)
{
    /* binding models */
    miot_server.pmodel_bound = &generic_on_off_server_1st;
    mijia_server.pmodel_bound = &miot_server;

    /* register miot model */
    miot_server.model_data_cb = miot_server_data;
    miot_server_reg(0, &miot_server);

    /* register mijia model */
    mijia_server.model_data_cb = mijia_server_data;
    mijia_server_reg(0, &mijia_server);

    /* register switch ctl models */
    generic_on_off_server_1st.model_data_cb = generic_on_off_server_data;
    generic_on_off_server_reg(0, &generic_on_off_server_1st);

    generic_on_off_server_2nd.model_data_cb = generic_on_off_server_data;
    generic_on_off_server_reg(1, &generic_on_off_server_2nd);

    cfg_server.model_receive = cfg_server_receive_peek;

    /* publish test */
#if 0
    mesh_model_pub_params_t pub_params;
    pub_params.pub_addr = 0xFFFE;
    pub_params.pub_key_info.app_key_index = 0;
    pub_params.pub_key_info.frnd_flag = 0;
    pub_params.pub_key_info.rfu = 0;
    pub_params.pub_ttl = 5;
    mesh_model_pub_params_set(generic_on_off_server_1st.pmodel, pub_params);
    mesh_model_pub_params_set(generic_on_off_server_2nd.pmodel, pub_params);
#endif
}

void handle_demo_switch_revert(uint8_t num, bool re_pub)
{
    current_target = num;
    if (re_pub)
    {
        if (num == 0)
        {
            generic_on_off_publish(&generic_on_off_server_1st, switch_1st_on_off);
//            DBG_DIRECT("Re-publish**************switch %d, state %d", current_target, switch_1st_on_off);
        }
        else
        {
            generic_on_off_publish(&generic_on_off_server_2nd, switch_2nd_on_off);
//            DBG_DIRECT("Re-publish**************switch %d, state %d", current_target, switch_2nd_on_off);
        }
    }
    else
    {
        if (num == 0)
        {
            if (switch_1st_on_off == GENERIC_OFF)
            {
                switch_1st_on_off = GENERIC_ON;
                light_lighten_red(8192);
                generic_on_off_publish(&generic_on_off_server_1st, GENERIC_ON);
            }
            else
            {
                switch_1st_on_off = GENERIC_OFF;
                light_lighten_red(0);
                generic_on_off_publish(&generic_on_off_server_1st, GENERIC_OFF);
            }
        }
        else
        {
            if (switch_2nd_on_off == GENERIC_OFF)
            {
                switch_2nd_on_off = GENERIC_ON;
                light_lighten_green(8192);
                generic_on_off_publish(&generic_on_off_server_2nd, GENERIC_ON);
            }
            else
            {
                switch_2nd_on_off = GENERIC_OFF;
                light_lighten_green(0);
                generic_on_off_publish(&generic_on_off_server_2nd, GENERIC_OFF);
            }
        }
        os_timer_restart(&xTimerPubDelay, 300);
    }
}


void vTimerPubCallback(void *pxTimer)
{
    handle_demo_switch_revert(current_target, true);
}


void sw_timer_init(void)
{
    bool ret = os_timer_create(&xTimerPubDelay, "xTimerPubDelay", 0, 300, false, vTimerPubCallback);
}
