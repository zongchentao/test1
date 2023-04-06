/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      light_ctl_server_app.c
* @brief     Smart mesh demo light ctl application
* @details
* @author    hector_huang
* @date      2018-8-16
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
#include "miot_model.h"
#include "miot_model_server_app.h"
#include "configuration.h"
#include "mijia_model.h"
#include "mijia_mesh_config.h"
#include "mijia_mesh_publish.h"

/* light subscribe address */
static const uint16_t light_ctl_sub_addr[] = {0xFE00};

/* light ctl value */
static int16_t ctl_delta_uv;

/* ctl light models */
mesh_model_info_t generic_on_off_server;
static mesh_model_info_t light_lightness_server;
static mesh_model_info_t light_ctl_temperature_server;
static mesh_model_info_t mijia_server;

/**
 * @brief light gradual change done callback
 */
static void light_lightness_change_done(light_t *light)
{
    light_state_store();
}

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
            if ((light_get_green()->lightness) ||
                (light_get_blue()->lightness))
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
                uint32_t trans_time = generic_transition_time_convert(pdata->total_time);
                if (trans_time > 0)
                {
                    if (GENERIC_ON == pdata->on_off)
                    {
                        if ((0 == light_get_green()->lightness_last) &&
                            (0 == light_get_blue()->lightness_last))
                        {
                            light_set_lightness_linear(light_get_green(), 65535, trans_time, NULL);
                            light_set_lightness_linear(light_get_blue(), 0x4e20, trans_time, light_lightness_change_done);
                        }
                        else
                        {
                            light_set_lightness_linear(light_get_green(), light_get_green()->lightness_last, trans_time,
                                                       NULL);
                            light_set_lightness_linear(light_get_blue(), light_get_blue()->lightness_last, trans_time,
                                                       NULL);
                        }
                        light_on_state_store(TRUE);
                    }
                    else
                    {
                        light_set_lightness_linear(light_get_green(), 0, trans_time, NULL);
                        light_set_lightness_linear(light_get_blue(), 0, trans_time, NULL);
                        light_on_state_store(FALSE);
                    }
                }
                else
                {
                    if (GENERIC_ON == pdata->on_off)
                    {
                        if ((0 == light_get_green()->lightness_last) &&
                            (0 == light_get_blue()->lightness_last))

                        {
                            light_set_green_lightness(65535);
                            light_set_blue_lightness(0x4e20);
                            light_state_store();
                        }
                        else
                        {
                            light_lighten(light_get_green(), light_get_green()->lightness_last);
                            light_lighten(light_get_blue(), light_get_blue()->lightness_last);
                        }
                        light_on_state_store(TRUE);
                    }
                    else
                    {
                        light_lighten(light_get_green(), 0);
                        light_lighten(light_get_blue(), 0);
                        light_on_state_store(FALSE);
                    }
                }
#if MI_PUB_NEW_STRATEGY
                mi_publish_single_start(pmodel_info);
#endif
            }
        }
        break;
    default:
        break;
    }

    return 0;
}

/**
 * @brief light lightness server data callback
 * @param[in] pmodel_info: light lightness server model handler
 * @param[in] type: data type
 * @param[in, out] pargs: data need to process
 */
static int32_t light_lightness_server_data(const mesh_model_info_p pmodel_info, uint32_t type,
                                           void *pargs)
{
    switch (type)
    {
    case LIGHT_LIGHTNESS_SERVER_GET:
        {
            light_lightness_server_get_t *pdata = pargs;
            pdata->lightness = light_get_green()->lightness;
        }
        break;
    case LIGHT_LIGHTNESS_SERVER_GET_LINEAR:
        {
        }
        break;
    case LIGHT_LIGHTNESS_SERVER_GET_DEFAULT:
        {
        }
        break;
    case LIGHT_LIGHTNESS_SERVER_GET_LAST:
        {
        }
        break;
    case LIGHT_LIGHTNESS_SERVER_GET_RANGE:
        {
        }
        break;
    case LIGHT_LIGHTNESS_SERVER_GET_DEFAULT_TRANSITION_TIME:
        {
        }
        break;
    case LIGHT_LIGHTNESS_SERVER_SET:
        {
            light_lightness_server_set_t *pdata = pargs;
            if (pdata->total_time.num_steps == pdata->remaining_time.num_steps)
            {
                uint32_t trans_time = generic_transition_time_convert(pdata->total_time);
                if (trans_time > 0)
                {
                    light_set_lightness_linear(light_get_green(), pdata->lightness, trans_time,
                                               light_lightness_change_done);
                }
                else
                {
                    light_set_green_lightness(pdata->lightness);
                    light_state_store();
                }
#if MI_PUB_NEW_STRATEGY
                mi_publish_single_start(pmodel_info);
#endif
            }
        }
        break;
    case LIGHT_LIGHTNESS_SERVER_SET_LINEAR:
        {
        }
        break;
    case LIGHT_LIGHTNESS_SERVER_SET_LAST:
        {
        }
        break;
    case LIGHT_LIGHTNESS_SERVER_SET_DEFAULT:
        {
        }
        break;
    case LIGHT_LIGHTNESS_SERVER_SET_RANGE:
        {
        }
        break;
    default:
        break;
    }

    return 0;
}

/**
 * @brief light ctl server data callback
 * @param[in] pmodel_info: light ctl server model handler
 * @param[in] type: data type
 * @param[in, out] pargs: data need to process
 */
static int32_t light_ctl_server_data(const mesh_model_info_p pmodel_info, uint32_t type,
                                     void *pargs)
{
    switch (type)
    {
    case LIGHT_CTL_SERVER_GET:
        {
            light_ctl_server_get_t *pdata = pargs;
            pdata->lightness = light_get_green()->lightness;
            pdata->temperature = lightness_to_temperature(light_get_blue()->lightness);
        }
        break;
    case LIGHT_CTL_SERVER_GET_DEFAULT:
        {
        }
        break;
    case LIGHT_CTL_SERVER_GET_TEMPERATURE:
        {
            light_ctl_server_get_temperature_t *pdata = pargs;
            pdata->temperature = lightness_to_temperature(light_get_blue()->lightness);;
            pdata->delta_uv = ctl_delta_uv;
        }
        break;
    case LIGHT_CTL_SERVER_GET_TEMPERATURE_RANGE:
        {
        }
        break;
    case LIGHT_CTL_SERVER_GET_DEFAULT_TRANSITION_TIME:
        {
        }
        break;
    case LIGHT_CTL_SERVER_SET:
        {
        }
        break;
    case LIGHT_CTL_SERVER_SET_TEMPERATURE:
        {
            light_ctl_server_set_temperature_t *pdata = pargs;
            if (pdata->total_time.num_steps == pdata->remaining_time.num_steps)
            {
                ctl_delta_uv = pdata->delta_uv;
                uint32_t trans_time = generic_transition_time_convert(pdata->total_time);
                if (trans_time > 0)
                {
                    light_set_lightness_linear(light_get_blue(), temperature_to_lightness(pdata->temperature),
                                               trans_time, light_lightness_change_done);
                }
                else
                {
                    light_set_blue_lightness(temperature_to_lightness(pdata->temperature));
                    light_state_store();
                }
#if MI_PUB_NEW_STRATEGY
                mi_publish_single_start(pmodel_info);
#endif
            }
        }
        break;
    case LIGHT_CTL_SERVER_SET_DEFAULT:
        {
        }
        break;
    case LIGHT_CTL_SERVER_SET_TEMPERATURE_RANGE:
        {
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
                    pmodel = light_ctl_temperature_server.pmodel;
                    sub_addr = pmsg->addr + 1;
                    primary_sub_addr = pmsg->addr;
                }
                else
                {
                    pmodel = generic_on_off_server.pmodel;
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
                light_ctl_server_models_sub();
            }
            break;
        }
    }

    return ret;
}

void light_ctl_server_models_sub(void)
{
    for (uint8_t i = 0; i < sizeof(light_ctl_sub_addr) / sizeof(uint16_t); ++i)
    {
        mesh_model_sub(generic_on_off_server.pmodel, light_ctl_sub_addr[i]);
        mesh_model_sub(light_ctl_temperature_server.pmodel, light_ctl_sub_addr[i]);
    }
}

void light_ctl_server_models_init(void)
{
    /* binding models */
    light_lightness_server.pmodel_bound = &generic_on_off_server;
    miot_server.pmodel_bound = &light_lightness_server;
    mijia_server.pmodel_bound = &miot_server;

    //light_ctl_temperature_server.pmodel_bound = &light_lightness_server;

    /* register light ctl models */
    generic_on_off_server.model_data_cb = generic_on_off_server_data;
    generic_on_off_server_reg(0, &generic_on_off_server);

    light_lightness_server.model_data_cb = light_lightness_server_data;
    light_lightness_server_reg(0, &light_lightness_server);
    light_lightness_period_pub_enable(TRUE, FALSE);

    light_ctl_temperature_server.model_data_cb = light_ctl_server_data;
    light_ctl_temperature_server_reg(1, &light_ctl_temperature_server);

    /* register miot model */
    miot_server.model_data_cb = miot_server_data;
    miot_server_reg(0, &miot_server);

    mijia_server.model_data_cb = mijia_server_data;
    mijia_server_reg(0, &mijia_server);

    cfg_server.model_receive = cfg_server_receive_peek;
}

