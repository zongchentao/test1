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

/* light subscribe address */
static const uint16_t light_ctl_sub_addr[] = {0xFE00};

/* ctl light models */
mesh_model_info_t generic_on_off_server;
static mesh_model_info_t light_lightness_server;
static mesh_model_info_t light_ctl_temperature_server;

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
            if ((light_get_cold()->lightness) ||
                (light_get_warm()->lightness))
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
                        if ((0 == light_get_cold()->lightness_last) &&
                            (0 == light_get_warm()->lightness_last))
                        {
                            light_set_lightness_linear(light_get_cold(), 65535, trans_time, NULL);
                            light_set_lightness_linear(light_get_warm(), 0x4e20, trans_time, light_lightness_change_done);
                        }
                        else
                        {
                            light_set_lightness_linear(light_get_cold(), light_get_cold()->lightness_last, trans_time,
                                                       NULL);
                            light_set_lightness_linear(light_get_warm(), light_get_warm()->lightness_last, trans_time,
                                                       NULL);
                        }
                    }
                    else
                    {
                        light_set_lightness_linear(light_get_cold(), 0, trans_time, NULL);
                        light_set_lightness_linear(light_get_warm(), 0, trans_time, NULL);
                    }
                }
                else
                {
                    if (GENERIC_ON == pdata->on_off)
                    {
                        light_cw_turn_on();
                        light_state_store();
                    }
                    else
                    {
                        light_cw_turn_off();
                    }
                }
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
            pdata->lightness = light_get_cold()->lightness;
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
                    light_set_lightness_linear(light_get_cold(), pdata->lightness, trans_time,
                                               light_lightness_change_done);
                }
                else
                {
                    light_set_cold_lightness(pdata->lightness);
                    light_state_store();
                }
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
            light_ctl_t ctl = light_get_ctl();
            pdata->lightness = ctl.lightness;
            pdata->temperature = ctl.temperature;
        }
        break;
    case LIGHT_CTL_SERVER_GET_DEFAULT:
        {
        }
        break;
    case LIGHT_CTL_SERVER_GET_TEMPERATURE:
        {
            light_ctl_server_get_temperature_t *pdata = pargs;
            light_ctl_t ctl = light_get_ctl();
            pdata->temperature = ctl.temperature;
            pdata->delta_uv = ctl.delta_uv;
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
                uint32_t trans_time = generic_transition_time_convert(pdata->total_time);
                if (trans_time > 0)
                {
                    light_set_lightness_linear(light_get_warm(), temperature_to_lightness(pdata->temperature),
                                               trans_time, light_lightness_change_done);
                }
                else
                {
                    light_ctl_t ctl = light_get_ctl();
                    ctl.temperature = pdata->temperature;
                    ctl.delta_uv = pdata->delta_uv;
                    light_set_ctl(ctl);
                    light_state_store();
                }
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

void light_ctl_server_models_sub(void)
{
    for (uint8_t i = 0; i < sizeof(light_ctl_sub_addr) / sizeof(uint16_t); ++i)
    {
        mesh_model_sub(generic_on_off_server.pmodel, light_ctl_sub_addr[i]);
        mesh_model_sub(light_lightness_server.pmodel, light_ctl_sub_addr[i]);
        mesh_model_sub(light_ctl_temperature_server.pmodel, light_ctl_sub_addr[i]);
    }
}

void light_ctl_server_models_init(void)
{
    /* binding models */
    light_lightness_server.pmodel_bound = &generic_on_off_server;
    //light_ctl_temperature_server.pmodel_bound = &light_lightness_server;

    /* register light ctl models */
    generic_on_off_server.model_data_cb = generic_on_off_server_data;
    generic_on_off_server_reg(0, &generic_on_off_server);

    light_lightness_server.model_data_cb = light_lightness_server_data;
    light_lightness_server_reg(0, &light_lightness_server);
    light_lightness_period_pub_enable(TRUE, FALSE);

    light_ctl_temperature_server.model_data_cb = light_ctl_server_data;
    light_ctl_temperature_server_reg(1, &light_ctl_temperature_server);
}

