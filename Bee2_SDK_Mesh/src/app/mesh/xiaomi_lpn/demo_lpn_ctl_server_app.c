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
#include "demo_lpn_ctl_server_app.h"
#include "generic_on_off.h"
#include "light_lightness.h"
#include "light_ctl.h"
#include "light_cwrgb_app.h"
#include "light_storage_app.h"
#include "light_controller_app.h"
#include "generic_transition_time.h"

/* light subscribe address */
static const uint16_t light_ctl_sub_addr[] = {0xFE00};

/* models */
mesh_model_info_t generic_on_off_server;

/* state */
generic_on_off_t lpn_state = GENERIC_OFF;

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
            pdata->on_off = lpn_state;
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
                    lpn_state = GENERIC_ON;
                }
                else
                {
                    printi("received message turn off");
                    lpn_state = GENERIC_OFF;
                }
            }
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
    }
}

void light_ctl_server_models_init(void)
{
    /* binding models */

    /* register light ctl models */
    generic_on_off_server.model_data_cb = generic_on_off_server_data;
    generic_on_off_server_reg(0, &generic_on_off_server);
}

