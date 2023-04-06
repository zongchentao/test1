/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      miot_model_client_app.c
* @brief     xiaomi vendor model client
* @details
* @author    hector_huang
* @date      2019-04-16
* @version   v1.0
* *********************************************************************************************************
*/
#include "miot_model_client_app.h"
#include "miot_model.h"

/* miot client model */
mesh_model_info_t miot_client;

/**
 * @brief miot client model data process
 * @param[in] pmodel_info: miot model client handler
 * @param[in] type: data type
 * @param[in] pargs: data need to process
 * @return process status
 */
static int32_t miot_client_data(const mesh_model_info_p pmodel_info, uint32_t type,
                                void *pargs)
{
    switch (type)
    {
    case MIOT_CLIENT_STATUS:
        {
            miot_client_status_t *pdata = pargs;
            printi("miot_client_staus:");
            dprinti(pdata->parameters, pdata->param_len);
        }
        break;
    case MIOT_CLIENT_INDICATION:
        {
            miot_client_indication_t *pdata = pargs;
            printi("miot_client_indication:");
            dprinti(pdata->parameters, pdata->param_len);

            /* @note use actual data to replace sample data */
            uint8_t indication_ack[MIOT_PARAM_MAX_LEN] = {pdata->parameters[0], pdata->parameters[1], 4, 3, 2, 1};
            miot_indication_ack(pmodel_info, pdata->src, pdata->app_key_index, indication_ack, 6);
        }
        break;
    default:
        break;
    }

    return 0;
}

void miot_client_model_init(void)
{
    /* binding models */

    /* register miot model */
    miot_client.model_data_cb = miot_client_data;
    miot_client_reg(0, &miot_client);
}

