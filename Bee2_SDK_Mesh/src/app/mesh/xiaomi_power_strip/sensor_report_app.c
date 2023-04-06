/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      sensor_report_app.c
* @brief     Smart mesh demo power strip application
* @details
* @author    astor zhang
* @date      2019-07-01
* @version   v1.0
* *********************************************************************************************************
*/

#include "mesh_api.h"
#include "sensor_report_app.h"
#include "mijia_model.h"
#include "generic_on_off.h"
#include "light_ctl.h"
#include "light_cwrgb_app.h"
#include "sensor.h"
#include "os_timer.h"
#include "generic_transition_time.h"
#include "miot_model.h"
#include "miot_model_server_app.h"

#define XIAOMI_PROPERTY_ID_INPUT_POWER  0x52
#define XIAOMI_PROPERTY_ID_TOTAL_ENERGY    0x6A

/* power strip subscribe address */
static const uint16_t power_ctl_sub_addr[] = {0xFE00};

/* power strip models */
mesh_model_info_t generic_on_off_server;
mesh_model_info_t sensor_report_server;
mesh_model_info_t mijia_power_strip_server;

/* sensor demo data */
uint8_t sample_raw_data_power[] = {0xFE, 0x01};
uint8_t sample_raw_data_consume[] = {0x10, 0xDE};

sensor_db_t *sensor_db;

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
            if (light_get_red()->lightness)
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
                    light_lighten_red(4096);
                }
                else
                {
                    light_lighten_red(0);
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
 * @brief sensor server data callback
 * @param[in] pmodel_info: sensor server model handler
 * @param[in] type: data type
 * @param[in, out] pargs: data need to process
 */
static int32_t sensor_report_server_data(const mesh_model_info_p pmodel_info, uint32_t type,
                                         void *pargs)
{
    switch (type)
    {
    case SENSOR_SERVER_GET:
        {
            sensor_server_get_t *pdata = pargs;
            if (pdata->property_id == XIAOMI_PROPERTY_ID_INPUT_POWER)
            {
                pdata->raw_data = sample_raw_data_power;
            }
            else if (pdata->property_id == XIAOMI_PROPERTY_ID_TOTAL_ENERGY)
            {
                pdata->raw_data = sample_raw_data_consume;
            }
        }
    default:
        break;
    }
    return 0;
}

/**
 * @brief sensor server data callback
 * @param[in] pmodel_info: sensor server model handler
 * @param[in] type: data type
 * @param[in, out] pargs: data need to process
 */
static int32_t mijia_server_data(const mesh_model_info_p pmodel_info, uint32_t type,
                                 void *pargs)
{
    switch (type)
    {
    default:
        break;
    }
    return 0;
}

void sensor_server_models_sub(void)
{
    for (uint8_t i = 0; i < sizeof(power_ctl_sub_addr) / sizeof(uint16_t); ++i)
    {
        mesh_model_sub(generic_on_off_server.pmodel, power_ctl_sub_addr[i]);
    }
}



void sensor_server_models_init(void)
{
    /* binding models */
    sensor_report_server.pmodel_bound = &generic_on_off_server;
    miot_server.pmodel_bound = &sensor_report_server;
    mijia_power_strip_server.pmodel_bound = &miot_server;

    /* register power strip models */
    generic_on_off_server.model_data_cb = generic_on_off_server_data;
    generic_on_off_server_reg(0, &generic_on_off_server);

    sensor_report_server.model_data_cb = sensor_report_server_data;
    sensor_server_reg(0, &sensor_report_server);

    /* register miot model */
    miot_server.model_data_cb = miot_server_data;
    miot_server_reg(0, &miot_server);

    mijia_power_strip_server.model_data_cb = mijia_server_data;
    mijia_server_reg(0, &mijia_power_strip_server);

    if (NULL == sensor_db)
    {
        sensor_db = plt_malloc(sizeof(sensor_db_t) * 2, RAM_TYPE_DATA_ON);
        if (NULL == sensor_db)
        {
            printe("sensor_server_reg: fail to allocate memory for the new model extension data!");
            return;
        }
        memset(sensor_db, 0, sizeof(sensor_db_t) * 2);
    }

    sensor_db[0].descriptor.property_id = 0x52;
    sensor_db[0].sensor_raw_data_len = 2;

    sensor_db[1].descriptor.property_id = 0x6A;
    sensor_db[1].sensor_raw_data_len = 2;

    sensor_server_set_db(&sensor_report_server, sensor_db, 2);

#if 0
    /* publish test */
    mesh_model_pub_params_t pub_params;
    pub_params.pub_addr = 0xFFFE;
    pub_params.pub_key_info.app_key_index = 0;
    pub_params.pub_key_info.frnd_flag = 0;
    pub_params.pub_key_info.rfu = 0;
    pub_params.pub_ttl = 5;
    mesh_model_pub_params_set(generic_on_off_server.pmodel, pub_params);
    mesh_model_pub_params_set(sensor_report_server.pmodel, pub_params);
#endif
}


void handle_demo_data(void)
{
    sample_raw_data_power[0] -= sample_raw_data_consume[1] % 8;
    sample_raw_data_consume[1] += sample_raw_data_power[0] % 9;
}

