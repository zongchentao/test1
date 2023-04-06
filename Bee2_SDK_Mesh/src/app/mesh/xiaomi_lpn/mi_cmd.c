/**
************************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
************************************************************************************************************
* @file     mi_cmd.c
* @brief    User defined mi commands.
* @details  User command interfaces.
* @author   hector_huang
* @date     2019-1-11
* @version  v0.1
*************************************************************************************************************
*/
#include "mi_cmd.h"
#include "mesh_api.h"
#include "mijia_mesh_config.h"
#include "mijia_mesh_app.h"
#include "cryptography/mi_mesh_otp_config.h"
#include "rtl876x_lib_platform.h"
#include "mi_dummy_cert.h"
#include "light_cwrgb_app.h"
#include "light_storage_app.h"
#include "generic_on_off.h"
#include "lpn_app.h"
#include "light_config.h"
#include "mi_config.h"
#include "cryptography/mi_mesh_otp_config.h"
#include "cryptography/mi_mesh_otp.h"
#include "miot_model.h"
#include "miot_model_server_app.h"

/* external generic on off server */
extern mesh_model_info_t generic_on_off_server;
/* external miot model server and client*/
extern mesh_model_info_t miot_client;

extern void get_device_id(uint8_t *pdid);

/** flash sector size */
#define FLASH_SECTOR_SIZE    0x1000

/**
 * @brief list mesh node data
 * @param[in] pparse_value: command parameters
 * @return command process status
 */
user_cmd_parse_result_t user_cmd_list(user_cmd_parse_value_t *pparse_value)
{
    data_uart_debug("ProductID:\t%d\r\n", PRODUCT_ID);
    uint32_t device_id[2] = {0};
    get_device_id((uint8_t *)device_id);
    data_uart_debug("DeviceID:\t0x%08x%08x\r\n", device_id[1], device_id[0]);
    data_uart_debug("Firmware:\t" MIBLE_LIB_AND_DEVELOPER_VERSION "\r\n");
    data_uart_debug("MeshState:\t%d\r\n", mesh_node.node_state);
    data_uart_debug("DevUUID:\t");
    data_uart_dump(mesh_node.dev_uuid, 16);
    uint8_t bt_addr[6];
    gap_get_param(GAP_PARAM_BD_ADDR, bt_addr);
    data_uart_debug("BTAddr:\t\t%02x-%02x-%02x-%02x-%02x-%02x\r\n",
                    bt_addr[5], bt_addr[4], bt_addr[3], bt_addr[2], bt_addr[1], bt_addr[0]);
    for (uint16_t index = 0; index < mesh_node.dev_key_num; index++)
    {
        if (mesh_node.dev_key_list[index].used && mesh_node.dev_key_list[index].element_num != 0)
        {
            data_uart_debug("DevKey:\t\t%d-0x%04x-%d-", index, mesh_node.dev_key_list[index].unicast_addr,
                            mesh_node.dev_key_list[index].element_num);
            data_uart_dump(mesh_node.dev_key_list[index].dev_key, 16);
        }
    }
    for (uint16_t index = 0; index < mesh_node.app_key_num; index++)
    {
        if (mesh_node.app_key_list[index].key_state != MESH_KEY_STATE_INVALID)
        {
            data_uart_debug("AppKey:\t\t%d-0x%04x-%d-%d-%d\r\n", index,
                            mesh_node.app_key_list[index].app_key_index_g, mesh_node.app_key_list[index].key_state,
                            key_state_to_tx_loop(mesh_node.app_key_list[index].key_state),
                            mesh_node.app_key_list[index].net_key_binding);
            for (uint8_t loop = 0; loop < 2; loop++)
            {
                if (mesh_node.app_key_list[index].papp_key[loop] != NULL)
                {
                    data_uart_debug("\t\t");
                    data_uart_dump(mesh_node.app_key_list[index].papp_key[loop]->app_key, 16);
                }
            }
        }
    }
    for (uint16_t index = 0; index < mesh_node.net_key_num; index++)
    {
        if (mesh_node.net_key_list[index].key_state != MESH_KEY_STATE_INVALID)
        {
            data_uart_debug("NetKey:\t\t%d-0x%04x-%d-%d-%d\r\n", index,
                            mesh_node.net_key_list[index].net_key_index_g, mesh_node.net_key_list[index].key_state,
                            key_state_to_tx_loop(mesh_node.net_key_list[index].key_state),
                            key_state_to_key_refresh_phase(mesh_node.net_key_list[index].key_state));
            if (mesh_node.net_key_list[index].net_key_index_g & 0x8000)
            {
                break;
            }
            for (uint8_t loop = 0; loop < 2; loop++)
            {
                if (mesh_node.net_key_list[index].pnet_key[loop] != NULL)
                {
                    data_uart_debug("\t\t");
                    data_uart_dump(mesh_node.net_key_list[index].pnet_key[loop]->net_key, 16);
                }
            }
        }
    }
    data_uart_debug("IVindex:\t%d-0x%x\r\n", mesh_node.iv_update_flag, mesh_node.iv_index);
    data_uart_debug("Seq:\t\t0x%06x\r\n", mesh_node.seq);
    data_uart_debug("NodeAddr:\t0x%04x-%d-%d\r\n", mesh_node.unicast_addr,
                    mesh_node.element_queue.count, mesh_node.model_num);
    mesh_element_p pelement = (mesh_element_p)mesh_node.element_queue.pfirst;
    while (pelement != NULL)
    {
        data_uart_debug("Element:\t%d-%d\r\n", pelement->element_index, pelement->model_queue.count);
        mesh_model_p pmodel = (mesh_model_p)pelement->model_queue.pfirst;
        while (pmodel != NULL)
        {
            data_uart_debug("Model:\t\t%d-%d-0x%08x", pmodel->pmodel_info->model_index,
                            pmodel->model_index, pmodel->pmodel_info->model_id);
            uint8_t key_flag = true;
            for (uint16_t index = 0; index < mesh_node.app_key_num; index++)
            {
                if (plt_bit_pool_get(pmodel->app_key_binding, index) &&
                    mesh_node.app_key_list[index].key_state != MESH_KEY_STATE_INVALID)
                {
                    if (key_flag)
                    {
                        key_flag = false;
                        data_uart_debug("-(key:%d", index);
                    }
                    else
                    {
                        data_uart_debug("-%d", index);
                    }
                }
            }
            if (!key_flag)
            {
                data_uart_debug(")");
            }
            if (MESH_NOT_UNASSIGNED_ADDR(pmodel->pub_params.pub_addr))
            {
                data_uart_debug("-(pub:0x%04x-%d-%d)", pmodel->pub_params.pub_addr, pmodel->pub_params.pub_ttl,
                                pmodel->pub_params.pub_key_info.app_key_index);
            }
            mesh_model_p pmodelb = pmodel;
            while (pmodelb->pmodel_info->pmodel_bound != NULL)
            {
                pmodelb = (mesh_model_p)pmodelb->pmodel_info->pmodel_bound->pmodel;
            }
            mesh_addr_member_p paddr_element = (mesh_addr_member_p)pmodelb->sub_queue.pfirst;
            while (paddr_element != NULL)
            {
                if (paddr_element == (mesh_addr_member_p)pmodelb->sub_queue.pfirst)
                {
                    if (pmodelb != pmodel)
                    {
                        data_uart_debug("-(sub:-%d-%d-0x%04x",
                                        ((mesh_model_p)pmodel->pmodel_info->pmodel_bound->pmodel)->model_index,
                                        pmodelb->model_index, paddr_element->mesh_addr);
                    }
                    else
                    {
                        data_uart_debug("-(sub:0x%04x", paddr_element->mesh_addr);
                    }
                }
                else
                {
                    data_uart_debug("-0x%04x", paddr_element->mesh_addr);
                }
                paddr_element = paddr_element->pnext;
                if (paddr_element == NULL)
                {
                    data_uart_debug(")");
                }
            }
            pmodel = pmodel->pnext;
            data_uart_debug("\r\n");
        }
        pelement = pelement->pnext;
    }

    return USER_CMD_RESULT_OK;
}

/**
 * @brief reset node
 * @param[in] pparse_value: command parameters
 * @return command process status
 */
static user_cmd_parse_result_t user_cmd_node_reset(user_cmd_parse_value_t *pparse_value)
{
    switch (pparse_value->dw_parameter[0])
    {
    case 0:
        mesh_node_reset();
        break;
    case 1:
        mesh_node_clean();
        break;
    default:
        break;
    }
    return USER_CMD_RESULT_OK;
}

/**
 * @brief verify certificates
 * @param[in] pparse_value: command parameters
 * @return command process status
 */
static user_cmd_parse_result_t user_cmd_mi_otp_verify(user_cmd_parse_value_t *pparse_value)
{
    int ret = mi_mesh_otp_verify();
    data_uart_debug("verify otp: %d\r\n", ret);
    return USER_CMD_RESULT_OK;
}

/**
 * @brief erase dfu region
 * @param[in] pparse_value: command parameters
 * @return command process status
 */
static user_cmd_parse_result_t user_cmd_mi_dfu_erase(user_cmd_parse_value_t *pparse_value)
{
    uint32_t sector_addr = DFU_NVM_START;
    sector_addr &= ~(FLASH_SECTOR_SIZE - 1);
    uint32_t size = DFU_NVM_SIZE;
    uint32_t sector_cnt = size / FLASH_SECTOR_SIZE;
    data_uart_debug("erase dfu sectors: start address(0x%x), sector count(%d)\r\n", sector_addr,
                    sector_cnt);
    for (uint32_t i = 0; i < sector_cnt; ++i)
    {
        if (!flash_erase_locked(FLASH_ERASE_SECTOR, sector_addr))
        {
            data_uart_debug("erase dfu sector %d(0x%x) failed\r\n", i, sector_addr);
            return USER_CMD_RESULT_ERROR;
        }
        data_uart_debug("erase dfu sector %d(0x%x) success\r\n", i, sector_addr);
        sector_addr += FLASH_SECTOR_SIZE;
    }

    return USER_CMD_RESULT_OK;
}

/**
 * @brief change light status
 * @param[in] pparse_value: command parameters
 * @return command process status
 */
static user_cmd_parse_result_t user_cmd_gooc(user_cmd_parse_value_t *pparse_value)
{
    uint32_t onoff = pparse_value->dw_parameter[0];
    if (onoff)
    {
        light_cw_turn_on();
        light_state_store();
    }
    else
    {
        light_cw_turn_off();
    }

    generic_on_off_publish(&generic_on_off_server, (generic_on_off_t)onoff);

    return USER_CMD_RESULT_OK;
}

#if MI_VENDOR_MODEL_CLIENT_ENABLE
/**
 * @brief miot get
 * @param[in] pparse_value: command parameters
 * @return command process status
 */
static user_cmd_parse_result_t user_cmd_miotg(user_cmd_parse_value_t *pparse_value)
{
    if (pparse_value->para_count < 2)
    {
        return USER_CMD_RESULT_WRONG_NUM_OF_PARAMETERS;
    }

    uint8_t data[MIOT_PARAM_MAX_LEN];
    for (uint8_t i = 0; i < pparse_value->para_count - 2; ++i)
    {
        data[i] = pparse_value->dw_parameter[i + 2];
    }
    miot_get(&miot_client, pparse_value->dw_parameter[0], pparse_value->dw_parameter[1],
             data, pparse_value->para_count - 2);

    return USER_CMD_RESULT_OK;
}

/**
 * @brief miot set
 * @param[in] pparse_value: command parameters
 * @return command process status
 */
static user_cmd_parse_result_t user_cmd_miots(user_cmd_parse_value_t *pparse_value)
{
    if (pparse_value->para_count < 3)
    {
        return USER_CMD_RESULT_WRONG_NUM_OF_PARAMETERS;
    }

    uint8_t data[MIOT_PARAM_MAX_LEN];
    for (uint8_t i = 0; i < pparse_value->para_count - 3; ++i)
    {
        data[i] = pparse_value->dw_parameter[i + 3];
    }
    miot_set(&miot_client, pparse_value->dw_parameter[0], pparse_value->dw_parameter[1],
             data, pparse_value->para_count - 3, pparse_value->dw_parameter[2]);

    return USER_CMD_RESULT_OK;
}
#endif

/**
 * @brief miot indication
 * @param[in] pparse_value: command parameters
 * @return command process status
 */
static user_cmd_parse_result_t user_cmd_mioti(user_cmd_parse_value_t *pparse_value)
{
    if (pparse_value->para_count < 2)
    {
        return USER_CMD_RESULT_WRONG_NUM_OF_PARAMETERS;
    }

    uint8_t data[MIOT_PARAM_MAX_LEN];
    for (uint8_t i = 0; i < pparse_value->para_count - 2; ++i)
    {
        data[i] = pparse_value->dw_parameter[i + 2];
    }

    miot_server_indication(pparse_value->dw_parameter[0], pparse_value->dw_parameter[1],
                           data, pparse_value->para_count - 2);

    return USER_CMD_RESULT_OK;
}

/**
 * @brief configuration server network transition set
 * @param[in] pparse_value: command parameters
 * @return command process status
 */
static user_cmd_parse_result_t user_cmd_cnts(user_cmd_parse_value_t *pparse_value)
{
    uint8_t count = pparse_value->dw_parameter[0];
    uint8_t steps = pparse_value->dw_parameter[1];
    cfg_server_set_net_trans(count, steps);

    data_uart_debug("set cfg server trans: count(%d) steps(%d)\r\n", count, steps);

    return USER_CMD_RESULT_OK;
}

/**
 * @brief configuration server response with segmented message
 * @param[in] pparse_value: command parameters
 * @return command process status
 */
static user_cmd_parse_result_t user_cmd_csrs(user_cmd_parse_value_t *pparse_value)
{
    bool use_seg = pparse_value->dw_parameter[0];
    cfg_server_resp_with_seg_msg(use_seg);
    return USER_CMD_RESULT_OK;
}

/*----------------------------------------------------
 * command table
 * --------------------------------------------------*/
const user_cmd_table_entry_t mi_cmd_table[] =
{
    // mi cmd
    {
        "ls",
        "ls\r\n",
        "list node state info\r\n",
        user_cmd_list
    },
    {
        "nr",
        "nr [mode]\r\n",
        "node reset\r\n",
        user_cmd_node_reset
    },
    {
        "miotpv",
        "miotpv\r\n",
        "verify mi otp\r\n",
        user_cmd_mi_otp_verify
    },
    {
        "midfue",
        "midfue\r\n",
        "erase mi dfu sectors\r\n",
        user_cmd_mi_dfu_erase
    },
    {
        "gooc",
        "gooc [on_off]\r\n",
        "change generic on/off status\r\n",
        user_cmd_gooc,
    },
#if MI_VENDOR_MODEL_CLIENT_ENABLE
    {
        "miotg",
        "miotg [dst] [app key index] [data...]\r\n",
        "miot get\r\n",
        user_cmd_miotg,
    },
    {
        "miots",
        "miots [dst] [app key index] [ack] [data...]\r\n",
        "miot set\r\n",
        user_cmd_miots,
    },
#endif
    {
        "mioti",
        "mioti [dst] [app key index] [data...]\r\n",
        "miot indication\r\n",
        user_cmd_mioti,
    },
    {
        "cnts",
        "cnts [count] [steps]\r\n",
        "cfg server net trans set\r\n",
        user_cmd_cnts,
    },
    {
        "csrs",
        "csrs [flag]\r\n",
        "cfg server rsp with seg\r\n",
        user_cmd_csrs,
    },
    /* MUST be at the end: */
    {
        0,
        0,
        0,
        0
    }
};

