#include "rible_api.h"
#include "gap_adv.h"

static rible_status_t err_code_convert(T_GAP_CAUSE cause)
{
    rible_status_t status;
    switch (cause)
    {
        case GAP_CAUSE_SUCCESS:
            status = RI_SUCCESS;
            break;
        case GAP_CAUSE_ALREADY_IN_REQ:
            status = RI_ERR_BUSY;
            break;
        case GAP_CAUSE_INVALID_STATE:
            status = RI_ERR_INVALID_STATE;
            break;
        case GAP_CAUSE_INVALID_PARAM:
            status = RI_ERR_INVALID_PARAM;
            break;
        case GAP_CAUSE_NON_CONN:
            status = RIBLE_ERR_INVALID_CONN_HANDLE;
            break;
        case GAP_CAUSE_NOT_FIND_IRK:
            status = RIBLE_ERR_UNKNOWN;
            break;
        case GAP_CAUSE_ERROR_CREDITS:
            status = RIBLE_ERR_UNKNOWN;
            break;
        case GAP_CAUSE_SEND_REQ_FAILED:
            status = RIBLE_ERR_UNKNOWN;
            break;
        case GAP_CAUSE_NO_RESOURCE:
            status = RI_ERR_RESOURCES;
            break;
        case GAP_CAUSE_INVALID_PDU_SIZE:
            status = RI_ERR_INVALID_LENGTH;
            break;
        case GAP_CAUSE_NOT_FIND:
            status = RI_ERR_NOT_FOUND;
            break;
        case GAP_CAUSE_CONN_LIMIT:
            status = RIBLE_ERR_UNKNOWN;
            break;
        case GAP_CAUSE_NO_BOND:
            status = RIBLE_ERR_UNKNOWN;
            break;
        case GAP_CAUSE_ERROR_UNKNOWN:
            status = RIBLE_ERR_UNKNOWN;
            break;
        default:
            status = RIBLE_ERR_UNKNOWN;
            break;
    }

    return status;
}

static T_GAP_CAUSE rtk_gap_adv_start(rible_gap_adv_param_t *p_param);
static T_GAP_CAUSE rtk_gap_adv_data_set(uint8_t const *p_data,
                                 uint8_t dlen, uint8_t const *p_sr_data, uint8_t srdlen);


rible_status_t rible_gap_address_get(rible_addr_t mac)
{
    T_GAP_CAUSE err = gap_get_param(GAP_PARAM_BD_ADDR, mac);
    return err_code_convert(err);
}

static T_GAP_CAUSE rtk_gap_adv_start(rible_gap_adv_param_t *p_param)
{
    uint8_t  adv_evt_type;
    uint8_t  adv_chann_map = GAP_ADVCHAN_ALL;
    uint16_t adv_int_min = p_param->interval_min;
    uint16_t adv_int_max = p_param->interval_max;
    if (RIBLE_ADV_TYPE_CONNECTABLE_UNDIRECTED == p_param->adv_type)
    {
        adv_evt_type = GAP_ADTYPE_ADV_IND;
    }
    else if (RIBLE_ADV_TYPE_SCANNABLE_UNDIRECTED == p_param->adv_type)
    {
        adv_evt_type = GAP_ADTYPE_ADV_SCAN_IND;
    }
    else
    {
        adv_evt_type = GAP_ADTYPE_ADV_NONCONN_IND;
    }

    if (p_param->ch_mask.ch_37_off)
    {
        adv_chann_map &= ~GAP_ADVCHAN_37;
    }

    if (p_param->ch_mask.ch_38_off)
    {
        adv_chann_map &= ~GAP_ADVCHAN_38;
    }

    if (p_param->ch_mask.ch_39_off)
    {
        adv_chann_map &= ~GAP_ADVCHAN_39;
    }

    /* set advertising parameters */
    le_adv_set_param(GAP_PARAM_ADV_EVENT_TYPE, sizeof(adv_evt_type), &adv_evt_type);
    le_adv_set_param(GAP_PARAM_ADV_CHANNEL_MAP, sizeof(adv_chann_map), &adv_chann_map);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MIN, sizeof(adv_int_min), &adv_int_min);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MAX, sizeof(adv_int_max), &adv_int_max);

    return le_adv_start();
}

rible_status_t rible_gap_adv_start(rible_gap_adv_param_t *p_param)
{
    rible_status_t err = RI_SUCCESS;
// #if MIBLE_API_SYNC
//     rtk_gap_task_t *ptask = plt_malloc(sizeof(rtk_gap_task_t), RAM_TYPE_DATA_ON);
//     if (NULL != ptask)
//     {
//         ptask->task_type = RTK_GAP_TASK_TYPE_ADV;
//         ptask->adv.adv_enable = TRUE;
//         ptask->adv.adv_param = *p_param;
//         rtk_gap_task_try(ptask);
//     }
//     else
//     {
//         MI_LOG_ERROR("adv start failed: out of memory!");
//         err = MI_ERR_NO_MEM;
//     }
// #else
    T_GAP_CAUSE ret = rtk_gap_adv_start(p_param);
    err = err_code_convert(ret);
// #endif
    return err;
}

static T_GAP_CAUSE rtk_gap_adv_data_set(uint8_t const *p_data,
                                 uint8_t dlen, uint8_t const *p_sr_data, uint8_t srdlen)
{
    if ((NULL != p_data) && (dlen > 0))
    {
        le_adv_set_param(GAP_PARAM_ADV_DATA, dlen, (void *)p_data);
    }

    if ((NULL != p_sr_data) && (srdlen > 0))
    {
        le_adv_set_param(GAP_PARAM_SCAN_RSP_DATA, srdlen, (void *)p_sr_data);
    }

    return le_adv_update_param();
}

rible_status_t rible_gap_adv_data_set(uint8_t const *p_data,
                                      uint8_t dlen, uint8_t const *p_sr_data, uint8_t srdlen)
{
    rible_status_t err = RI_SUCCESS;
// #if MIBLE_API_SYNC
//     rtk_gap_task_t *ptask = plt_malloc(sizeof(rtk_gap_task_t), RAM_TYPE_DATA_ON);
//     if (NULL != ptask)
//     {
//         uint8_t len = (dlen > RTK_GAP_MAX_ADV_DATA_LEN) ? RTK_GAP_MAX_ADV_DATA_LEN : dlen;
//         ptask->task_type = RTK_GAP_TASK_TYPE_UPDATE_ADV_PARAM;
//         memcpy(ptask->update_adv_param.adv_data, p_data, len);
//         ptask->update_adv_param.adv_data_len = len;

//         len = (srdlen > RTK_GAP_MAX_ADV_DATA_LEN) ? RTK_GAP_MAX_ADV_DATA_LEN : srdlen;
//         memcpy(ptask->update_adv_param.scan_rsp_data, p_sr_data, len);
//         ptask->update_adv_param.scan_rsp_data_len = len;
//         rtk_gap_task_try(ptask);
//     }
//     else
//     {
//         MI_LOG_ERROR("adv data set failed: out of memory!");
//         err = MI_ERR_NO_MEM;
//     }
// #else
    T_GAP_CAUSE ret = rtk_gap_adv_data_set(p_data, dlen, p_sr_data, srdlen);
    err = err_code_convert(ret);
// #endif
    return err;
}

rible_status_t rible_gap_adv_stop(void)
{
    rible_status_t err = RI_SUCCESS;
// #if MIBLE_API_SYNC
//     rtk_gap_task_t *ptask = plt_malloc(sizeof(rtk_gap_task_t), RAM_TYPE_DATA_ON);
//     if (NULL != ptask)
//     {
//         ptask->task_type = RTK_GAP_TASK_TYPE_ADV;
//         ptask->adv.adv_enable = FALSE;
//         rtk_gap_task_try(ptask);
//     }
//     else
//     {
//         MI_LOG_ERROR("adv stop failed: out of memory!");
//         err = MI_ERR_NO_MEM;
//     }
// #else
    T_GAP_CAUSE ret = le_adv_stop();
    err = err_code_convert(ret);
// #endif
    return err;
}

