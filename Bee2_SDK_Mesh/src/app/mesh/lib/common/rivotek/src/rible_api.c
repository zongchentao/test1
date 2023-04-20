#include "rible_api.h"
#include "rible_log.h"

/**
 * @brief   Get BLE mac address.
 * @param   [out] mac: pointer to data
 * @return  RI_SUCCESS          The requested mac address were written to mac
 *          RI_ERR_INTERNAL     No mac address found.
 * @note:   You should copy gap mac to mac[6]
 * */
__WEAK rible_status_t rible_gap_address_get(rible_addr_t mac)
{
    return RI_SUCCESS;
}

/**
 * @brief   Start advertising
 * @param   [in] p_adv_param : pointer to advertising parameters, see
 * rible_gap_adv_param_t for details
 * @return  RI_SUCCESS             Successfully initiated advertising procedure.
 *          RI_ERR_INVALID_STATE   Initiated connectable advertising procedure
 * when connected.
 *          RI_ERR_INVALID_PARAM   Invalid parameter(s) supplied.
 *          RI_ERR_BUSY            The stack is busy, process pending events and
 * retry.
 *          RI_ERR_RESOURCES       Stop one or more currently active roles
 * (Central, Peripheral or Observer) and try again.
 * @note    Other default advertising parameters: local public address , no
 * filter policy
 * */
__WEAK rible_status_t rible_gap_adv_start(rible_gap_adv_param_t *p_param)
{
    return RI_SUCCESS;
}

/**
 * @brief   Config advertising data
 * @param   [in] p_data : Raw data to be placed in advertising packet. If NULL, no changes are made to the current advertising packet.
 * @param   [in] dlen   : Data length for p_data. Max size: 31 octets. Should be 0 if p_data is NULL, can be 0 if p_data is not NULL.
 * @param   [in] p_sr_data : Raw data to be placed in scan response packet. If NULL, no changes are made to the current scan response packet data.
 * @param   [in] srdlen : Data length for p_sr_data. Max size: BLE_GAP_ADV_MAX_SIZE octets. Should be 0 if p_sr_data is NULL, can be 0 if p_data is not NULL.
 * @return  RI_SUCCESS             Successfully set advertising data.
 *          RI_ERR_INVALID_ADDR    Invalid pointer supplied.
 *          RI_ERR_INVALID_PARAM   Invalid parameter(s) supplied.
 * */
__WEAK rible_status_t rible_gap_adv_data_set(uint8_t const * p_data,
        uint8_t dlen, uint8_t const *p_sr_data, uint8_t srdlen)
{
    return RI_SUCCESS;
}

/**
 * @brief   Stop advertising
 * @param   void
 * @return  RI_SUCCESS             Successfully stopped advertising procedure.
 *          RI_ERR_INVALID_STATE   Not in advertising state.
 * */
__WEAK rible_status_t rible_gap_adv_stop(void)
{
    return RI_SUCCESS;
}

/**
 * @brief   Add a Service to a GATT server
 * @param   [in|out] p_server_db: pointer to rible service data type
 * of rible_gatts_db_t, see TYPE rible_gatts_db_t for details.
 * @return  RI_SUCCESS             Successfully added a service declaration.
 *          RI_ERR_INVALID_ADDR    Invalid pointer supplied.
 *          RI_ERR_INVALID_PARAM   Invalid parameter(s) supplied.
 *          RI_ERR_NO_MEM          Not enough memory to complete operation.
 * @note    This function can be implemented asynchronous. When service inition complete, call rible_arch_event_callback function and pass in RIBLE_ARCH_EVT_GATTS_SRV_INIT_CMP event and result.
 * */
__WEAK rible_status_t rible_gatts_service_init(rible_gatts_db_t *p_server_db)
{
    return RI_SUCCESS;
}

/**
 * @brief   Set characteristic value
 * @param   [in] srv_handle: service handle
 *          [in] value_handle: characteristic value handle
 *          [in] offset: the offset from which the attribute value has
 *to be updated
 *          [in] p_value: pointer to data
 *          [in] len: data length
 * @return  RI_SUCCESS             Successfully retrieved the value of the
 *attribute.
 *          RI_ERR_INVALID_ADDR    Invalid pointer supplied.
 *          RI_ERR_INVALID_PARAM   Invalid parameter (offset) supplied.
 *          RI_ERR_INVALID_LENGTH   Invalid length supplied.
 *          RIBLE_ERR_ATT_INVALID_HANDLE     Attribute not found.
 *          RIBLE_ERR_GATT_INVALID_ATT_TYPE  Attributes are not modifiable by
 *the application.
 * */
__WEAK rible_status_t rible_gatts_value_set(uint16_t srv_handle,
        uint16_t value_handle, uint8_t offset, uint8_t* p_value, uint8_t len)
{
    return RI_SUCCESS;
}

/**
 * @brief   Get charicteristic value as a GATTS.
 * @param   [in] srv_handle: service handle
 *          [in] value_handle: characteristic value handle
 *          [out] p_value: pointer to data which stores characteristic value
 *          [out] p_len: pointer to data length.
 * @return  RI_SUCCESS             Successfully get the value of the attribute.
 *          RI_ERR_INVALID_ADDR    Invalid pointer supplied.
 *          RI_ERR_INVALID_PARAM   Invalid parameter (offset) supplied.
 *          RI_ERR_INVALID_LENGTH   Invalid length supplied.
 *          RIBLE_ERR_ATT_INVALID_HANDLE     Attribute not found.
 **/
__WEAK rible_status_t rible_gatts_value_get(uint16_t srv_handle,
        uint16_t value_handle, uint8_t* p_value, uint8_t *p_len)
{
    return RI_SUCCESS;
}

/**
 * @brief   Set characteristic value and notify it to client.
 * @param   [in] conn_handle: conn handle
 *          [in] srv_handle: service handle
 *          [in] char_value_handle: characteristic  value handle
 *          [in] offset: the offset from which the attribute value has to
 * be updated
 *          [in] p_value: pointer to data
 *          [in] len: data length
 *          [in] type : notification = 1; indication = 2;
 *
 * @return  RI_SUCCESS             Successfully queued a notification or
 * indication for transmission,
 *          RI_ERR_INVALID_ADDR    Invalid pointer supplied.
 *          RI_ERR_INVALID_PARAM   Invalid parameter (offset) supplied.
 *          RI_ERR_INVALID_STATE   Invalid Connection State or notifications
 * and/or indications not enabled in the CCCD.
 *          RI_ERR_INVALID_LENGTH   Invalid length supplied.
 *          RI_ERR_BUSY            Procedure already in progress.
 *          RIBLE_ERR_ATT_INVALID_HANDLE     Attribute not found.
 *          RIBLE_ERR_GATT_INVALID_ATT_TYPE   //Attributes are not modifiable by
 * the application.
 * @note    This function checks for the relevant Client Characteristic
 * Configuration descriptor value to verify that the relevant operation (notification or
 * indication) has been enabled by the client.
 * */
__WEAK rible_status_t rible_gatts_notify_or_indicate(uint16_t conn_handle,
        uint16_t srv_handle, uint16_t char_value_handle, uint8_t offset,
        uint8_t* p_value, uint8_t len, uint8_t type)
{
    return RI_SUCCESS;
}

/**
 * @brief   Respond to a Read/Write user authorization request.
 * @param   [in] conn_handle: conn handle
 *          [in] status:  1: permit to change value ; 0: reject to change value
 *          [in] char_value_handle: characteristic handle
 *          [in] offset: the offset from which the attribute value has to
 * be updated
 *          [in] p_value: Pointer to new value used to update the attribute value.
 *          [in] len: data length
 *          [in] type : read response = 1; write response = 2;
 *
 * @return  RI_SUCCESS             Successfully queued a response to the peer, and in the case of a write operation, GATT updated.
 *          RI_ERR_INVALID_ADDR    Invalid pointer supplied.
 *          RI_ERR_INVALID_PARAM   Invalid parameter (offset) supplied.
 *          RI_ERR_INVALID_STATE   Invalid Connection State or no authorization request pending.
 *          RI_ERR_INVALID_LENGTH  Invalid length supplied.
 *          RI_ERR_BUSY            Procedure already in progress.
 *          RIBLE_ERR_ATT_INVALID_HANDLE     Attribute not found.
 * @note    This call should only be used as a response to a RIBLE_GATTS_EVT_READ/WRITE_PERMIT_REQ
 * event issued to the application.
 * */
__WEAK rible_status_t rible_gatts_rw_auth_reply(uint16_t conn_handle,
        uint8_t status, uint16_t char_value_handle, uint8_t offset,
        uint8_t* p_value, uint8_t len, uint8_t type)
{
    return RI_SUCCESS;
}

__WEAK rible_status_t rible_log_printf(const char * sFormat, ...)
{
    return RI_SUCCESS;
}

__WEAK rible_status_t rible_log_hexdump(void* array_base, uint16_t array_size)
{
    return RI_SUCCESS;
}
