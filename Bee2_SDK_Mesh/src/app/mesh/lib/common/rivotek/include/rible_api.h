#ifndef RIVOTEK_BLE_API_H__
#define RIVOTEK_BLE_API_H__

#include "rible_type.h"

rible_status_t rible_gap_address_get(rible_addr_t mac);

//advertising
rible_status_t rible_gap_adv_start(rible_gap_adv_param_t *p_adv_param);

rible_status_t rible_gap_adv_data_set(uint8_t const * p_data, uint8_t dlen,
        uint8_t const *p_sr_data, uint8_t srdlen);

rible_status_t rible_gap_adv_stop(void);

rible_status_t rible_gatts_service_init(
        rible_gatts_db_t *rible_service_database);

rible_status_t rible_gatts_value_set(uint16_t srv_handle, uint16_t char_handle,
        uint8_t offset, uint8_t* buf, uint8_t len);

rible_status_t rible_gatts_value_get(uint16_t srv_handle, uint16_t char_handle,
        uint8_t* pdata, uint8_t *plen);

rible_status_t rible_gatts_notify_or_indicate(uint16_t conn_handle,
        uint16_t srv_handle, uint16_t char_value_handle, uint8_t offset,
        uint8_t* p_value, uint8_t len, uint8_t type);

rible_status_t rible_gatts_rw_auth_reply(uint16_t conn_handle, uint8_t status,
        uint16_t char_value_handle, uint8_t offset, uint8_t* p_value,
        uint8_t len, uint8_t type);

// logging
rible_status_t rible_log_printf(const char * sFormat, ...);

rible_status_t rible_log_hexdump(void* array_base, uint16_t array_size);

#endif
