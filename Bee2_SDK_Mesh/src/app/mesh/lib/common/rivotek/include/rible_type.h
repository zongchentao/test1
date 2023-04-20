#ifndef RIVOTEK_BLE_TYPE_H__
#define RIVOTEK_BLE_TYPE_H__

#include "rible_commom.h"

typedef uint8_t rible_addr_t[6];

typedef uint32_t rible_cfm_t;

typedef enum {
    RI_SUCCESS      = 0x00,
    RI_ERR_INTERNAL,
    RI_ERR_NOT_FOUND,
    RI_ERR_NO_EVENT,
    RI_ERR_NO_MEM,
    RI_ERR_INVALID_ADDR,     // Invalid pointer supplied
    RI_ERR_INVALID_PARAM,    // Invalid parameter(s) supplied.
    RI_ERR_INVALID_STATE,    // Invalid state to perform operation.
    RI_ERR_INVALID_LENGTH,
    RI_ERR_DATA_SIZE,
    RI_ERR_TIMEOUT,
    RI_ERR_BUSY,
    RI_ERR_RESOURCES,
    RIBLE_ERR_INVALID_CONN_HANDLE,
    RIBLE_ERR_ATT_INVALID_ATT_HANDLE,
    RIBLE_ERR_GAP_INVALID_BLE_ADDR,
    RIBLE_ERR_GATT_INVALID_ATT_TYPE,
    RIBLE_ERR_UNKNOWN, // other ble stack errors
} rible_status_t;

//advertise parameter
typedef enum {
    RIBLE_ADV_TYPE_CONNECTABLE_UNDIRECTED,      // ADV_IND
    RIBLE_ADV_TYPE_SCANNABLE_UNDIRECTED,        // ADV_SCAN_IND
    RIBLE_ADV_TYPE_NON_CONNECTABLE_UNDIRECTED,  // ADV_NONCONN_INC
} rible_gap_adv_type_t;

typedef struct {
    uint16_t interval_min;               // Range: 0x0020 to 0x4000  Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec
    uint16_t interval_max;               // Range: 0x0020 to 0x4000  Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec
    rible_gap_adv_type_t adv_type;

    struct {
        uint8_t ch_37_off : 1;  /**< Setting this bit to 1 will turn off advertising on channel 37 */
        uint8_t ch_38_off : 1;  /**< Setting this bit to 1 will turn off advertising on channel 38 */
        uint8_t ch_39_off : 1;  /**< Setting this bit to 1 will turn off advertising on channel 39 */
    } ch_mask;
} rible_gap_adv_param_t;

//gatt service database
typedef struct {
    uint32_t type;                                     // RIBLE_UUID_16 = 0; RIBLE_UUID_128 = 1
    union {
        uint16_t uuid16;
        uint8_t uuid128[16];
    };
} rible_uuid_t;

typedef enum {
    RIBLE_PRIMARY_SERVICE = 1,
    RIBLE_SECONDARY_SERVICE,
} rible_gatts_service_t;

typedef struct{
    uint16_t reliable_write     :1;
    uint16_t writeable          :1;
} rible_gatts_char_desc_ext_prop_t;

typedef struct{
    char *string;
    uint8_t len;
} rible_gatts_char_desc_user_desc_t;

typedef struct{
    uint8_t  format;
    uint8_t  exponent;
    uint16_t unit;
    uint8_t  name_space;
    uint16_t desc;
} rible_gatts_char_desc_cpf_t;

/*
 * NOTE: if char property contains notify , then SHOULD include cccd(client characteristic configuration descriptor automatically). The same to sccd when BROADCAST enabled
 * */
typedef struct{
    rible_gatts_char_desc_ext_prop_t  *extend_prop;
    rible_gatts_char_desc_cpf_t       *char_format;     // See more details at Bluetooth SPEC 4.2 [Vol 3, Part G] Page 539
    rible_gatts_char_desc_user_desc_t *user_desc;       // read only
} rible_gatts_char_desc_db_t;

// gatts characteristic
// default:  no authentication ; no encrption; configurable authorization

typedef struct{
    rible_uuid_t char_uuid;
    uint8_t char_property;                             // See TYPE mible_gatts_char_property for details
    uint8_t *p_value;                                  // initial characteristic value
    uint8_t char_value_len;
    uint16_t char_value_handle;                        // [out] where the assigned handle be stored.
    bool is_variable_len;
    bool rd_author;                                    // read authorization. Enabel or Disable MIBLE_GATTS_READ_PERMIT_REQ event
    bool wr_author;                                    // write authorization. Enabel or Disable MIBLE_GATTS_WRITE_PERMIT_REQ event
    rible_gatts_char_desc_db_t char_desc_db;
} rible_gatts_char_db_t;

typedef struct{
    rible_gatts_service_t srv_type;                    // primary service or secondary service
    uint16_t srv_handle;                               // [out] dynamically allocated
    rible_uuid_t srv_uuid;                             // 16-bit or 128-bit uuid
    uint8_t char_num;
    rible_gatts_char_db_t *p_char_db;                  // p_char_db[charnum-1]
} rible_gatts_srv_db_t;                                // Regardless of service inclusion service

typedef struct{
    rible_gatts_srv_db_t *p_srv_db;                    // p_srv_db[srv_num]
    uint8_t srv_num;
} rible_gatts_db_t;


typedef enum {
    RIBLE_BROADCAST           = 0x01,
    RIBLE_READ                = 0x02,
    RIBLE_WRITE_WITHOUT_RESP  = 0x04,
    RIBLE_WRITE               = 0x08,
    RIBLE_NOTIFY              = 0x10,
    RIBLE_INDICATE            = 0x20,
    RIBLE_AUTH_SIGNED_WRITE   = 0x40,
} rible_gatts_char_property;

#endif
