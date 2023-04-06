/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     dfu_service.h
* @brief
* @details
* @author   ken_mei
* @date     02-09-2016
* @version  v1.0.0
******************************************************************************
* @attention
* <h2><center>&copy; COPYRIGHT 2015 Realtek Semiconductor Corporation</center></h2>
******************************************************************************
*/

#ifndef _DFU_SERVICE_H_
#define _DFU_SERVICE_H_

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

#include "profile_server.h"
#include "flash_device.h"

#define  GATT_UUID128_DFU_PACKET        0x12, 0xA2, 0x4D, 0x2E, 0xFE, 0x14, 0x48, 0x8e, 0x93, 0xD2, 0x17, 0x3C, 0x87, 0x63, 0x00, 0x00
#define  GATT_UUID128_DFU_CONTROL_POINT 0x12, 0xA2, 0x4D, 0x2E, 0xFE, 0x14, 0x48, 0x8e, 0x93, 0xD2, 0x17, 0x3C, 0x87, 0x64, 0x00, 0x00


/*each control point procedure*/
#define DFU_OPCODE_MIN                          0x00
#define DFU_OPCODE_START_DFU                    0x01
#define DFU_OPCODE_RECEIVE_FW_IMAGE_INFO        0x02
#define DFU_OPCODE_VALID_FW                     0x03
#define DFU_OPCODE_ACTIVE_IMAGE_RESET           0x04
#define DFU_OPCODE_SYSTEM_RESET                 0x05
#define DFU_OPCODE_REPORT_TARGET_INFO           0x06
#define DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ      0x07

#define DFU_OPCODE_PKT_RX_NOTIFICATION_VOICE    0x08
#define DFU_OPCODE_BUFFER_CHECK_EN              0x09 /*report current ota function version information*/
#define DFU_OPCODE_REPORT_BUFFER_CRC            0x0a /*report current buffer CRC*/

#define DFU_OPCODE_RECEIVE_IC_TYPE              0x0b
#define DFU_OPCODE_COPY_IMG                     0x0c
#define DFU_OPCODE_MAX                          0x0d

/*length of each control point procedure*/
#define DFU_LENGTH_START_DFU                    (1+12+4)/*4 bytes is padding for encrypt*/
#define DFU_LENGTH_RECEIVE_FW_IMAGE_INFO        (1+2+4) //img_id + cur_offset
#define DFU_LENGTH_VALID_FW                     (1+2)   //img_id
#define DFU_LENGTH_ACTIVE_IMAGE_RESET           0x01
#define DFU_LENGTH_SYSTEM_RESET                 0x01
#define DFU_LENGTH_REPORT_TARGET_INFO           (1+2) //img_id
#define DFU_LENGTH_CONN_PARA_TO_UPDATE_REQ      (1+2+2+2+2) //conn_interval_min,conn_interval_max,conn_latency,superv_tout
#define DFU_LENGTH_BUFFER_CHECK_EN              (1)
#define DFU_LENGTH_REPORT_BUFFER_CRC            (1+2+2) //buf_size, buf_crc


/*notification opcode*/
#define DFU_OPCODE_NOTIFICATION                 0x10

#define DFU_NOTIFY_ENABLE                       1
#define DFU_NOTIFY_DISABLE                      2

/*length of notification*/
#define DFU_NOTIFY_LENGTH_ARV                   3  //others opcode notification length
//#define DFU_NOTIFY_LENGTH_START_DFU             (DFU_NOTIFY_LENGTH_ARV)
//#define DFU_NOTIFY_LENGTH_VALID_FW              (DFU_NOTIFY_LENGTH_ARV)
#define DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO    (DFU_NOTIFY_LENGTH_ARV+4+4)  //img_ver, cur_offset
#define DFU_NOTIFY_LENGTH_BUFFER_CHECK_EN       (DFU_NOTIFY_LENGTH_ARV+2+2)  //buf_size, mtu_size
#define DFU_NOTIFY_LENGTH_REPORT_BUFFER_CRC     (DFU_NOTIFY_LENGTH_ARV+4)   //cur_offset
#define DFU_NOTIFY_LENGTH_RECEIVE_IC_TYPE       (DFU_NOTIFY_LENGTH_ARV+1)   //ic_type

/*max length*/
#define DFU_NOTIFY_LENGTH_MAX                   DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO

/*attribut index*/
#define INDEX_DFU_PACKET_VALUE                  0x02
#define INDEX_DFU_CONTROL_POINT_CHAR_VALUE      0x04
#define INDEX_DFU_CHAR_CCCD_INDEX               0x05


/*dfu error code*/
#define DFU_ARV_SUCCESS                         0x01
#define DFU_ARV_FAIL_INVALID_PARAMETER          0x02
#define DFU_ARV_FAIL_OPERATION                  0x03
#define DFU_ARV_FAIL_DATA_SIZE_EXCEEDS_LIMIT    0x04
#define DFU_ARV_FAIL_CRC_ERROR                  0x05
#define DFU_ARV_FAIL_LENGTH_ERROR               0x06
#define DFU_ARV_FAIL_PROG_ERROR                 0x07
#define DFU_ARV_FAIL_ERASE_ERROR                0x08


typedef void (*P_FUN_DFU_OPCODE_CB)(uint8_t opcode);

typedef struct _T_START_DFU_PARA
{
    uint8_t ic_type;
    uint8_t secure_version;
    union
    {
        uint16_t value;
        struct
        {
            uint16_t xip: 1; // payload is executed on flash
            uint16_t enc: 1; // all the payload is encrypted
            uint16_t load_when_boot: 1; // load image when boot
            uint16_t enc_load: 1; // encrypt load part or not
            uint16_t enc_key_select: 3; // referenced to ENC_KEY_SELECT
            uint16_t not_ready : 1; //for copy image in ota
            uint16_t not_obsolete : 1; //for copy image in ota
            uint16_t rsvd: 7;
        };
    } ctrl_flag;
    uint16_t signature;
    uint16_t crc16;
    uint32_t image_length;
} T_START_DFU_PARA;

typedef struct _T_PKT_RX_NOTIF_REQ
{
    uint16_t packet_num;
} T_PKT_RX_NOTIF_REQ;


typedef struct _T_DFU_CTRL_POINT
{
    uint8_t opcode;
    union
    {
        T_START_DFU_PARA start_dfu;
        T_PKT_RX_NOTIF_REQ pkt_rx_notify_req;
    } p;
} T_DFU_CTRL_POINT, * P_DFU_CTRL_POINT;

typedef struct
{
    uint32_t origin_image_version;

    uint32_t cur_offset;
    uint32_t image_total_length;

    uint8_t ic_type;
    uint8_t secure_version;
    union
    {
        uint16_t value;
        struct
        {
            uint16_t xip: 1; // payload is executed on flash
            uint16_t enc: 1; // all the payload is encrypted
            uint16_t load_when_boot: 1; // load image when boot
            uint16_t enc_load: 1; // encrypt load part or not
            uint16_t enc_key_select: 3; // referenced to ENC_KEY_SELECT
            uint16_t not_ready : 1; //for copy image in ota
            uint16_t not_obsolete : 1; //for copy image in ota
            uint16_t rsvd: 7;
        };
    } ctrl_flag;
    uint16_t signature;
    uint16_t crc16;
    uint32_t image_length;

    bool ota_conn_para_upd_in_progress;
    uint8_t mtu_size;
} T_DFU_CB;


/*Notifications defined here*/

typedef struct _TNOTIFICATION_TARGET_IMAGE_INFO
{

    uint16_t nOrigFwVersion;
    uint32_t nImageUpdateOffset;
} TNOTIFICATION_TARGET_IMAGE_INFO;

typedef struct _TNOTIFICATION_REPORT_PKT_NUM
{
    uint16_t PacketNum;
} TNOTIFICATION_REPORT_PKT_NUM;

typedef struct _TNOTIFICATION_REPORT_OTA_FUNC
{
    uint16_t OtaFuncVersion;
    uint32_t invalid;
} TNOTIFICATION_REPORT_OTA_FUNC;


typedef enum _TNOTIFICATION_REPORT_FUNC_VERSION
{
    NORMAL_FUNCTION = 0x0000,  /*normal function*/
    IMAGE_CHECK_FUNCTION = 0x0001  /*image check function*/
} TNOTIFICATION_REPORT_FUNC_VERSION;

typedef enum _DFU_BUFFER_IS_VALID
{
    DFU_BUFFER_VALID = 0x00,
    DFU_BUFFER_INVALID = 0x01
} DFU_BUFFER_IS_VALID;

typedef struct _DFUNotification
{
    uint8_t opCode;
    uint8_t reqOpCode;
    uint8_t respValue;
    union
    {
        TNOTIFICATION_TARGET_IMAGE_INFO NotifyTargetImageInfo;
        TNOTIFICATION_REPORT_PKT_NUM NotifyPktNum;
    } p;
} TDFUNotification, * PDFUNotification;

typedef union _TDFU_UPSTREAM_MSG_DATA
{
    uint8_t notification_indification_index;
    uint8_t write_value_index;
} TDFU_UPSTREAM_MSG_DATA;
/** Dfu service data to inform application */
typedef struct _TDFU_CALLBACK_DATA
{
    uint8_t                 conn_id;
    T_SERVICE_CALLBACK_TYPE     msg_type;
    TDFU_UPSTREAM_MSG_DATA    msg_data;
} TDFU_CALLBACK_DATA;

extern T_DFU_CB g_dfu_para;
extern uint8_t *p_ota_temp_buffer_head;
extern uint16_t g_ota_tmp_buf_used_size;
extern uint16_t mBufSize;
extern uint32_t g_sil_dfu_resend_offset;

/* attribute index / client characteristic configuration descriptor (CCCD) pair */
typedef struct _T_GATT_ATTR_IDX_CCCD
{
    uint16_t    attr_idx;
    uint16_t    ccc_bits;
} T_GATT_ATTR_IDX_CCCD, * P_GATT_ATTR_IDX_CCCD;

uint8_t dfu_add_service(void *pFunc);

#ifdef __cplusplus
}
#endif

#endif
