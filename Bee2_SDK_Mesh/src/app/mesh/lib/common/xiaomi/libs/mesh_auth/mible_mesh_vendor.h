/*
 * mible_mesh_vendor.h
 *
 *  Created on: 2020Äê12ÔÂ10ÈÕ
 *      Author: mi
 */

#ifndef MIJIA_BLE_LIBS_MESH_AUTH_MIBLE_MESH_VENDOR_H_
#define MIJIA_BLE_LIBS_MESH_AUTH_MIBLE_MESH_VENDOR_H_

#define MIBLE_MESH_NODE_SUB_MAX_NUM     10
#define MIBLE_MESH_NODE_PRESUB_MAX_NUM  3

#if defined ( __CC_ARM )
__PACKED typedef struct {
    uint8_t siid;
    uint8_t piid;
}vendor_mesh_get_t;

__PACKED typedef struct {
    uint8_t siid;
    uint8_t piid;
    uint8_t payload[4];
    uint8_t tid;
    uint8_t type;
}vendor_mesh_state_t;

__PACKED typedef struct {
    uint8_t siid;
    uint8_t piid;
    uint8_t tid;
    uint8_t retry_times;
}vendor_mesh_sync_t;

__PACKED typedef struct {
    uint8_t siid;
    uint8_t piid;
    uint8_t tid;
    uint8_t type;
    uint8_t payload[4];
}vendor_mesh_sync_ack_t;

__PACKED typedef struct {
    uint8_t siid;
    uint8_t eiid;
    uint8_t tid;
    uint8_t piid;
    uint8_t payload[4];
}vendor_mesh_event_t;

__PACKED typedef struct {
    uint8_t siid;
    uint8_t eiid;
    uint8_t tid;
    uint8_t arg[];
}vendor_mesh_event_tlv_t;

__PACKED typedef struct {
    uint8_t siid;
    uint8_t aiid;
    uint8_t tid;
    uint8_t piid;
    uint8_t payload[4];
}vendor_mesh_action_t;

__PACKED typedef union {
    vendor_mesh_get_t       get;
    vendor_mesh_state_t     state;
    vendor_mesh_sync_t      sync;
    vendor_mesh_sync_ack_t  sync_ack;
    vendor_mesh_event_t     event;
    vendor_mesh_event_tlv_t event_tlv;
    vendor_mesh_action_t    action;
}mible_mesh_vendor_message_t;

__PACKED typedef struct {
    uint16_t scene_id;
    uint8_t siid;
    uint8_t piid;
    uint8_t payload[4];
}vendor_mesh_scene_store_t;

__PACKED typedef struct {
    uint16_t scene_id;
    uint8_t siid;
    uint8_t piid;
}vendor_mesh_scene_delete_t;

__PACKED typedef struct {
    uint16_t scene_id;
    uint8_t tid;
}vendor_mesh_scene_recall_t;

__PACKED typedef struct {
    uint16_t scene_id;
    uint8_t tid;
    uint8_t position;
    uint8_t mode;
}vendor_mesh_scene_pos_t;

__PACKED typedef struct {
    uint16_t scene_id;
    uint8_t siid;
    uint8_t piid;
    uint8_t code;
}vendor_mesh_scene_state_t;

__PACKED typedef union {
    vendor_mesh_scene_state_t       state;
    vendor_mesh_scene_store_t       store;
    vendor_mesh_scene_delete_t      del;
    vendor_mesh_scene_recall_t      recall;
    vendor_mesh_scene_pos_t         pos;
}mible_mesh_scene_message_t;

__PACKED typedef struct {
    uint16_t primary_grp_addr;
} simple_subscribe_rsp_t;

__PACKED typedef struct {
    uint8_t tid;
} discover_node_req_t;

__PACKED typedef struct {
    uint8_t reserved[1];
} discover_node_rsp_t;

__PACKED typedef struct {
    uint8_t tid;
    uint8_t retry;
} net_param_req_t;

__PACKED typedef struct {
    uint8_t tid;
    uint8_t pub_interval;
}net_param_rsp_t;

__PACKED typedef struct {
    uint8_t type;
    __PACKED union {
        simple_subscribe_rsp_t   sub_rsp;
        discover_node_req_t discover_req;
        discover_node_rsp_t discover_rsp;
        net_param_req_t     net_para_req;
        net_param_rsp_t     net_para_rsp;
    } value;
}vendor_mesh_config_t;

__PACKED typedef struct {
    uint8_t tid;
    uint16_t group_list[MIBLE_MESH_NODE_SUB_MAX_NUM];
}node_group_sync_t;

__PACKED typedef struct {
    uint8_t tid;
    uint16_t group_list[MIBLE_MESH_NODE_PRESUB_MAX_NUM];
}node_pre_group_sync_t;

__PACKED typedef struct {
    uint8_t tid;
    uint8_t enable;
    uint8_t comb;
    uint16_t min;
    uint16_t max;
}node_group_ack_t;

__PACKED typedef struct {
    uint8_t tid;
    uint8_t relay;
    uint8_t cnt_step;
}node_relay_config_t;

__PACKED typedef struct {
    uint8_t tid;
    uint8_t ttl;
    uint8_t cnt_step;
}node_net_config_t;

__PACKED typedef struct {
    uint8_t tid;
    uint16_t window;
    uint16_t interval;
}node_scan_config_t;

__PACKED typedef struct {
    uint8_t tid;
    int16_t code;
}node_error_t;

__PACKED typedef struct {
    uint8_t type;
    __PACKED union {
        node_group_sync_t       group;
        node_pre_group_sync_t   pre_group;
        node_group_ack_t        ack;
        node_relay_config_t     relay;
        node_net_config_t       net;
        node_scan_config_t      scan;
        node_error_t            error;
    } value;
}mible_mesh_dev_config_t;

__PACKED typedef struct {
    uint8_t payload[8];     //power up: iv_flag+iv index[4]+seq[3]
}mible_mesh_dev_trace_t;

#elif defined   ( __GNUC__ )
typedef struct __PACKED{
    uint8_t siid;
    uint8_t piid;
}vendor_mesh_get_t;

typedef struct __PACKED{
    uint8_t siid;
    uint8_t piid;
    uint8_t payload[4];
    uint8_t tid;
    uint8_t type;
}vendor_mesh_state_t;

typedef struct __PACKED{
    uint8_t siid;
    uint8_t piid;
    uint8_t tid;
    uint8_t retry_times;
}vendor_mesh_sync_t;

typedef struct __PACKED{
    uint8_t siid;
    uint8_t piid;
    uint8_t tid;
    uint8_t type;
    uint8_t payload[4];
}vendor_mesh_sync_ack_t;

typedef struct __PACKED{
    uint8_t siid;
    uint8_t eiid;
    uint8_t tid;
    uint8_t piid;
    uint8_t payload[4];
}vendor_mesh_event_t;

typedef struct __PACKED{
    uint8_t siid;
    uint8_t eiid;
    uint8_t tid;
    uint8_t arg[];
}vendor_mesh_event_tlv_t;

typedef struct __PACKED{
    uint8_t siid;
    uint8_t aiid;
    uint8_t tid;
    uint8_t piid;
    uint8_t payload[4];
}vendor_mesh_action_t;

typedef union __PACKED{
    vendor_mesh_get_t       get;
    vendor_mesh_state_t     state;
    vendor_mesh_sync_t      sync;
    vendor_mesh_sync_ack_t  sync_ack;
    vendor_mesh_event_t     event;
    vendor_mesh_event_tlv_t event_tlv;
    vendor_mesh_action_t    action;
}mible_mesh_vendor_message_t;

typedef struct __PACKED{
    uint16_t scene_id;
    uint8_t siid;
    uint8_t piid;
    uint8_t payload[4];
}vendor_mesh_scene_store_t;

typedef struct __PACKED{
    uint16_t scene_id;
    uint8_t siid;
    uint8_t piid;
}vendor_mesh_scene_delete_t;

typedef struct __PACKED{
    uint16_t scene_id;
    uint8_t tid;
}vendor_mesh_scene_recall_t;

typedef struct __PACKED{
    uint16_t scene_id;
    uint8_t tid;
    uint8_t position;
    uint8_t mode;
}vendor_mesh_scene_pos_t;

typedef struct __PACKED{
    uint16_t scene_id;
    uint8_t siid;
    uint8_t piid;
	uint8_t code;
}vendor_mesh_scene_state_t;

typedef union __PACKED{
    vendor_mesh_scene_store_t       store;
    vendor_mesh_scene_delete_t      del;
    vendor_mesh_scene_recall_t      recall;
    vendor_mesh_scene_state_t       state;
    vendor_mesh_scene_pos_t         pos;
}mible_mesh_scene_message_t;

typedef struct __PACKED{
    uint16_t primary_grp_addr;
} simple_subscribe_rsp_t;

typedef struct __PACKED{
    uint8_t tid;
} discover_node_req_t;

typedef struct __PACKED{
    uint8_t reserved[1];
} discover_node_rsp_t;

typedef struct __PACKED{
    uint8_t tid;
    uint8_t retry;
} net_param_req_t;

typedef struct __PACKED{
    uint8_t tid;
    uint8_t pub_interval;
}net_param_rsp_t;

typedef struct __PACKED{
    uint8_t type;
    union __PACKED{
        simple_subscribe_rsp_t   sub_rsp;
        discover_node_req_t discover_req;
        discover_node_rsp_t discover_rsp;
        net_param_req_t     net_para_req;
        net_param_rsp_t     net_para_rsp;
    } value;
}vendor_mesh_config_t;

typedef struct __PACKED{
    uint8_t tid;
    uint16_t group_list[MIBLE_MESH_NODE_SUB_MAX_NUM];
}node_group_sync_t;

typedef struct __PACKED{
    uint8_t tid;
    uint16_t group_list[MIBLE_MESH_NODE_SUB_MAX_NUM];
}node_pre_group_sync_t;

typedef struct __PACKED{
    uint8_t tid;
    uint8_t enable;
    uint8_t comb;
    uint16_t min;
    uint16_t max;
}node_group_ack_t;

typedef struct __PACKED{
    uint8_t tid;
    uint8_t relay;
    uint8_t cnt_step;
}node_relay_config_t;

typedef struct __PACKED{
    uint8_t tid;
    uint8_t ttl;
    uint8_t cnt_step;
}node_net_config_t;

typedef struct __PACKED{
    uint8_t tid;
    uint16_t window;
    uint16_t interval;
}node_scan_config_t;

typedef struct __PACKED{
    uint8_t tid;
    int16_t code;
}node_error_t;

typedef struct __PACKED{
    uint8_t type;
    union __PACKED{
        node_group_sync_t       group;
        node_pre_group_sync_t   pre_group;
        node_group_ack_t        ack;
        node_relay_config_t     relay;
        node_net_config_t       net;
        node_scan_config_t      scan;
        node_error_t            error;
    } value;
}mible_mesh_dev_config_t;

typedef struct __PACKED{
    uint8_t payload[8];
}mible_mesh_dev_trace_t;
#endif

#endif /* MIJIA_BLE_LIBS_MESH_AUTH_MIBLE_MESH_VENDOR_H_ */
