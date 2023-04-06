/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      app_task.c
   * @brief     Routines to create App task and handle events & messages
   * @author    jane
   * @date      2017-06-02
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <os_msg.h>
#include <os_task.h>
#include <gap.h>
#include <gap_le.h>
#include <app_task.h>
#include <app_msg.h>
#include <app_task.h>
#include <peripheral_app.h>
#include <string.h>
#include "mible_type.h"

/** @defgroup  PERIPH_APP_TASK Peripheral App Task
    * @brief This file handles the implementation of application task related functions.
    *
    * Create App task and handle events & messages
    * @{
    */
/*============================================================================*
 *                              Macros
 *============================================================================*/
#define APP_TASK_PRIORITY             1         //!< Task priorities
#define APP_TASK_STACK_SIZE           256 * 8   //!<  Task stack size
#define MAX_NUMBER_OF_GAP_MESSAGE     0x20      //!<  GAP message queue size
#define MAX_NUMBER_OF_IO_MESSAGE      0x20      //!<  IO message queue size
#define MAX_NUMBER_OF_EVENT_MESSAGE   (MAX_NUMBER_OF_GAP_MESSAGE + MAX_NUMBER_OF_IO_MESSAGE)    //!< Event message queue size

#define EVENT_MI                      0x81

/*============================================================================*
 *                              Variables
 *============================================================================*/
void *app_task_handle;   //!< APP Task handle ,system task
void *evt_queue_handle;  //!< Event queue handle
void *io_queue_handle;   //!< IO queue handle

void *mible_task_handle;	// mible api task
void *mi_queue_handle;   // mi auth
void *gap_lock_seg_handle = NULL;
void *gap_wait_seg_handle = NULL;

typedef struct {
    mible_handler_t handler;
    void *arg;
} mi_event_t;

/*============================================================================*
 *                              Functions
 *============================================================================*/
void app_main_task(void *p_param);
void mible_task(void *p_param);
extern mible_status_t mible_record_init(void);

/**
 * @brief  Initialize App task
 * @return void
 */
void app_task_init()
{
    os_task_create(&app_task_handle, "app", app_main_task, 0, APP_TASK_STACK_SIZE,
                   APP_TASK_PRIORITY);
	
		os_task_create(&mible_task_handle, "mible", mible_task, 0, APP_TASK_STACK_SIZE,
                   APP_TASK_PRIORITY);
}


/*mible_std_authen*/
#include "mible_api.h"
#include "mible_log.h"
#include "mible_beacon.h"
#include "mible_server.h"
#include "mi_config.h"
#include "rtl876x_gpio.h"
#include "rtl876x_nvic.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"

#include "board.h"

//Push Button2 -- FAST_PAIR Advertising last time -- 5s
#define ADV_FAST_PAIR_TIME 5000
//Advertising interval time -- 100ms
#define ADV_INTERVAL_TIME 100

static void* fastpair_timer;

#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))
enum
{
    UNIT_0_625_MS = 625,        /**< Number of microseconds in 0.625 milliseconds. */
    UNIT_1_25_MS  = 1250,       /**< Number of microseconds in 1.25 milliseconds. */
    UNIT_10_MS    = 10000       /**< Number of microseconds in 10 milliseconds. */
};

device_info dev_info = {
        .bonding = WEAK_BONDING,//,STRONG_BONDING // can be modified according to product
        .pid = PRODUCT_ID, //,930 // product id, can be modified according to product
        .version = "0000",  // can be modified according to product
};

static mible_op_mode_t MIBLE_FUNCTION = MODE_REMOTE;

/**@brief Function for advertising.
 */
void advertising_stop(void *arg)
{
		mible_status_t status;
    MI_LOG_INFO("advertising stop...\n");
    status = mible_gap_adv_stop();
		if(MI_SUCCESS != status){
        MI_LOG_ERROR("stop adv failed (err %d) \r\n", status);
        return;
    }
}

static void push_key_mibeacon(uint8_t key)
{		
		uint8_t key_val[3] = {key, 0, 0};
		mibeacon_obj_enque_oneshot(MI_STA_BUTTON, 3, key_val);
}

static void ble_fastpair_event(void)
{
    mible_status_t status;
    
    MI_LOG_INFO("ble_fastpair advertising init...\n");
    status = mible_timer_stop(fastpair_timer);
    if (MI_SUCCESS != status) {
        MI_LOG_WARNING("stop fastpair timer fail (err %d)\n", status);
        return;
    }
		
		status = mible_timer_start(fastpair_timer, ADV_FAST_PAIR_TIME, NULL);
    if(MI_SUCCESS != status){
        MI_LOG_ERROR("start fastpair timer failed (err %d) \r\n", status);
        return;
    }

		mible_addr_t dev_mac;
    mibeacon_frame_ctrl_t frame_ctrl = {
        .is_encrypt = 0,
        .mac_include = 1,
        .cap_include = 1,
        .obj_include = 1,
        .bond_confirm = 1,
        .version = 0x03,
    };
    mibeacon_capability_t cap = {.connectable = 1,
                                 .encryptable = 1,
                                 .bondAbility = 1};
		
		status = mible_gap_address_get(dev_mac);
    if (MI_SUCCESS != status) {
        MI_LOG_WARNING("get device MAC fail (err %d)\n", status);
        return;
    }
    mibeacon_obj_t fastpair_obj = {.type = MI_EVT_SIMPLE_PAIR,
                                   .len = 2,
                                   .val[0] = 0x01,
                                   .val[1] = 0x10,};
    mibeacon_config_t mibeacon_cfg = {
        .frame_ctrl = frame_ctrl,
        .pid =dev_info.pid,
        .p_mac = (mible_addr_t*)dev_mac, 
        .p_capability = &cap,
        .p_obj = (mibeacon_obj_t*)&fastpair_obj,
        .obj_num = 1,
    };
 
    uint8_t service_data[31] = {0};
    uint8_t service_data_len = 0;

    status = fastpair_data_set(&mibeacon_cfg, service_data, &service_data_len);
    if(MI_SUCCESS != status){
        MI_LOG_ERROR("fastpair_data_set failed (err %d) \r\n", status);
        return;
    }
		
    if (0 == service_data_len || service_data_len > 28) {
        MI_LOG_ERROR("service data len is %d \r\n", service_data_len);
        return;
    }
    
    uint8_t adv_data[31];
		uint8_t adv_len = 3;
		*adv_data = 2;
		*(adv_data+1) = 1;
		*(adv_data+2) = 6;
      
    memcpy(adv_data+3, service_data, service_data_len);
    adv_len += service_data_len;
		
		MI_LOG_INFO("fastpair adv data");
		MI_LOG_HEXDUMP(adv_data, adv_len);
		status = mible_gap_adv_data_set(adv_data, adv_len, NULL, 0);
    if(MI_SUCCESS != status){
        MI_LOG_ERROR("set adv data failed (err %d) \r\n", status);
        return;
    }
		
		mible_gap_adv_param_t adv_param = {
        .adv_type = MIBLE_ADV_TYPE_CONNECTABLE_UNDIRECTED,
        .adv_interval_min = MSEC_TO_UNITS(ADV_INTERVAL_TIME, UNIT_0_625_MS),
        .adv_interval_max = MSEC_TO_UNITS(ADV_INTERVAL_TIME, UNIT_0_625_MS),
        .ch_mask = {0},
    };
		MI_LOG_INFO("advertising start...\n");
		status = mible_gap_adv_start(&adv_param);
		if(MI_SUCCESS != status){
        MI_LOG_ERROR("start adv failed (err %d) \r\n", status);
        return;
    }
}


void mible_service_init_cmp(void)
{
    MI_LOG_INFO("mible_service_init_cmp\r\n");
}

void mible_connected(void)
{
    MI_LOG_INFO("mible_connected \r\n");
}

void mible_disconnected(void)
{
    MI_LOG_INFO("mible_disconnected \r\n");
    //advertising_init();
}

void mible_bonding_evt_callback(mible_bonding_state state)
{
    if(state == BONDING_FAIL){
        MI_LOG_INFO("BONDING_FAIL\r\n");
        mible_gap_disconnect(mible_server_connection_handle);
    }else if(state == BONDING_SUCC){
        MI_LOG_INFO("BONDING_SUCC\r\n");
    }else if(state == LOGIN_FAIL){
        MI_LOG_INFO("LOGIN_FAIL\r\n");
        mible_gap_disconnect(mible_server_connection_handle);
    }else{
        MI_LOG_INFO("LOGIN_SUCC\r\n");
    }
}

void std_authen_event_cb(mible_std_auth_evt_t evt,
        mible_std_auth_evt_param_t* p_param)
{
    switch(evt){
    case MIBLE_STD_AUTH_EVT_SERVICE_INIT_CMP:
        mible_service_init_cmp();
        break;
    case MIBLE_STD_AUTH_EVT_CONNECT:
        mible_connected();
        break;
    case MIBLE_STD_AUTH_EVT_DISCONNECT:
        mible_disconnected();
        break;
    case MIBLE_STD_AUTH_EVT_RESULT:
        mible_bonding_evt_callback(p_param->result.state);
        break;
    default:
        MI_LOG_ERROR("Unkown std authen event\r\n");
        break;
    }
}
/**
 * @brief        App task to handle events & messages
 * @param[in]    p_param    Parameters sending to the task
 * @return       void
 */
void app_main_task(void *p_param)
{
    uint8_t event;
    os_msg_queue_create(&io_queue_handle, MAX_NUMBER_OF_IO_MESSAGE, sizeof(T_IO_MSG));
    os_msg_queue_create(&evt_queue_handle, MAX_NUMBER_OF_EVENT_MESSAGE, sizeof(uint8_t));
    
    gap_start_bt_stack(evt_queue_handle, io_queue_handle, MAX_NUMBER_OF_GAP_MESSAGE);

    while (true)
    {
        if (os_msg_recv(evt_queue_handle, &event, 0xFFFFFFFF) == true)
        {
            if (event == EVENT_IO_TO_APP)
            {
                T_IO_MSG io_msg;
                if (os_msg_recv(io_queue_handle, &io_msg, 0) == true)
                {
                    app_handle_io_msg(io_msg);
                }
            }
            else
            {
                gap_handle_msg(event);
            }
        }
    }
}
/** @} */ /* End of group PERIPH_APP_TASK */

void mible_task(void *p_param)
{
		mible_status_t ret;
	
		os_sem_create(&gap_lock_seg_handle,1,1);
		os_sem_create(&gap_wait_seg_handle,0,1);
		os_msg_queue_create(&mi_queue_handle, 10, sizeof(mi_event_t));
	
		mible_record_init();	//init record
		driver_init();				//init gpio
		
		/* Add mijia auth code */
    mible_std_auth_evt_register(std_authen_event_cb);       
    mible_server_info_init(&dev_info, MIBLE_FUNCTION/*STD_AUTHEN or REMOTE_CONTROL*/);
    mible_server_miservice_init();
	
    ret = mible_timer_create(&fastpair_timer, advertising_stop, MIBLE_TIMER_SINGLE_SHOT);
    if(ret != MI_SUCCESS){
        MI_LOG_ERROR("fastpair_timer_create failed. code = %x .\r\n",ret);
    }else{
        MI_LOG_DEBUG("fastpair_timer_create success. \r\n");
    }
		
		while (true)
    {
				mible_tasks_exec();
		}
}

mible_status_t mible_task_post(mible_handler_t handler, void *arg)
{
    mi_event_t event;

    event.handler = handler;
    event.arg = arg;
    if (os_msg_send(mi_queue_handle, &event, 0))
    {
        return MI_SUCCESS;
    }
    else
    {
        return MI_ERR_INTERNAL;
    }
}

void mible_tasks_exec(void)
{
    mi_event_t event;

    if (os_msg_recv(mi_queue_handle, &event, 0xFFFFFFFF) &&
        NULL != event.handler) {

        event.handler(event.arg);
    }
}

static void app_key_handler(void *arg)
{
		int index = (int)arg;

    MI_LOG_DEBUG("Key index -- %d \r\n", index);
    if (2 == index) {		
        ble_fastpair_event();
    } else {
        push_key_mibeacon(index);
    }
}

void GPIO28_Handler(void)
{
    /*key_0 handler*/
    GPIO_INTConfig(GPIO_GetPin(P4_0), DISABLE);
    GPIO_MaskINTConfig(GPIO_GetPin(P4_0), ENABLE);

    mible_task_post(app_key_handler, (void *)0);
    GPIO_ClearINTPendingBit(GPIO_GetPin(P4_0));

    GPIO_MaskINTConfig(GPIO_GetPin(P4_0), DISABLE);
    GPIO_INTConfig(GPIO_GetPin(P4_0), ENABLE);
}

void GPIO29_Handler(void)
{
    /*key_1 handler*/
    GPIO_INTConfig(GPIO_GetPin(P4_1), DISABLE);
    GPIO_MaskINTConfig(GPIO_GetPin(P4_1), ENABLE);

    mible_task_post(app_key_handler, (void *)1);
    GPIO_ClearINTPendingBit(GPIO_GetPin(P4_1));

    GPIO_MaskINTConfig(GPIO_GetPin(P4_1), DISABLE);
    GPIO_INTConfig(GPIO_GetPin(P4_1), ENABLE);
}

void GPIO20_Handler(void)
{
    /*key_2 handler*/
    GPIO_INTConfig(GPIO_GetPin(P2_4), DISABLE);
    GPIO_MaskINTConfig(GPIO_GetPin(P2_4), ENABLE);

    mible_task_post(app_key_handler, (void *)2);
    GPIO_ClearINTPendingBit(GPIO_GetPin(P2_4));

    GPIO_MaskINTConfig(GPIO_GetPin(P2_4), DISABLE);
    GPIO_INTConfig(GPIO_GetPin(P2_4), ENABLE);
}


