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
#define APP_TASK_STACK_SIZE           1024*8 //256 * 8   //!<  Task stack size
#define MAX_NUMBER_OF_GAP_MESSAGE     0x20      //!<  GAP message queue size
#define MAX_NUMBER_OF_IO_MESSAGE      0x20      //!<  IO message queue size
#define MAX_NUMBER_OF_EVENT_MESSAGE   (MAX_NUMBER_OF_GAP_MESSAGE + MAX_NUMBER_OF_IO_MESSAGE)    //!< Event message queue size

#define EVENT_MI                      0x81

#define DEVICE_NAME                     "stand_demo"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Xiaomi Inc."                           /**< Manufacturer. Will be passed to Device Information Service. */

#define BLE_GATEWAY_TEST 0

/*============================================================================*
 *                              Variables
 *============================================================================*/
void *app_task_handle;   //!< APP Task handle
void *evt_queue_handle;  //!< Event queue handle
void *io_queue_handle;   //!< IO queue handle
void *mi_queue_handle;   // mi auth

void *sched_task_handle;   //!< mi schedule Task handle


typedef struct {
    mible_handler_t handler;
    void *arg;
} mi_event_t;

/*============================================================================*
 *                              Functions
 *============================================================================*/
void app_main_task(void *p_param);

extern mible_status_t mible_record_init(void);

extern void mi_schd_process();
void sched_task(void *p_param)
{
    while(1)
    {
        mi_schd_process();
    }
}

/**
 * @brief  Initialize App task
 * @return void
 */
void app_task_init()
{
    os_task_create(&app_task_handle, "app", app_main_task, 0, APP_TASK_STACK_SIZE,
                   APP_TASK_PRIORITY);
    
    os_task_create(&sched_task_handle, "sched", sched_task, 0, 2048,
                   APP_TASK_PRIORITY);
}


/*mible_std_authen*/
#include "mible_api.h"
#include "mible_log.h"
#include "common/mible_beacon_internal.h"
#include "common/mible_beacon.h"
#include "mi_config.h"
#include "standard_auth/mible_standard_auth.h"
#include "mijia_profiles/mi_service_server.h"
#include "mijia_profiles/stdio_service_server.h"

#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))
enum
{
    UNIT_0_625_MS = 625,        /**< Number of microseconds in 0.625 milliseconds. */
    UNIT_1_25_MS  = 1250,       /**< Number of microseconds in 1.25 milliseconds. */
    UNIT_10_MS    = 10000       /**< Number of microseconds in 10 milliseconds. */
};

static uint8_t qr_code[16] = {
0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,
};

extern mible_status_t mible_service_data_set(mibeacon_config_t const * const config, uint8_t *p_output, uint8_t *p_output_len);

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(bool solicite_bind)
{
    MI_LOG_INFO("advertising init...\n");

    uint8_t user_data[31], user_dlen;
    user_data[0] = 1 + strlen(DEVICE_NAME);
    user_data[1] = 9;  // complete local name
    strcpy((char*)&user_data[2], DEVICE_NAME);
    user_dlen = 2 + strlen(DEVICE_NAME);
    if (MI_SUCCESS != mibeacon_adv_data_set(solicite_bind, 0, user_data, user_dlen)) {
        MI_LOG_ERROR("encode mibeacon data failed. \r\n");
    }
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t errno = mibeacon_adv_start(300);
    MI_ERR_CHECK(errno);
}

void mi_schd_event_handler(schd_evt_t *p_event)
{
    MI_LOG_INFO("USER CUSTOM CALLBACK RECV EVT ID %d\n", p_event->id);
    switch (p_event->id) {
    case SCHD_EVT_OOB_REQUEST:
        MI_LOG_INFO("App selected IO cap is 0x%04X\n", p_event->data.IO_capability);
        switch (p_event->data.IO_capability) {
        case 0x0080:
            mi_schd_oob_rsp(qr_code, 16);
            MI_LOG_INFO(MI_LOG_COLOR_GREEN "Please scan device QR code.\n");
            break;

        default:
            MI_LOG_ERROR("Selected IO cap is not supported.\n");
            mible_gap_disconnect(0);
        }
        break;

    case SCHD_EVT_KEY_DEL_SUCC:
        // device has been reset, restart adv mibeacon contains IO cap.
        advertising_init(0);
        break;

    default:
        break;
    }
}

void stdio_rx_handler(uint8_t* p, uint8_t l)
{
    int errno;
    /* RX plain text (It has been decrypted) */
    MI_LOG_INFO("RX raw data\n");
    MI_LOG_HEXDUMP(p, l);

    /* TX plain text (It will be encrypted before send out.) */
    errno = stdio_tx(p, l);
    MI_ERR_CHECK(errno);
}


#if BLE_GATEWAY_TEST
static void * gatewaytest_timer;
static void enqueue_new_objs(void)
{
    static int16_t temp;
    static int16_t hum;
    static int8_t  battery;

    temp = temp < 500 ? temp + 1 : -500;
    mibeacon_obj_enque(MI_STA_TEMPERATURE, sizeof(temp), &temp, 0);

    hum = hum < 1000 ? hum + 1 : 0;
    mibeacon_obj_enque(MI_STA_HUMIDITY, sizeof(hum), &hum, 0);

    battery = battery < 100 ? battery + 1 : 0;
    mibeacon_obj_enque(MI_STA_BATTERY, sizeof(battery), &battery, 0);
}
static void gatewaytest_handler(void * p_context)
{
    enqueue_new_objs();
}
#endif

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
    os_msg_queue_create(&mi_queue_handle, 2, sizeof(mi_event_t));

    gap_start_bt_stack(evt_queue_handle, io_queue_handle, MAX_NUMBER_OF_GAP_MESSAGE);

    driver_init();

    /* Add mijia auth code */
    mible_record_init();
    advertising_init(0);

    /* <!> mi_scheduler_init() must be called after ble_stack_init(). */
    mi_scheduler_init(10, mi_schd_event_handler, NULL);
    mi_scheduler_start(SYS_KEY_RESTORE);
    
    mi_service_init();
    stdio_service_init(stdio_rx_handler);

    // Start execution.
    //application_timers_start();
    advertising_start();
		
#if BLE_GATEWAY_TEST
    uint32_t delay_ms=10000;
    MI_LOG_INFO("start object adv after %d ms\n", delay_ms);
    mible_timer_create(&gatewaytest_timer,gatewaytest_handler,MIBLE_TIMER_REPEATED);
    mible_timer_start(gatewaytest_timer,delay_ms,NULL);
#endif

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
            else if (event == EVENT_MI)
            {
                mible_tasks_exec();
            }
            else
            {
                gap_handle_msg(event);
            }
        }
    }
}

/** @} */ /* End of group PERIPH_APP_TASK */

mible_status_t mible_task_post(mible_handler_t handler, void *arg)
{
    mi_event_t event;
    uint8_t event_mi = EVENT_MI;

    event.handler = handler;
    event.arg = arg;
    if (os_msg_send(mi_queue_handle, &event, 0))
    {
        os_msg_send(evt_queue_handle, &event_mi, 0);
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

    if (os_msg_recv(mi_queue_handle, &event, 0) &&
        NULL != event.handler) {

        event.handler(event.arg);
    }
}

