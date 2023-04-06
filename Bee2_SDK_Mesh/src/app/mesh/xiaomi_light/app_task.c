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
#include <stdlib.h>
#include <os_msg.h>
#include <os_task.h>
#include <gap.h>
#include <gap_le.h>
#include <gap_msg.h>
#include <trace.h>
#include <app_task.h>
#include <app_msg.h>
#include <rtl876x.h>
#include <rtl876x_nvic.h>
#include <rtl876x_uart.h>
#include <rtl876x_rcc.h>
#include <rtl876x_pinmux.h>

#include "mesh_api.h"
#include "light_app.h"
#include "otp_config.h"

#include "mijia_mesh_config.h"
#if MI_MANU_TEST_ENABLE
#include "mijia_mp_cmd.h"
#endif

#if MI_USER_CMD_ENABLE
#include "mi_cmd.h"
#endif

#include "mi_config.h"
#include "mijia_mesh_app.h"
#include "patch_header_check.h"
#include "version_app.h"
#include "gatt_dfu/mible_dfu_main.h"

/*============================================================================*
 *                              Macros
 *============================================================================*/
#define EVENT_MESH                    0x80
#define EVENT_MI                      0x81
#define APP_TASK_PRIORITY             2         //!< Task priorities
#define APP_TASK_STACK_SIZE           256 * 12  //!< Task stack size
#define MAX_NUMBER_OF_GAP_MESSAGE     0x20      //!< GAP message queue size
#define MAX_NUMBER_OF_IO_MESSAGE      0x20      //!< IO message queue size
#define MAX_NUMBER_OF_EVENT_MESSAGE   (MAX_NUMBER_OF_GAP_MESSAGE + MAX_NUMBER_OF_IO_MESSAGE + MESH_INNER_MSG_NUM + MI_INNER_MSG_NUM) //!< Event message queue size

/*============================================================================*
 *                              Variables
 *============================================================================*/
void *app_task_handle;   //!< APP Task handle
static void *evt_queue_handle;  //!< Event queue handle
static void *io_queue_handle;   //!< IO queue handle

/*============================================================================*
 *                              Functions
 *============================================================================*/
extern void driver_init(void);
static void app_main_task(void *p_param);
static void app_system_info(void);
extern void get_device_id(uint8_t *pdid);

bool app_send_msg_to_apptask(T_IO_MSG *p_msg)
{
    uint8_t event = EVENT_IO_TO_APP;

    if (os_msg_send(io_queue_handle, p_msg, 0) == false)
    {
        APP_PRINT_ERROR0("send_io_msg_to_app fail");
        return false;
    }
    if (os_msg_send(evt_queue_handle, &event, 0) == false)
    {
        APP_PRINT_ERROR0("send_evt_msg_to_app fail");
        return false;
    }
    return true;
}

/**
 * @brief send uart message to app task
 * @param[in] data: data need to send
 * @return void
 */
void app_send_uart_msg(uint8_t data)
{
    uint8_t event = EVENT_IO_TO_APP;
    T_IO_MSG msg;
    msg.type = IO_MSG_TYPE_UART;
    msg.subtype = data;
    if (os_msg_send(io_queue_handle, &msg, 0) == false)
    {
    }
    else if (os_msg_send(evt_queue_handle, &event, 0) == false)
    {
    }
}

/**
 * @brief  Initialize App task
 * @return void
 */
void app_task_init(void)
{
    os_task_create(&app_task_handle, "app", app_main_task, 0, APP_TASK_STACK_SIZE,
                   APP_TASK_PRIORITY);
}

static void app_dfu_callback(mible_dfu_state_t state, mible_dfu_param_t *param)
{
    switch (state)
    {
    case MIBLE_DFU_STATE_START:
        DBG_DIRECT("fragment size is %d", param->start.fragment_size);
        break;
    case MIBLE_DFU_STATE_VERIFY:
        DBG_DIRECT("verify result is %d", param->verify.value);
        break;
    case MIBLE_DFU_STATE_SWITCH:
        DBG_DIRECT("switch to new firmware");
        break;
    case MIBLE_DFU_STATE_CANCEL:
        DBG_DIRECT("the sequence is canceled");
        break;
    default:
        DBG_DIRECT("state of DFU is unknown");
        break;
    }
}


/**
 * @brief        App task to handle events & messages
 * @param[in]    p_param    Parameters sending to the task
 * @return       void
 */
static void app_main_task(void *p_param)
{
    mi_startup_delay();

    uint8_t event;

    os_msg_queue_create(&io_queue_handle, MAX_NUMBER_OF_IO_MESSAGE, sizeof(T_IO_MSG));
    os_msg_queue_create(&evt_queue_handle, MAX_NUMBER_OF_EVENT_MESSAGE, sizeof(uint8_t));

    gap_start_bt_stack(evt_queue_handle, io_queue_handle, MAX_NUMBER_OF_GAP_MESSAGE);
    mesh_start(EVENT_MESH, EVENT_IO_TO_APP, evt_queue_handle, io_queue_handle);

    app_system_info();
    mi_mesh_start(EVENT_MI, evt_queue_handle);
    mible_dfu_callback_register(app_dfu_callback);

#if MI_MANU_TEST_ENABLE || MI_USER_CMD_ENABLE
    data_uart_init(P3_0, P3_1, app_send_uart_msg);
#endif

#if MI_MANU_TEST_ENABLE
    mijia_mp_cmd_init();
#endif

#if MI_USER_CMD_ENABLE
    user_cmd_init("Mijia Mesh");
#endif

#if (ROM_WATCH_DOG_ENABLE == 1)
    extern void reset_watch_dog_timer_enable(void);
    reset_watch_dog_timer_enable();
#endif

    driver_init();

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
            else if (event == EVENT_MESH)
            {
                mesh_inner_msg_handle(event);
            }
            else if (event == EVENT_MI)
            {
                mi_inner_msg_handle(event);
            }
            else
            {
                gap_handle_msg(event);
            }
        }
    }
}

/**
 * @brief output system information
 */
void app_system_info(void)
{
    T_IMG_ID image_id;
    T_IMAGE_VERSION image_version;
    const char *name = NULL;
    uint8_t mac[6] = {0};
    uint32_t device_id[2] = {0};

    DBG_DIRECT(" ");
    DBG_DIRECT("_|      _|  _|_|_|  _|_|_|    _|_|  ");
    DBG_DIRECT("_|_|  _|_|    _|      _|    _|    _|");
    DBG_DIRECT("_|  _|  _|    _|      _|    _|    _|");
    DBG_DIRECT("_|      _|    _|      _|    _|    _|");
    DBG_DIRECT("_|      _|  _|_|_|  _|_|_|    _|_|  ");
    DBG_DIRECT(" ");

    for (image_id = OTA; image_id < IMAGE_MAX; ++image_id)
    {
        switch (image_id)
        {
        case OTA:
            name = "OTA header";
            break;
        case SecureBoot:
            name = "Secure boot";
            break;
        case RomPatch:
            name = "ROM patch";
            break;
        case AppPatch:
            name = "SDK";
            break;
        case AppData2:
            name = "Patch extention";
            break;
        default:
            name = NULL;
            break;
        }

        if (NULL == name)
        {
            continue;
        }

        if (get_active_bank_image_version(image_id, &image_version))
        {
            if (OTA == image_id)
            {
                DBG_DIRECT("%s version: %d.%d.%d.%d", name,
                           image_version.ver_info.header_sub_version._version_major,
                           image_version.ver_info.header_sub_version._version_minor,
                           image_version.ver_info.header_sub_version._version_revision,
                           image_version.ver_info.header_sub_version._version_reserve);
            }
            else
            {
                DBG_DIRECT("%s version: %d.%d.%d.%d", name,
                           image_version.ver_info.img_sub_version._version_major,
                           image_version.ver_info.img_sub_version._version_minor,
                           image_version.ver_info.img_sub_version._version_revision,
                           image_version.ver_info.img_sub_version._version_reserve);
            }

        }
        else
        {
            DBG_DIRECT("Fail to get %s version !!!", name);
        }
    }
    DBG_DIRECT("GCID: %08x", VERSION_GCID);

    DBG_DIRECT(" ");
    DBG_DIRECT("Product ID: %d", PRODUCT_ID);
    get_device_id((uint8_t *)device_id);
    if (0 == device_id[1])
    {
        DBG_DIRECT("Device ID: %u", device_id[0]);
    }
    else
    {
        DBG_DIRECT("Device ID: 0x%08X%08X", device_id[1], device_id[0]);
    }
    if (GAP_CAUSE_SUCCESS == gap_get_param(GAP_PARAM_BD_ADDR, mac))
    {
        DBG_DIRECT("MAC address: %02X %02X %02X %02X %02X %02X",
                   mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    }
    else
    {
        DBG_DIRECT("Fail to get MAC address !!!");
    }
    DBG_DIRECT("Firmware version: " MIBLE_LIB_AND_DEVELOPER_VERSION);
    DBG_DIRECT(" ");
}

