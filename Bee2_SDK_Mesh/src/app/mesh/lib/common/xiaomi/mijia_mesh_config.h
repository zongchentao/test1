/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file     mijia_mesh_config.h
* @brief    Head file for mijia mesh application.
* @details  Data types and external functions declaration.
* @author   hector_huang
* @date     2019-1-10
* @version  v1.0
* *************************************************************************************
*/
#ifndef _MIJIA_MESH_CONFIG_H_
#define _MIJIA_MESH_CONFIG_H_

#include "flash_map.h"

/* enable manufacture and test module */
#define MI_MANU_TEST_ENABLE                      0

/* enable user command module */
#define MI_USER_CMD_ENABLE                       1

#if MI_MANU_TEST_ENABLE && MI_USER_CMD_ENABLE
#error must NOT enable both of MANU_TEST and USER_CMD
#endif

#define MI_PUB_NEW_STRATEGY                      1

/* enable vendor model client */
#define MI_VENDOR_MODEL_CLIENT_ENABLE            0

/* product id */
/**
 * 0x3B4: Yeelight
 * 0x379: Develop Board
 */
//#define PRODUCT_ID                               0x0379

/* the number of mijia mesh inner message */
#define MI_INNER_MSG_NUM                         16

/* device version */
#define MI_DEVICE_VERSION                        5

/* device beacon time interval unprov */
#define MI_BEACON_INTERVAL_UNPROV                100 //100ms

/* device beacon time interval proved */
#define MI_BEACON_INTERVAL_PROVED                500 //500ms

/* enable stop udb function */
#define MI_ENABLE_STOP_UDB                       0

/* stop advertising period */
#define MI_STOP_UDB_PERIOD                       1800000  //30 minutes

/* default ttl number */
#define MI_DEFAULT_TTL                           5

/* model publish default parameters */
#define MI_PUB_ADDRESS                           0xFEFF
#define MI_PUB_STEP_RESOLUTION                   2 //0:100ms 1:1s 2:10s 3:10 minutes
#define MI_PUB_NUM_STEPS                         6
#define MI_PUB_RETRANS_COUNT                     0
#define MI_PUB_RETRANS_INTERVAL_STEPS            0 //(n + 1) * 50ms

/* network default parameters */
#define MI_NET_RETRANS_COUNT                     2
#define MI_NET_RETRANS_INTERVAL_STEPS            4 //(n + 1) * 10ms

/* relay default parameters */
#define MI_RELAY_RETRANS_COUNT                   2
#define MI_RELAY_RETRANS_INTERVAL_STEPS          4 //(n + 1) * 10ms

/* network message cache */
#define MI_NMC_SIZE                              96

/* reply protection list size */
#define MI_RPL_SIZE                              32

/* iv update trigger sequence */
#define MI_IV_UPDATE_TRIGGER_SEQUENCE_NUM        0xf00000

/* time of expecting to receive segment acknowledge */
#define MI_TIME_RECV_SEG_ACK                     350

/* time after sending segment acknowledgement */
#define MI_TIME_SEND_SEG_ACK                     300

/* scheduler task number */
#define MI_GAP_SCHED_TASK_NUM                    15

/* mi scheduler interval */
#define MI_SCHEDULER_INTERVAL                    10  //10ms

/* interwave scan window */
#define MI_GAP_SCHED_INTERWAVE_SCAN_WINDOW       158

/* interwave scan interval */
#define MI_GAP_SCHED_INTERWAVE_SCAN_INTERVAL     160

/* scan window */
#define MI_GAP_SCHED_SCAN_WINDOW                 158

/* scan interval */
#define MI_GAP_SCHED_SCAN_INTERVAL               160

/* relay parallel max number */
#define MI_GAP_SCHED_RELAY_PARALLEL_MAX_NUM      5

/* mi record param offset */
#define MI_RECORD_OFFSET                         2000 //!< Shall be bigger than or equal to the size of mesh stack flash usage

#define DFU_NVM_START                            OTA_TMP_ADDR
#define DFU_NVM_SIZE                             OTA_TMP_SIZE

#endif

