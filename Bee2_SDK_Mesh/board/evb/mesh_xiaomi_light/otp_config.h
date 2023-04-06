/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    otp_config.h
  * @brief   Update Configuration in APP
  * @date    2017.6.6
  * @version v1.0
  * *************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   * *************************************************************************************
  */

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef OTP_CONFIG_H
#define OTP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtl876x_wdg.h"

#define BT_STACK_CONFIG_ENABLE

#ifdef BT_STACK_CONFIG_ENABLE
void bt_stack_config_init(void);
#endif

/*============================================================================*
 *                        debug configuration
 *============================================================================*/
/** @brief just for debug task hang*/
#define DEBUG_TASK_HANG_ENABLE                     0
/** @brief just for debug not enter dlps reason*/
#define DEBUG_DLPS_ERROR_IN_APP_ENABLE             0


/*============================================================================*
 *                        flash configuration
 *============================================================================*/
/** @brief support for puran flash*/
#define FTL_APP_CALLBACK_ENABLE                    0
/** @brief enable BP, set lock level depend on flash layout and selected flash id */
#define FLASH_BLOCK_PROTECT_ENABLE                 0
/** @brief modify delay time for wakeup flash from power down mode to standby mode*/
#define AFTER_TOGGLE_CS_DELAY                      6


/*============================================================================*
 *                        platform configuration
 *============================================================================*/
/** @brief default enable swd pinmux */
#define SWD_PINMUX_ENABLE                          1
/** @brief default disable watch dog in rom */
#define ROM_WATCH_DOG_ENABLE                       1
/** @brief set wdg mode, default reset all */
#define ROM_WATCH_DOG_MODE                         RESET_ALL
/** @brief Watch Dog Timer Config, default 4s timeout
   * div_factor: 16Bit: 32.768k/(1+divfactor).
   * cnt_limit: 2^(cnt_limit+1) - 1 ; max 11~15 = 0xFFF.
   * wdg_mode:
   *            1: RESET_ALL_EXCEPT_AON
   *            3: RESET_ALL
**/
#define ROM_WATCH_DOG_CFG_DIV_FACTOR               79
#define ROM_WATCH_DOG_CFG_CNT_LIMIT                15
/**************************************************/
/** @brief config enable write hardfault record to flash example */
//config enable write hardfault record to flash
//if enable, must define 1; if disable, define 0 or not define
#define ENABLE_WRITE_HARDFAULT_RECORD_TO_FLASH     0
//if enable must define hardfault record begin and end flash addr, and record cfg depend on user flash usage
#define HARDFAULT_RECORD_BEG_ADDR                  0x842000  //change to user unused space
#define HARDFAULT_RECORD_END_ADDR                 (HARDFAULT_RECORD_BEG_ADDR + FMC_PAGE_SIZE)
#define HARDFAULT_RECORD_CFG                      ( BIT_ENABLE_SAVE_HARDFAULT | BIT_CLEAR_HISTROY_BEFORE_SAVING )
/*============================================================================*
 *                        upperstack configuration
 *============================================================================*/
//add more here


/*============================================================================*
 *                        app configuration
 *============================================================================*/
#define OTA_TIMEOUT_TOTAL                          240
#define OTA_TIMEOUT_WAIT4_CONN                     60
#define OTA_TIMEOUT_WAIT4_IMAGE_TRANS              200
#define OTA_TIMEOUT_CTITTV                         0xFF
//add more here



#ifdef __cplusplus
}
#endif


/** @} */ /* End of group OTP_CONFIG */
#endif
