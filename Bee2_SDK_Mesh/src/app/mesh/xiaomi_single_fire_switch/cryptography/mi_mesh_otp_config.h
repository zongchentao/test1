/*
 * mi_mesh_otp_config.h
 *
 *  Created on: 2018年9月18日
 *      Author: JT
 */

#ifndef MIJIA_BLE_LIBS_MANUFACTURE_MI_MESH_OTP_CONFIG_H_
#define MIJIA_BLE_LIBS_MANUFACTURE_MI_MESH_OTP_CONFIG_H_

#include "flash_map.h"

#define POTP_BASE          ((uint8_t *)(BKP_DATA1_ADDR))
#define POTP_FULL_SIZE     4096

#endif /* MIJIA_BLE_LIBS_MANUFACTURE_MI_MESH_OTP_CONFIG_H_ */
