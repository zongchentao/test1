/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file    flash_map.h
* @brief   Flash Layout Configuration, and flash layout must be changed with config file!
* @note    flash_map.h must be generated by FlashMapGenerateTool!
* *************************************************************************************
*/

#ifndef _FLASH_MAP_H_
#define _FLASH_MAP_H_

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
*                        Flash Layout
*============================================================================*/
/*  Flash total size                                512KB
example:
    1) Reaerved:                                       4K (0x00800000)
    2) OEM Header:                                     4K (0x00801000)
    3) OTA Bank0:                                    280K (0x00802000)
        a) OTA Header                    4K (0x00802000)
        b) Secure boot loader            4K (0x0080D000)
        c) Patch code                   40K (0x00803000)
        d) APP code                    220K (0x0080E000)
        e) APP data1                     0K (0x00845000)
        f) APP data2                    12K (0x00845000)
    4) OTA Bank1:                                      0K (0x00848000)
        a) OTA Header                    0K (0x00000000)
        b) Secure boot loader            0K (0x00000000)
        c) Patch code                    0K (0x00000000)
        d) APP code                      0K (0x00000000)
        e) APP data1                     0K (0x00000000)
        f) APP data2                     0K (0x00000000)
    5) FTL:                                           16K (0x00848000)
    6) OTA Tmp:                                      204K (0x0084C000)
    7) APP Defined Section:                            4K (0x0087F000)
*/

/*============================================================================*
*            Flash Layout Configuration (Generated by FlashMapGenerateTool)
*============================================================================*/

#define FLASH_ADDR                      0x00800000  //Fixed
#define FLASH_SIZE                      0x00080000  //512K Bytes

/* ========== High Level Flash Layout Configuration ========== */
#define RESERVED_ADDR                   0x00800000
#define RESERVED_SIZE                   0x00001000  //4K Bytes
#define OEM_CFG_ADDR                    0x00801000
#define OEM_CFG_SIZE                    0x00001000  //4K Bytes
#define OTA_BANK0_ADDR                  0x00802000
#define OTA_BANK0_SIZE                  0x00046000  //280K Bytes
#define OTA_BANK1_ADDR                  0x00848000
#define OTA_BANK1_SIZE                  0x00000000  //0K Bytes
#define FTL_ADDR                        0x00848000
#define FTL_SIZE                        0x00004000  //16K Bytes
#define OTA_TMP_ADDR                    0x0084C000
#define OTA_TMP_SIZE                    0x00033000  //204K Bytes
#define BKP_DATA1_ADDR                  0x0087F000
#define BKP_DATA1_SIZE                  0x00001000  //4K Bytes
#define BKP_DATA2_ADDR                  0x00000000
#define BKP_DATA2_SIZE                  0x00000000  //0K Bytes

/* ========== OTA Bank0 Flash Layout Configuration ========== */
#define BANK0_OTA_HEADER_ADDR           0x00802000
#define BANK0_OTA_HEADER_SIZE           0x00001000  //4K Bytes
#define BANK0_SECURE_BOOT_ADDR          0x0080D000
#define BANK0_SECURE_BOOT_SIZE          0x00001000  //4K Bytes
#define BANK0_ROM_PATCH_ADDR            0x00803000
#define BANK0_ROM_PATCH_SIZE            0x0000A000  //40K Bytes
#define BANK0_APP_ADDR                  0x0080E000
#define BANK0_APP_SIZE                  0x00037000  //220K Bytes
#define BANK0_APP_DATA1_ADDR            0x00845000
#define BANK0_APP_DATA1_SIZE            0x00000000  //0K Bytes
#define BANK0_APP_DATA2_ADDR            0x00845000
#define BANK0_APP_DATA2_SIZE            0x00003000  //12K Bytes

/* ========== OTA Bank1 Flash Layout Configuration ========== */
#define BANK1_OTA_HEADER_ADDR           0x00000000
#define BANK1_OTA_HEADER_SIZE           0x00000000  //0K Bytes
#define BANK1_SECURE_BOOT_ADDR          0x00000000
#define BANK1_SECURE_BOOT_SIZE          0x00000000  //0K Bytes
#define BANK1_ROM_PATCH_ADDR            0x00000000
#define BANK1_ROM_PATCH_SIZE            0x00000000  //0K Bytes
#define BANK1_APP_ADDR                  0x00000000
#define BANK1_APP_SIZE                  0x00000000  //0K Bytes
#define BANK1_APP_DATA1_ADDR            0x00000000
#define BANK1_APP_DATA1_SIZE            0x00000000  //0K Bytes
#define BANK1_APP_DATA2_ADDR            0x00000000
#define BANK1_APP_DATA2_SIZE            0x00000000  //0K Bytes


#ifdef __cplusplus
}
#endif
/** @} */ /* _FLASH_MAP_H_ */
#endif
