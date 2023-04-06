#include <string.h>
#include "trace.h"
#include "app_section.h"
#include "flash_device.h"
#include "flash_adv_cfg.h"
#include "patch_header_check.h"
#include "hw_aes.h"
#include "rtl876x_hw_aes.h"
#include "os_sync.h"
#include "otp.h"
#include "dfu_api.h"
#include "silent_dfu_flash.h"
#include "board.h"

uint8_t prev_bp_lv = 0xff;
uint32_t appdefine_offset = 0;

/**
*  @brief: unlock flash is need when erase or write flash.
*/
bool unlock_flash_all(void)
{
    DFU_PRINT_TRACE0("**********[Flash Set] Flash unlock ***********");
    if (FLASH_SUCCESS == flash_sw_protect_unlock_by_addr_locked(0x00800000, &prev_bp_lv))
    {
        DFU_PRINT_TRACE1("[Flash Set] Flash unlock address = 0x800000, prev_bp_lv = %d", prev_bp_lv);
        return true;
    }
    return false;
}

/**
*  @brief: lock flash after erase or write flash.
*/
void lock_flash(void)
{
    if (prev_bp_lv != 0xff)
    {
        flash_set_block_protect_locked(prev_bp_lv);
    }
}

/**
 * @brief erase a sector of the flash.
 *
 * @param  addr          flash addr in sector to be erase.
 * @return  0 if erase successfully, error line number otherwise
*/
uint32_t flash_erase_sector(uint32_t addr)
{
    DFU_PRINT_INFO1("==> flash_erase_sector :%x \r\n", addr);
    return flash_erase_locked(FLASH_ERASE_SECTOR, addr);
}

/**
 * @brief erase a sector of the flash.
 *
 * @param  signature          signature to identify FW.
 * @param  offset             offset of the image.
 * @return  0 if erase successfully, error line number otherwise
*/
uint32_t sil_dfu_flash_erase(uint16_t signature, uint32_t offset)
{
    uint32_t result = 0;
    uint32_t dfu_base_addr;
    dfu_base_addr = get_temp_ota_bank_addr_by_img_id((T_IMG_ID)signature);
    if (dfu_base_addr == 0)
    {
        result = __LINE__;
        goto L_Return;
    }

    flash_erase_sector(dfu_base_addr + offset);
L_Return:
    DFU_PRINT_INFO1("<==sil_dfu_flash_erase result:%d \r\n", result);
    return result;
}

#if USER_DATA_COPY_ENABLE
extern uint8_t *p_ota_temp_buffer_head;
/**
 * @brief   copy appdata from active bank to updating bank.
 *
 * @param   image_id    signature to identify image.
 * @param   dlAddress   address the img copy to.
 * @param   dlSize      copy size.
 * @return  true if the image copied success, false otherwise
*/
bool sil_dfu_copy_img(uint16_t image_id, uint32_t dlAddress, uint32_t dlSize)
{
    uint32_t error_code = 0;
    uint32_t source_base_addr;
    uint32_t dest_base_addr;
    int remain_size = dlSize;
    uint32_t s_val;
    uint32_t dlOffset, tmp_offset;

    if ((image_id != AppData1) && (image_id != AppData2) && (image_id != AppData3)
        && (image_id != AppData4) && (image_id != AppData5) && (image_id != AppData6))
    {
        error_code = __LINE__;
        goto L_Return;
    }
    if (dlAddress % 4096)
    {
        error_code = __LINE__;
        goto L_Return;
    }

    source_base_addr = get_active_ota_bank_addr() & 0xffffff;

    if (flash_get_bank_addr(FLASH_OTA_BANK_0) == get_active_ota_bank_addr())
    {
        dest_base_addr = flash_get_bank_addr(FLASH_OTA_BANK_1) & 0xffffff;
    }
    else
    {
        dest_base_addr = flash_get_bank_addr(FLASH_OTA_BANK_0) & 0xffffff;
    }
    if ((source_base_addr % 4096) || (dest_base_addr % 4096))
    {
        error_code = __LINE__;
        goto L_Return;
    }
    if (dest_base_addr >= dlAddress)
    {
        error_code = __LINE__;
        goto L_Return;
    }
    dlOffset = dlAddress - dest_base_addr;
    tmp_offset = dlOffset;
    if (dlOffset % 4096)
    {
        error_code = __LINE__;
        goto L_Return;
    }
    T_IMG_HEADER_FORMAT *p_data_header;
    p_data_header = (T_IMG_HEADER_FORMAT *)(source_base_addr + dlOffset);
    if (p_data_header->ctrl_header.image_id != image_id)
    {
        error_code = __LINE__;
        goto L_Return;
    }

    while (remain_size > 0)
    {
        if (!((dest_base_addr + tmp_offset) % 4096)) //must 4k align
        {
            flash_erase_sector(dest_base_addr + tmp_offset);
        }
        if (remain_size > 2048)
        {
            memcpy(p_ota_temp_buffer_head, (uint8_t *)(source_base_addr + tmp_offset), 2048);
            if (remain_size ==  dlSize)
            {
                T_IMG_CTRL_HEADER_FORMAT *p_header = (T_IMG_CTRL_HEADER_FORMAT *) p_ota_temp_buffer_head;
                p_header->ctrl_flag.flag_value.not_ready = 0x1; /*make sure image is not ready, will use it later*/
            }
            for (int i = 0; i < 2048; i = i + 4)
            {
                flash_auto_write_locked(dest_base_addr + tmp_offset + i, *(uint32_t *)p_ota_temp_buffer_head);

                flash_auto_read_locked(dest_base_addr + tmp_offset + i | FLASH_OFFSET_TO_NO_CACHE, &s_val);
                if (s_val != *(uint32_t *)p_ota_temp_buffer_head)
                {
                    DFU_PRINT_TRACE3("s_val:0x%08x, *p_void:0x%08x, i:0x%08x",
                                     s_val, *(uint32_t *)p_ota_temp_buffer_head, i);
                    error_code = __LINE__;
                    goto L_Return;
                }
                else
                {
                    p_ota_temp_buffer_head += 4;
                }
            }


            remain_size -= 2048;
        }
        else
        {
            memcpy(p_ota_temp_buffer_head, (uint8_t *)(source_base_addr + tmp_offset), remain_size);
            for (int i = 0; i < remain_size; i = i + 4)
            {
                flash_auto_write_locked(dest_base_addr + tmp_offset + i, *(uint32_t *)p_ota_temp_buffer_head);

                flash_auto_read_locked(dest_base_addr + tmp_offset + i | FLASH_OFFSET_TO_NO_CACHE, &s_val);
                if (s_val != *(uint32_t *)p_ota_temp_buffer_head)
                {
                    DFU_PRINT_TRACE3("s_val:0x%08x, *p_void:0x%08x, i:0x%08x",
                                     s_val, *(uint32_t *)p_ota_temp_buffer_head, i);
                    error_code = __LINE__;
                    goto L_Return;
                }
                else
                {
                    p_ota_temp_buffer_head += 4;
                }
            }
            remain_size = 0;
        }
        tmp_offset += 2048;
    }

L_Return:
    DFU_PRINT_INFO1("<====dfu_copy_img  error_code:%d", error_code);
    if (error_code)
    {
        return false;
    }
    return true;
}
#endif//#if USER_DATA_COPY_ENABLE
/**
*  @brief: get temp image size accord to the image id.
*/
uint32_t get_temp_ota_bank_size_by_img_id(T_IMG_ID image_id)
{
    uint32_t image_size = 0;

    bool enable_old_ota = !is_ota_support_bank_switch();
    if (enable_old_ota)
    {
#ifdef SDK_8772
        if (image_id == SecureBoot || image_id == RomPatch || image_id == AppPatch
            || image_id == AppData1 || image_id == AppData2 || image_id == AppData3
            || image_id == AppData4 || image_id == AppData5 || image_id == AppData6)
#else

#if (SUPPORT_OTA_APP_DATA_EXTENSION == 1)
        if (image_id == SecureBoot || image_id == RomPatch || image_id == AppPatch
            || image_id == AppData1 || image_id == AppData2 || image_id == AppData3
            || image_id == AppData4 || image_id == AppData5 || image_id == AppData6)
#else
        if (image_id == SecureBoot || image_id == RomPatch || image_id == AppPatch
            || image_id == AppData1 || image_id == AppData2)
#endif

#endif
        {
            image_size = flash_get_bank_size(FLASH_OTA_TMP);
        }
        //others will return 0
    }
    else
    {
        uint32_t ota_bank0_addr = flash_get_bank_addr(FLASH_OTA_BANK_0);
        uint32_t temp_bank_addr;
        if (ota_bank0_addr == get_active_ota_bank_addr())
        {
            temp_bank_addr = flash_get_bank_addr(FLASH_OTA_BANK_1);
        }
        else
        {
            temp_bank_addr = ota_bank0_addr;
        }

        if (image_id == OTA)
        {
            image_size = OTA_HEADER_SIZE;
        }
#ifdef SDK_8772
        else if (image_id == SecureBoot || image_id == RomPatch || image_id == AppPatch
                 || image_id == AppData1 || image_id == AppData2 || image_id == AppData3
                 || image_id == AppData4 || image_id == AppData5 || image_id == AppData6)
#else

#if (SUPPORT_OTA_APP_DATA_EXTENSION == 1)
        else if (image_id == SecureBoot || image_id == RomPatch || image_id == AppPatch
                 || image_id == AppData1 || image_id == AppData2 || image_id == AppData3
                 || image_id == AppData4 || image_id == AppData5 || image_id == AppData6)
#else
        else if (image_id == SecureBoot || image_id == RomPatch || image_id == AppPatch
                 || image_id == AppData1 || image_id == AppData2)
#endif

#endif
        {
            // auth ota temp bank and get address
            // image_authencation will fail after secure boot, so remove it
            if (!check_header_valid(temp_bank_addr, OTA))
            {
                image_size = 0;
            }
            else
            {
                image_size = HAL_READ32((uint32_t) & ((T_OTA_HEADER_FORMAT *)temp_bank_addr)->secure_boot_size,
                                        (image_id - SecureBoot) * 8);

                //attention: if use old ota header generate tool, app data3-6 addr will be default value 0xffffffff
                if (OTA_HEADER_DEFAULT_VALUE == image_size)
                {
                    image_size = 0;
                }
            }
        }
        else //others will return 0
        {
        }
    }

    return image_size;
}

/**
* @brief check ota image size whether exceed flash layout address.
*/
bool check_dfu_update_image_length(uint16_t signature, uint32_t offset, uint32_t length,
                                   void *p_void, uint32_t *ret)
{
    uint32_t temp_bank_size = 0;
    *ret = 0;

    if (p_void == NULL)
    {
        *ret = __LINE__;
        return false;
    }

    if (SIGNATURE_APP_DEFINE == signature)
    {
        temp_bank_size = APP_DEFINE_DATA_SIZE;
    }
    else
    {
        temp_bank_size = get_temp_ota_bank_size_by_img_id((T_IMG_ID)signature);
    }


    if (offset == 0)
    {
        T_IMG_CTRL_HEADER_FORMAT *p_header = (T_IMG_CTRL_HEADER_FORMAT *) p_void;
        uint32_t total_length = p_header->payload_len + IMG_HEADER_SIZE;

        if (total_length > temp_bank_size)
        {
            DFU_PRINT_ERROR2("New Image too large! total_length = %d, temp_bank_size = %d", total_length,
                             temp_bank_size);
            *ret = __LINE__;
            return false;
        }
    }

    if (offset + length > temp_bank_size)
    {
        DFU_PRINT_ERROR3("New Image single packet too large! offset = %d, length = %d, temp_bank_size = %d",
                         offset, length, temp_bank_size);
        *ret = __LINE__;
        return false;
    }

    //check pass
    return true;
}

/**
 * @brief  write specified image data with specified length to flash
 * @param  signature          signature to identify FW.
 * @param  offset             offset of the image.
 * @param  length             length of data.
 * @param  p_void             pointer to data.
 * @return 0 if write FW image successfully, error line number otherwise
*/
uint32_t sil_dfu_update(uint16_t signature, uint32_t offset, uint32_t length,
                        uint32_t/*void*/ *p_void)
{
    uint32_t result = 0;
    uint32_t dfu_base_addr;
    uint32_t start_addr;
    uint32_t s_val;

    DFU_PRINT_INFO2("==> dfu_update offset;%d, length:%d", offset, length);

    if (length % 4)
    {
        result = __LINE__;
        goto L_Return;
    }

    if (p_void == 0)
    {
        result = __LINE__;
        goto L_Return;
    }

    if (SIGNATURE_APP_DEFINE != signature)
    {
        /*get back up area address*/
#ifdef SDK_8772
        dfu_base_addr = get_temp_ota_bank_addr_by_img_id((T_IMG_ID)signature);
#else

#if (SUPPORT_OTA_APP_DATA_EXTENSION == 1)
        dfu_base_addr = flash_ioctl(flash_ioctl_get_temp_bank_addr_by_image_id_ext, (T_IMG_ID)signature, 0);
#else
        dfu_base_addr = get_temp_ota_bank_addr_by_img_id((T_IMG_ID)signature);
#endif

#endif
    }
    else
    {
        /*get back up area address*/
        dfu_base_addr = APP_DEFINE_DATA_ADDR;
    }

    if (dfu_base_addr == 0)
    {
        result = __LINE__;
        goto L_Return;
    }

    /* before erase temp image or write image to flash temp, check access length depend flash layout */
    if (!check_dfu_update_image_length(signature, offset, length, p_void, &result))
    {
        goto L_Return;
    }

    /*if it's start_packet*/
    if (offset == 0)
    {
        /*ASSERT(length>=sizeof(image_header_t));*/
        T_IMG_CTRL_HEADER_FORMAT *p_header = (T_IMG_CTRL_HEADER_FORMAT *) p_void;
        p_header->ctrl_flag.flag_value.not_ready = 0x1; /*make sure image is not ready, will use it later*/
        DFU_PRINT_TRACE3("dfu_update New Image Header:0x%08x, Signature:0x%08x, dfu_base_addr:0x%08x",
                         length, signature, dfu_base_addr);
    }

    if ((offset % FMC_SEC_SECTION_LEN) == 0)   //new page starts
    {
        flash_erase_sector(dfu_base_addr + offset);
    }
    else  // cross page
    {
        if ((offset / FMC_SEC_SECTION_LEN) != ((offset + length) / FMC_SEC_SECTION_LEN))
        {
            flash_erase_sector((dfu_base_addr + offset + length) & ~(FMC_SEC_SECTION_LEN - 1));
        }
    }
    start_addr = dfu_base_addr + offset;
    for (int i = 0; i < length; i = i + 4)
    {
        flash_auto_write_locked(start_addr + i, *(uint32_t *)p_void);

        flash_auto_read_locked(start_addr + i | FLASH_OFFSET_TO_NO_CACHE, &s_val);
        if (s_val != *(uint32_t *)p_void)
        {
            DFU_PRINT_TRACE3("s_val:0x%08x, *p_void:0x%08x, i:0x%08x",
                             s_val, *(uint32_t *)p_void, i);
            result = __LINE__;
            goto L_Return;
        }
        else
        {
            p_void++;
        }
    }

//    ota_offset = offset + length; //for re-ota
    if (SIGNATURE_APP_DEFINE == signature)
    {
        appdefine_offset = offset + length;
    }

L_Return:

    DFU_PRINT_INFO1("<==dfu_update result:%d \r\n", result);
    return result;
}

/**
 * @brief calculate checksum of the image and compare with given checksum value.
 *
 * @param   image_id     signature to identify image.
 * @return  true if the image integrity check passes, false otherwise
*/
bool silent_dfu_check_checksum(uint16_t image_id)
{
    bool check_result;

    if (image_id == SIGNATURE_APP_DEFINE)
    {
        silent_dfu_reset(image_id);

        /* because APP_DEFINE_DATA_ADDR is not located flash block protect range, needn't unlock bp */
        T_IMG_CTRL_HEADER_FORMAT *p_appdefine_header = (T_IMG_CTRL_HEADER_FORMAT *)(
                                                           APP_DEFINE_DATA_ADDR | FLASH_OFFSET_TO_NO_CACHE);
        /*use flash auto mode if OTP->image_split_read == 0, can skip flash_lock and unlock*/
        if (OTP->image_split_read)
        {
            flash_lock(FLASH_LOCK_USER_MODE_READ);
        }
        check_result = check_image_chksum(p_appdefine_header);
        /*flash auto mode when OTP->image_split_read = 0, skip flash_lock and unlock*/
        if (OTP->image_split_read)
        {
            flash_unlock(FLASH_LOCK_USER_MODE_READ);
        }
        /*don't need unlock bp*/

        /*
        mark image as ready:
        p_appdefine_header->ctrl_flag.flag_value.not_ready = 0;
        there are two reasons we must use user mode API:
        1. support flash larger than 64M bits.
        2. support 4 bit mode: PRM enabled and programming non cache space doesn't supported.
        */
        uint32_t r_data;
        flash_read_locked((uint32_t)p_appdefine_header, sizeof(r_data), (uint8_t *)&r_data);
        r_data &= ~BIT23; /* not_ready */
        flash_write_locked((uint32_t)p_appdefine_header, sizeof(r_data), (uint8_t *)&r_data);
    }
    else
    {
        /* need modify p_header->ctrl_flag.not_ready, so first unlock flash bp*/
        unlock_flash_all();
        /*use flash auto mode if OTP->image_split_read == 0, can skip flash_lock and unlock*/
        if (OTP->image_split_read)
        {
            flash_lock(FLASH_LOCK_USER_MODE_READ);
        }
        check_result = dfu_check_checksum(image_id);
        /*flash auto mode when OTP->image_split_read = 0, skip flash_lock and unlock*/
        if (OTP->image_split_read)
        {
            flash_unlock(FLASH_LOCK_USER_MODE_READ);
        }
        lock_flash();
    }

    return check_result;
}

bool silent_dfu_reset(uint16_t image_id)
{
    bool ret = true;
    if (image_id == SIGNATURE_APP_DEFINE)
    {
        appdefine_offset = 0;
    }
    else
    {
        ret = dfu_reset(image_id);
    }
    return ret;
}


uint32_t dfu_flash_check_appdefine_blank(uint32_t appdefine_addr, uint32_t offset, uint16_t size)
{
    uint32_t i = 0;
    uint32_t result = 0;
    uint32_t start_addr = appdefine_addr + offset;
    DFU_PRINT_INFO2("<====dfu_flash_check_appdefine_blank :start_addr = %x, size:%d", start_addr, size);

    for (i = 0; i < size / 4; i++)
    {
        uint32_t r_data;
        flash_auto_read_locked(start_addr, &r_data);
        if (r_data != 0xffffffff)
        {
            result = __LINE__;
            goto L_Return;
        }
        start_addr += 4;
    }

L_Return:
    DFU_PRINT_INFO2("<====dfu_flash_check_appdefine_blank :i = %x, result:%d", i, result);
    return result;
}






