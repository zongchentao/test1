/**
*********************************************************************************************************
*               Copyright(c) 2018, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      rtk_nvm.c
* @brief     xiaomi ble nvm api
* @details   NVM data types and functions.
* @author    hector_huang
* @date      2018-12-26
* @version   v1.0
* *********************************************************************************************************
*/
#include <string.h>
#include "mible_api.h"
#include "miio_user_api.h"
#include "flash_device.h"
#undef  MI_LOG_MODULE_NAME
#define MI_LOG_MODULE_NAME "RTK_NVM"
#include "mible_log.h"
#include "ftl.h"
#include "ftl_map.h"
#include "flash_map.h"
#include "platform_types.h"
#include "silent_dfu_flash.h"

#include "patch_header_check.h"
#include "otp.h"
#include "platform_misc.h"

#define CREATE_WHEN_WRITE           1

#define SYS_RECORD_NUM              5
#define USER_RECORD_NUM             16

#define INVALID_RECORD_INDEX        0xFF
#define CALC_LEN(len)               (((len) + 3) / 4 * 4)

typedef struct
{
    uint16_t record_id;
    uint16_t offset;
    uint8_t len;
    uint8_t valid;
    uint8_t padding[2];
} record_t;

static record_t records[SYS_RECORD_NUM];
#define OFFSET_HEADER               FTL_MAP_MI_RECORD_OFFSET_OFFSET
#define OFFSET_RECORD               (FTL_MAP_MI_RECORD_OFFSET_OFFSET + sizeof(records))

#if USER_RECORD_NUM
static record_t user_records[USER_RECORD_NUM];
#define USER_OFFSET_HEADER          FTL_MAP_USER_RECORD_OFFSET_OFFSET
#define USER_OFFSET_RECORD          (FTL_MAP_USER_RECORD_OFFSET_OFFSET + sizeof(user_records))
#endif

/* dfu releated parameters, support pasue and resume download */
#define PATCH_SIZE                  (40 * 1024)
#define SEC_BOOT_SIZE               (4 * 1024)
#define OTA_RECORD_OFFSET           (FTL_MAP_MI_RECORD_OFFSET_OFFSET - 4)

typedef union
{
    struct
    {
        //bool patch_exist;
        bool secboot_exist;
        bool app_exist;
    };
    uint8_t pad[4];
} ota_ctx_t;

static ota_ctx_t ota_ctx;


static void dump_record_header(void)
{
    MI_LOG_DEBUG("record header:");
    for (uint8_t i = 0; i < SYS_RECORD_NUM; ++i)
    {
        MI_LOG_DEBUG("id: %d, offset: %d, len: %d, valid: %d",
                     records[i].record_id, records[i].offset,
                     records[i].len, records[i].valid);
    }
}

static mible_status_t mible_record_save_header(void)
{
    if (0 != ftl_save((void *)records, OFFSET_HEADER, sizeof(records)))
    {
        return MI_ERR_INVALID_LENGTH;
    }

    return MI_SUCCESS;
}

static mible_status_t mible_record_load_header(void)
{
    if (0 != ftl_load((void *)records, OFFSET_HEADER, sizeof(records)))
    {
        return MI_ERR_INVALID_LENGTH;
    }

    return MI_SUCCESS;
}

mible_status_t mible_record_init(void)
{
    MI_LOG_DEBUG("mible record init");
    mible_record_load_header();
    uint8_t index = 0;
    /* find exists record */
    for (index = 0; index < SYS_RECORD_NUM; ++index)
    {
        if ((0 != records[index].valid) && (1 != records[index].valid))
        {
            /* new record aera, initialize it */
            memset(&records[index], 0, sizeof(record_t));
        }
    }
    mible_record_save_header();

    dump_record_header();

    return MI_SUCCESS;
}

void mible_record_clear(void)
{
    memset(records, 0, sizeof(records));
    ftl_save((void *)records, OFFSET_HEADER, sizeof(records));
}

#if USER_RECORD_NUM
static void dump_user_record_header(void)
{
    MI_LOG_DEBUG("record header:");
    for (uint8_t i = 0; i < USER_RECORD_NUM; ++i)
    {
        MI_LOG_DEBUG("id: %d, offset: %d, len: %d, valid: %d",
                     user_records[i].record_id, user_records[i].offset,
                     user_records[i].len, user_records[i].valid);
    }
}
    
static mible_status_t user_record_save_header(void)
{
    if (0 != ftl_save((void *)user_records, USER_OFFSET_HEADER, sizeof(user_records)))
    {
        return MI_ERR_INVALID_LENGTH;
    }

    return MI_SUCCESS;
}

static mible_status_t user_record_load_header(void)
{
    if (0 != ftl_load((void *)user_records, USER_OFFSET_HEADER, sizeof(user_records)))
    {
        return MI_ERR_INVALID_LENGTH;
    }

    return MI_SUCCESS;
}

mible_status_t user_record_init(void)
{
    MI_LOG_DEBUG("user record init");
    user_record_load_header();
    uint8_t index = 0;
    /* find exists record */
    for (index = 0; index < USER_RECORD_NUM; ++index)
    {
        if ((0 != user_records[index].valid) && (1 != user_records[index].valid))
        {
            /* new record aera, initialize it */
            memset(&user_records[index], 0, sizeof(record_t));
        }
    }
    user_record_save_header();

    dump_user_record_header();

    return MI_SUCCESS;
}

void user_record_clear(void)
{
    memset(user_records, 0, sizeof(user_records));
    ftl_save((void *)user_records, USER_OFFSET_HEADER, sizeof(user_records));
}
#endif

static uint8_t mible_record_find_index(uint16_t record_id, uint8_t len)
{
    uint8_t max_record_num = SYS_RECORD_NUM;
    record_t *p_record = records;
#if USER_RECORD_NUM
    if(record_id >= MIBLE_USER_REC_ID_BASE){
        max_record_num = USER_RECORD_NUM;
        p_record = user_records;
    }
#endif  
    uint8_t index;
    /* find exists record */
    for (index = 0; index < max_record_num; ++index)
    {
        if ((p_record[index].record_id == record_id) && (p_record[index].len == len))
        {
            break;
        }
    }

    if (index >= max_record_num)
    {
        /* find empty record */
        for (index = 0; index < max_record_num; ++index)
        {
            if (!p_record[index].valid)
            {
                break;
            }
        }
        if (index >= max_record_num)
        {
            index = INVALID_RECORD_INDEX;
        }
    }

    return index;
}

static uint16_t mible_record_get_offset(uint16_t record_id)
{
    uint8_t max_record_num = SYS_RECORD_NUM;
    record_t *p_record = records;
#if USER_RECORD_NUM
    if(record_id >= MIBLE_USER_REC_ID_BASE){
        max_record_num = USER_RECORD_NUM;
        p_record = user_records;
    }
#endif
    uint8_t max_index = 0;
    uint16_t max_offset = 0;
    for (uint8_t i = 0; i < max_record_num; ++i)
    {
        if (p_record[i].offset > max_offset)
        {
            max_index = i;
            max_offset = p_record[i].offset;
        }
    }
    if (0 == max_offset)
    {
#if USER_RECORD_NUM
        if(record_id >= MIBLE_USER_REC_ID_BASE){
            max_offset = USER_OFFSET_RECORD;
        }else
#endif
        max_offset = OFFSET_RECORD;
    }
    max_offset += CALC_LEN(p_record[max_index].len);

    return max_offset;
}

mible_status_t mible_record_create(uint16_t record_id, uint8_t len)
{
    record_t *p_record = records;
    MI_LOG_DEBUG("mible record create: id(%d), len(%d)", record_id, len);
#if USER_RECORD_NUM
    if(record_id >= MIBLE_USER_REC_ID_BASE){
        p_record = user_records;
    }
#endif
    uint8_t index = mible_record_find_index(record_id, len);
    if (INVALID_RECORD_INDEX == index)
    {
        return MI_ERR_NO_MEM;
    }

    uint16_t offset = p_record[index].offset;
    if (p_record[index].record_id != record_id)
    {
        offset = mible_record_get_offset(record_id);
    }

    p_record[index].record_id = record_id;
    p_record[index].len = len;
    p_record[index].offset = offset;
    //records[index].valid = 1;
#if USER_RECORD_NUM
    if(record_id >= MIBLE_USER_REC_ID_BASE){
        user_record_save_header();
    }
#endif
    mible_record_save_header();

    return MI_SUCCESS;
}

mible_status_t mible_record_delete(uint16_t record_id)
{
    uint8_t max_record_num = SYS_RECORD_NUM;
    record_t *p_record = records;
    MI_LOG_DEBUG("mible record delete: %d", record_id);
#if USER_RECORD_NUM
    if(record_id >= MIBLE_USER_REC_ID_BASE){
        max_record_num = USER_RECORD_NUM;
        p_record = user_records;
    }
#endif    
    uint8_t index = 0;
    for (; index < max_record_num; ++index)
    {
        if (p_record[index].valid && (p_record[index].record_id == record_id))
        {
            p_record[index].valid = 0;
#if USER_RECORD_NUM
            if(record_id >= MIBLE_USER_REC_ID_BASE){
                user_record_save_header();
            }
#endif
            mible_record_save_header();
            break;
        }
    }

    mible_arch_evt_param_t param;
    memset(&param, 0, sizeof(param));
    param.record.status = MI_SUCCESS;
    param.record.id = record_id;
    mible_arch_event_callback(MIBLE_ARCH_EVT_RECORD_DELETE, &param);

    return MI_SUCCESS;
}

mible_status_t mible_record_read(uint16_t record_id, uint8_t *p_data,
                                 uint8_t len)
{
    uint8_t max_record_num = SYS_RECORD_NUM;
    record_t *p_record = records;
    MI_LOG_DEBUG("mible record read: id(%d), len(%d)", record_id, len);
#if USER_RECORD_NUM
    if(record_id >= MIBLE_USER_REC_ID_BASE){
        max_record_num = USER_RECORD_NUM;
        p_record = user_records;
    }
#endif
    uint8_t index = 0;
    for (; index < max_record_num; ++index)
    {
        if (p_record[index].valid && (p_record[index].record_id == record_id))
        {
            break;
        }
    }

    if (index >= max_record_num)
    {
        return MI_ERR_INVALID_PARAM;
    }

    if (NULL == p_data)
    {
        return MI_ERR_INVALID_ADDR;
    }

    if ((0 == len) || (len > p_record[index].len))
    {
        return MI_ERR_INVALID_LENGTH;
    }

    if (0 != ftl_load((void *)p_data, p_record[index].offset, len))
    {
        return MI_ERR_INVALID_LENGTH;
    }

    return MI_SUCCESS;
}

mible_status_t mible_record_write(uint16_t record_id, const uint8_t *p_data,
                                  uint8_t len)
{
    uint8_t max_record_num = SYS_RECORD_NUM;
    record_t *p_record = records;
    MI_LOG_DEBUG("mible record write: id(%d), len(%d)", record_id, len);
#if USER_RECORD_NUM
    if(record_id >= MIBLE_USER_REC_ID_BASE){
        max_record_num = USER_RECORD_NUM;
        p_record = user_records;
    }
#endif
    uint8_t index = 0;
    for (; index < max_record_num; ++index)
    {
        if (p_record[index].record_id == record_id)
        {
            break;
        }
    }

    if (index >= max_record_num)
    {
#if !CREATE_WHEN_WRITE
        return MI_ERR_INVALID_PARAM;
#else
        mible_status_t ret = mible_record_create(record_id, len);
        if (MI_SUCCESS != ret)
        {
            return ret;
        }

        /* find index again */
        for (index = 0; index < max_record_num; ++index)
        {
            if (p_record[index].record_id == record_id)
            {
                break;
            }
        }

        if (index >= max_record_num)
        {
            return MI_ERR_INVALID_PARAM;
        }
#endif
    }

    if (NULL == p_data)
    {
        return MI_ERR_INVALID_ADDR;
    }

    if ((0 == len) || (len > p_record[index].len))
    {
        return MI_ERR_INVALID_LENGTH;
    }

    if (0 != ftl_save((void *)p_data, p_record[index].offset, len))
    {
        return MI_ERR_INVALID_LENGTH;
    }

    if (0 == p_record[index].valid)
    {
        p_record[index].valid = 1;
#if USER_RECORD_NUM
        if(record_id >= MIBLE_USER_REC_ID_BASE){
            user_record_save_header();
        }
#endif
        mible_record_save_header();
    }

    mible_arch_evt_param_t param;
    memset(&param, 0, sizeof(param));
    param.record.status = MI_SUCCESS;
    param.record.id = record_id;
    mible_arch_event_callback(MIBLE_ARCH_EVT_RECORD_WRITE, &param);

    return MI_SUCCESS;
}

#ifdef MI_MESH_ENABLED
#include "common/crc32.h"
/**
 * @brief   Use OTA_TMP partition last 8K for Scene store
 * @format          ADDRESS          1B  2B  3B  4B  5B  6B  7B  8B
 *          [0x0087D000-0x0087D008]  -   -   M   I   R   E   C   O
 *          [0x0087D008-0x0087D010]  R   D   -   -  (----CRC32----)
 *          [0x0087D010-0x0087D018] (REC_ID)(LEN)(------VALUE------)
 *          [0x0087Dxxx-0x0087Dxxx] (REC_ID)(LEN)(------VALUE------)
 * */
#define FLASH_ADR_MI_RECORD         0x0087D000
#define FLASH_ADR_MI_RECORD_TMP     0x0087E000
#define FLASH_ADR_MI_RECORD_MAX     0x0087F000
#define RECORD_MAGIC_NUM            "--MIRECORD--"
#define RECORD_MAGIC_LEN            12
#define RECORD_CRC_LEN              4
#define RECORD_MAX_LEN              250
#define RECORD_RESERVE_SPACE        16

#define MIBLE_SCENE_REC_ID_BASE     0x8000

typedef struct{
    uint16_t rec_id;
    uint8_t len ;
    uint8_t dat[29];
}rtk_record_t;

static rtk_record_t rtk_record;
static uint32_t flash_idx_adr = FLASH_ADR_MI_RECORD+RECORD_RESERVE_SPACE;

/**
 * @brief   Sequential search RECORD_MAIN partition
 * @param   [in] record_id: scene id in 0x80~0xA0, and scene map 0x17F
 *          [in/out] *p_adr: address of record value 
 * @return  TRUE    Record Successfully Found 
 *          FALSE   Record not Found
 * */
uint8_t find_record_adr(uint16_t record_id,uint32_t *p_adr) 
{
    uint32_t flash_record_idx =0;
    
    uint8_t  record_buf[512];
    uint32_t record_buf_idx =0;
    rtk_record_t *p_record;
    
    flash_read_locked(FLASH_ADR_MI_RECORD+flash_record_idx, sizeof(record_buf), record_buf);
    
    //check magic num
    if(strncmp((const char *)record_buf, RECORD_MAGIC_NUM, RECORD_MAGIC_LEN)){
        MI_LOG_WARNING("[find_record_adr] invalid MAGIC_NUM");
        return FALSE;
    }
    record_buf_idx += RECORD_RESERVE_SPACE;
    flash_record_idx += RECORD_RESERVE_SPACE;
    
    while(FLASH_ADR_MI_RECORD + flash_record_idx < FLASH_ADR_MI_RECORD_TMP){
        // if record head Incomplete, Read flash from idx
        if((record_buf_idx + 3) >= sizeof(record_buf)){
            flash_read_locked(FLASH_ADR_MI_RECORD+flash_record_idx, sizeof(record_buf), record_buf);
            record_buf_idx = 0;
        }
        
        p_record = (rtk_record_t *)(record_buf + record_buf_idx);
        
        if(p_record->rec_id == 0xffff){
            //search none
            MI_LOG_DEBUG("find_record_adr: id(%d), none", record_id);
            return FALSE;
        }else if (p_record->rec_id == record_id){
            //search success
            *p_adr = FLASH_ADR_MI_RECORD + flash_record_idx;
            //MI_LOG_DEBUG("find_record_adr: id(%d), addr(%d)", record_id, *p_adr);
            return TRUE;
        }else{
            //search next
            flash_record_idx += p_record->len+3;
            record_buf_idx   += p_record->len+3;
        }
    }
    MI_LOG_WARNING("find_record_adr: id(%d), overflow", record_id);
    return FALSE;
}

/**
 * @brief   Sequential write data in RECORD_MAIN partition and increase Pointer
 * @param   [in/out] *p_adr: address of write value 
 *          [in/out] *p_buf: write value buffer
 *          [in] len: value length
 * @return  TRUE    Record Successfully write 
 * */
uint8_t rtk_write_flash(uint32_t *p_adr,uint8_t *p_buf,uint8_t len)
{
    uint32_t tmp_adr = *p_adr;
    flash_write_locked(tmp_adr, len, p_buf);
    tmp_adr += len;
    *p_adr = tmp_adr;
    return TRUE;
}

/**
 * @brief   Find first freedom address
 * @param   [globe] record_adr: blank address
 * */
void rtk_record_part_init(void)
{
    uint8_t *p_buf = (uint8_t *)(&rtk_record);
    uint32_t record_adr = FLASH_ADR_MI_RECORD + RECORD_RESERVE_SPACE;
    // find freedom record address
    while(1){
        flash_read_locked(record_adr, sizeof(rtk_record_t), p_buf);
        if(rtk_record.rec_id == 0xffff || record_adr >= FLASH_ADR_MI_RECORD_TMP){
            flash_idx_adr = record_adr;
            break;
        }
        record_adr += rtk_record.len +3 ;
        MI_LOG_DEBUG("rtk_record_part: id(%d), len(%d)", rtk_record.rec_id, rtk_record.len);
    }
    MI_LOG_DEBUG("rtk_record_part_init: flash_idx_adr(%08x)", flash_idx_adr);
}

/**
 * @brief   Copy origin data from RECORD_MAIN to RECORD_TEMP
 * @step    1. check main page magic
 *          2. erase tmp_area 4k flash.
 *          3. parse all 4k flash & calc crc
 *          4. write magic & crc32 to tmp
 * @return  0 : Successful, others : Failed
 * */
int rtk_record_main_to_temp(void)
{
    uint32_t record_adr = FLASH_ADR_MI_RECORD + RECORD_RESERVE_SPACE;
    uint32_t record_adr_cpy = FLASH_ADR_MI_RECORD_TMP + RECORD_RESERVE_SPACE;
    uint8_t *p_buf = (uint8_t *)(&rtk_record);

    //1. check main page magic
    flash_read_locked(FLASH_ADR_MI_RECORD, sizeof(rtk_record_t), p_buf);
    if(strncmp((const char *)p_buf, RECORD_MAGIC_NUM, RECORD_MAGIC_LEN) != 0){
        MI_LOG_ERROR("[main_to_temp]can't find magic num!!!");
        flash_erase_locked(FLASH_ERASE_SECTOR, FLASH_ADR_MI_RECORD);
        return MI_ERR_NOT_FOUND;
    }

    //2. erase tmp_area 4k flash.
    //flash_erase_locked(FLASH_ERASE_SECTOR, FLASH_ADR_MI_RECORD_TMP);
    //3. parse all 4k flash & calc crc
    uint32_t origin_crc = 0;
    while(1){
        flash_read_locked(record_adr,sizeof(rtk_record_t),p_buf);
        record_adr += sizeof(rtk_record_t);
        if(record_adr >= FLASH_ADR_MI_RECORD_TMP){
            uint32_t last_len = FLASH_ADR_MI_RECORD_TMP + sizeof(rtk_record_t) - record_adr;
            flash_write_locked(record_adr_cpy, last_len, p_buf);
            record_adr_cpy += last_len;
            //calc last crc
            origin_crc = soft_crc32(p_buf, last_len, origin_crc);
            break;
        }
        flash_write_locked(record_adr_cpy, sizeof(rtk_record_t), p_buf);
        record_adr_cpy += sizeof(rtk_record_t);
        //calc crc
        origin_crc = soft_crc32(p_buf, sizeof(rtk_record_t), origin_crc);
    }
    //4. write magic & crc32 to tmp
    MI_LOG_DEBUG("[main_to_temp]origin crc %08x", origin_crc);
    memcpy(p_buf, RECORD_MAGIC_NUM, RECORD_MAGIC_LEN);
    memcpy(p_buf+RECORD_MAGIC_LEN, (uint8_t *)&origin_crc, sizeof(origin_crc));
    flash_write_locked(FLASH_ADR_MI_RECORD_TMP, RECORD_RESERVE_SPACE, p_buf);
    return 0;
}

/**
 * @brief   Copy origin data from RECORD_TEMP to RECORD_MAIN
 * @step    1. check temp page magic & crc
 *          2. erase main_area 4k flash.
 *          3. copy temp to main & delete invalid record
 *          4. erase temp_area 4k flash
 * @return  0 : Successful, others : Failed
 * */
int rtk_record_temp_to_main(void)
{
    int result = 0;
    uint32_t record_adr     = FLASH_ADR_MI_RECORD_TMP + RECORD_RESERVE_SPACE;
    uint32_t record_adr_cpy = FLASH_ADR_MI_RECORD + RECORD_RESERVE_SPACE;
    uint8_t *p_buf = (uint8_t *)(&rtk_record);
    
    //1. check temp page magic & crc
    uint32_t origin_crc = 0, crc32 = 0;
    flash_read_locked(FLASH_ADR_MI_RECORD_TMP, sizeof(rtk_record_t), p_buf);
    if(strncmp((const char *)p_buf, RECORD_MAGIC_NUM, RECORD_MAGIC_LEN) == 0){
        memcpy((uint8_t *)&origin_crc, p_buf+RECORD_MAGIC_LEN, sizeof(origin_crc));
        while(1){
            flash_read_locked(record_adr,sizeof(rtk_record_t),p_buf);
            record_adr += sizeof(rtk_record_t);
            if(record_adr >= FLASH_ADR_MI_RECORD_MAX){
                uint32_t last_len = FLASH_ADR_MI_RECORD_MAX + sizeof(rtk_record_t) - record_adr;
                crc32 = soft_crc32(p_buf, last_len, crc32);
                break;
            }
            crc32 = soft_crc32(p_buf, sizeof(rtk_record_t), crc32);
        }
        MI_LOG_DEBUG("[temp_to_main]origin crc %08x, calc crc %08x", origin_crc, crc32);
        if(origin_crc != crc32){
            MI_LOG_ERROR("[temp_to_main]can't match crc!!!");
            result = MI_ERR_INVALID_PARAM;
            goto end;
        }
        record_adr = FLASH_ADR_MI_RECORD_TMP + RECORD_RESERVE_SPACE;
    }else{
        MI_LOG_ERROR("[temp_to_main]can't find magic num!!!");
        result = MI_ERR_NOT_FOUND;
        goto end;
    }

    //2. erase main_area 4k flash.
    flash_erase_locked(FLASH_ERASE_SECTOR, FLASH_ADR_MI_RECORD);
    //3. copy temp to main
    while(1){
        if(record_adr >= FLASH_ADR_MI_RECORD_MAX){
            MI_LOG_ERROR("[temp_to_main]out of memery, addr %08x!!!", record_adr);
            break;
        }
        flash_read_locked(record_adr,sizeof(rtk_record_t),p_buf);
        if(rtk_record.rec_id == 0xffff){
            break;
        }else if(rtk_record.rec_id == 0){
            // skip erased data
            record_adr += rtk_record.len +3 ;
        }else{
            // need to cpy
            MI_LOG_DEBUG("temp_to_main: rec_id(%d), len(%d)", rtk_record.rec_id, rtk_record.len);
            uint8_t total_len =0;
            total_len = rtk_record.len+3;
            while(total_len){
                if(total_len > sizeof(rtk_record_t)){
                    rtk_write_flash(&record_adr_cpy,p_buf,sizeof(rtk_record_t));
                    total_len -= sizeof(rtk_record_t);
                    record_adr += sizeof(rtk_record_t);
                    flash_read_locked(record_adr,sizeof(rtk_record_t),p_buf);
                }else{
                    rtk_write_flash(&record_adr_cpy,p_buf,total_len);
                    record_adr += total_len;
                    total_len =0;
                }
            }
        }
    }
    //4. erase temp_area 4k flash
    memcpy(p_buf, RECORD_MAGIC_NUM, RECORD_MAGIC_LEN);
    flash_write_locked(FLASH_ADR_MI_RECORD, RECORD_MAGIC_LEN, p_buf);
end:
    flash_erase_locked(FLASH_ERASE_SECTOR, FLASH_ADR_MI_RECORD_TMP);
    return result;
}

/**
 * @brief   Record partition initialization
 * @step    1. check if temp magic & crc valid, copy to main
 *          2. check if main is valid, carry to temp
 *          3. success, carry temp to main 
 *          4. fail, erase main & write magic
 *          5. find first freedom address
 * */
void rtk_record_sort_out(void)
{
    uint8_t *p_buf = (uint8_t *)(&rtk_record);
    //1. check temp is valid
    int result = rtk_record_temp_to_main();
    MI_ERR_CHECK(result);
    //2. check main is valid, carry to temp
    if(result){
        MI_LOG_DEBUG("rtk_record_sort_out: temp invalid, carry main to tmp");
        result = rtk_record_main_to_temp();
        MI_ERR_CHECK(result);
        //3. success, carry to main 
        if(result == 0){
            MI_LOG_DEBUG("rtk_record_sort_out: success, carry tmp to main");
            result = rtk_record_temp_to_main();
            MI_ERR_CHECK(result);
        //4. fail, erase main & write magic
        }else{
            MI_LOG_DEBUG("rtk_record_sort_out: fail, erase main");
            flash_erase_locked(FLASH_ERASE_SECTOR, FLASH_ADR_MI_RECORD);
            memcpy(p_buf, RECORD_MAGIC_NUM, RECORD_MAGIC_LEN);
            flash_write_locked(FLASH_ADR_MI_RECORD, RECORD_MAGIC_LEN, p_buf);
        }
    }
    rtk_record_part_init();
}

/**
 * @brief   delete record header
 * @param   [in] record_id: scene id in 0x80~0xA0, or scene map 0x17F
 * @return  MI_SUCCESS    Record Successfully delete 
 * */
mible_status_t rtk_record_delete(uint16_t record_id)
{
    uint32_t record_adr =0;
    uint8_t *p_buf = (uint8_t *)(&rtk_record);

    if(!find_record_adr(record_id,&record_adr)){
        return MI_ERR_INVALID_PARAM;
    }

    // just set the record id to 0
    rtk_record.rec_id =0;
    flash_write_locked(record_adr, 2, p_buf);
    //MI_LOG_DEBUG("rtk_record_delete: record_id(%d)", record_id);
    return MI_SUCCESS;
}

/**
 * @brief   read record data
 * @param   [in] record_id: scene id in 0x80~0xA0, or scene map 0x17F
 *          [in/out] *p_data: read value buffer
 *          [in] len: value length
 * @return  MI_SUCCESS    Record Successfully read 
 * */
mible_status_t rtk_record_read(uint16_t record_id, uint8_t* p_data,uint8_t len)
{
    uint32_t record_adr =0;
    uint8_t *p_buf = (uint8_t *)(&rtk_record);
    if(len > RECORD_MAX_LEN ||len ==0  ){
        return  MI_ERR_INVALID_LENGTH;
    }else if(p_data == NULL){
        return MI_ERR_INVALID_ADDR;
    }else if(!find_record_adr(record_id,&record_adr)){
        return MI_ERR_INVALID_PARAM;
    }
    flash_read_locked(record_adr, sizeof(rtk_record_t), p_buf);
    //MI_LOG_DEBUG("rtk_record_read: record_id(%d) req_len(%d) saved_len(%d)", record_id, len, rtk_record.len);
    if(len > rtk_record.len){
        return MI_ERR_INVALID_LENGTH;
    }

    if(len <= sizeof(rtk_record.dat)){
        // read form buffer
        memcpy(p_data, rtk_record.dat, len);
        return MI_SUCCESS;
    }else{
        // directly read all the buf part 
        flash_read_locked(record_adr+3, len ,p_data);
        return MI_SUCCESS;
    }
}

/**
 * @brief   write record data
 * @param   [in] record_id: scene id in 0x80~0xA0, or scene map 0x17F
 *          [in] *p_data: write value buffer
 *          [in] len: value length
 * @step    1. check freedom length and sort out
 *          2. write data & calculate crc32
 *          3. delete forward record_id
 * @return  MI_SUCCESS    Record Successfully read 
 * */
mible_status_t rtk_record_write(uint16_t record_id,const uint8_t* p_data,uint8_t len)
{
    uint8_t total_len = len;
    uint8_t buf_idx =0;
    uint8_t *p_buf = (uint8_t *)(&rtk_record);
    if(len > RECORD_MAX_LEN || len == 0){
        return MI_ERR_INVALID_LENGTH;
    }

    if(flash_idx_adr + (len+3)+RECORD_RESERVE_SPACE > FLASH_ADR_MI_RECORD_TMP){
        MI_LOG_DEBUG("rtk_record_write: sort!!!");
        // need to clean the flash first .
        rtk_record_sort_out();
    }

    if(flash_idx_adr + (len+3)+RECORD_RESERVE_SPACE > FLASH_ADR_MI_RECORD_TMP){
        MI_LOG_ERROR("rtk_record_write: out of memery!!!");
        return MI_ERR_NO_MEM;
    }
    
    // write part 
    memset(p_buf,0,sizeof(rtk_record_t));
    rtk_record.rec_id = record_id;
    rtk_record.len = total_len;
    
    // calc origin crc
    uint32_t flash_idx_adr_save = flash_idx_adr, flash_idx_adr_tmp = flash_idx_adr;
    uint32_t origin_crc = 0, crc32 = 0;
    origin_crc = soft_crc32(p_buf, 3, 0);               //calc header
    origin_crc = soft_crc32(p_data, len, origin_crc);   //calc data
    
    // write the header part 
    if(total_len > sizeof(rtk_record.dat)){
        memcpy(rtk_record.dat,p_data,sizeof(rtk_record.dat));
        rtk_write_flash(&flash_idx_adr,p_buf,sizeof(rtk_record));
        total_len -= sizeof(rtk_record.dat);
        buf_idx += sizeof(rtk_record.dat);
    }else{
        memcpy(rtk_record.dat,p_data,total_len);
        rtk_write_flash(&flash_idx_adr,p_buf,total_len+3);
        total_len = 0;
    }
    
    //read back & calc first page crc
    flash_read_locked(flash_idx_adr_tmp, sizeof(rtk_record_t), p_buf);
    crc32 = soft_crc32(p_buf, total_len? sizeof(rtk_record_t) : (len+3), 0);
    
    //MI_LOG_DEBUG("rtk_record_write: id(%d), len(%d)", rtk_record.rec_id, rtk_record.len);
    // write the continus part 
    while(total_len >0){
        flash_idx_adr_tmp += sizeof(rtk_record_t);
        memset(p_buf,0,sizeof(rtk_record_t));
        if(total_len>sizeof(rtk_record_t)){
            memcpy(p_buf,p_data+buf_idx,sizeof(rtk_record_t));
            rtk_write_flash(&flash_idx_adr,p_buf,sizeof(rtk_record_t));
            total_len -= sizeof(rtk_record_t);
            buf_idx += sizeof(rtk_record_t);
        }else{
            memcpy(p_buf,p_data+buf_idx,total_len);
            rtk_write_flash(&flash_idx_adr,p_buf,total_len);
            total_len =0;
            //buf_idx =0;
        }
        //read back & calc continue page crc
        flash_read_locked(flash_idx_adr_tmp, sizeof(rtk_record_t), p_buf);
        crc32 = soft_crc32(p_buf, total_len? sizeof(rtk_record_t) : (len - buf_idx), crc32);
    }
    
    if(origin_crc != crc32){
        MI_LOG_ERROR("check crc fail, origin %08x, calc %08x", origin_crc, crc32);
        flash_read_locked(flash_idx_adr_save, sizeof(rtk_record_t), p_buf);
        rtk_record.rec_id =0;
        flash_write_locked(flash_idx_adr_save, 2, p_buf);
        return MI_ERR_INTERNAL;
    }
    //MI_LOG_DEBUG("check origin crc %08x, calc crc %08x", origin_crc, crc32);

    // delete record ahead
    while(1)
    {
        uint32_t record_adr =0;
        if(!find_record_adr(record_id,&record_adr) || record_adr >= flash_idx_adr_save){
            break;
        }
        rtk_record.rec_id =0;
        flash_write_locked(record_adr, 2, p_buf);
    }
    
    return MI_SUCCESS;
}

mible_status_t mible_scene_delete(uint16_t record_id)
{
    return rtk_record_delete(MIBLE_SCENE_REC_ID_BASE+record_id);
}

mible_status_t mible_scene_read(uint16_t record_id, uint8_t* p_data, uint8_t len)
{
    return rtk_record_read(MIBLE_SCENE_REC_ID_BASE+record_id, p_data, len);
}

mible_status_t mible_scene_write(uint16_t record_id, const uint8_t* p_data, uint8_t len)
{
    return rtk_record_write(MIBLE_SCENE_REC_ID_BASE+record_id, p_data, len);
}

#endif

mible_status_t mible_nvm_init(void)
{
    return MI_SUCCESS;
}

static mible_status_t mible_ota_ctx_save(void)
{
    if (0 != ftl_save((void *)&ota_ctx, OTA_RECORD_OFFSET, sizeof(ota_ctx_t)))
    {
        return MI_ERR_INVALID_LENGTH;
    }

    return MI_SUCCESS;
}

static mible_status_t mible_ota_ctx_load(void)
{
    if (0 != ftl_load((void *)&ota_ctx, OTA_RECORD_OFFSET, sizeof(ota_ctx_t)))
    {
        return MI_ERR_INVALID_LENGTH;
    }

    return MI_SUCCESS;
}

mible_status_t mible_upgrade_firmware(void)
{
    /* load ota context */
    mible_ota_ctx_load();
    MI_LOG_DEBUG("mible_upgrade_firmware: app_exist(%d), secboot_exist(%d)", ota_ctx.app_exist,
                 ota_ctx.secboot_exist);

    uint32_t base_addr = 0;
    T_IMG_CTRL_HEADER_FORMAT *p_ota_header;

    unlock_flash_all();
    if (ota_ctx.app_exist)
    {
        base_addr = PATCH_SIZE + ota_ctx.secboot_exist * SEC_BOOT_SIZE + get_temp_ota_bank_addr_by_img_id(
                        RomPatch);
        MI_LOG_DEBUG("mible_upgrade_firmware: app base addr: 0x%x", base_addr);
        p_ota_header = (T_IMG_CTRL_HEADER_FORMAT *)base_addr;
        p_ota_header->ctrl_flag.flag_value.not_ready = 0;
    }

    if (ota_ctx.secboot_exist)
    {
        base_addr = PATCH_SIZE + get_temp_ota_bank_addr_by_img_id(RomPatch);
        MI_LOG_DEBUG("mible_upgrade_firmware: secure boot base addr: 0x%x", base_addr);
        p_ota_header = (T_IMG_CTRL_HEADER_FORMAT *)base_addr;
        p_ota_header->ctrl_flag.flag_value.not_ready = 0;
    }

    base_addr = get_temp_ota_bank_addr_by_img_id(RomPatch);
    MI_LOG_DEBUG("mible_upgrade_firmware: patch base addr: 0x%x", base_addr);
    p_ota_header = (T_IMG_CTRL_HEADER_FORMAT *)base_addr;
    p_ota_header->ctrl_flag.flag_value.not_ready = 0;

    MI_LOG_DEBUG("upgrade verify ok, restarting....");
    plt_reset(0);
    return MI_SUCCESS;
}

mible_status_t mible_nvm_read(void *p_data, uint32_t length, uint32_t address)
{
    if (!flash_read_locked(address, length, p_data))
    {
        return MI_ERR_INTERNAL;
    }

    return MI_SUCCESS;
}

mible_status_t mible_nvm_write(void *p_data, uint32_t length, uint32_t address)
{
    MI_LOG_DEBUG("mible_nvm_write: address = 0x%x, length = %d, p_data = 0x%x", address, length,
                 p_data);

    if (NULL == p_data)
    {
        MI_LOG_WARNING("mible_nvm_write: invalid data");
        return MI_ERR_INVALID_PARAM;
    }

    if ((address >= OTP->ota_tmp_addr) && (address < (OTP->ota_tmp_addr + OTP->ota_tmp_size)))
    {
        if (address == OTP->ota_tmp_addr)
        {
            /* ota started, clear context first */
            //ota_ctx.patch_exist = FALSE;
            ota_ctx.app_exist = FALSE;
            ota_ctx.secboot_exist = FALSE;
            mible_ota_ctx_save();

            T_IMG_CTRL_HEADER_FORMAT *p_header = (T_IMG_CTRL_HEADER_FORMAT *)p_data;
            if (p_header->image_id == RomPatch)
            {
                //ota_ctx.patch_exist = TRUE;
                if (p_header->ctrl_flag.flag_value.rsvd & 0x08)
                {
                    ota_ctx.secboot_exist = TRUE;
                }
                if (p_header->ctrl_flag.flag_value.rsvd & 0x10)
                {
                    ota_ctx.app_exist = TRUE;
                }
                mible_ota_ctx_save();
            }
            if (AppPatch == p_header->image_id)
            {
                MI_LOG_WARNING("mible_nvm_write: invalid id(AppPatch)");
                return MI_ERR_INTERNAL;
            }

            if (SecureBoot == p_header->image_id)
            {
                MI_LOG_WARNING("mible_nvm_write: invalid id(SecureBoot)");
                return MI_ERR_INTERNAL;
            }

            MI_LOG_DEBUG("mible_nvm_write: new image header:0x%08x, signature:0x%08x, dfu_base_addr:0x%08x",
                         length, p_header->image_id, address);
        }

        /* new page starts */
        if (0 == (address % FMC_SEC_SECTION_LEN))
        {
            flash_erase_locked(FLASH_ERASE_SECTOR, address);
        }
    }

    flash_write_locked(address, length, p_data);

    return MI_SUCCESS;
}

