#include "rible_api.h"
#include "rible_log.h"

#include "string.h"
#include "profile_server.h"
#include "platform_types.h"
#include "platform_os.h"

#define MAX_GATT_DB_NUM                      4 //To support multi services
#define GATT_TABLE_VALUE_ALLOC               1

static T_APP_RESULT result_g = APP_RESULT_SUCCESS;

static uint16_t srv_start_handle = 0;

typedef struct
{
    bool used;
    uint16_t start_handle;
    uint16_t service_id;
    uint16_t attr_count;
    T_ATTRIB_APPL *pattrs;
} service_db_t;

static service_db_t srv_dbs[MAX_GATT_DB_NUM];

static service_db_t *request_service_db(void)
{
    for (uint8_t i = 0; i < MAX_GATT_DB_NUM; ++i)
    {
        if (!srv_dbs[i].used)
        {
            return &srv_dbs[i];
        }
    }

    return NULL;
}


static uint16_t calculate_service_db_size(rible_gatts_srv_db_t *pservice)
{
    uint16_t size = 0;
    rible_gatts_char_db_t *pcharacter = NULL;
    rible_gatts_char_desc_db_t *pdescriptor = NULL;

    /* increase primary or secondary service */
    size ++;

    /* increase characteristics */
    for (uint8_t char_idx = 0; char_idx < pservice->char_num; ++char_idx)
    {
        pcharacter = pservice->p_char_db + char_idx;
        /* increase characterstic */
        size ++;
        /* increase characterstic value */
        size ++;

        /* increase characteristic descriptor */
        if (0 != (pcharacter->char_property & RIBLE_NOTIFY))
        {
            /* increase cccd */
            size ++;
        }

        if (0 != (pcharacter->char_property & RIBLE_BROADCAST))
        {
            /* increase sccd */
            size ++;
        }

        pdescriptor = &(pcharacter->char_desc_db);
        if (NULL != pdescriptor->extend_prop)
        {
            /* increase characteristic extended properties */
            size ++;
        }

        if (NULL != pdescriptor->char_format)
        {
            /* increase characteristic format */
            size ++;
        }

        if (NULL != pdescriptor->user_desc)
        {
            /* increase characteristic user description */
            size ++;
        }
    }

    //RI_LOG_DEBUG("service table count = %d", size);
    return size * sizeof(T_ATTRIB_APPL);
}

/******************************************************
 * gatt server interfaces
 ******************************************************/

static rible_status_t add_primary_service(T_ATTRIB_APPL *pattr, rible_gatts_srv_db_t *pservice)
{
    /* fill service */
    if (RIBLE_PRIMARY_SERVICE == pservice->srv_type)
    {
        pattr->type_value[0] = LO_WORD(GATT_UUID_PRIMARY_SERVICE);
        pattr->type_value[1] = HI_WORD(GATT_UUID_PRIMARY_SERVICE);
    }
    else
    {
        pattr->type_value[0] = LO_WORD(GATT_UUID_SECONDARY_SERVICE);
        pattr->type_value[1] = HI_WORD(GATT_UUID_SECONDARY_SERVICE);
    }

    pattr->permissions = GATT_PERM_READ;
    if (0 == pservice->srv_uuid.type)
    {
        /* uuid16 */
        pattr->flags = (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE);
        pattr->type_value[2] = LO_WORD(pservice->srv_uuid.uuid16);
        pattr->type_value[3] = HI_WORD(pservice->srv_uuid.uuid16);
        pattr->value_len = UUID_16BIT_SIZE;
        pattr->p_value_context = NULL;
    }
    else
    {
        /* uuid128 */
        pattr->flags = (ATTRIB_FLAG_VOID | ATTRIB_FLAG_LE);
        pattr->value_len = UUID_128BIT_SIZE;
#if GATT_TABLE_VALUE_ALLOC
        pattr->p_value_context = plt_malloc(UUID_128BIT_SIZE, RAM_TYPE_DATA_OFF);
        if (NULL == pattr->p_value_context)
        {
            return RI_ERR_NO_MEM;
        }
        memcpy(pattr->p_value_context, pservice->srv_uuid.uuid128, UUID_128BIT_SIZE);
#else
        pattr->p_value_context = pservice->srv_uuid.uuid128;
#endif
    }

    return RI_SUCCESS;
}


static rible_status_t add_characteristic(T_ATTRIB_APPL *pattr, rible_gatts_char_db_t *pcharacter)
{
    /* fill characterstic */
    pattr->flags = ATTRIB_FLAG_VALUE_INCL;
    pattr->type_value[0] = LO_WORD(GATT_UUID_CHARACTERISTIC);
    pattr->type_value[1] = HI_WORD(GATT_UUID_CHARACTERISTIC);
    pattr->type_value[2] = pcharacter->char_property;
    pattr->value_len  = 1;
    pattr->p_value_context = NULL;
    pattr->permissions = GATT_PERM_READ;
    return RI_SUCCESS;
}

static rible_status_t add_characteristic_value(T_ATTRIB_APPL *pattr,
                                               rible_gatts_char_db_t *pcharacter)
{
    /* fill characteristic value */
    if (0 == pcharacter->char_uuid.type)
    {
        /* uuid16 */
        pattr->type_value[0] = LO_WORD(pcharacter->char_uuid.uuid16);
        pattr->type_value[1] = HI_WORD(pcharacter->char_uuid.uuid16);
    }
    else
    {
        /* uuid128 */
        pattr->flags = ATTRIB_FLAG_UUID_128BIT;
        memcpy(pattr->type_value, pcharacter->char_uuid.uuid128, UUID_128BIT_SIZE);
    }

    /* get data from p_value_context */
    pattr->flags |= ATTRIB_FLAG_VALUE_APPL;
    pattr->value_len = pcharacter->char_value_len;
    pattr->value_len <<= 8;
#if GATT_TABLE_VALUE_ALLOC
    pattr->p_value_context = plt_malloc(pcharacter->char_value_len, RAM_TYPE_DATA_ON);
    memset(pattr->p_value_context, 0, pcharacter->char_value_len);
    if (NULL == pattr->p_value_context)
    {
        return RI_ERR_NO_MEM;
    }
    if (NULL != pcharacter->p_value)
    {
        memcpy(pattr->p_value_context, pcharacter->p_value, pcharacter->char_value_len);
    }
#else
    pattr->p_value_context = pcharacter->p_value;
#endif
    
    if (pcharacter->rd_author)
    {
        pattr->permissions |= GATT_PERM_READ_AUTHOR_REQ;
    }
    else
    {
        pattr->permissions |= GATT_PERM_READ;
    }

    if (pcharacter->wr_author)
    {
        pattr->permissions |= GATT_PERM_WRITE_AUTHOR_REQ;
    }
    else
    {
        pattr->permissions |= GATT_PERM_WRITE;
    }
   
    return RI_SUCCESS;
}

static rible_status_t add_cccd(T_ATTRIB_APPL *pattr)
{
    pattr->flags = (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_CCCD_APPL);
    pattr->type_value[0] = LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG);
    pattr->type_value[1] = HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG);
    pattr->type_value[2] = LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT);
    pattr->type_value[3] = HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT);
    pattr->value_len = 2;
    pattr->p_value_context = NULL;
    pattr->permissions = (GATT_PERM_READ | GATT_PERM_WRITE);
    return RI_SUCCESS;
}

static rible_status_t add_sccd(T_ATTRIB_APPL *pattr)
{
    pattr->flags = ATTRIB_FLAG_VALUE_INCL;
    pattr->type_value[0] = LO_WORD(GATT_UUID_CHAR_SERVER_CONFIG);
    pattr->type_value[1] = HI_WORD(GATT_UUID_CHAR_SERVER_CONFIG);
    //pattr->type_value[2] = LO_WORD(GATT_SERVER_CHAR_CONFIG_BROADCAST);
    //pattr->type_value[3] = HI_WORD(GATT_SERVER_CHAR_CONFIG_BROADCAST);
    pattr->type_value[2] = 0;
    pattr->type_value[3] = 0;
    pattr->value_len = 2;
    pattr->p_value_context = NULL;
    pattr->permissions = (GATT_PERM_READ | GATT_PERM_WRITE);
    return RI_SUCCESS;
}

static rible_status_t add_extended_properties(T_ATTRIB_APPL *pattr,
                                              rible_gatts_char_desc_ext_prop_t *pextended_properties)
{
    uint16_t property = 0;
    if (pextended_properties->reliable_write)
    {
        property |= 0x01;
    }

    if (pextended_properties->reliable_write)
    {
        property |= 0x02;
    }
    pattr->flags = ATTRIB_FLAG_VALUE_INCL;
    pattr->type_value[0] = LO_WORD(GATT_UUID_CHAR_EXTENDED_PROP);
    pattr->type_value[1] = HI_WORD(GATT_UUID_CHAR_EXTENDED_PROP);
    pattr->type_value[2] = LO_WORD(property);
    pattr->type_value[3] = HI_WORD(property);
    pattr->value_len = 2;
    pattr->p_value_context = NULL;
    pattr->permissions = GATT_PERM_READ;

    return RI_SUCCESS;
}

static rible_status_t add_char_format(T_ATTRIB_APPL *pattr,
                                      rible_gatts_char_desc_cpf_t *pchar_format)
{
    pattr->flags = ATTRIB_FLAG_VALUE_INCL;
    pattr->type_value[0] = LO_WORD(GATT_UUID_CHAR_FORMAT);
    pattr->type_value[1] = HI_WORD(GATT_UUID_CHAR_FORMAT);
    pattr->type_value[2] = pchar_format->format;
    pattr->type_value[3] = pchar_format->exponent;
    pattr->type_value[4] = LO_WORD(pchar_format->unit);
    pattr->type_value[5] = HI_WORD(pchar_format->unit);
    pattr->type_value[6] = pchar_format->name_space;
    pattr->type_value[7] = LO_WORD(pchar_format->desc);
    pattr->type_value[8] = HI_WORD(pchar_format->desc);
    pattr->value_len = 7;
    pattr->p_value_context = NULL;
    pattr->permissions = GATT_PERM_READ;

    return RI_SUCCESS;
}

static rible_status_t add_user_desc(T_ATTRIB_APPL *pattr,
                                    rible_gatts_char_desc_user_desc_t *puser_desc)
{
    pattr->flags = ATTRIB_FLAG_VOID;
    pattr->type_value[0] = LO_WORD(GATT_UUID_CHAR_USER_DESCR);
    pattr->type_value[1] = HI_WORD(GATT_UUID_CHAR_USER_DESCR);
    pattr->value_len = puser_desc->len;
#if GATT_TABLE_VALUE_ALLOC
    pattr->p_value_context = plt_malloc(puser_desc->len, RAM_TYPE_DATA_OFF);
    memset(pattr->p_value_context, 0, puser_desc->len);
    if (NULL == pattr->p_value_context)
    {
        return RI_ERR_NO_MEM;
    }
    if (NULL != puser_desc->string)
    {
        memcpy(pattr->p_value_context, puser_desc->string, puser_desc->len);
    }
#else
    pattr->p_value_context = puser_desc->string;
#endif
    pattr->permissions = (GATT_PERM_READ | GATT_PERM_WRITE);

    return RI_SUCCESS;
}

//��Ӧ�ص�����
//static T_APP_RESULT ri_server_read_cb(uint8_t conn_id, T_SERVER_ID service_id,
//                                      uint16_t attrib_index, uint16_t offset, uint16_t *p_length, uint8_t **pp_value)
//{
//    //MI_LOG_DEBUG("read data from handle(%d-%d)", service_id, attrib_index);
//    T_APP_RESULT result;
//    uint8_t index;
//    for (index = 0; index < MAX_GATT_DB_NUM; ++index)
//    {
//        if (srv_dbs[index].used && (srv_dbs[index].service_id == service_id))
//        {
//            break;
//        }
//    }

//    if (index >= MAX_GATT_DB_NUM)
//    {
//        return APP_RESULT_ATTR_NOT_FOUND;
//    }

//    if (attrib_index >= srv_dbs[index].attr_count)
//    {
//        return APP_RESULT_ATTR_NOT_FOUND;
//    }

//    rible_gatts_evt_t event;
//    if (srv_dbs[index].pattrs[attrib_index].permissions & GATT_PERM_READ_AUTHOR_REQ)
//    {
//        event = RIBLE_GATTS_EVT_READ_PERMIT_REQ;
//        rible_gatts_evt_param_t param;
//        memset(&param, 0, sizeof(param));
//        param.conn_handle = conn_id;
//        param.read.value_handle = srv_dbs[index].start_handle + attrib_index;
//        rible_gatts_event_callback(event, &param);
//    }

//    result = result_g;
//    if (APP_RESULT_SUCCESS == result)
//    {
//        if (srv_dbs[index].pattrs[attrib_index].flags & ATTRIB_FLAG_VALUE_APPL)
//        {
//            *p_length = (srv_dbs[index].pattrs[attrib_index].value_len & 0xff);
//            if (*p_length <= offset)
//            {
//                *p_length = 0;
//				*pp_value = NULL;
//            }
//            else
//            {
//				*p_length = (srv_dbs[index].pattrs[attrib_index].value_len & 0xff) - offset;
//                *pp_value = (uint8_t *)(srv_dbs[index].pattrs[attrib_index].p_value_context) + offset;
//            }
//        }
//    }
//    result_g = APP_RESULT_SUCCESS;
//    return result;
//}

//static T_APP_RESULT ri_server_write_cb(uint8_t conn_id, T_SERVER_ID service_id,
//                                       uint16_t attrib_index,
//                                       T_WRITE_TYPE write_type, uint16_t len, uint8_t *pvalue, P_FUN_WRITE_IND_POST_PROC *ppost_proc)
//{
//    //MI_LOG_DEBUG("write data to handle(%d-%d)", service_id, attrib_index);
//    T_APP_RESULT result;
//    uint8_t index;
//    for (index = 0; index < MAX_GATT_DB_NUM; ++index)
//    {
//        if (srv_dbs[index].used && (srv_dbs[index].service_id == service_id))
//        {
//            break;
//        }
//    }

//    if (index >= MAX_GATT_DB_NUM)
//    {
//        return APP_RESULT_ATTR_NOT_FOUND;
//    }

//    if (attrib_index >= srv_dbs[index].attr_count)
//    {
//        return APP_RESULT_ATTR_NOT_FOUND;
//    }

//    rible_gatts_evt_t event = RIBLE_GATTS_EVT_WRITE;
//    if (srv_dbs[index].pattrs[attrib_index].permissions & GATT_PERM_WRITE_AUTHOR_REQ)
//    {
//        event = RIBLE_GATTS_EVT_WRITE_PERMIT_REQ;
//    }

//    rible_gatts_evt_param_t param;
//    memset(&param, 0, sizeof(param));
//    param.conn_handle = conn_id;
//    param.write.value_handle = srv_dbs[index].start_handle + attrib_index;
//    param.write.offset = 0;
//    param.write.data = pvalue;
//    param.write.len = len;

//    rible_gatts_event_callback(event, &param);

//    result = result_g;
//    if (APP_RESULT_SUCCESS == result)
//    {
//        if (srv_dbs[index].pattrs[attrib_index].flags & ATTRIB_FLAG_VALUE_APPL)
//        {
//            //MI_LOG_DEBUG("gatt server set appl value by remote");
//            uint16_t max_len = (srv_dbs[index].pattrs[attrib_index].value_len >> 8);
//            uint16_t write_len = (max_len < len) ? max_len : len;
//            memcpy((uint8_t *)(srv_dbs[index].pattrs[attrib_index].p_value_context), pvalue, write_len);
//            srv_dbs[index].pattrs[attrib_index].value_len &= ~0xff;
//            srv_dbs[index].pattrs[attrib_index].value_len |= write_len;
//        }
//    }
//    result_g = APP_RESULT_SUCCESS;
//    return result;
//}

//static void ri_server_cccd_update_cb(uint8_t conn_id, T_SERVER_ID server_id,
//                                     uint16_t attrib_index,
//                                     uint16_t cccd_bits)
//{
//    //MI_LOG_DEBUG("update cccd data of handle(%d)", attrib_index);
//}

//const T_FUN_GATT_SERVICE_CBS ri_server_cbs =
//{
//    .read_attr_cb = ri_server_read_cb,  /** read callback function pointer */
//    .write_attr_cb = ri_server_write_cb, /** write callback function pointer */
//    .cccd_update_cb = ri_server_cccd_update_cb  /** cccd update callback function pointer */
//};


static void calculate_char_handle(rible_gatts_srv_db_t *pservice, uint16_t start_handle)
{
    uint16_t handle = start_handle;
    rible_gatts_char_db_t *pcharacter = NULL;
    rible_gatts_char_desc_db_t *pdescriptor = NULL;

    /* calculate characteristics handle */
    for (uint8_t char_idx = 0; char_idx < pservice->char_num; ++char_idx)
    {
        pcharacter = pservice->p_char_db + char_idx;
        /* calculate characterstic handle */
        handle ++;
        /* calculate characterstic value handle */
        handle ++;

        pcharacter->char_value_handle = handle;
        //MI_LOG_DEBUG("character value handle: %d", pcharacter->char_value_handle);

        /* increase characteristic descriptor handle */
        if (0 != (pcharacter->char_property & RIBLE_NOTIFY))
        {
            /* increase cccd handle */
            handle ++;
        }

        if (0 != (pcharacter->char_property & RIBLE_BROADCAST))
        {
            /* increase sccd handle */
            handle ++;
        }

        pdescriptor = &(pcharacter->char_desc_db);
        if (NULL != pdescriptor->extend_prop)
        {
            /* increase characteristic extended properties handle */
            handle ++;
        }

        if (NULL != pdescriptor->char_format)
        {
            /* increase characteristic format handle */
            handle ++;
        }

        if (NULL != pdescriptor->user_desc)
        {
            /* increase characteristic user description handle */
            handle ++;
        }
    }
}

/**
 * @brief   Add a Service to a GATT server
 * @param   [in|out] p_server_db: pointer to rible service data type
 * of rible_gatts_db_t, see TYPE rible_gatts_db_t for details.
 * @return  RI_SUCCESS             Successfully added a service declaration.
 *          RI_ERR_INVALID_ADDR    Invalid pointer supplied.
 *          RI_ERR_INVALID_PARAM   Invalid parameter(s) supplied.
 *          R_ERR_NO_MEM          Not enough memory to complete operation.
 * @note    This function can be implemented asynchronous. When service inition complete, call rible_arch_event_callback function and pass in RIBLE_ARCH_EVT_GATTS_SRV_INIT_CMP event and result.
 * */
rible_status_t rible_gatts_service_init(rible_gatts_db_t *p_server_db)
{
    T_ATTRIB_APPL *pattr = NULL;
    rible_gatts_srv_db_t *pservice = NULL;
    rible_gatts_char_db_t *pcharacter = NULL;
    rible_gatts_char_desc_db_t *pdescriptor = NULL;
    uint16_t service_table_size = 0;
    service_db_t *psrv_db = NULL;
    /* validation database space */
    uint8_t unused_count = 0;
    for (uint8_t i = 0; i < MAX_GATT_DB_NUM; ++i)
    {
        if (!srv_dbs[i].used)
        {
            unused_count ++;
        }
    }
    if (unused_count < p_server_db->srv_num)
    {
        return RI_ERR_NO_MEM;
    }

    /* add service databases */
    for (uint8_t srv_idx = 0; srv_idx < p_server_db->srv_num; ++srv_idx)
    {
        pservice = p_server_db->p_srv_db + srv_idx;

        /* find unused space */
        psrv_db = request_service_db();
        if (NULL == psrv_db)
        {
            return RI_ERR_NO_MEM;
        }

        /* allocate service table */
        service_table_size = calculate_service_db_size(pservice);
        psrv_db->pattrs = (T_ATTRIB_APPL *)plt_malloc(service_table_size, RAM_TYPE_DATA_OFF);
        if (NULL == psrv_db->pattrs)
        {
            return RI_ERR_NO_MEM;
        }
        /* zero service table */
        memset(psrv_db->pattrs, 0, service_table_size);
        psrv_db->used = TRUE;
        psrv_db->attr_count = service_table_size / sizeof(T_ATTRIB_APPL);
        pattr = psrv_db->pattrs;

        /* add primary or secondary service */
        add_primary_service(pattr, pservice);
        pattr ++;

        /* fill characteristics */
        for (uint8_t char_idx = 0; char_idx < pservice->char_num; ++char_idx)
        {
            /* add characterstic */
            pcharacter = pservice->p_char_db + char_idx;
            add_characteristic(pattr, pcharacter);
            pattr ++;
            add_characteristic_value(pattr, pcharacter);
            pattr ++;

            /* add characteristic descriptor */
            if (0 != (pcharacter->char_property & RIBLE_NOTIFY))
            {
                /* add cccd */
                add_cccd(pattr);
                pattr ++;
            }

            if (0 != (pcharacter->char_property & RIBLE_BROADCAST))
            {
                /* add sccd */
                add_sccd(pattr);
                pattr ++;
            }

            pdescriptor = &(pcharacter->char_desc_db);
            if (NULL != pdescriptor->extend_prop)
            {
                /* add characteristic extended properties */
                add_extended_properties(pattr, pdescriptor->extend_prop);
                pattr ++;
            }

            if (NULL != pdescriptor->char_format)
            {
                /* add characteristic format */
                add_char_format(pattr, pdescriptor->char_format);
                pattr ++;
            }

            if (NULL != pdescriptor->user_desc)
            {
                /* add characteristic user description */
                add_user_desc(pattr, pdescriptor->user_desc);
                pattr ++;
            }
        }

        /* add service */
        uint8_t service_id = SERVICE_PROFILE_GENERAL_ID;
//        if (false == server_add_service(&service_id, (uint8_t *)psrv_db->pattrs,
//                                        service_table_size, ri_server_cbs))
//        {
//            //MI_LOG_ERROR("server add failed!");
//            return RI_ERR_NO_MEM;
//        }
        psrv_db->service_id = service_id;

        /* calculate characteristic handle */
        calculate_char_handle(pservice, srv_start_handle);

        /* calculate start handle */
        psrv_db->start_handle = srv_start_handle;
        srv_start_handle += psrv_db->attr_count;
    }

//    rible_arch_evt_param_t param;
//    memset(&param, 0, sizeof(param));
//    param.srv_init_cmp.status = RI_SUCCESS;
//    param.srv_init_cmp.p_gatts_db = p_server_db;
    //rible_arch_event_callback(RIBLE_ARCH_EVT_GATTS_SRV_INIT_CMP, &param);

    //dump_attribute();

    return RI_SUCCESS;
}


static service_db_t *find_srv_db(uint8_t attr_handle)
{
    for (uint8_t i = 0; i < MAX_GATT_DB_NUM; ++i)
    {
        if (srv_dbs[i].used && ((srv_dbs[i].start_handle <= attr_handle) &&
                                ((srv_dbs[i].start_handle + srv_dbs[i].attr_count) > attr_handle)))
        {
            return &srv_dbs[i];
        }
    }
    return NULL;
}

static bool is_fixed_uuid(uint16_t uuid)
{
    if ((GATT_UUID_PRIMARY_SERVICE == uuid) ||
        (GATT_UUID_SECONDARY_SERVICE == uuid) ||
        (GATT_UUID_INCLUDE == uuid) ||
        (GATT_UUID_CHARACTERISTIC == uuid) ||
        (GATT_UUID_CHAR_EXTENDED_PROP == uuid) ||
        (GATT_UUID_CHAR_USER_DESCR == uuid) ||
        (GATT_UUID_CHAR_CLIENT_CONFIG == uuid) ||
        (GATT_UUID_CHAR_SERVER_CONFIG == uuid) ||
        (GATT_UUID_CHAR_FORMAT == uuid) ||
        (GATT_UUID_CHAR_AGGR_FORMAT == uuid) ||
        (GATT_UUID_CHAR_VALID_RANGE == uuid) ||
        (GATT_UUID_CHAR_EXTERNAL_REPORT_REFERENCE == uuid) ||
        (GATT_UUID_CHAR_REPORT_REFERENCE == uuid) ||
        (GATT_UUID_CHAR_DESCRIPTOR_NUM_OF_DIGITALS == uuid) ||
        (GATT_UUID_CHAR_DESCRIPTOR_VALUE_TRIGGER_SETTING == uuid) ||
        (GATT_UUID_CHAR_SENSING_CONFIGURATION == uuid) ||
        (GATT_UUID_CHAR_SENSING_MEASUREMENT == uuid) ||
        (GATT_UUID_CHAR_SENSING_TRIGGER_SETTING == uuid) ||
        (GATT_UUID_CHAR_DESCRIPTOR_TIME_TRIGGER_SETTING  == uuid))
    {
        return TRUE;
    }

    return FALSE;
}

/**
 * @brief   Set characteristic value
 * @param   [in] srv_handle: service handle
 *          [in] value_handle: characteristic value handle
 *          [in] offset: the offset from which the attribute value has
 *to be updated
 *          [in] p_value: pointer to data
 *          [in] len: data length
 * @return  RI_SUCCESS             Successfully retrieved the value of the
 *attribute.
 *          RI_ERR_INVALID_ADDR    Invalid pointer supplied.
 *          RI_ERR_INVALID_PARAM   Invalid parameter (offset) supplied.
 *          RI_ERR_INVALID_LENGTH   Invalid length supplied.
 *          RIBLE_ERR_ATT_INVALID_HANDLE     Attribute not found.
 *          RIBLE_ERR_GATT_INVALID_ATT_TYPE  Attributes are not modifiable by
 *the application.
 * */
rible_status_t rible_gatts_value_set(uint16_t srv_handle,
                                     uint16_t value_handle, uint8_t offset, uint8_t *p_value, uint8_t len)
{
    if (NULL == p_value)
    {
        //MI_LOG_ERROR("gatt server set failed: invalue address");
        return RI_ERR_INVALID_ADDR;
    }

    service_db_t *psrv_db = find_srv_db(value_handle);
    if (NULL == psrv_db)
    {
        return RIBLE_ERR_ATT_INVALID_ATT_HANDLE;
    }

    uint8_t attr_idx = value_handle - psrv_db->start_handle;
    /* get uuid */
    uint16_t uuid = psrv_db->pattrs[attr_idx].type_value[1];
    uuid <<= 8;
    uuid |= psrv_db->pattrs[attr_idx].type_value[0];
    if (is_fixed_uuid(uuid))
    {
        //MI_LOG_ERROR("gatt server set failed: fixed uuid(%d)", uuid);
        return RIBLE_ERR_GATT_INVALID_ATT_TYPE;
    }

    //MI_LOG_DEBUG("gatt server set uuid(%d) value", uuid);

    if (psrv_db->pattrs[attr_idx].flags == ATTRIB_FLAG_VOID)
    {
        //MI_LOG_DEBUG("gatt server set void value");
        uint16_t max_len = psrv_db->pattrs[attr_idx].value_len;
        uint16_t write_len = (max_len < (offset + len)) ? (max_len - offset) : len;
        memcpy((uint8_t *)(psrv_db->pattrs[attr_idx].p_value_context) + offset, p_value, write_len);
    }
    else if (psrv_db->pattrs[attr_idx].flags & ATTRIB_FLAG_VALUE_INCL)
    {
        //MI_LOG_DEBUG("gatt server set incl value");
        uint16_t max_len = psrv_db->pattrs[attr_idx].value_len;
        uint16_t write_len = (max_len < (offset + len)) ? (max_len - offset) : len;
        memcpy(psrv_db->pattrs[attr_idx].type_value + 2 + offset, p_value, write_len);
    }
    else if (psrv_db->pattrs[attr_idx].flags & ATTRIB_FLAG_VALUE_APPL)
    {
        //MI_LOG_DEBUG("gatt server set appl value");
        uint16_t max_len = (psrv_db->pattrs[attr_idx].value_len >> 8);
        uint16_t write_len = (max_len < (offset + len)) ? (max_len - offset) : len;
        memcpy((uint8_t *)(psrv_db->pattrs[attr_idx].p_value_context) + offset, p_value, write_len);
        psrv_db->pattrs[attr_idx].value_len &= ~0xff;
        psrv_db->pattrs[attr_idx].value_len |= (offset + write_len);
    }
    else
    {
        //MI_LOG_DEBUG("gatt server set faild: invalid flag(%d)", psrv_db->pattrs[attr_idx].flags);
        return RIBLE_ERR_GATT_INVALID_ATT_TYPE;
    }

    return RI_SUCCESS;
}

/**
 * @brief   Get charicteristic value as a GATTS.
 * @param   [in] srv_handle: service handle
 *          [in] value_handle: characteristic value handle
 *          [out] p_value: pointer to data which stores characteristic value
 *          [out] p_len: pointer to data length.
 * @return  RI_SUCCESS             Successfully get the value of the attribute.
 *          RI_ERR_INVALID_ADDR    Invalid pointer supplied.
 *          RI_ERR_INVALID_PARAM   Invalid parameter (offset) supplied.
 *          RI_ERR_INVALID_LENGTH   Invalid length supplied.
 *          RIBLE_ERR_ATT_INVALID_HANDLE     Attribute not found.
 **/
rible_status_t rible_gatts_value_get(uint16_t srv_handle,
                                     uint16_t value_handle, uint8_t *p_value, uint8_t *p_len)
{
    if (NULL == p_value)
    {
        return RI_ERR_INVALID_ADDR;
    }

    if (NULL == p_len)
    {
        return RI_ERR_INVALID_LENGTH;
    }

    service_db_t *psrv_db = find_srv_db(value_handle);
    if (NULL == psrv_db)
    {
        return RIBLE_ERR_ATT_INVALID_ATT_HANDLE;
    }

    uint8_t attr_idx = value_handle - psrv_db->start_handle;

    if (psrv_db->pattrs[attr_idx].flags == ATTRIB_FLAG_VOID)
    {
        *p_len = psrv_db->pattrs[attr_idx].value_len;
        memcpy(p_value, psrv_db->pattrs[attr_idx].p_value_context, *p_len);
    }
    else if (psrv_db->pattrs[attr_idx].flags & ATTRIB_FLAG_VALUE_INCL)
    {
        *p_len = psrv_db->pattrs[attr_idx].value_len;
        memcpy(p_value, psrv_db->pattrs[attr_idx].type_value + 2, *p_len);
    }
    else if (psrv_db->pattrs[attr_idx].flags & ATTRIB_FLAG_VALUE_APPL)
    {
        *p_len = (psrv_db->pattrs[attr_idx].value_len & 0xff);
        memcpy(p_value, psrv_db->pattrs[attr_idx].p_value_context, *p_len);
    }
    else
    {
        return RIBLE_ERR_ATT_INVALID_ATT_HANDLE;
    }

    return RI_SUCCESS;
}

/**
 * @brief   Set characteristic value and notify it to client.
 * @param   [in] conn_handle: conn handle
 *          [in] srv_handle: service handle
 *          [in] char_value_handle: characteristic  value handle
 *          [in] offset: the offset from which the attribute value has to
 * be updated
 *          [in] p_value: pointer to data
 *          [in] len: data length
 *          [in] type : notification = 1; indication = 2;
 *
 * @return  RI_SUCCESS             Successfully queued a notification or
 * indication for transmission,
 *          RI_ERR_INVALID_ADDR    Invalid pointer supplied.
 *          RI_ERR_INVALID_PARAM   Invalid parameter (offset) supplied.
 *          RI_ERR_INVALID_STATE   Invalid Connection State or notifications
 * and/or indications not enabled in the CCCD.
 *          RI_ERR_INVALID_LENGTH   Invalid length supplied.
 *          RI_ERR_BUSY            Procedure already in progress.
 *          RIBLE_ERR_ATT_INVALID_HANDLE     Attribute not found.
 *          RIBLE_ERR_GATT_INVALID_ATT_TYPE   //Attributes are not modifiable by
 * the application.
 * @note    This function checks for the relevant Client Characteristic
 * Configuration descriptor value to verify that the relevant operation (notification or
 * indication) has been enabled by the client.
 * */
rible_status_t rible_gatts_notify_or_indicate(uint16_t conn_handle,
                                              uint16_t srv_handle, uint16_t char_value_handle, uint8_t offset,
                                              uint8_t *p_value, uint8_t len, uint8_t type)
{
    if (NULL == p_value)
    {
        return RI_ERR_INVALID_ADDR;
    }

    service_db_t *psrv_db = find_srv_db(char_value_handle);
    if (NULL == psrv_db)
    {
        return RIBLE_ERR_ATT_INVALID_ATT_HANDLE;
    }

    uint8_t attr_idx = char_value_handle - psrv_db->start_handle;

    /* get uuid */
    uint16_t uuid = psrv_db->pattrs[attr_idx].type_value[1];
    uuid <<= 8;
    uuid |= psrv_db->pattrs[attr_idx].type_value[0];
    if (is_fixed_uuid(uuid))
    {
        return RIBLE_ERR_GATT_INVALID_ATT_TYPE;
    }

    /* TODO: use offset */
    if (!server_send_data(conn_handle, psrv_db->service_id, attr_idx,
                          p_value, len, (T_GATT_PDU_TYPE)type))
    {
        return RIBLE_ERR_UNKNOWN;
    }

    return RI_SUCCESS;
}

/**
 * @brief   Respond to a Read/Write user authorization request.
 * @param   [in] conn_handle: conn handle
 *          [in] status:  1: permit to change value ; 0: reject to change value
 *          [in] char_value_handle: characteristic handle
 *          [in] offset: the offset from which the attribute value has to
 * be updated
 *          [in] p_value: Pointer to new value used to update the attribute value.
 *          [in] len: data length
 *          [in] type : read response = 1; write response = 2;
 *
 * @return  RI_SUCCESS             Successfully queued a response to the peer, and in the case of a write operation, GATT updated.
 *          RI_ERR_INVALID_ADDR    Invalid pointer supplied.
 *          RI_ERR_INVALID_PARAM   Invalid parameter (offset) supplied.
 *          RI_ERR_INVALID_STATE   Invalid Connection State or no authorization request pending.
 *          RI_ERR_INVALID_LENGTH  Invalid length supplied.
 *          RI_ERR_BUSY            Procedure already in progress.
 *          RIBLE_ERR_ATT_INVALID_HANDLE     Attribute not found.
 * @note    This call should only be used as a response to a RIBLE_GATTS_EVT_READ/WRITE_PERMIT_REQ
 * event issued to the application.
 * */
rible_status_t rible_gatts_rw_auth_reply(uint16_t conn_handle,
                                         uint8_t status, uint16_t char_value_handle, uint8_t offset,
                                         uint8_t *p_value, uint8_t len, uint8_t type)
{
    if (0 == status)
    {
        result_g = (T_APP_RESULT)(ATT_ERR | ATT_ERR_INSUFFICIENT_AUTHOR);
    }

    return RI_SUCCESS;
}
