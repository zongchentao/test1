#ifndef RIVOTEK_BLE_LOG_H__
#define RIVOTEK_BLE_LOG_H__

#include "rible_api.h"

#define RI_LOG_ENABLED  TRUE

#ifndef RI_LOG_LEVEL
#define RI_LOG_LEVEL              4
#endif

#ifndef RI_LOG_MODULE_NAME
#define RI_LOG_MODULE_NAME        "RIBLE_SDK"
#endif

#define RI_LOG_LEVEL_ERROR        0x01
#define RI_LOG_LEVEL_WARNING      0x02
#define RI_LOG_LEVEL_INFO         0x03
#define RI_LOG_LEVEL_DEBUG        0x04

#if RI_LOG_ENABLED
#define RI_PRINTF(...)                      rible_log_printf(__VA_ARGS__)
#define RI_HEXDUMP(array_base, array_size)  rible_log_hexdump(array_base, array_size)
#else
#define RI_PRINTF(...)
#define RI_HEXDUMP(base_addr, bytes)
#endif

#define RI_LOG_ERROR(_fmt_, ...)                                       \
do {                                                                            \
    if (RI_LOG_LEVEL >= RI_LOG_LEVEL_ERROR)                                     \
    {                                                                           \
        RI_PRINTF("[E]%s::%d" _fmt_, __func__, __LINE__, ##__VA_ARGS__);   \
    }                                                                           \
} while(0)

#define RI_LOG_WARNING(_fmt_, ...)                                     \
do {                                                                            \
    if (RI_LOG_LEVEL >= RI_LOG_LEVEL_WARNING)                                   \
    {                                                                           \
    RI_PRINTF("[W]%s::%d" _fmt_, __func__, __LINE__, ##__VA_ARGS__);   \
    }                                                                           \
} while(0)

#define RI_LOG_INFO(_fmt_, ...)                                        \
do {                                                                            \
    if (RI_LOG_LEVEL >= RI_LOG_LEVEL_INFO)                                      \
    {                                                                           \
        RI_PRINTF("[I]%s::%d" _fmt_, __func__, __LINE__, ##__VA_ARGS__);   \
    }                                                                           \
} while(0)

#define RI_LOG_DEBUG(_fmt_, ...)                                       \
do {                                                                            \
    if (RI_LOG_LEVEL >= RI_LOG_LEVEL_DEBUG)                                     \
    {                                                                           \
        RI_PRINTF("[D]%s::%d" _fmt_, __func__, __LINE__, ##__VA_ARGS__);   \
    }                                                                           \
} while(0)

#define RI_LOG_HEXDUMP(p_data, len)                                    \
do {                                                                            \
    if (RI_LOG_LEVEL >= RI_LOG_LEVEL_DEBUG)                                     \
    {                                                                           \
        RI_HEXDUMP(p_data, len);                                                \
    }                                                                           \
} while(0)


#endif
