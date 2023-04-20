#include "rible_log.h"
#include "trace.h"
#include <stdio.h>
#include <stdarg.h>

rible_status_t rible_log_printf(const char * sFormat, ...)
{
    char buffer[256];
    va_list ParamList;
    
    va_start(ParamList, sFormat);
    vsprintf(buffer, sFormat, ParamList);
    log_direct(COMBINE_TRACE_INFO(TYPE_BEE2, SUBTYPE_DIRECT, MODULE_GAP, 0), (const char*)buffer);
    va_end(ParamList);

    return RI_SUCCESS;
}

rible_status_t rible_log_hexdump(void* array_base, uint16_t array_size)
{
    do {\
        static const char format[] TRACE_DATA = "[hex][%d] %b\n";\
        log_buffer(COMBINE_TRACE_INFO(TYPE_BEE2, SUBTYPE_FORMAT, MODULE_GAP, 0), (uint32_t)format, 2, array_size, TRACE_BINARY(array_size, array_base));\
    } while (0);

    return RI_SUCCESS;
}
