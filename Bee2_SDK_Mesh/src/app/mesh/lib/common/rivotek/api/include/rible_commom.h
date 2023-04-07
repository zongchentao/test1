#ifndef RIVOTEK_BLE_COMMOM_H__
#define RIVOTEK_BLE_COMMOM_H__

#include <stdbool.h>
#include <stdint.h>

#ifndef NULL
#define NULL 0
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/* Compiler Related Definitions */
#ifdef __CC_ARM                         /* ARM Compiler */
    #define __WEAK                     __weak
#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
    #define __WEAK                     __weak
#elif defined (__GNUC__)                /* GNU GCC Compiler */
    #define __WEAK                     __attribute__((weak))
#elif defined (__ADSPBLACKFIN__)        /* for VisualDSP++ Compiler */
    #define __WEAK                     __attribute__((weak))
#elif defined (_MSC_VER)
    #define __WEAK
#elif defined (__TI_COMPILER_VERSION__)
    #define __WEAK
#else
    #error not supported tool chain
#endif

#endif