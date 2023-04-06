/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file     mijia_mp_cmd.c
* @brief    Source file for mijia command.
* @details  Data types and external functions declaration.
* @author   hector_huang
* @date     2019-04-03
* @version  v1.0
* *************************************************************************************
*/
#include <string.h>
#include <gap.h>
#include <test_mode.h>
#include "data_uart.h"
#include "mijia_mp_cmd.h"
#include "mijia_mp_cmd_parser.h"
#include "rtl876x_lib_platform.h"
#include "rtl876x_wdg.h"
#include "cryptography/mi_crypto.h"
#include "cryptography/mi_crypto_backend_uECC.h"
#include "cryptography/mi_mesh_otp.h"
#include "cryptography/mi_mesh_otp_config.h"
#include "mible_api.h"
#include "mi_config.h"
#include "rtl876x_rcc.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_gpio.h"

/** flash sector size */
#define FLASH_SECTOR_SIZE    0x1000

/* opcode */
#define MIJIA_MP_CMD_READ_MAC                         0x1202
#define MIJIA_MP_CMD_IO_TEST                          0x1205
#define MIJIA_MP_CMD_ENTER_HCI_MODE                   0x120C
#define MIJIA_MP_CMD_RESTART_DUT                      0x1217
#define MIJIA_MP_CMD_PROGRAM_MAC                      0x1218
#define MIJIA_MP_CMD_CREATE_PUB_KEY                   0x1420
#define MIJIA_MP_CMD_READ_PUB_KEY                     0x1421
#define MIJIA_MP_CMD_PROGRAM_ROOT_CERT                0x1422
#define MIJIA_MP_CMD_PROGRAM_MANU_CERT                0x1423
#define MIJIA_MP_CMD_PROGRAM_DEV_CERT                 0x1424
#define MIJIA_MP_CMD_VERIFY_CERT_CHAIN                0x1425
#define MIJIA_MP_CMD_CHECK_DEV_INFO                   0x1426


/* device secret and public key */
static uint8_t dev_sk[32];
static uint8_t dev_pk[64];

/* device information */
typedef struct
{
    uint16_t product_id;
    uint8_t version[5];
    uint8_t mac[6];
    uint8_t did[8];
} _PACKED_ mijia_dev_info_t;

/* gpio pin number */
uint8_t GPIO_PIN_NUM[12][2] =
{
    {0, 3}, //P0_0    0, P0_3    3
    {1, 2}, //P0_1    1, P0_2    2
    {4, 36}, //P0_4    4, H_0     36
    {5, 6}, //P0_5    5, P0_6    6
    {8, 9}, //P1_0    8, P1_1    9
    {18, 19}, //P2_2    18,P2_3    19
    {20, 21}, //P2_4    20,P2_5    21
    {22, 23}, //P2_6    22,P2_7    23
    {26, 27}, //P3_2    26,P3_3    27
    {32, 33}, //P4_0    32,P4_1    33
    {34, 35}, //P4_2    34,P4_3    35
    {37, 38}, //H_1     37,H_2     38
};

/* gpio test result */
static uint8_t GPIO_Test_Result[12] = {0};

/**
 * @brief io delay function
 */
void io_delay(void)
{
    for (volatile uint32_t i = 0; i < 5000; i++);
}

/**
 * @brief test gpio forward
 */
static void mp_test_gpio_forward(void)
{
    uint32_t pin_input = 0;
    uint32_t pin_output = 0;
    uint8_t gpio_input_data[12];

    GPIO_DeInit();
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);
    for (uint8_t i = 1; i < 9; i++)
    {
        /* GPIO pin input */
        Pad_Config(GPIO_PIN_NUM[i][0], PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
                   PAD_OUT_HIGH);
        Pinmux_Config(GPIO_PIN_NUM[i][0], DWGPIO);
        pin_input |= GPIO_GetPin(GPIO_PIN_NUM[i][0]);

        /* GPIO pin output */
        Pad_Config(GPIO_PIN_NUM[i][1], PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
                   PAD_OUT_HIGH);
        Pinmux_Config(GPIO_PIN_NUM[i][1], DWGPIO);
        pin_output |= GPIO_GetPin(GPIO_PIN_NUM[i][1]);
    }
    GPIO_InitStruct.GPIO_Pin    = pin_input;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_ITCmd  = DISABLE;
    GPIO_Init(&GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin    = pin_output;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_ITCmd  = DISABLE;
    GPIO_Init(&GPIO_InitStruct);

    /* output low */
    GPIO_Write(~pin_output);
    io_delay();
    for (uint8_t i = 1; i < 9; i++)
    {
        gpio_input_data[i] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[i][0]));
        if (0 != gpio_input_data[i]) { GPIO_Test_Result[i]++; }
    }
    memset(gpio_input_data, 0, sizeof(gpio_input_data));

    /* output high */
    GPIO_Write(pin_output);
    io_delay();
    for (uint8_t i = 1; i < 9; i++)
    {
        gpio_input_data[i] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[i][0]));
        if (1 != gpio_input_data[i]) { GPIO_Test_Result[i]++; }
    }
    memset(gpio_input_data, 0, sizeof(gpio_input_data));

    GPIO_Write(~pin_output);
    io_delay();
    for (uint8_t i = 1; i < 9; i++)
    {
        gpio_input_data[i] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[i][0]));
        if (0 != gpio_input_data[i]) { GPIO_Test_Result[i]++; }
    }

    /* P0_0 input always */
    Pad_Config(GPIO_PIN_NUM[0][0], PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);
    Pinmux_Config(GPIO_PIN_NUM[0][0], DWGPIO);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin    = GPIO_GetPin(GPIO_PIN_NUM[0][0]);
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_ITCmd  = DISABLE;
    GPIO_Init(&GPIO_InitStruct);
    gpio_input_data[0] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[0][0]));
    if (0 != gpio_input_data[0]) { GPIO_Test_Result[0]++; }

    memset(gpio_input_data, 0, sizeof(gpio_input_data));

    GPIO_DeInit();
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    GPIO_StructInit(&GPIO_InitStruct);
    pin_input = 0;
    pin_output = 0;
    for (uint8_t i = 1; i < 9; i++)
    {
        Pinmux_Config(GPIO_PIN_NUM[i][0], IDLE_MODE);
        Pinmux_Config(GPIO_PIN_NUM[i][1], IDLE_MODE);
    }

    for (uint8_t i = 9; i < 12; i++)
    {
        /* GPIO pin input */
        Pad_Config(GPIO_PIN_NUM[i][0], PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
                   PAD_OUT_HIGH);
        Pinmux_Config(GPIO_PIN_NUM[i][0], DWGPIO);
        pin_input |= GPIO_GetPin(GPIO_PIN_NUM[i][0]);

        /* GPIO pin output */
        Pad_Config(GPIO_PIN_NUM[i][1], PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
                   PAD_OUT_HIGH);
        Pinmux_Config(GPIO_PIN_NUM[i][1], DWGPIO);
        pin_output |= GPIO_GetPin(GPIO_PIN_NUM[i][1]);
    }
    GPIO_InitStruct.GPIO_Pin    = pin_input;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_ITCmd  = DISABLE;
    GPIO_Init(&GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin    = pin_output;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_ITCmd  = DISABLE;
    GPIO_Init(&GPIO_InitStruct);

    /* output low */
    GPIO_Write(~pin_output);
    io_delay();
    for (uint8_t i = 9; i < 12; i++)
    {
        gpio_input_data[i] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[i][0]));
        if (0 != gpio_input_data[i]) { GPIO_Test_Result[i]++; }
    }
    memset(gpio_input_data, 0, sizeof(gpio_input_data));

    /* output high */
    GPIO_Write(pin_output);
    io_delay();
    for (uint8_t i = 9; i < 12; i++)
    {
        gpio_input_data[i] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[i][0]));
        if (1 != gpio_input_data[i]) { GPIO_Test_Result[i]++; }
    }
    memset(gpio_input_data, 0, sizeof(gpio_input_data));

    /* output low */
    GPIO_Write(~pin_output);
    io_delay();
    for (uint8_t i = 9; i < 12; i++)
    {
        gpio_input_data[i] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[i][0]));
        if (0 != gpio_input_data[i]) { GPIO_Test_Result[i]++; }
    }
    memset(gpio_input_data, 0, sizeof(gpio_input_data));
}

/**
 * @brief test gpio backward
 */
static void mp_test_gpio_backward(void)
{
    uint32_t pin_input = 0;
    uint32_t pin_output = 0;
    uint8_t gpio_input_data[12];

    GPIO_DeInit();
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);
    for (uint8_t i = 1; i < 9; i++)
    {
        /* GPIO pin input */
        Pad_Config(GPIO_PIN_NUM[i][1], PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
                   PAD_OUT_HIGH);
        Pinmux_Config(GPIO_PIN_NUM[i][1], DWGPIO);
        pin_input |= GPIO_GetPin(GPIO_PIN_NUM[i][1]);

        /* GPIO pin output */
        Pad_Config(GPIO_PIN_NUM[i][0], PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
                   PAD_OUT_HIGH);
        Pinmux_Config(GPIO_PIN_NUM[i][0], DWGPIO);
        pin_output |= GPIO_GetPin(GPIO_PIN_NUM[i][0]);
    }
    GPIO_InitStruct.GPIO_Pin    = pin_input;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_ITCmd  = DISABLE;
    GPIO_Init(&GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin    = pin_output;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_ITCmd  = DISABLE;
    GPIO_Init(&GPIO_InitStruct);

    /* output low */
    GPIO_Write(~pin_output);
    io_delay();
    for (uint8_t i = 1; i < 9; i++)
    {
        gpio_input_data[i] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[i][1]));
        if (0 != gpio_input_data[i]) { GPIO_Test_Result[i]++; }
    }
    memset(gpio_input_data, 0, sizeof(gpio_input_data));

    /* output high */
    GPIO_Write(pin_output);
    io_delay();
    for (uint8_t i = 1; i < 9; i++)
    {
        gpio_input_data[i] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[i][1]));
        if (1 != gpio_input_data[i]) { GPIO_Test_Result[i]++; }
    }
    memset(gpio_input_data, 0, sizeof(gpio_input_data));

    /* output low */
    GPIO_Write(~pin_output);
    io_delay();
    for (uint8_t i = 1; i < 9; i++)
    {
        gpio_input_data[i] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[i][1]));
        if (0 != gpio_input_data[i]) { GPIO_Test_Result[i]++; }
    }

    /* P0_0 input always */
    Pad_Config(GPIO_PIN_NUM[0][0], PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);
    Pinmux_Config(GPIO_PIN_NUM[0][0], DWGPIO);
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin    = GPIO_GetPin(GPIO_PIN_NUM[0][0]);
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_ITCmd  = DISABLE;
    GPIO_Init(&GPIO_InitStruct);
    gpio_input_data[0] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[0][0]));
    if (0 != gpio_input_data[0]) { GPIO_Test_Result[0]++; }

    memset(gpio_input_data, 0, sizeof(gpio_input_data));

    GPIO_DeInit();
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    GPIO_StructInit(&GPIO_InitStruct);
    pin_input = 0;
    pin_output = 0;
    for (uint8_t i = 1; i < 9; i++)
    {
        Pinmux_Config(GPIO_PIN_NUM[i][0], IDLE_MODE);
        Pinmux_Config(GPIO_PIN_NUM[i][1], IDLE_MODE);
    }

    for (uint8_t i = 9; i < 12; i++)
    {
        /* GPIO pin input */
        Pad_Config(GPIO_PIN_NUM[i][1], PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
                   PAD_OUT_HIGH);
        Pinmux_Config(GPIO_PIN_NUM[i][1], DWGPIO);
        pin_input |= GPIO_GetPin(GPIO_PIN_NUM[i][1]);

        /* GPIO pin output */
        Pad_Config(GPIO_PIN_NUM[i][0], PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
                   PAD_OUT_HIGH);
        Pinmux_Config(GPIO_PIN_NUM[i][0], DWGPIO);
        pin_output |= GPIO_GetPin(GPIO_PIN_NUM[i][0]);
    }
    GPIO_InitStruct.GPIO_Pin    = pin_input;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_ITCmd  = DISABLE;
    GPIO_Init(&GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin    = pin_output;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_ITCmd  = DISABLE;
    GPIO_Init(&GPIO_InitStruct);

    /* output low */
    GPIO_Write(~pin_output);
    io_delay();
    for (uint8_t i = 9; i < 12; i++)
    {
        gpio_input_data[i] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[i][1]));
        if (0 != gpio_input_data[i]) { GPIO_Test_Result[i]++; }
    }
    memset(gpio_input_data, 0, sizeof(gpio_input_data));

    /* output high */
    GPIO_Write(pin_output);
    io_delay();
    for (uint8_t i = 9; i < 12; i++)
    {
        gpio_input_data[i] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[i][1]));
        if (1 != gpio_input_data[i]) { GPIO_Test_Result[i]++; }
    }
    memset(gpio_input_data, 0, sizeof(gpio_input_data));

    /* output low */
    GPIO_Write(~pin_output);
    io_delay();
    for (uint8_t i = 9; i < 12; i++)
    {
        gpio_input_data[i] = GPIO_ReadInputDataBit(GPIO_GetPin(GPIO_PIN_NUM[i][1]));
        if (0 != gpio_input_data[i]) { GPIO_Test_Result[i]++; }
    }
}

/**
 * @brief read mac address
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
void mijia_mp_cmd_read_mac(uint16_t opcode, const uint8_t *pdata, uint16_t len)
{
    uint8_t mac_addr[6];
    gap_get_param(GAP_PARAM_BD_ADDR, mac_addr);
    mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, mac_addr, 6);
}

/**
 * @brief enter hci mode
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
static void mijia_mp_cmd_enter_hci_mode(uint16_t opcode, const uint8_t *pdata, uint16_t len)
{
    mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, NULL, 0);

    set_hci_mode_flag(true);
    WDG_SystemReset(RESET_ALL_EXCEPT_AON, SWITCH_HCI_MODE);
}

/**
 * @brief restart dut
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
static void mijia_mp_cmd_restart_dut(uint16_t opcode, const uint8_t *pdata, uint16_t len)
{
    mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, NULL, 0);
    WDG_SystemReset(RESET_ALL, UART_CMD_RESET);
}

/**
 * @brief program mac
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
static void mijia_mp_cmd_program_mac(uint16_t opcode, const uint8_t *pdata, uint16_t len)
{
    uint8_t state = 0;
    if (!UpdateMAC((uint8_t *)pdata))
    {
        state = 1;
    }

    mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, &state, 1);
}

/**
 * @brief io test
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
void mijia_mp_cmd_io_test(uint16_t opcode, const uint8_t *pdata, uint16_t len)
{
    memset(GPIO_Test_Result, 0, sizeof(GPIO_Test_Result));
    mp_test_gpio_forward();
    mp_test_gpio_backward();

    mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, GPIO_Test_Result, 12);
    return;
}

/**
 * @brief erase otp region
 */
static bool mijia_otp_erase(void)
{
    uint32_t sector_addr = (uint32_t)POTP_BASE;
    sector_addr &= ~(FLASH_SECTOR_SIZE - 1);
    /* erase otp */
    return flash_erase_locked(FLASH_ERASE_SECTOR, sector_addr);
}

/**
 * @brief generite key
 * @param key: key need to generate
 */
static void mijia_keygen(uint8_t k[16])
{
    memset(k, 0, 16);
    mible_gap_address_get(k);
    mible_aes128_encrypt(k, k, 16, k);
}

/**
 * @brief create public key
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
static void mijia_mp_cmd_create_pub_key(uint16_t opcode, const uint8_t *pdata, uint16_t len)
{
    uint8_t state = 0;
    /* erase key area */
    if (!mijia_otp_erase())
    {
        state = 3;
        mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, &state, 1);
        return;
    }

    /* write header */
    mi_mesh_otp_seal_tag(POTP_BASE);

    /* check header */
    if (!mi_mesh_otp_is_existed())
    {
        state = 4;
        mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, &state, 1);
        return ;
    }

    /* generate dev cert secrect and public key */
    micro_ecc_init(NULL);
    if (0 != micro_ecc_keypair_gen(NULL, dev_sk, dev_pk))
    {
        state = 1;
        mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, &state, 1);
        return ;
    }

    /* storage private key */
    uint8_t buffer[40];
    uint8_t key[16];
    mijia_keygen(key);
    mi_crypto_ccm_encrypt(key, key + 8, 8, NULL, 0, dev_sk, 32, buffer,
                          buffer + 32, 4);
    if (0 != mi_mesh_otp_write(OTP_DEV_CERT_PRI, buffer, 36))
    {
        state = 2;
        mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, &state, 1);
        return ;
    }

    mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, &state, 1);
}

/**
 * @brief read public key
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
static void mijia_mp_cmd_read_pub_key(uint16_t opcode, const uint8_t *data, uint16_t len)
{
    mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, dev_pk, sizeof(dev_pk));
}

/**
 * @brief program root certificate
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
static void mijia_mp_cmd_program_root_cert(uint16_t opcode, const uint8_t *data, uint16_t len)
{
    uint8_t state = 0;
    if (0 != mi_mesh_otp_write(OTP_ROOT_CERT, data, len))
    {
        state = 1;
    }
    mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, &state, 1);
}

/**
 * @brief program manufacture certificate
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
static void mijia_mp_cmd_program_manu_cert(uint16_t opcode, const uint8_t *data, uint16_t len)
{
    uint8_t state = 0;
    if (0 != mi_mesh_otp_write(OTP_MANU_CERT, data, len))
    {
        state = 1;
    }
    mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, &state, 1);
}

/**
 * @brief program device certificate
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
static void mijia_mp_cmd_program_dev_cert(uint16_t opcode, const uint8_t *data, uint16_t len)
{
    uint8_t state = 0;
    if (0 != mi_mesh_otp_write(OTP_DEV_CERT, data, len))
    {
        state = 1;
    }
    mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, &state, 1);
}

/**
 * @brief verify certificate chain
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
static void mijia_mp_cmd_verify_cert_chain(uint16_t opcode, const uint8_t *data, uint16_t len)
{
    uint8_t state = 0;
    if (0 != mi_mesh_otp_verify())
    {
        state = 1;
    }
    mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, &state, 1);
}

/**
 * @brief get device id
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
void get_device_id(uint8_t *pdid)
{
    uint8_t buf[512];
    int buf_len;
    msc_crt_t crt;

    buf_len = mi_mesh_otp_read(OTP_DEV_CERT, buf, sizeof(buf));
    if (buf_len > 0)
    {
        mi_crypto_crt_parse_der(buf, buf_len, NULL, &crt);
        if (crt.sn.len <= 8)
        {
            for (int i = 0; i < crt.sn.len; i++)
            {
                pdid[crt.sn.len - 1 - i] = crt.sn.p[i];
            }
        }
    }
}

/**
 * @brief get device information
 * @param[in] opcode: command opcode
 * @param[in] pdata: command data
 * @param[in] len: command length
 */
static void mijia_mp_cmd_check_dev_info(uint16_t opcode, const uint8_t *data, uint16_t len)
{
    mijia_dev_info_t dev_info;
    dev_info.product_id = PRODUCT_ID;
    dev_info.version[0] = MI_MESH_MAJOR;
    dev_info.version[1] = MI_MESH_MINOR;
    dev_info.version[2] = MI_MESH_REVISION;
    dev_info.version[3] = DEVELOPER_VERSION;
    dev_info.version[4] = DEVELOPER_VERSION >> 8;
    gap_get_param(GAP_PARAM_BD_ADDR, dev_info.mac);
    memset(dev_info.did, 0, 8);
    get_device_id(dev_info.did);
    mijia_mp_cmd_response(opcode, MIJIA_MP_SUCCESS, (const uint8_t *)&dev_info,
                          sizeof(mijia_dev_info_t));
}

/**
 * @brief command table
 */
static const mijia_mp_cmd_table_t mijia_mp_cmd_table[] =
{
    {MIJIA_MP_CMD_READ_MAC, mijia_mp_cmd_read_mac},
    {MIJIA_MP_CMD_ENTER_HCI_MODE, mijia_mp_cmd_enter_hci_mode},
    {MIJIA_MP_CMD_RESTART_DUT, mijia_mp_cmd_restart_dut},
    {MIJIA_MP_CMD_PROGRAM_MAC, mijia_mp_cmd_program_mac},
    {MIJIA_MP_CMD_IO_TEST, mijia_mp_cmd_io_test},
    {MIJIA_MP_CMD_CREATE_PUB_KEY, mijia_mp_cmd_create_pub_key},
    {MIJIA_MP_CMD_READ_PUB_KEY, mijia_mp_cmd_read_pub_key},
    {MIJIA_MP_CMD_PROGRAM_ROOT_CERT, mijia_mp_cmd_program_root_cert},
    {MIJIA_MP_CMD_PROGRAM_MANU_CERT, mijia_mp_cmd_program_manu_cert},
    {MIJIA_MP_CMD_PROGRAM_DEV_CERT, mijia_mp_cmd_program_dev_cert},
    {MIJIA_MP_CMD_VERIFY_CERT_CHAIN, mijia_mp_cmd_verify_cert_chain},
    {MIJIA_MP_CMD_CHECK_DEV_INFO, mijia_mp_cmd_check_dev_info},
};

static __INLINE uint16_t mijia_mp_cmd_table_length(void)
{
    return sizeof(mijia_mp_cmd_table) / sizeof(mijia_mp_cmd_table[0]);
}

void mijia_mp_cmd_init(void)
{
    mijia_mp_cmd_parser_init(mijia_mp_cmd_table, mijia_mp_cmd_table_length());
}
