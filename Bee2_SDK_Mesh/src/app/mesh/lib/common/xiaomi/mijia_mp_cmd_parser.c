/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file     mijia_mp_cmd_parser.c
* @brief    Source file for command parse.
* @details  Parse mp commands and execute right commands.
* @author   hector_huang
* @date     2019-04-02
* @version  v1.0
* *************************************************************************************
*/
#include "mijia_mp_cmd_parser.h"
#include "platform_diagnose.h"
#include "platform_os.h"
#include "crc16btx.h"
#include "data_uart.h"


#define MIJIA_MP_HEAD               0x87
#define MIJIA_MP_HEAD_LEN           1
#define MIJIA_MP_OPCODE_LEN         2
#define MIJIA_MP_LENGTH_LEN         4
#define MIJIA_MP_CRC_LEN            2

typedef enum
{
    PARSE_HEAD,
    PARSE_OPCODE,
    PARSE_LENGTH,
    PARSE_PAYLOAD,
    PARSE_CRC
} mijia_mp_parse_state_t;

typedef struct
{
    mijia_mp_parse_state_t state;
    uint8_t *pdata;
    const mijia_mp_cmd_table_t *pcmd_info;
    uint16_t opcode;
    uint32_t payload_len;
    uint32_t state_count;
    uint8_t data[MIJIA_MP_CMD_REQUEST_MAX_SIZE];
} mijia_mp_packet_t;

static mijia_mp_packet_t *mp_pkt = NULL;
static const mijia_mp_cmd_table_t *cmd_table = NULL;
static uint16_t cmd_table_len = 0;

static const mijia_mp_cmd_table_t *mijia_mp_get_cmd_info(uint16_t opcode)
{
    if (NULL == cmd_table)
    {
        return NULL;
    }

    for (uint16_t i = 0; i < cmd_table_len; ++i)
    {
        if (cmd_table[i].opcode == opcode)
        {
            return &cmd_table[i];
        }
    }

    return NULL;
}

void mijia_mp_cmd_response(uint16_t opcode, mijia_mp_cmd_status_t status, const uint8_t *payload,
                           uint32_t payload_len)
{
    diag_assert((payload_len > 0) ? (NULL != payload) : TRUE);
    uint8_t buf[MIJIA_MP_CMD_RESPONSE_MAX_SIZE];
    uint8_t *pbuf = buf;
    *pbuf ++ = MIJIA_MP_HEAD;
    *pbuf ++ = opcode;
    *pbuf ++ = opcode >> 8;
    *pbuf ++ = (uint8_t)status;
    *pbuf ++ = payload_len;
    *pbuf ++ = payload_len >> 8;
    *pbuf ++ = payload_len >> 16;
    *pbuf ++ = payload_len >> 24;
    while (payload_len --)
    {
        *pbuf ++ = *payload ++;
    }
    uint16_t calc_crc = btxfcs(0, buf, pbuf - buf);
    *pbuf ++ = calc_crc;
    *pbuf ++ = calc_crc >> 8;
    data_uart_send_string(buf, pbuf - buf);
}

void mijia_mp_cmd_parse(const uint8_t *pdata, uint8_t len)
{
    diag_assert(NULL != mp_pkt);
    while (len --)
    {
        switch (mp_pkt->state)
        {
        case PARSE_HEAD:
            if (MIJIA_MP_HEAD == *pdata)
            {
                mp_pkt->state = PARSE_OPCODE;
                mp_pkt->state_count = MIJIA_MP_OPCODE_LEN;
                mp_pkt->pdata = mp_pkt->data;
                *mp_pkt->pdata ++ = MIJIA_MP_HEAD;
            }
            break;
        case PARSE_OPCODE:
            *mp_pkt->pdata ++ = *pdata;
            mp_pkt->state_count --;
            if (0 == mp_pkt->state_count)
            {
                mp_pkt->opcode = (mp_pkt->pdata[-1] << 8) + mp_pkt->pdata[-2];
                mp_pkt->state = PARSE_LENGTH;
                mp_pkt->state_count = MIJIA_MP_LENGTH_LEN;
            }
            break;
        case PARSE_LENGTH:
            *mp_pkt->pdata ++ = *pdata;
            mp_pkt->state_count --;
            if (0 == mp_pkt->state_count)
            {
                mp_pkt->payload_len = ((uint32_t)mp_pkt->pdata[-1] << 24) + ((uint32_t)mp_pkt->pdata[-2] << 16) +
                                      ((uint32_t)mp_pkt->pdata[-3] << 8) + (uint32_t)mp_pkt->pdata[-4];
                if (mp_pkt->payload_len > 0)
                {
                    mp_pkt->state = PARSE_PAYLOAD;
                    mp_pkt->state_count = mp_pkt->payload_len;
                }
                else
                {
                    mp_pkt->state = PARSE_CRC;
                    mp_pkt->state_count = MIJIA_MP_CRC_LEN;
                }
            }
            break;
        case PARSE_PAYLOAD:
            *mp_pkt->pdata ++ = *pdata;
            mp_pkt->state_count --;
            if (0 == mp_pkt->state_count)
            {
                mp_pkt->state = PARSE_CRC;
                mp_pkt->state_count = MIJIA_MP_CRC_LEN;
            }
            break;
        case PARSE_CRC:
            *mp_pkt->pdata ++ = *pdata;
            mp_pkt->state_count --;
            if (0 == mp_pkt->state_count)
            {
                /* get recived crc */
                uint16_t recv_crc = (mp_pkt->pdata[-1] << 8) + mp_pkt->pdata[-2];

                /* validate crc */
                uint16_t calc_crc = btxfcs(0, mp_pkt->data, mp_pkt->pdata - mp_pkt->data - MIJIA_MP_CRC_LEN);
                if (calc_crc == recv_crc)
                {
                    /* validate opcode */
                    /* get command table */
                    mp_pkt->pcmd_info = mijia_mp_get_cmd_info(mp_pkt->opcode);
                    if (NULL == mp_pkt->pcmd_info)
                    {
                        /* invalid opcode */
                        printe("mijia_mp_cmd_parse: receive invalid opcode = 0x%x", mp_pkt->opcode);
                        mijia_mp_cmd_response(mp_pkt->opcode, MIJIA_MP_ERR_OPCODE, NULL, 0);
                    }
                    else
                    {
                        /* get valid packet */
                        if (NULL != mp_pkt->pcmd_info->cmd_process)
                        {
                            uint8_t *ppayload = NULL;
                            if (mp_pkt->payload_len > 0)
                            {
                                ppayload = mp_pkt->data + MIJIA_MP_HEAD_LEN + MIJIA_MP_OPCODE_LEN +
                                           MIJIA_MP_LENGTH_LEN;
                            }
                            mp_pkt->pcmd_info->cmd_process(mp_pkt->opcode, ppayload, mp_pkt->payload_len);
                        }
                    }
                }
                else
                {
                    printe("mijia_mp_cmd_parse: received crc does not match calculated crc!");
                    /* crc check failed */
                    mijia_mp_cmd_response(mp_pkt->opcode, MIJIA_MP_ERR_CRC, NULL, 0);
                }

                mp_pkt->state = PARSE_HEAD;
            }
            break;
        default:
            /* should never reach here */
            diag_assert(0);
            break;
        }

        pdata ++;
    }
}

bool mijia_mp_cmd_parser_init(const mijia_mp_cmd_table_t *pcmd_table, uint16_t table_len)
{
    cmd_table = pcmd_table;
    cmd_table_len = table_len;
    mp_pkt = plt_malloc(sizeof(mijia_mp_packet_t), RAM_TYPE_DATA_OFF);
    if (NULL == mp_pkt)
    {
        printe("mijia_mp_cmd_init: initialize mp parser failed!");
        return FALSE;
    }
    mp_pkt->state = PARSE_HEAD;
    printi("mijia_mp_cmd_init: mp parser ready");

    return TRUE;
}

