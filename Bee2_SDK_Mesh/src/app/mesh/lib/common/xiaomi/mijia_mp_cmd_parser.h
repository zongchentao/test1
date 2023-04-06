/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file     mijia_mp_cmd_parser.h
* @brief    Source file for command parse.
* @details  Data types and external functions declaration.
* @author   hector_huang
* @date     2019-04-02
* @version  v1.0
* *************************************************************************************
*/

#ifndef _MIJIA_MP_CMD_PARSER_H_
#define _MIJIA_MP_CMD_PARSER_H_

#include "platform_types.h"

BEGIN_DECLS

/* brief mijia mp command max length */
#define MIJIA_MP_CMD_REQUEST_MAX_SIZE      512
#define MIJIA_MP_CMD_RESPONSE_MAX_SIZE     80

/* mijia mp command response status */
typedef enum
{
    MIJIA_MP_SUCCESS,      /* operation success */
    MIJIA_MP_ERR_CRC,      /* crc error */
    MIJIA_MP_ERR_OPCODE,   /* invalid opcode */
    MIJIA_MP_ERR_PARAM,    /* invalid parameter */
} mijia_mp_cmd_status_t;

/* mijia mp command process callback */
typedef void (*mijia_mp_cmd_process_t)(uint16_t opcode, const uint8_t *pdata, uint16_t len);

/* command table structure */
typedef struct
{
    uint16_t opcode;
    mijia_mp_cmd_process_t cmd_process;
} mijia_mp_cmd_table_t;


/**
 * @brief initialize mijia mp command
 * @param[in] pcmd_table: user defined command table array
 * @param[in] table_len: command table length
 */
bool mijia_mp_cmd_parser_init(const mijia_mp_cmd_table_t *pcmd_table, uint16_t table_len);

/**
 * @brief parse mijia mp command
 * @param[in] pdata: data need to parse
 * @param[in] len: data length
 */
void mijia_mp_cmd_parse(const uint8_t *pdata, uint8_t len);

/**
 * @brief response to request message
 * @param[in] opcode: message opcode
 * @param[in] status: message process status
 * @param[in] payload: response message payload
 * @param[in] payload_len: response message payload length
 */
void mijia_mp_cmd_response(uint16_t opcode, mijia_mp_cmd_status_t status, const uint8_t *ppayload,
                           uint32_t payload_len);


END_DECLS

#endif /* _MIJIA_PT_CMD_PARSE_H_ */

