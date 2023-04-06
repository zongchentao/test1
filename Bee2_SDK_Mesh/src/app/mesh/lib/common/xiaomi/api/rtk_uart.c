/*
 * mible_mcu.c
 *
 *  Created on: 2021/3/15
 *      Author: mi
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <os_msg.h>
#include <os_task.h>
#include "rtl876x_nvic.h"
#include "rtl876x_rcc.h"
#include "rtl876x_rtc.h"
#include "rtl876x_uart.h"
#include "rtl876x_gpio.h"
#include "rtl876x_pinmux.h"
#include "mible_api.h"
#include "mible_log.h"
#include "mible_mcu.h"
#include "gatt_dfu/mible_dfu_main.h"
#include "xmodem.h"
#include "rtk_uart.h"

#if defined(CUSTOMIZED_MI_CONFIG_FILE)
#include CUSTOMIZED_MI_CONFIG_FILE
#endif

volatile static enum {
    MCU_TRANS_IDLE = 0,
    MCU_TRANS_MASTER = 1,
    MCU_TRANS_SLAVE = 2,
} rtk_mcu_trans_role = MCU_TRANS_IDLE;

static struct
{
    uint8_t buf[256];
    volatile uint16_t rd_pos;
    volatile uint16_t wt_pos;
} mible_mcu_fifo_rx;

static pt_t pt_recv;

static void rtk_uart_recv_byte(uint8_t byte)
{
    uint16_t pos = mible_mcu_fifo_rx.wt_pos;

    pos = (pos + 1) % sizeof(mible_mcu_fifo_rx.buf);
    if (pos == mible_mcu_fifo_rx.rd_pos)
    {
        MI_LOG_ERROR("OTA RX Overflow!!!");
        return;
    }

    mible_mcu_fifo_rx.buf[mible_mcu_fifo_rx.wt_pos] = byte;
    mible_mcu_fifo_rx.wt_pos = pos;
}

static void MI_UART1(void)
{
    uint8_t rx_data;
    uint32_t int_status;
    uint32_t line_error;
    uint8_t fifo_len;

    /* read interrupt id */
    int_status = UART_GetIID(UART1);
    /* disable interrupt */
    UART_INTConfig(UART1, UART_INT_RD_AVA | UART_INT_LINE_STS, DISABLE);

    if (UART_GetFlagState(UART1, UART_FLAG_RX_IDLE) == SET)
    {
		UART_INTConfig(UART1, UART_INT_IDLE, DISABLE);
		/* TODO: Wait MCU event */
		UART_INTConfig(UART1, UART_INT_IDLE, ENABLE);
    }

    switch (int_status)
    {
    /* tx fifo empty */
    case UART_INT_ID_TX_EMPTY:
        /* do nothing */
        break;
    /* rx data valiable */
    case UART_INT_ID_RX_LEVEL_REACH:
    case UART_INT_ID_RX_TMEOUT:
        fifo_len = UART_GetRxFIFOLen(UART1);
        for (uint8_t i = 0; i < fifo_len; ++i)
        {
            UART_ReceiveData(UART1, &rx_data, 1);
            rtk_uart_recv_byte(rx_data);
        }
        break;
    /* receive line status interrupt */
    case UART_INT_ID_LINE_STATUS:
        line_error = UART1->LSR;
        printe("data_uart_isr Line status error, fail = %d!", line_error);
        break;
    default:
        break;
    }

    /* enable interrupt again */
    UART_INTConfig(UART1, UART_INT_RD_AVA | UART_INT_LINE_STS, ENABLE);
    return;
}

static void rtk_wakeup_pin_config(GPIOIT_PolarityType type)
{
	/* config wakeup pin input pull-up mode */
	Pad_Config(WAKE_PIN_NUM, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);
	Pinmux_Config(WAKE_PIN_NUM, DWGPIO);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = WAKE_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_ITCmd = ENABLE;
	GPIO_InitStruct.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
	GPIO_InitStruct.GPIO_DebounceTime = 5;
	GPIO_InitStruct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
	GPIO_InitStruct.GPIO_ITPolarity = type;
	GPIO_Init(&GPIO_InitStruct);
}

/* TODO */
void WAKE_PIN_Handler(void)
{
	/* wakeup by slave */
	GPIO_MaskINTConfig(WAKE_PIN, ENABLE);

	if (!GPIO_ReadInputDataBit(WAKE_PIN))
	{
		MI_LOG_INFO("wakeup by slave\n");
		System_WakeUpPinDisable(WAKE_PIN_NUM);

		rtk_mcu_trans_role = MCU_TRANS_SLAVE;
		rtk_wakeup_pin_config(GPIO_INT_POLARITY_ACTIVE_HIGH);

		goto EXIT;
	}

	MI_LOG_INFO("slave trans complete\n");
	rtk_mcu_trans_role = MCU_TRANS_IDLE;
	rtk_wakeup_pin_config(GPIO_INT_POLARITY_ACTIVE_LOW);

EXIT:
	GPIO_ClearINTPendingBit(WAKE_PIN);
	GPIO_MaskINTConfig(WAKE_PIN, DISABLE);
}

void rtk_mcu_enter_master(void)
{
	MI_LOG_INFO("mible_mcu_enter_master\n");

	while (rtk_mcu_trans_role == MCU_TRANS_SLAVE)
		;

	MI_LOG_INFO("start trans\n");

	GPIO_MaskINTConfig(WAKE_PIN, ENABLE);

	System_WakeUpPinDisable(WAKE_PIN_NUM);

	rtk_mcu_trans_role = MCU_TRANS_MASTER;

	/* config wakeup pin output down mode */
	Pad_Config(WAKE_PIN_NUM, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = WAKE_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_ITCmd = DISABLE;
	GPIO_Init(&GPIO_InitStruct);

	GPIO_WriteBit(WAKE_PIN, Bit_RESET);

    /* delay 10ms for slave wakeup */
	uint32_t exp = RTC_GetCounter() + 10;
	while (RTC_GetCounter() <= exp)
		;
}

void rtk_mcu_exit_master(void)
{
	MI_LOG_INFO("mible_mcu_exit_master\n");

	if (rtk_mcu_trans_role != MCU_TRANS_MASTER)
	{
		MI_LOG_ERROR("Invalid role %d\n", rtk_mcu_trans_role);
		return;
	}

	rtk_wakeup_pin_config(GPIO_INT_POLARITY_ACTIVE_LOW);

	GPIO_INTConfig(WAKE_PIN, ENABLE);
	GPIO_ClearINTPendingBit(WAKE_PIN);
	GPIO_MaskINTConfig(WAKE_PIN, DISABLE);

	rtk_mcu_trans_role = MCU_TRANS_IDLE;
}

void rtk_mcu_wakeup(void)
{
	if (!System_WakeUpInterruptValue(WAKE_PIN_NUM))
	{
		MI_LOG_WARNING("not for us\n");
		return;
	}

	if (rtk_mcu_trans_role != MCU_TRANS_IDLE)
	{
		MI_LOG_ERROR("invalid role %d\n", rtk_mcu_trans_role);
		return;
	}

	WAKE_PIN_Handler();
}

bool rtk_mcu_is_idle(void)
{
	return (rtk_mcu_trans_role == MCU_TRANS_IDLE);
}

void rtk_mcu_status(mible_dfu_state_t state, mible_dfu_param_t *param)
{
	MI_LOG_INFO("mcu update state %d", state);

	if (state != MIBLE_DFU_STATE_CANCEL)
	{
		return;
	}

	if (rtk_mcu_trans_role != MCU_TRANS_MASTER)
	{
		return;
	}

	MI_LOG_WARNING("Unexpected cancel, exit master");

	PT_INIT(&pt_recv);

	rtk_mcu_exit_master();
}

void rtk_mcu_perip_power_off(void)
{
	UART_INTConfig(UART1, UART_INT_RD_AVA | UART_INT_LINE_STS | UART_INT_IDLE, DISABLE);
	UART_DeInit(UART1);

	/* Turn off UART clock */
	RCC_PeriphClockCmd(APBPeriph_UART1, APBPeriph_UART1_CLOCK, DISABLE);

	/* pad config */
	Pad_Config(TX_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);
	Pad_Config(RX_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);

	Pad_Config(WAKE_PIN_NUM, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);

	// use this pin to wakeup the chip from Deep Low Power Mode.
	System_WakeUpPinEnable(WAKE_PIN_NUM, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_DISABLE);
}

void rtk_mcu_perip_power_on(void)
{
	/* Turn on UART clock */
	RCC_PeriphClockCmd(APBPeriph_UART1, APBPeriph_UART1_CLOCK, ENABLE);

	/* pinmux config */
	Pinmux_Config(TX_PIN, UART1_TX);
	Pinmux_Config(RX_PIN, UART1_RX);

	/* pad config */
	Pad_Config(TX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);
	Pad_Config(RX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);

	/* uart init */
	UART_InitTypeDef uartInitStruct;
	UART_StructInit(&uartInitStruct);
	/* change default rx trigger level */
	uartInitStruct.rxTriggerLevel = UART_RX_FIFO_TRIGGER_LEVEL_8BYTE;
	uartInitStruct.idle_time = UART_RX_IDLE_8BYTE;

	UART_Init(UART1, &uartInitStruct);
	/* enable rx interrupt and line status interrupt */
	UART_INTConfig(UART1, UART_INT_RD_AVA | UART_INT_LINE_STS | UART_INT_IDLE, ENABLE);

	Pad_Config(WAKE_PIN_NUM, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
}

void mible_mcu_init(void)
{
    PT_INIT(&pt_recv);
    
    UART_DeInit(UART1);
    rtk_mcu_perip_power_on();
    
    /*  Enable UART IRQ  */
	NVIC_InitTypeDef nvic_init_struct;
	nvic_init_struct.NVIC_IRQChannel = UART1_IRQn;
	nvic_init_struct.NVIC_IRQChannelCmd = ENABLE;
	nvic_init_struct.NVIC_IRQChannelPriority = 5;
	NVIC_Init(&nvic_init_struct);

	RamVectorTableUpdate(Uart1_VECTORn, MI_UART1);

	/* Initialize GPIO peripheral */
	RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    rtk_wakeup_pin_config(GPIO_INT_POLARITY_ACTIVE_LOW);
    
    GPIO_INTConfig(WAKE_PIN, ENABLE);
	GPIO_ClearINTPendingBit(WAKE_PIN);
	GPIO_MaskINTConfig(WAKE_PIN, DISABLE);
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannel = WAKE_PIN_IRQn;
	NVIC_Init(&NVIC_InitStruct);
}

void mible_mcu_deinit(void)
{
	rtk_mcu_perip_power_off();

	GPIO_INTConfig(WAKE_PIN, DISABLE);

	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelCmd = DISABLE;
	NVIC_InitStruct.NVIC_IRQChannel = WAKE_PIN_IRQn;
	NVIC_Init(&NVIC_InitStruct);

	System_WakeUpPinDisable(WAKE_PIN_NUM);
}

void mible_mcu_send_buffer(const uint8_t *pSend_Buf, uint16_t vCount)
{
	uint8_t count;

	while (vCount / UART_TX_FIFO_SIZE > 0)
	{
		while (UART_GetFlagState(UART1, UART_FLAG_THR_EMPTY) == 0)
			;
		for (count = UART_TX_FIFO_SIZE; count > 0; count--)
		{
			UART1->RB_THR = *pSend_Buf++;
		}
		vCount -= UART_TX_FIFO_SIZE;
	}

	while (UART_GetFlagState(UART1, UART_FLAG_THR_EMPTY) == 0)
		;
	while (vCount--)
	{
		UART1->RB_THR = *pSend_Buf++;
	}
}

void mible_mcu_send_byte(uint8_t byte)
{
	while (UART_GetFlagState(UART1, UART_FLAG_THR_EMPTY) == 0)
		;
	UART1->RB_THR = byte;
}

void mible_mcu_flushinput(void)
{
	UART_ClearRxFifo(UART1);

    mible_mcu_fifo_rx.rd_pos = 0;
    mible_mcu_fifo_rx.wt_pos = 0;
}

static uint16_t mible_mcu_fifo_recv(uint8_t *bytes, uint16_t pos, uint16_t require)
{
    for (uint8_t i = 0; (mible_mcu_fifo_rx.rd_pos != mible_mcu_fifo_rx.wt_pos) && (pos < require); ++i)
    {
        bytes[pos++] = mible_mcu_fifo_rx.buf[mible_mcu_fifo_rx.rd_pos++];
        mible_mcu_fifo_rx.rd_pos %= sizeof(mible_mcu_fifo_rx.buf);
    }

    return pos;
}

PT_THREAD(mible_mcu_recv_bytes(void *bytes, uint16_t req, uint32_t timeout))
{
    static uint8_t *out;
    static uint32_t exp;
    static uint16_t req_len, recv_len;

    PT_BEGIN(&pt_recv);

    out = bytes;
    req_len = req;
    recv_len = 0;
    exp = RTC_GetCounter() + timeout;

    PT_WAIT_UNTIL(&pt_recv, (req_len == (recv_len = mible_mcu_fifo_recv(out, recv_len, req_len))) || (RTC_GetCounter() > exp));

    if (RTC_GetCounter() > exp)
    {
        MI_LOG_ERROR("Timeout req_len %d, recv_len %d", req_len, recv_len);
        goto EXIT;
    }

    PT_END(&pt_recv);

EXIT:
    PT_EXIT(&pt_recv);
}

static struct
{
	miio_xmodem_t xmodem;
	void *data;
	uint32_t length;
} mijia_mcu_ota;

mible_status_t mible_mcu_cmd_send(mible_mcu_cmd_t cmd, void *arg)
{
	MI_LOG_INFO("cmd %d", cmd);

	/* add your own code here*/
	mible_status_t ret = MI_SUCCESS;
	unsigned int last_index = 0;
	unsigned int crc32 = 0;
	unsigned int recv_bytes = 0;
	unsigned char *buf = mijia_mcu_ota.xmodem.xbuff; /* borrow xmodem */

	rtk_mcu_enter_master();

	switch (cmd)
	{
	case MIBLE_MCU_GET_VERSION:
		MI_LOG_INFO("down get_ver\r");
		mible_mcu_send_buffer("down get_ver\r", strlen("down get_ver\r"));
		break;
	case MIBLE_MCU_READ_DFUINFO:
		MI_LOG_INFO("down rd_info\r");
		mible_mcu_send_buffer("down rd_info\r", strlen("down dfu_rdinfo\r"));
		break;
	case MIBLE_MCU_WRITE_DFUINFO:
		last_index = ((mible_dfu_info_t *)arg)->last_index;
		crc32 = ((mible_dfu_info_t *)arg)->crc32;
		recv_bytes = ((mible_dfu_info_t *)arg)->recv_bytes;
		sprintf((char *)buf, "down wr_info %d %08x %d\r", last_index, crc32, recv_bytes);
		MI_LOG_INFO("%s", buf);
		mible_mcu_send_buffer(buf, strlen((char *)buf));
		break;
	case MIBLE_MCU_WRITE_FIRMWARE:
		sprintf((char *)buf, "down nvm_write %08x\r", (unsigned int)(((mible_mcu_nvminfo_t *)arg)->address));
		MI_LOG_INFO("%s", buf);
		mible_mcu_send_buffer(buf, strlen((char *)buf));
		break;
	case MIBLE_MCU_VERIFY_FIRMWARE:
		mible_mcu_send_buffer("down dfu_verify\r", strlen("down dfu_verify\r"));
		break;
	case MIBLE_MCU_UPGRADE_FIRMWARE:
		mible_mcu_send_buffer("down dfu_active\r", strlen("down dfu_active\r"));
		break;
	case MIBLE_MCU_TRANSFER:
		if (MI_SUCCESS != miio_xmodem_create_instance(&mijia_mcu_ota.xmodem))
		{
			MI_LOG_ERROR("miio_xmodem_create_instance FAIL!!\n");
			ret = MI_ERR_NO_MEM;
			break;
		}

		MI_LOG_DEBUG("mible_mcu_nvm_write addr %08x, len %d\n",
					 ((mible_mcu_nvminfo_t *)arg)->address, ((mible_mcu_nvminfo_t *)arg)->length);

		mijia_mcu_ota.data = ((mible_mcu_nvminfo_t *)arg)->p_data;
		mijia_mcu_ota.length = ((mible_mcu_nvminfo_t *)arg)->length;

		break;
	default:
		break;
	}

	mible_mcu_flushinput();

	if (ret != MI_SUCCESS)
	{
		rtk_mcu_exit_master();
	}

	return ret;
}

static const struct
{
	uint8_t req_len;
	uint32_t timeout;
} mible_mcu_req_table[MIBLE_MCU_TRANSFER] =
	{
		[MIBLE_MCU_GET_VERSION] = {4, 100},
		[MIBLE_MCU_READ_DFUINFO] = {26, 200},
		[MIBLE_MCU_WRITE_DFUINFO] = {3, 100},
		[MIBLE_MCU_WRITE_FIRMWARE] = {3, 100},
		[MIBLE_MCU_VERIFY_FIRMWARE] = {3, 3000},
		[MIBLE_MCU_UPGRADE_FIRMWARE] = {3, 100},
};

mible_status_t mible_mcu_cmd_wait(mible_mcu_cmd_t cmd, void *arg)
{
	uint8_t *recv = mijia_mcu_ota.xmodem.xbuff; /* borrow xmodem */
	uint8_t read_cnt;
	/* add your own code here*/
	mible_status_t ret = MI_ERR_BUSY;
	unsigned int last_index = 0;
	unsigned int crc32 = 0;
	unsigned int recv_bytes = 0;
	char status;

	if (cmd == MIBLE_MCU_TRANSFER)
	{
		status = xmodem_transfer_data_pt(&mijia_mcu_ota.xmodem, mijia_mcu_ota.data, mijia_mcu_ota.length);
		if (status == PT_WAITING)
		{
			ret = MI_ERR_BUSY;
		}
		else if (status == PT_EXITED)
		{
			MI_LOG_ERROR("xmodem_transfer_data failed\n");
			ret = MI_ERR_TIMEOUT;
		}
		else
		{
			MI_LOG_INFO("xmodem_transfer_data successful\n");
			ret = MI_SUCCESS;
		}
		goto EXIT;
	}

	status = mible_mcu_recv_bytes(recv, mible_mcu_req_table[cmd].req_len, mible_mcu_req_table[cmd].timeout);
	if (PT_WAITING == status)
	{
		ret = MI_ERR_BUSY;
		goto EXIT;
	}
	else if (PT_EXITED == status)
	{
		ret = MI_ERR_TIMEOUT;
		goto EXIT;
	}
	else
	{
		ret = MI_SUCCESS;
	}

	MI_LOG_HEXDUMP(recv, mible_mcu_req_table[cmd].req_len);

	switch (cmd)
	{
	case MIBLE_MCU_GET_VERSION:
		for (read_cnt = 0; read_cnt < 4; read_cnt++)
		{
			if (recv[read_cnt] < '0' || recv[read_cnt] > '9')
			{
				MI_LOG_ERROR("MIBLE_MCU_GET_VERSION invalid formate!!!\n");
				ret = MI_ERR_INVALID_PARAM;
				goto EXIT;
			}
		}

		memcpy((uint8_t *)arg, recv, 4);
		break;
	case MIBLE_MCU_READ_DFUINFO:
		MI_LOG_INFO("mible_mcu_read_dfuinfo\n");
		if (0 == strncmp((const char *)recv, "ok", strlen("ok")))
		{
			read_cnt = sscanf((const char *)recv, "%*s%04x%08x%08x", &last_index, &crc32, &recv_bytes);
			if (read_cnt == 3)
			{
				MI_LOG_INFO("mible_mcu_read_dfuinfo succ index %d, cec32 %08x, recv %d!!!\n",
							last_index, crc32, recv_bytes);
				((mible_dfu_info_t *)arg)->last_index = last_index;
				((mible_dfu_info_t *)arg)->crc32 = crc32;
				((mible_dfu_info_t *)arg)->recv_bytes = recv_bytes;
			}
			else
			{
				MI_LOG_ERROR("mible_mcu_read_dfuinfo fail : param %d!!!\n", read_cnt);
				ret = MI_ERR_INVALID_PARAM;
			}
		}
		else
		{
			MI_LOG_ERROR("mible_mcu_read_dfuinfo fail !!!\n");
			ret = MI_ERR_NOT_FOUND;
		}
		break;
	case MIBLE_MCU_WRITE_DFUINFO:
		MI_LOG_INFO("mible_mcu_write_dfuinfo\n");
		if (0 == strncmp((const char *)recv, "ok", strlen("ok")))
		{
			MI_LOG_INFO("mible_mcu_write_dfuinfo succ !!!\n");
		}
		else
		{
			MI_LOG_ERROR("mible_mcu_write_dfuinfo fail !!!\n");
			ret = MI_ERR_NOT_FOUND;
		}
		break;
	case MIBLE_MCU_WRITE_FIRMWARE:
		MI_LOG_INFO("mible_mcu_nvm_write\n");

		if (0 == strncmp((const char *)recv, "ok", strlen("ok")))
		{
			MI_LOG_INFO("receive ok, start xmodem!!!\n");
		}
		else
		{
			MI_LOG_ERROR("mible_mcu_nvm_write fail !!!\n");
			ret = MI_ERR_NOT_FOUND;
		}
		break;
	case MIBLE_MCU_VERIFY_FIRMWARE:
		MI_LOG_INFO("mible_mcu_verify_firmware\n");
		if (0 == strncmp((const char *)recv, "ok", strlen("ok")))
		{
			MI_LOG_INFO("mible_mcu_verify_firmware succ !!!\n");
		}
		else
		{
			MI_LOG_ERROR("mible_mcu_verify_firmware fail !!!\n");
			ret = MI_ERR_NOT_FOUND;
		}
		break;
	case MIBLE_MCU_UPGRADE_FIRMWARE:
		MI_LOG_INFO("mible_mcu_upgrade_firmware\n");
		if (0 == strncmp((const char *)recv, "ok", strlen("ok")))
		{
			MI_LOG_INFO("mible_mcu_upgrade_firmware succ !!!\n");
		}
		else
		{
			MI_LOG_ERROR("mible_mcu_upgrade_firmware fail !!!\n");
			ret = MI_ERR_NOT_FOUND;
		}
		break;
	default:
		break;
	}

EXIT:

	if (ret != MI_ERR_BUSY)
	{
		rtk_mcu_exit_master();
	}

	return ret;
}
