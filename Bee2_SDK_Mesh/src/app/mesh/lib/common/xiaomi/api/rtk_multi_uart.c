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
#include "gatt_dfu/mible_dfu_auth.h"
#include "xmodem.h"
#include "rtk_uart.h"

#if defined(CUSTOMIZED_MI_CONFIG_FILE)
#include CUSTOMIZED_MI_CONFIG_FILE
#endif

#define ALIGN_SIZE 2048
#define MI_HEADER_SIZE 128
#define MI_MAX_HEADER 10

#define MI_MAGIC_HEADER                                                                                        \
	{                                                                                                      \
		0x00, 0x00, 0x01, 0x00, 0x4D, 0x49, 0xEF, 0x54, 0x46, 0x4F, 0x54, 0x41, 0x82, 0x56, 0x26, 0x47 \
	}

#define MI_MAGIC_TAG                                                                                           \
	{                                                                                                      \
		0x00, 0x00, 0x00, 0x00, 0x4D, 0x49, 0xEF, 0x54, 0x46, 0x4F, 0x54, 0x41, 0x82, 0x56, 0x26, 0x47 \
	}

/* Private define ------------------------------------------------------------*/

#define MIBLE_DFU_TLV_NEW_VERSION 0x01
#define MIBLE_DFU_TLV_OLD_VERSION 0x02
#define MIBLE_DFU_TLV_SIGN_TYPE 0x03
#define MIBLE_DFU_TLV_MODEL 0x04
#define MIBLE_DFU_TLV_PID 0x05
#define MIBLE_DFU_CERT_MAX_SIZE 512

/* Private typedef -----------------------------------------------------------*/
#if defined(__CC_ARM)
__packed struct mible_dfu_ver
{
	uint8_t major;
	uint8_t minor;
	uint8_t revision;
	uint16_t build;
};
typedef struct mible_dfu_ver mible_dfu_ver_t;

__packed struct mible_dfu_tag
{
	uint32_t magic[4];
	uint16_t tag_size;
	uint16_t product_id;
	uint32_t firmware_size;
	uint16_t certificates_size;
	uint8_t flag;
	uint8_t reserved;
	uint8_t payload[1];
};
typedef struct mible_dfu_tag mible_dfu_tag_t;

__packed struct mible_dfu_tlv
{
	uint8_t type;
	uint8_t length;
	uint8_t value[1];
};
typedef struct mible_dfu_tlv mible_dfu_tlv_t;
#elif defined(__GNUC__)

struct __PACKED mible_dfu_ver
{
	uint8_t major;
	uint8_t minor;
	uint8_t revision;
	uint16_t build;
};
typedef struct mible_dfu_ver mible_dfu_ver_t;

struct __PACKED mible_dfu_tag
{
	uint8_t magic[16];
	uint16_t tag_size;
	uint16_t product_id;
	uint32_t firmware_size;
	uint16_t certificates_size;
	uint8_t flag;
	uint8_t reserved;
	uint8_t payload[1];
};
typedef struct mible_dfu_tag mible_dfu_tag_t;

struct __PACKED mible_dfu_tlv
{
	uint8_t type;
	uint8_t length;
	uint8_t value[1];
};
typedef struct mible_dfu_tlv mible_dfu_tlv_t;
#endif

#define MI_MULTI_OTA_RECORD_ID 6

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

__PACKED typedef struct
{
	uint16_t last;
	uint32_t crc32;
	uint32_t recv_bytes;

	__PACKED struct
	{
		uint16_t port;
		uint16_t count;
		uint16_t left;
		uint32_t recv_bytes;
	} frag;

	__PACKED struct
	{
		uint32_t total[2]; /*!< The number of Bytes processed.  */
		uint32_t state[8]; /*!< The intermediate digest state.  */
		unsigned char buffer[64];
	} hash;
} mcu_header_t;

static mcu_header_t hdr;

__PACKED struct mcu_end_header
{
	uint16_t port;
	uint32_t crc32;
	uint8_t version[5];
	uint8_t priority[1];
} mcu_table[2];

static uint8_t mcu_switch_pos, mcu_switch_end;

typedef void (*reg_t)(UART_TypeDef *uart);
typedef void (*deg_t)(UART_TypeDef *uart);

#define MCU0_PORT_TX_PIN P3_2
#define MCU0_PORT_RX_PIN P3_3

#define MCU1_PORT_TX_PIN P3_0
#define MCU1_PORT_RX_PIN P3_1

static const struct mcu_ins_t
{
	UART_TypeDef *uart;
} mcu_ins[2] = {
    {UART},
    {UART1},
};

#define MCU_PORT_DEFAULT 0x00
#define MCU_PORT_INVALID 0xff

static uint8_t mcu_port_pos = MCU_PORT_INVALID;

static pt_t pt_recv;

static inline bool mible_mcu_is_trans_complete(void)
{
	return !hdr.frag.left;
}

static inline bool mible_mcu_switch_is_end(void)
{
	return mcu_switch_pos >= mcu_switch_end;
}

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

static void MI_UART(void)
{
	uint8_t rx_data;
	uint32_t int_status;
	uint32_t line_error;
	uint8_t fifo_len;
	UART_TypeDef *uart;

	if (mcu_port_pos == MCU_PORT_INVALID)
	{
		printe("invalid state!!!\n");
		return;
	}

	uart = mcu_ins[mcu_port_pos].uart;

	/* read interrupt id */
	int_status = UART_GetIID(uart);
	/* disable interrupt */
	UART_INTConfig(uart, UART_INT_RD_AVA | UART_INT_LINE_STS, DISABLE);

	if (UART_GetFlagState(uart, UART_FLAG_RX_IDLE) == SET)
	{
		UART_INTConfig(uart, UART_INT_IDLE, DISABLE);
		/* TODO: Wait MCU event */
		UART_INTConfig(uart, UART_INT_IDLE, ENABLE);
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
		fifo_len = UART_GetRxFIFOLen(uart);
		for (uint8_t i = 0; i < fifo_len; ++i)
		{
			UART_ReceiveData(uart, &rx_data, 1);
			rtk_uart_recv_byte(rx_data);
		}
		break;
	/* receive line status interrupt */
	case UART_INT_ID_LINE_STATUS:
		line_error = uart->LSR;
		printe("data_uart_isr Line status error, fail = %d!", line_error);
		break;
	default:
		break;
	}

	/* enable interrupt again */
	UART_INTConfig(uart, UART_INT_RD_AVA | UART_INT_LINE_STS, ENABLE);
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

static void rtk_mcu_enter_master(void)
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

static void rtk_mcu_exit_master(void)
{
	MI_LOG_INFO("mible_mcu_exit_master\n");

	if (rtk_mcu_trans_role != MCU_TRANS_MASTER)
	{
		return;
	}

	/* TODO */
	rtk_wakeup_pin_config(GPIO_INT_POLARITY_ACTIVE_LOW);

	GPIO_INTConfig(WAKE_PIN, ENABLE);
	GPIO_ClearINTPendingBit(WAKE_PIN);
	GPIO_MaskINTConfig(WAKE_PIN, DISABLE);

	rtk_mcu_trans_role = MCU_TRANS_IDLE;
}

static void rtk_mcu_regis(UART_TypeDef *uart)
{
	UART_INTConfig(uart, UART_INT_RD_AVA | UART_INT_LINE_STS | UART_INT_IDLE, ENABLE);
}

static void rtk_mcu_deregis(UART_TypeDef *uart)
{
	/* TODO */
	rtk_mcu_exit_master();

	UART_INTConfig(uart, UART_INT_RD_AVA | UART_INT_LINE_STS | UART_INT_IDLE, DISABLE);
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
	/* not support */
	return false;
}

static mible_status_t mible_mcu_switch_port(uint8_t new_port)
{
	if (new_port >= sizeof(mcu_ins) / sizeof(struct mcu_ins_t))
	{
		MI_LOG_ERROR("unsupport port %d", new_port);
		return MI_ERR_INVALID_PARAM;
	}

	if (mcu_port_pos == new_port)
	{
		return MI_SUCCESS;
	}

	if (mcu_port_pos != MCU_PORT_INVALID ||
	    new_port == MCU_PORT_INVALID)
	{
		rtk_mcu_deregis(mcu_ins[mcu_port_pos].uart);
		MI_LOG_INFO("close port %d\n", mcu_port_pos);
		mcu_port_pos = MCU_PORT_INVALID;
	}

	if (new_port != MCU_PORT_INVALID)
	{
		mcu_port_pos = new_port;
		rtk_mcu_regis(mcu_ins[mcu_port_pos].uart);
		MI_LOG_INFO("open port %d\n", mcu_port_pos);

		return MI_SUCCESS;
	}

	return MI_SUCCESS;
}

void rtk_mcu_status(mible_dfu_state_t state, mible_dfu_param_t *param)
{
	MI_LOG_INFO("mcu update state %d", state);

	if (state != MIBLE_DFU_STATE_CANCEL)
	{
		return;
	}

	mcu_switch_pos = mcu_switch_end = 0;

	mible_mcu_switch_port(MCU_PORT_INVALID);

	PT_INIT(&pt_recv);
}

void rtk_mcu_perip_power_off(void)
{
	UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS | UART_INT_IDLE, DISABLE);
	UART_DeInit(UART);

	/* Turn off UART clock */
	RCC_PeriphClockCmd(APBPeriph_UART0, APBPeriph_UART0_CLOCK, DISABLE);

	/* pad config */
	Pad_Config(MCU0_PORT_TX_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);
	Pad_Config(MCU0_PORT_RX_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);

	UART_INTConfig(UART1, UART_INT_RD_AVA | UART_INT_LINE_STS | UART_INT_IDLE, DISABLE);
	UART_DeInit(UART1);

	/* Turn off UART clock */
	RCC_PeriphClockCmd(APBPeriph_UART1, APBPeriph_UART1_CLOCK, DISABLE);

	/* pad config */
	Pad_Config(MCU1_PORT_TX_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);
	Pad_Config(MCU1_PORT_RX_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);

	Pad_Config(WAKE_PIN_NUM, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);

	// use this pin to wakeup the chip from Deep Low Power Mode.
	System_WakeUpPinEnable(WAKE_PIN_NUM, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_DISABLE);
}

void rtk_mcu_perip_power_on(void)
{
	RCC_PeriphClockCmd(APBPeriph_UART0, APBPeriph_UART0_CLOCK, ENABLE);
	RCC_PeriphClockCmd(APBPeriph_UART1, APBPeriph_UART1_CLOCK, ENABLE);

	/* pinmux config */
	Pinmux_Config(MCU0_PORT_TX_PIN, UART0_TX);
	Pinmux_Config(MCU0_PORT_RX_PIN, UART0_RX);

	/* pad config */
	Pad_Config(MCU0_PORT_TX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);
	Pad_Config(MCU0_PORT_RX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);

	/* pinmux config */
	Pinmux_Config(MCU1_PORT_TX_PIN, UART1_TX);
	Pinmux_Config(MCU1_PORT_RX_PIN, UART1_RX);

	/* pad config */
	Pad_Config(MCU1_PORT_TX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);
	Pad_Config(MCU1_PORT_RX_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);

	/* uart init */
	UART_InitTypeDef uartInitStruct;
	UART_StructInit(&uartInitStruct);
	/* change default rx trigger level */
	uartInitStruct.rxTriggerLevel = UART_RX_FIFO_TRIGGER_LEVEL_8BYTE;
	uartInitStruct.idle_time = UART_RX_IDLE_8BYTE;

	UART_Init(UART, &uartInitStruct);
	UART_Init(UART1, &uartInitStruct);
	Pad_Config(WAKE_PIN_NUM, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
}

void mible_mcu_init(void)
{
	PT_INIT(&pt_recv);

	rtk_mcu_perip_power_on();

	/*  Enable UART IRQ  */
	NVIC_InitTypeDef nvic_init_struct;
	nvic_init_struct.NVIC_IRQChannel = UART0_IRQn;
	nvic_init_struct.NVIC_IRQChannelCmd = ENABLE;
	nvic_init_struct.NVIC_IRQChannelPriority = 5;
	NVIC_Init(&nvic_init_struct);

	nvic_init_struct.NVIC_IRQChannel = UART1_IRQn;
	nvic_init_struct.NVIC_IRQChannelCmd = ENABLE;
	nvic_init_struct.NVIC_IRQChannelPriority = 5;
	NVIC_Init(&nvic_init_struct);

	RamVectorTableUpdate(Uart0_VECTORn, MI_UART);
	RamVectorTableUpdate(Uart1_VECTORn, MI_UART);

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
	UART_TypeDef *uart = mcu_ins[mcu_port_pos].uart;

	while (vCount / UART_TX_FIFO_SIZE > 0)
	{
		while (UART_GetFlagState(uart, UART_FLAG_THR_EMPTY) == 0)
			;
		for (count = UART_TX_FIFO_SIZE; count > 0; count--)
		{
			uart->RB_THR = *pSend_Buf++;
		}
		vCount -= UART_TX_FIFO_SIZE;
	}

	while (UART_GetFlagState(uart, UART_FLAG_THR_EMPTY) == 0)
		;
	while (vCount--)
	{
		uart->RB_THR = *pSend_Buf++;
	}
}

static mible_status_t send_cmd(const uint8_t *pSend_Buf, uint16_t vCount)
{
	if (mcu_port_pos == MCU_PORT_INVALID)
	{
		printe("invalid state!!!\n");
		return MI_ERR_INVALID_STATE;
	}

	rtk_mcu_enter_master();

	mible_mcu_send_buffer(pSend_Buf, vCount);

	return MI_SUCCESS;
}

void mible_mcu_send_byte(uint8_t byte)
{
	UART_TypeDef *uart;

	if (mcu_port_pos == MCU_PORT_INVALID)
	{
		printe("invalid state!!!\n");
		return;
	}

	uart = mcu_ins[mcu_port_pos].uart;

	while (UART_GetFlagState(uart, UART_FLAG_THR_EMPTY) == 0)
		;
	uart->RB_THR = byte;
}

void mible_mcu_flushinput(void)
{
	if (mcu_port_pos == MCU_PORT_INVALID)
	{
		printe("invalid state!!!\n");
		return;
	}

	UART_ClearRxFifo(mcu_ins[mcu_port_pos].uart);

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
		MI_LOG_ERROR("Timeout");
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
	uint16_t offset;
} mijia_mcu_ota;

#include "common/crc32.h"
#include "third_party/mbedtls/sha256.h"
static void mible_mcu_sha256(bool is_init, bool is_end, uint8_t *const data, uint16_t length, uint8_t *out)
{
	mbedtls_sha256_context ctx;
	uint32_t offset = 0, copy_size;
	uint8_t buffer[256];

	if (is_init)
	{
		MI_LOG_INFO("mcu hash init\n");

		mbedtls_sha256_init(&ctx);
		mbedtls_sha256_starts(&ctx, 0);
		memcpy((void *)hdr.hash.state, ctx.state, 32);
		return;
	}

	/* Recovery previous state and values */
	memcpy(ctx.total, (void *)hdr.hash.total, 8);
	memcpy(ctx.state, (void *)hdr.hash.state, 32);
	memcpy(ctx.buffer, (void *)hdr.hash.buffer, 64);
	ctx.is224 = 0;

	while (offset < length)
	{
		copy_size = MIN(length - offset, sizeof(buffer));
		memcpy(buffer, data + offset, copy_size);
		mbedtls_sha256_update(&ctx, buffer, copy_size);
		offset += copy_size;
	}

	if (is_end)
	{
		MI_LOG_INFO("mcu hash end\n");

		mbedtls_sha256_finish(&ctx, out);
		mbedtls_sha256_free(&ctx);		
		return;
	}

	/* Store currently state and values */
	memcpy((void *)hdr.hash.total, ctx.total, 8);
	memcpy((void *)hdr.hash.state, ctx.state, 32);
	memcpy((void *)hdr.hash.buffer, ctx.buffer, 64);
}

#include "cryptography/mi_crypto.h"
#include "cryptography/mi_crypto_backend_uECC.h"
static mible_status_t mble_mcu_verify(void)
{
	int err;
	char *pem_crt;
	mible_dfu_tag_t *tag;
	uint16_t cert_len, signature_size;
	msc_crt_t server_crt, develop_crt;
	uint8_t const margic[] = MI_MAGIC_TAG;
	uint8_t srv_der_crt[MIBLE_DFU_CERT_MAX_SIZE];
	uint8_t dev_der_crt[MIBLE_DFU_CERT_MAX_SIZE];
	uint16_t length = mijia_mcu_ota.length - MI_HEADER_SIZE;
	uint8_t child_sha[32], pack_sig[64], *signature, *certs, *ptr;

	const ecc256_pk_t parent_pub = {
       		0xbe,0xf5,0x8b,0x02,0xda,0xe3,0xff,0xf8,0x54,0x1a,0xa0,0x44,0x8f,0xba,0xc4,0x4d,
        	0xb7,0xc6,0x9a,0x2f,0xa8,0xf0,0xb1,0xb6,0xff,0x7a,0xd9,0x51,0xdb,0x66,0x28,0xfa,
        	0xd7,0xf0,0x20,0xea,0x39,0xa2,0xee,0x86,0x7f,0xdd,0x78,0x3f,0xdc,0x2f,0xb0,0x86,
        	0x09,0x5c,0xc2,0x85,0x04,0x13,0xa2,0x80,0x2c,0x62,0x7d,0xbd,0xc7,0x15,0xf4,0xf9,
	};

	ptr = (uint8_t *)mijia_mcu_ota.data + MI_HEADER_SIZE;
	if (memcmp(ptr, margic, sizeof(margic)))
	{
		MI_LOG_HEXDUMP((void *)ptr, sizeof(margic));
		MI_LOG_ERROR("Invalid header");
		return MI_ERR_INVALID_PARAM;
	}

	tag = (mible_dfu_tag_t *)ptr;
	MI_LOG_INFO("tag size %d pid %d Firmware Size %d Cert Size %d Flag %d",
		    tag->tag_size, tag->product_id, tag->firmware_size, tag->certificates_size, tag->flag);

	certs = ptr + tag->tag_size;
	pem_crt = strstr((const char *)certs, PEM_HEADER);

	cert_len = mbedtls_crt_pem2der((const unsigned char *)pem_crt, tag->certificates_size,
				       srv_der_crt, MIBLE_DFU_CERT_MAX_SIZE);

	err = mbedtls_crt_parse_der(srv_der_crt, cert_len, NULL, &server_crt);
	if (err < 0)
	{
		MI_LOG_WARNING("Can't find server certificate: %d\n", err);
		return MI_ERR_INVALID_PARAM;
	}
	else
	{
		MI_LOG_DEBUG("server cert OU: %s\n", server_crt.sub_o.p);
	}

	/* get developer certificate */
	pem_crt = strstr((const char *)pem_crt + 1, PEM_HEADER);

	cert_len = mbedtls_crt_pem2der((const unsigned char *)pem_crt, tag->certificates_size,
				       dev_der_crt, MIBLE_DFU_CERT_MAX_SIZE);
	err = mbedtls_crt_parse_der(dev_der_crt, cert_len, NULL, &develop_crt);
	if (err < 0)
	{
		MI_LOG_WARNING("Can't parse developer certificate: %d\n", err);
		return MI_ERR_INVALID_PARAM;
	}
	else
	{
		MI_LOG_DEBUG("developer cert OU: %s\n", develop_crt.sub_o.p);
	}

	/* fetch signature */
	signature_size = length - tag->tag_size - tag->certificates_size;
	MI_LOG_INFO("tag size %d cert size %d signature size %d\n",
		    tag->tag_size, tag->certificates_size, signature_size);

	signature = ptr + tag->tag_size + tag->certificates_size;
	err = mbedtls_read_signature(signature, signature_size, pack_sig, 64);
	if (err < 0)
	{
		MI_LOG_WARNING("fail to get signature from DFU package.\n");
		return MI_ERR_INVALID_PARAM;
	}

	mbedtls_sha256(server_crt.tbs.p, server_crt.tbs.len, child_sha, 0);
	err = micro_ecc_verify(P256R1, parent_pub, child_sha, server_crt.sig);
	if (0 != err)
	{
		MI_LOG_ERROR("Server cert is invalid %d.\n", err);
		return MI_ERR_INTERNAL;
	}
	else
	{
		MI_LOG_WARNING("Server cert is valid.\n");
	}

	mbedtls_sha256(develop_crt.tbs.p, develop_crt.tbs.len, child_sha, 0);
	err = micro_ecc_verify(P256R1, server_crt.pub.p, child_sha, develop_crt.sig);
	if (0 != err)
	{
		MI_LOG_ERROR("Developer cert is invalid %d.\n", err);
		return MI_ERR_INTERNAL;
	}
	else
	{
		MI_LOG_WARNING("Developer cert is valid.\n");
	}

	mible_mcu_sha256(false, true, (void *)mijia_mcu_ota.data, MI_HEADER_SIZE + tag->tag_size, child_sha);

	err = micro_ecc_verify(P256R1, develop_crt.pub.p, child_sha, pack_sig);
	if (0 != err)
	{
		MI_LOG_ERROR("FW pack signature is invalid %d.\n", err);
		return MI_ERR_INTERNAL;
	}
	else
	{
		MI_LOG_WARNING("FW pack signature is valid.\n");
	}

	return MI_SUCCESS;
}

static mible_status_t mible_mcu_header_decode(void)
{
	uint16_t pos, len, tag_size, pid, offset = 0;
	uint8_t const *ptr = mijia_mcu_ota.data;
	uint8_t const margic[] = MI_MAGIC_HEADER;

	if (mijia_mcu_ota.length <= MI_HEADER_SIZE)
	{
		MI_LOG_ERROR("too small header length %d\n", mijia_mcu_ota.length);
		return MI_ERR_INVALID_LENGTH;
	}

	if (memcmp(ptr, margic, sizeof(margic)))
	{
		MI_LOG_HEXDUMP((void *)ptr, sizeof(margic));
		MI_LOG_ERROR("invalid header");
		return MI_ERR_INVALID_PARAM;
	}

	offset = sizeof(margic);

	memcpy(&tag_size, ptr + offset, 2);
	offset += 2;

	memcpy(&pid, ptr + offset, 2);
	offset += 2;

	memcpy((void *)&hdr.frag.count, ptr + offset, 2);
	offset += 4;

	memcpy((void *)&hdr.frag.port, ptr + offset, 1);
	offset += 1 + 3;

	MI_LOG_INFO("tag size %d pid %d", tag_size, pid);

	if (tag_size == offset + 4)
	{
		/* TODO Check CRC32 */

		hdr.frag.left = hdr.frag.count;
		MI_LOG_INFO("mcu %d fragment %d", hdr.frag.port, hdr.frag.count);
		return MI_SUCCESS;
	}

	if (ptr[offset] != 0x40)
	{
		MI_LOG_ERROR("invalid type 0x%02x != 0x40", ptr[offset]);
		return MI_ERR_INVALID_PARAM;
	}

	offset++;

	mcu_switch_pos = 0;
	mcu_switch_end = ptr[offset++];

	if (mcu_switch_end > sizeof(mcu_table) / sizeof(struct mcu_end_header))
	{
		MI_LOG_ERROR("unsupport too many devices %d\n", mcu_switch_end);
		return MI_ERR_RESOURCES;
	}

	for (pos = 0; pos < mcu_switch_end; pos++)
	{
		mcu_table[pos].port = ptr[offset++];
		MI_LOG_INFO("[%d] mcu 0x%02x\n", pos, mcu_table[pos].port);
	}

	for (pos = 0; pos < mcu_switch_end; pos++)
	{
		if (ptr[offset] != 0x41)
		{
			MI_LOG_ERROR("invalid type 0x%02x != 0x41", ptr[offset]);
			return MI_ERR_INVALID_PARAM;
		}

		offset++;

		len = ptr[offset++];
		memcpy((void *)&mcu_table[pos].crc32, ptr + offset, len);
		offset += len;
	}

	if (tag_size != offset + 4)
	{
		MI_LOG_ERROR("invalid tag_size %d != %d + 4", tag_size, offset);
		return MI_ERR_INVALID_PARAM;
	}

	return mble_mcu_verify();
}

static mible_status_t mcu_verify_or_switch(bool is_verify)
{
	mible_status_t ret;
	unsigned char *buf = mijia_mcu_ota.xmodem.xbuff; /* borrow xmodem */

	if (mible_mcu_switch_is_end())
	{
		return MI_SUCCESS;
	}

	ret = mible_mcu_switch_port(mcu_table[mcu_switch_pos].port);
	if (ret != MI_SUCCESS)
	{
		MI_LOG_ERROR("open port failed\n");
		return MI_ERR_INVALID_PARAM;
	}

	if (is_verify)
	{
		sprintf((char *)buf, "down dfu_verify %08x\r", mcu_table[mcu_switch_pos].crc32);
	}
	else
	{
		sprintf((char *)buf, "down dfu_active\r");
	}

	MI_LOG_INFO("%s", buf);

	return send_cmd(buf, strlen((char *)buf));
}

mible_status_t mible_mcu_cmd_send(mible_mcu_cmd_t cmd, void *arg)
{
	MI_LOG_INFO("cmd %d", cmd);

	/* add your own code here*/
	mible_status_t ret = MI_SUCCESS;
	uint32_t init_crc32;
	unsigned char *buf = mijia_mcu_ota.xmodem.xbuff; /* borrow xmodem */

	switch (cmd)
	{
	case MIBLE_MCU_GET_VERSION:
		mible_mcu_switch_port(MCU_PORT_DEFAULT);

		MI_LOG_INFO("down get_ver\r");
		ret = send_cmd("down get_ver\r", strlen("down get_ver\r"));
		break;
	case MIBLE_MCU_READ_DFUINFO:
		MI_LOG_INFO("read dfuinfo\r");
		break;
	case MIBLE_MCU_WRITE_DFUINFO:
		hdr.last = ((mible_dfu_info_t *)arg)->last_index;
		hdr.crc32 = ((mible_dfu_info_t *)arg)->crc32;
		hdr.recv_bytes = ((mible_dfu_info_t *)arg)->recv_bytes;

		if (mible_mcu_is_trans_complete())
		{
			break;
		}

		sprintf((char *)buf, "down wr_info %d %08x %d\r",
			hdr.frag.count - hdr.frag.left, hdr.crc32, hdr.frag.recv_bytes);
		MI_LOG_INFO("%s", buf);
		ret = send_cmd(buf, strlen((char *)buf));
		break;
	case MIBLE_MCU_WRITE_FIRMWARE:
		mijia_mcu_ota.data = ((mible_mcu_nvminfo_t *)arg)->p_data;
		mijia_mcu_ota.length = ((mible_mcu_nvminfo_t *)arg)->length;
		mijia_mcu_ota.offset = 0;

		if (!mible_mcu_is_trans_complete() && hdr.frag.left < hdr.frag.count)
		{
			init_crc32 = hdr.crc32;
			goto SEND;
		}

		ret = mible_mcu_header_decode();
		if (ret != MI_SUCCESS)
		{
			goto EXIT;
		}

		hdr.frag.recv_bytes = 0x00;

		if (mible_mcu_is_trans_complete())
		{
			MI_LOG_INFO("total trans complete\n");
			break;
		}

		ret = mible_mcu_switch_port(hdr.frag.port);
		if (ret != MI_SUCCESS)
		{
			MI_LOG_ERROR("open port failed\n");
			goto EXIT;
		}

		mijia_mcu_ota.offset = MI_HEADER_SIZE;
		init_crc32 = soft_crc32(mijia_mcu_ota.data, MI_HEADER_SIZE, hdr.crc32);

SEND:
		sprintf((char *)buf, "down nvm_write %08x %08x\r", hdr.frag.recv_bytes, init_crc32);
		MI_LOG_INFO("%s", buf);
		ret = send_cmd(buf, strlen((char *)buf));
		break;
	case MIBLE_MCU_VERIFY_FIRMWARE:
		mible_record_delete(MI_MULTI_OTA_RECORD_ID);
		mcu_switch_pos = 0;
		ret = mcu_verify_or_switch(true);
		break;
	case MIBLE_MCU_UPGRADE_FIRMWARE:
		mcu_switch_pos = 0;
		ret = mcu_verify_or_switch(false);
		break;
	case MIBLE_MCU_TRANSFER:
		if (MI_SUCCESS != miio_xmodem_create_instance(&mijia_mcu_ota.xmodem))
		{
			MI_LOG_ERROR("miio_xmodem_create_instance FAIL!!\n");
			ret = MI_ERR_NO_MEM;
			break;
		}

		rtk_mcu_enter_master();

		break;
	default:
		break;
	}

EXIT:
	mible_mcu_flushinput();
	
	if (ret != MI_SUCCESS)
	{
		mible_record_delete(MI_MULTI_OTA_RECORD_ID);
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
	[MIBLE_MCU_WRITE_DFUINFO] = {3, 500},
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
	char status;

	if (cmd == MIBLE_MCU_TRANSFER)
	{
		if (mible_mcu_is_trans_complete())
		{
			ret = MI_SUCCESS;
			goto EXIT;
		}

		status = xmodem_transfer_data_pt(&mijia_mcu_ota.xmodem,
						 (uint8_t *)mijia_mcu_ota.data + mijia_mcu_ota.offset,
						 mijia_mcu_ota.length - mijia_mcu_ota.offset);
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
			hdr.frag.recv_bytes += mijia_mcu_ota.length - mijia_mcu_ota.offset;
			MI_LOG_INFO("xmodem_transfer_data successful left %d\n", hdr.frag.left);
			ret = MI_SUCCESS;
		}

		goto EXIT;
	}
	else if (cmd == MIBLE_MCU_READ_DFUINFO)
	{
		ret = mible_record_read(MI_MULTI_OTA_RECORD_ID, (void *)&hdr, sizeof(hdr));
		if (ret != MI_SUCCESS)
		{
			MI_LOG_WARNING("record not found ret %d", ret);
			memset(&hdr, 0x00, sizeof(hdr));
			mible_mcu_sha256(true, false, NULL, 0, NULL);

			ret = MI_ERR_NOT_FOUND;
		}
		else
		{
			MI_LOG_INFO("mible_mcu_read_dfuinfo succ index %d, cec32 %08x, recv %d!!!\n",
				    hdr.last, hdr.crc32, hdr.recv_bytes);
			((mible_dfu_info_t *)arg)->last_index = hdr.last;
			((mible_dfu_info_t *)arg)->crc32 = hdr.crc32;
			((mible_dfu_info_t *)arg)->recv_bytes = hdr.recv_bytes;

			ret = mible_mcu_switch_port(hdr.frag.port);
		}
		
		goto EXIT;
	}
	else if ((cmd == MIBLE_MCU_WRITE_FIRMWARE || cmd == MIBLE_MCU_WRITE_DFUINFO) &&
		 mible_mcu_is_trans_complete())
	{
		ret = MI_SUCCESS;
		goto EXIT;
	}
	else if ((cmd == MIBLE_MCU_VERIFY_FIRMWARE || cmd == MIBLE_MCU_UPGRADE_FIRMWARE) &&
		 mible_mcu_switch_is_end())
	{
		ret = MI_SUCCESS;
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
	case MIBLE_MCU_WRITE_DFUINFO:
		MI_LOG_INFO("mible_mcu_write_dfuinfo\n");
		if (0 == strncmp((const char *)recv, "ok", strlen("ok")))
		{
			MI_LOG_INFO("mible_mcu_write_dfuinfo succ !!!\n");

			mible_mcu_sha256(false, false, mijia_mcu_ota.data, mijia_mcu_ota.length, NULL);

			hdr.frag.left--;

			mible_record_write(MI_MULTI_OTA_RECORD_ID, (void *)&hdr, sizeof(hdr));
		}
		else
		{
			MI_LOG_ERROR("mible_mcu_write_dfuinfo fail !!!\n");
			ret = MI_ERR_NOT_FOUND;
		}

		rtk_mcu_exit_master();

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

			mcu_switch_pos++;

			ret = mcu_verify_or_switch(true);
			if (ret != MI_SUCCESS)
			{
				break;
			}

			ret = MI_ERR_BUSY;
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

			mcu_switch_pos++;

			ret = mcu_verify_or_switch(false);
			if (ret != MI_SUCCESS)
			{
				break;
			}

			ret = MI_ERR_BUSY;
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
		
		/* Capture exception */
		if (ret != MI_SUCCESS)
		{
			mible_record_delete(MI_MULTI_OTA_RECORD_ID);
		}
	}

	return ret;
}
