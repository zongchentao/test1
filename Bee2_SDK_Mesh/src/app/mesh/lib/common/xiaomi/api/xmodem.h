#ifndef __XMODEM_H__
#define __XMODEM_H__

#include "third_party/pt/pt.h"

typedef enum _miio_xmodem_type {
	XMODEM = 0,
	XMODEM_1K,
} miio_xmodem_type;

typedef struct _miio_xmodem_t{
	miio_xmodem_type type;

	/*    xmodem -  134 bytes : 1[head] + 1[num] + 1[~num] +  128[data] + 2[crc] + 1[null] */
	/* xmodem-1k - 1030 bytes : 1[head] + 1[num] + 1[~num] + 1024[data] + 2[crc] + 1[null] */
	unsigned char xbuff[134];
} miio_xmodem_t;

int miio_xmodem_create_instance(miio_xmodem_t *xmodem);

void miio_xmodem_destroy(miio_xmodem_t *x);

int xmodem_recv_data(miio_xmodem_t *x, unsigned char *dest, int destsz);

/**
 * @brief  transfer xmodem data to mcu module
 *
 * @param[in]  handle: miio_handle_t struct pointer
 * @return
 *             - XMODEM_OK: receive xmodem data success
 *             - XMODEM_ERR: receive xmodem data failed
 */
PT_THREAD(xmodem_transfer_data_pt(miio_xmodem_t *x, unsigned char *src, int srcsz));

#endif
