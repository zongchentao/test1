/**
 * @file	: xmodem.c
 * @author	: suhang7@xiaomi.com
 * @brief	:
 * @version : 0.1
 * @date	: 2020-10-30
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "xmodem.h"

#include <string.h>
#include "mible_log.h"
#include "mible_api.h"
#include "mible_mcu.h"
#include "third_party/pt/pt.h"

/* Xmodem flag definition */
#define SOH		(0x01)
#define STX		(0x02)
#define EOT		(0x04)
#define ACK		(0x06)
#define NAK		(0x15)
#define CAN		(0x18)
#define CTRLZ	(0x1A)

#define DLY_1S 100
#define MAXRETRANS 25

#define XMODEM_OK			MI_SUCCESS
#define XMODEM_ERR			MI_ERR_INTERNAL
#define XMODEM_PARAM_ERR	MI_ERR_INVALID_PARAM

static unsigned short crc16_ccitt(const unsigned char* buf, int len)
{
	unsigned short crc = 0;
	while (len--) {
		int i;
		crc ^= *buf++ << 8;

		for (i = 0; i < 8; ++i) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc = crc << 1;
		}
	}
	return crc;
}

static int check(int crc, const unsigned char *buf, int sz)
{
    if (crc) {
        unsigned short crc = crc16_ccitt(buf, sz);
        unsigned short tcrc = (buf[sz]<<8)+buf[sz+1];
        if (crc == tcrc)
            return 1;
    }
    else {
        int i;
        unsigned char cks = 0;
        for (i = 0; i < sz; ++i) {
            cks += buf[i];
        }
        if (cks == buf[sz])
        return 1;
    }

    return 0;
}

int _inbyte(unsigned short timeout)
{
	char ret;
    uint8_t c;

	while((ret = mible_mcu_recv_bytes(&c, 1, timeout)) == PT_WAITING);
	
	if (ret == PT_EXITED)
	{
        return -1;
	}
	
	return c;
}

void _outbyte(int c)
{
	MI_LOG_DEBUG("[xmodem]send byte [%02x]", c);
    mible_mcu_send_byte((uint8_t)c);
}

static inline void flushinput(void)
{
    /*
    while (_inbyte(((DLY_1S)*3)>>1) >= 0)
        ;
    */
    mible_mcu_flushinput();
}

int xmodem_recv_data(miio_xmodem_t *x, unsigned char *dest, int destsz)
{
    //unsigned char xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
    unsigned char *xbuff = x->xbuff;
	unsigned char *p;
	int bufsz, crc = 0;
	unsigned char trychar = 'C';
	unsigned char packetno = 1;
	int i, c, len = 0;
	int retry, retrans = MAXRETRANS;

	for(;;) {
		for( retry = 0; retry < 16; ++retry) {
			if (trychar) _outbyte(trychar);
			if ((c = _inbyte((DLY_1S)<<1)) >= 0) {
				switch (c) {
				case SOH:
					bufsz = 128;
                    MI_LOG_DEBUG("[xmodem]XMODEM recv SOH, goto start_recv\n");
					goto start_recv;
				case STX:
					bufsz = 1024;
					goto start_recv;
				case EOT:
                    MI_LOG_DEBUG("[xmodem]XMODEM recv EOT, send ACK\n");
                    _outbyte(ACK);
                    /* check valid length */
                    c = len-1;
                    while(dest[c] == 0){
                        c--;
                    }
                    if(dest[c] == CTRLZ){
                        len = c;
                    }
					flushinput();
					return len; /* normal end */
				case CAN:
					if ((c = _inbyte(DLY_1S)) == CAN) {
                        _outbyte(ACK);
						flushinput();
						return -1; /* canceled by remote */
					}
					break;
				default:
					break;
				}
			}
		}
		if (trychar == 'C') { trychar = NAK; continue; }
		flushinput();
		_outbyte(CAN);
		_outbyte(CAN);
		_outbyte(CAN);
		return -2; /* sync error */

	start_recv:
		if (trychar == 'C') crc = 1;
		trychar = 0;
		p = xbuff;
		*p++ = c;
		for (i = 0;  i < (bufsz+(crc?1:0)+3); ++i) {
			if ((c = _inbyte(DLY_1S)) < 0) goto reject;
			*p++ = c;
		}
        MI_LOG_DEBUG("[xmodem]XMODEM recv %d byte: [1]%02x [2]%02x\n", i, xbuff[1], xbuff[2]);
        MI_LOG_HEXDUMP(xbuff, i+1);

		if (xbuff[1] == (unsigned char)(~xbuff[2]) && 
			(xbuff[1] == packetno || xbuff[1] == (unsigned char)packetno-1) &&
			check(crc, &xbuff[3], bufsz)) {
			if (xbuff[1] == packetno)	{
				register int count = destsz - len;
				if (count > bufsz) count = bufsz;
				if (count > 0) {
					memcpy (&dest[len], &xbuff[3], count);
					len += count;
				}
				++packetno;
				retrans = MAXRETRANS+1;
			}
			if (--retrans <= 0) {
				flushinput();
				_outbyte(CAN);
				_outbyte(CAN);
				_outbyte(CAN);
				return -3; /* too many retry error */
			}
			_outbyte(ACK);
			continue;
		}
	reject:
        MI_LOG_DEBUG("[xmodem]XMODEM recv reject, send NAK");
		flushinput();
		_outbyte(NAK);
	}
}

char __inbyte(int *c, unsigned short timeout)
{
	char ret;
	static uint8_t val;

	ret = mible_mcu_recv_bytes(&val, 1, timeout);
	if (ret == PT_WAITING)
	{
		goto EXIT;
	}
	else if (ret == PT_EXITED)
	{
		*c = -1;
	}
	else if (ret == PT_ENDED)
	{
        *c = val;
	}

EXIT:
	return ret;
}

static pt_t pt1;

PT_THREAD(xmodem_transfer_data_pt(miio_xmodem_t *x, unsigned char *src, int srcsz))
{
    static unsigned char *xbuff;
    static int bufsz, crc;
    static unsigned char packetno;
    static int i, c, len;
    static int retry;
	
	PT_BEGIN(&pt1);
	
	MI_LOG_INFO("234\n");
	
	xbuff = x->xbuff;
	crc = -1;
	packetno = 1;
	len = 0;

    for(;;) {
        for(retry = 0; retry < 16; ++retry) {
			PT_WAIT_UNTIL(&pt1, PT_WAITING != __inbyte(&c, DLY_1S << 1));
			if (c >= 0) {
                switch (c) {
                case 'C':
                    crc = 1;
                    goto start_trans;
                case NAK:
                    crc = 0;
                    goto start_trans;
                case CAN:
					PT_WAIT_UNTIL(&pt1, PT_WAITING != __inbyte(&c, DLY_1S));
					if (c == CAN) {
                        _outbyte(ACK);
                        flushinput();
                        goto EXIT;
                    }
                    break;
                default:
                    break;
                }
            }
        }
        _outbyte(CAN);
        _outbyte(CAN);
        _outbyte(CAN);
        flushinput();
        goto EXIT;

        for(;;) {
        start_trans:
#ifdef TRANSMIT_XMODEM_1K
            xbuff[0] = STX; bufsz = 1024;
#else
            xbuff[0] = SOH; bufsz = 128;
#endif
            xbuff[1] = packetno;
            xbuff[2] = ~packetno;
            c = srcsz - len;
            MI_LOG_DEBUG("XMODEM start_trans packetno = %d, c = %d, len = %d\n", packetno,c,len);
            if (c > bufsz) c = bufsz;
            if (c > 0) {
                memset (&xbuff[3], 0, bufsz);
                memcpy (&xbuff[3], &src[len], c);
                if (c < bufsz) xbuff[3+c] = CTRLZ;
                if (crc) {
                    unsigned short ccrc = crc16_ccitt(&xbuff[3], bufsz);
					MI_LOG_INFO("crcc 0x%04x", ccrc);
                    xbuff[bufsz+3] = (ccrc>>8) & 0xFF;
                    xbuff[bufsz+4] = ccrc & 0xFF;
                }
                else {
                    unsigned char ccks = 0;
                    for (i = 3; i < bufsz+3; ++i) {
                        ccks += xbuff[i];
                    }
                    xbuff[bufsz+3] = ccks;
                }
                for (retry = 0; retry < MAXRETRANS; ++retry) {
					flushinput();
					
                    MI_LOG_DEBUG("XMODEM send byte retry = %d\n", retry);

					mible_mcu_send_buffer(xbuff, bufsz+4+(crc?1:0));

					PT_WAIT_UNTIL(&pt1, PT_WAITING != __inbyte(&c, DLY_1S));
					if ( c >= 0 ) {
                        switch (c) {
                        case ACK:
                            ++packetno;
                            len += bufsz;
                            goto start_trans;
                        case CAN:
							PT_WAIT_UNTIL(&pt1, PT_WAITING != __inbyte(&c, DLY_1S << 1));
							if (c == CAN) {
                                _outbyte(ACK);
                                flushinput();
                                goto EXIT;
                            }
                            break;
                        case NAK:
                        default:
                            break;
                        }
                    }
                }
                _outbyte(CAN);
                _outbyte(CAN);
                _outbyte(CAN);
                flushinput();
                goto EXIT;
            }
            else {
                for (retry = 0; retry < 10; ++retry) {
                    MI_LOG_DEBUG("XMODEM send EOT retry %d\n", retry);
                    _outbyte(EOT);
					
					PT_WAIT_UNTIL(&pt1, PT_WAITING != __inbyte(&c, DLY_1S << 1));
					MI_LOG_DEBUG("c = %d\n", c);
                    if (c == ACK) break;
                }
                flushinput();
                MI_LOG_DEBUG("XMODEM return %d\n", (c == ACK)?len:-5);
				
				if (c == ACK)
				{
					goto END;
				}
				else
				{
					goto EXIT;
				}
            }
        }
    }

END:
	PT_END(&pt1);
	
EXIT:
    PT_EXIT(&pt1);
}

int miio_xmodem_create_instance(miio_xmodem_t *xmodem)
{
	int ret = XMODEM_OK;
	
	PT_INIT(&pt1);

	if(NULL == xmodem){
		MI_LOG_ERROR("uart or xmodem pointer is NULL");
		return XMODEM_PARAM_ERR;
	}

	xmodem->type = XMODEM;

	return ret;
}

void miio_xmodem_destroy(miio_xmodem_t *x)
{
	MI_LOG_INFO("destroy xmodem instance");
	return;
}
