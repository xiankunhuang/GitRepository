#include "config.h"
#include "backplane.h"


static unsigned int rs485_rx_frames = 0;
static unsigned int rs485_rx_frames_badcrc = 0;
static unsigned int rs485_rx_bytes = 0;
static unsigned int rs485_rx_bytes_ignored = 0;
static unsigned int rs485_tx_frames = 0;
static unsigned int rs485_tx_frames_failed = 0;
static unsigned int rs485_tx_bytes = 0;

void rs485_print_stats()
{
	printk("RX fr=%d crc=%d b=%d bi=%d TX fr=%d fail=%d b=%d\n", rs485_rx_frames, rs485_rx_frames_badcrc, rs485_rx_bytes, rs485_rx_bytes_ignored, rs485_tx_frames, rs485_tx_frames_failed, rs485_tx_bytes);
}

int rs485_is_busy()
{
	if (DMACC0Config_bit.E || (U0LSR_bit.TEMT == 0)) {		// DMA_TX发完了，而且uart fifo里也发完了
		return 1;
	} else {
		gpio_rs485_tx_onoff(0);
		return 0;
	}
}

// 上一帧还没发完就不处理接收
// 发完了就关闭rs485_tx并处理接收
// 接收及成帧逻辑
//	直到接收到^
//	其后的第1个字节是addr，只处理自己的地址、FF或者FE
//	其后的第2个字节是func_id，需要设定预期长度
//	其后的第3第4个字节是little endian的帧长度，只处理与预期长度相符的
//	接收到预期的长度，检查crc如果合法就交给协议处理
//	上述过程任何失败都导致重新开始
void rs485_rx_proc()
{
	if (rs485_is_busy())      // 上一个帧还没发完，不处理接收
		return;

	static unsigned char frame[CONFIG_FRAME_BUF_SZ];
	static unsigned short frame_len = 0;
	static unsigned short frame_len_expected;
	
	while (U0LSR & 0x01) {
		unsigned char b = U0RBR;
//  printk("%x ", b);
		switch (frame_len) {
			case 0:
				if (b == '^') {
					frame[frame_len++] = b;
				} else {
					rs485_rx_bytes_ignored++;
				}
				break;

			case 1:
				if ((b == 0xFF) || (b == 0xFE) || (b == g_data_runtime.backplane_addr)) {
					// 注意所有包都处理广播地址0xFF和0xFE，其中0xFF需要应答（目前只有hello帧），0xFE不需应答（目前有hello/下发profile/ioctl/以后有可能有批量升级）
					frame[frame_len++] = b;
				} else {
					rs485_rx_bytes_ignored += frame_len + 1;
					frame_len = 0;
				}
				break;

			case 2:
				if (b <= PROTOCOL_CMD_FUNC_ID_MAX) {
					frame[frame_len++] = b;
					printk("F%x\n", b);
				} else {
					rs485_rx_bytes_ignored += frame_len + 1;
					frame_len = 0;
				}
				break;

			case 3:
				frame[frame_len++] = b;
				break;

			case 4: {
				unsigned short len = frame[3] | (b << 8);
				if ((len == protocol_frame_len(frame[2])) || ((frame[2] == 0x08) && (len >= PROTOCOL_CMD_FUNC_08_88_LEN_MIN) && (len <= PROTOCOL_CMD_FUNC_08_88_LEN_MAX))) {
					frame[frame_len++] = b;
					frame_len_expected = len;
				} else {
					rs485_rx_bytes_ignored += frame_len + 1;
					frame_len = 0;
				}
				break;
			}
			default:
				if (frame_len < (frame_len_expected - 1)) {
					frame[frame_len++] = b;
				} else if (frame_len == (frame_len_expected - 1)) {
					if (b == '$') {
						frame[frame_len++] = b;
						unsigned short crc = frame[frame_len - 3] | (frame[frame_len - 2] << 8);
						if (crc == crc16(frame, frame_len - 3)) {
							rs485_rx_bytes += frame_len;
							rs485_rx_frames++;
							g_data_runtime.last_rs485_frame_received_tick = g_tick_100us;
							protocol_exec(frame, frame_len);
						} else {
							rs485_rx_frames_badcrc++;
						}
						frame_len = 0;
					} else {
						rs485_rx_bytes_ignored += frame_len + 1;
						frame_len = 0;
					}
				} else {
					rs485_rx_bytes_ignored += frame_len + 1;
					frame_len = 0;
				}
				break;
					
		}

	}

}

// <0表示失败，=0表示成功
int rs485_crc_and_tx(unsigned char *frame, unsigned short frame_len)
{
	if (DMACC0Config_bit.E) {		// 上一个帧还没发完，直接报警返回
		printk("why packets overlap len=%d\n", frame_len);
		rs485_tx_frames_failed++;
		return -1;
	}
	rs485_tx_frames++;
	rs485_tx_bytes += frame_len;

	unsigned short crc = crc16(frame, frame_len - 3);
	frame[frame_len - 3] = crc & 0xFF;
	frame[frame_len - 2] = (crc & 0xFF00) >> 8;

	IRQDisable();
	gpio_rs485_tx_onoff(1);

	DMACC0SrcAddr = (unsigned int)frame;
	DMACC0Control_bit.TRANSFERSIZE = frame_len;
	DMACC0Config_bit.E = 1;
	IRQEnable();
	return 0;
}

void rs485_init()
{
	// UART0 用作485端口，8n1，无中断
	PCONP_bit.PCUART0 = 1;
	U0LCR  = 0x83;
	U0DLM  = _F_PCLK / 16 / CONFIG_UART0_BAUDRATE / 256;
	U0DLL  = _F_PCLK / 16 / CONFIG_UART0_BAUDRATE % 256;
	U0LCR  = 0x03;
	U0FCR  = 0xCF;
    U0IER = 0;				// disable all uart1 interrupts

	// DMA channel 2用作UART0 TX
	PCONP_bit.PCGPDMA = 1;
	DMACConfig_bit.E = 1;			// enable DMA controller
	*((volatile unsigned int *)0x400FC1C4) &= ~(1 << 0);	// DMASEL08 = uart0 tx
	DMACIntErrClr = 0xFF;
    DMACIntTCClear = 0xFF;
	DMACC0SrcAddr = 0;
    DMACC0DestAddr = 0x4000C000;	// UART0 Transmitter Holding Register
    DMACC0LLI = 0;
    DMACC0Control = (0 << 0)		// transfer size
                  | (0x00 << 12)	// src burst size = 1，这里跟datasheet不同，但不设0 dma就完成不了
                  | (0x00 << 15)	// dest burst = 1
                  | (0x00 << 18)	// src width = 8-bit
                  | (0x00 << 21)	// dest width = 8-bit
                  | (0x01 << 26)	// src increment = 1
                  | (0x00 << 27)	// dest increment = 0
                  | (0x00 << 31);	// terminal count interrupt disabled
    DMACC0Config = (0 << 0)			// disabled
                 | (0x0 << 1)		// src = mem
                 | (8 << 6)			// dest = uart0 tx
                 | (0x1 << 11)		// TransferType: Memory to peripheral
                 | (1 << 14)		// Interrupt error mask
                 | (1 << 15);		// Terminal count interrupt mask

}

unsigned short crc16(const unsigned char * data, unsigned short data_len)
{
	static const unsigned short crc_table[] = {
		0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
		0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
		0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
		0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
		0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
		0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
		0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
		0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
		0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
		0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
		0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
		0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
		0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
		0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
		0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
		0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
		0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
		0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
		0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
		0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
		0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
		0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
		0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
		0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
		0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
		0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
		0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
		0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
		0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
		0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
		0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
		0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
	};
	unsigned char tmp;
	unsigned short crc = 0xFFFF;

	if (data_len < 4)
		return 0;
	while (data_len--) {
		tmp = *data++ ^ crc;
		crc >>= 8;
		crc ^= crc_table[tmp];
	}
	return crc;
}

