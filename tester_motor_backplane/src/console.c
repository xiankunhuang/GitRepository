#include <stdarg.h>

#include "config.h"
#include "backplane.h"


static const unsigned char HEX_CHARS[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
#define isdigit(ch)		(((ch) >= HEX_CHARS[0]) && ((ch) <= HEX_CHARS[9]))
#define valueofchar(ch)	((ch) - HEX_CHARS[0])


static int put_char(unsigned char ch);


static inline int atoi(const unsigned char *p)
{
	unsigned int values[16];
	int has_sign = 0;

	if (!p) return 0;
	if (p[0] == '-') has_sign = 1;
	
	int count = 0;
	for (int i = has_sign; (count <= 10) && p[i]; i++) {		// 32位最多10位
		if (isdigit(p[i])) {
			values[count++] = valueofchar(p[i]);
		} else {
			break;
		}
	}
	if (count == 0)
		return 0;

	int value = 0;
	for (int i = 0; i < count; i++) {
		value = (value * 10) + values[i];
	}
	return has_sign ? -value : value;
}

// 返回字符个数
static inline int itoa(int value, unsigned char *string_output)
{
	int has_sign = 0;

	if (value == 0) {
		string_output[0] = '0';
		string_output[1] = 0;
		return 1;
	}

	if (value < 0) {
		string_output[0] = '-';
		value = -value;
		has_sign = 1;
	}

	int digit_count = 0;
	unsigned char reverted_str[16];		// 32位最多10位
	while (value) {
		reverted_str[digit_count++] = HEX_CHARS[value % 10];
		value /= 10;
	}

	for (int i = 0; i < digit_count; i++) {
		string_output[i + has_sign] = reverted_str[digit_count - 1 - i];
	}
	string_output[digit_count + has_sign] = 0;

	return digit_count + has_sign;
}



static const char* COMMANDS[] = {
	"help", 				    // 0
	"reboot yes",				// 1
	"uptime",				    // 2
	"set_rs485_addr#",			// 3
	"rs485_stats",				// 4
	"i2c_stats",				// 5
	"set_tray_state#",			// 6
	"set_running_state#",		// 7
	"print_adc",				// 8
	"set_power0#",				// 9
	"set_pin_config",			// 10
	"set_tray_hw_rev#",			// 11
	"set_tray_sn#",				// 12
	"set_step_motor_x",			// 13
	"set_step_motor_y",			// 14
	"set_step_motor_z",			// 15
	"restart_push_pull_tray"    // 16
};

void console_cmd(const unsigned char cmd_id, const unsigned char * arg)
{
	switch (cmd_id) {
		case 1:
			if ((arg != 0) && (arg[0] == 'y') && (arg[1] == 'e') && (arg[2] == 's')) {
				boot_jump(0);
			} else {
				printk("need \'yes\'\n");
			}
			break;

		case 2:
			if ((arg != 0) && (arg[0] == 'p') && (arg[1] == 'c') && (arg[2] == 'b') && (arg[3] == 't') && (arg[4] == 'e') && (arg[5] == 's') && (arg[6] == 't')) {
				printk("pcb test mode\n");
			} else {
				printk("uptime:%d loops:%d\n", g_tick_100us, g_data_runtime.loops_per_second);
				printk("config:485_timeout=%d  mcu=%d~%d vcc3=%d~%d\n",
					g_global_config.rs485_offline_timeout_100uS,
					g_global_config.vcc3_mcu_lower_limit_mV, g_global_config.vcc3_mcu_upper_limit_mV,
					g_global_config.vcc3_lower_limit_mV, g_global_config.vcc3_upper_limit_mV);
			}
			break;

		case 3:
			if (arg) {
				int v = atoi(arg);
				printk("set rs485_addr=%d\n", (unsigned char)v);
				g_data_backplane_eeprom.addr = v;
				flash_write_backplane_eeprom();
			} else {
				flash_print_backplane_eeprom();
			}
			break;

		case 4:
			rs485_print_stats();
			break;

		case 5:
			i2c_print_stats();
			break;

		
		case 8:
			adc_print();
			print_step_motor_postion();
			break;

		case 9: 		
			if (arg) {
				unsigned int v = atoi(arg);
				(v > 0) ? gpio_electromagnet_onoff(1) : gpio_electromagnet_onoff(0);
			}
			break;

		case 10:
			if (arg) {
				unsigned int v = atoi(arg);
				if (v > 0) FIO0SET_bit.P0_7 = 1;
				else FIO0CLR_bit.P0_7 = 1;
			}
			break;

		case 11:
			if (arg) {
				unsigned int v = atoi(arg);
				if (v > 0) FIO1SET_bit.P1_16 = 1;
				else FIO1CLR_bit.P1_16 = 1;
			}
			break;
		
		
		case 12:
			if (arg) {
				unsigned int v = atoi(arg);
				if (v > 0) FIO1SET_bit.P1_17 = 1; else FIO1CLR_bit.P1_17 = 1;
			}
			break;

		case 13:
			if (arg) {
				unsigned int v = atoi(arg);
				step_motor_goto(0, v, (unsigned int)sm[1].position << 2, (unsigned int)sm[2].position << 2);
			} else 
				print_step_motor_postion();
			break;
		case 14:
			if (arg) {
				unsigned int v = atoi(arg);
				step_motor_goto(0, (unsigned int)sm[0].position << 2, v, (unsigned int)sm[2].position << 2);
			} else {
				print_step_motor_postion();
			}
			break;
		case 15:
			if (arg) {
				unsigned int v = atoi(arg);
				step_motor_goto(0, (unsigned int)sm[0].position << 2, (unsigned int)sm[1].position << 2, v);
			} else {
				print_step_motor_postion();
			}
 			break; 
		case 16:
			if(3 == g_data_runtime.tested){   // 确定是测试流程出错才重新启动测试 
			    g_data_runtime.tested = 0;
			}
			break;
		default:
			printk("unsupported command\n");
			break;
	}
}

// 收到一个回车就执行命令，否则就继续收
void console_rx_proc()
{
	static unsigned char rx_buf[64];
	static unsigned char rx_buf_idx = 0;

	if (U1LSR_bit.DR) {
		rx_buf[rx_buf_idx] = U1RBR;
		put_char(rx_buf[rx_buf_idx]);

		if (rx_buf[rx_buf_idx] == '\n') {
			rx_buf[rx_buf_idx] = 0;

			if (rx_buf_idx == 0) {
				return;
			}

			int cmd_len;
			if (rx_buf_idx && (rx_buf[rx_buf_idx - 1] == '\r')) {
				rx_buf[rx_buf_idx - 1] = 0;
				cmd_len = rx_buf_idx - 1;
			} else {
				cmd_len = rx_buf_idx;
			}

			int cmdid = atoi(rx_buf);
			if ((cmdid <= 0) || (cmdid >= DIMOFARRAY(COMMANDS))) {
				for (unsigned char i = 0; i < DIMOFARRAY(COMMANDS); i++) {
					printk("%d. %s\n", i, COMMANDS[i]);
				}
			} else {
				unsigned char *arg = 0;
				for (int i = 0; i < (cmd_len - 1); i++) {
					if ((rx_buf[i] == ' ') && (rx_buf[i + 1])) {
						arg = rx_buf + (i + 1);
						break;
					}
				}
				console_cmd(cmdid, arg);
			}
			rx_buf_idx = 0;
			return;
		}

		rx_buf_idx++;
		if (rx_buf_idx >= (sizeof(rx_buf) - 3)) {
			printk("cmd too long\n");
			rx_buf_idx = 0;
		}
	}
}

void console_init()
{
    // UART1 用作调试端口，8n1，无中断
    PCONP_bit.PCUART1 = 1;
    U1LCR_bit.WLS = 3;		// 8n1
    U1LCR_bit.DLAB = 1;		// set baudrate
    U1DLM = _F_PCLK / 16 / CONFIG_UART1_BAUDRATE / 256;
    U1DLL = _F_PCLK / 16 / CONFIG_UART1_BAUDRATE % 256;
    U1LCR_bit.DLAB = 0;		// end set baudrate
    U1FCR = 0xCF;			// enable fifo|DMA and reset rx/tx fifo, 14B to trigger RXINT
    U1IER = 0;				// disable all uart1 interrupts

    // DMA channel 1用作UART1 TX
    PCONP_bit.PCGPDMA = 1;
    DMACConfig_bit.E = 1;			// enable DMA controller
    *((volatile unsigned int *)0x400FC1C4) &= ~(1 << 2);	// DMASEL10 = uart1 tx
    DMACIntErrClr = 0xFF;
    DMACIntTCClear = 0xFF;
    DMACC1SrcAddr = 0;
    DMACC1DestAddr = 0x40010000;	// UART1 Transmitter Holding Register
    DMACC1LLI = 0;
    DMACC1Control = (0 << 0)		// transfer size
                  | (0x00 << 12)	// src burst size = 1，这里跟datasheet不同，但不设0 dma就完成不了
                  | (0x00 << 15)	// dest burst = 1
                  | (0x00 << 18)	// src width = 8-bit
                  | (0x00 << 21)	// dest width = 8-bit
                  | (0x01 << 26)	// src increment = 1
                  | (0x00 << 27)	// dest increment = 0
                  | (0x00 << 31);	// terminal count interrupt disabled
    DMACC1Config = (0 << 0)			// disabled
                 | (0x0 << 1)		// src = mem
                 | (10 << 6)		// dest = uart1 tx
                 | (0x1 << 11)		// TransferType: Memory to peripheral
                 | (1 << 14)		// Interrupt error mask
                 | (1 << 15);		// Terminal count interrupt mask
}

// 返回字符个数
static inline int utohex(unsigned int value, unsigned char *string_output)
{
	int non_0_found = 0;
	int idx = 0;
	for (int i = 0; i < 32; i += 4) {
		unsigned char fourbit = (value >> (28 - i)) & 0xF;
		if (!non_0_found && !fourbit)
			continue;
		string_output[idx++] = HEX_CHARS[fourbit];
		non_0_found = 1;
	}
	if (idx == 0) {
		string_output[0] = '0';
		string_output[1] = 'h';
		string_output[2] = 0;
		return 2;
	}
	string_output[idx] = 'h';
	string_output[idx + 1] = 0;	// null terminated
	return idx + 1;
}

//#define BUFFER_SZ 1020		// 不能超过DMA最大的长度4K
#define BUFFER_SZ 4020	


#pragma pack(4)
struct tx_buf_s {
	unsigned char data[BUFFER_SZ];
	int data_len;
};
#pragma pack()

#pragma data_alignment=4
static struct tx_buf_s tx_bufs[2];			// ping-pong

static unsigned char DMAed_idx = 1;	// 取初值1是为了初次进行tx的时候从buf[0]开始

static unsigned char console_ring_buf[BUFFER_SZ];
static unsigned int console_ring_buf_head = 0;
static int console_ring_buf_sz = 0;
#define IDX_OF_console_ring_buf(_start_, _idx_) (((_start_ + _idx_) < BUFFER_SZ) ? (_start_ + _idx_) : (_start_ + _idx_ - BUFFER_SZ))

//到len最大也只可能是BUFFER_SZ，不用考虑len > BUFFER_SZ的问题
static void console_ring_buf_put(unsigned char * data, int len)
{
	int current_tail = console_ring_buf_head + console_ring_buf_sz;
	if (current_tail >= BUFFER_SZ)
		current_tail -= BUFFER_SZ;
	for (int i = 0; i < len; i++) {
		console_ring_buf[IDX_OF_console_ring_buf(current_tail, i)] = data[i];
	}
	if ((BUFFER_SZ - console_ring_buf_sz) >= len) {			// 空间够就只增加sz
		console_ring_buf_sz += len;
	} else {	// 空间不够就得合适的摆head的位置，保持最新的BUFFER_SZ个字节的数据
		int tail = console_ring_buf_head + console_ring_buf_sz + len;
		while (tail >= BUFFER_SZ) {
			tail -= BUFFER_SZ;
		}
		console_ring_buf_head = tail;
		console_ring_buf_sz = BUFFER_SZ;
	}
}

int console_ring_buf_get(unsigned char *buf, unsigned short max_length)
{
	int len = MIN(console_ring_buf_sz, max_length);
	for (int i = 0; i < len; i++) {
		buf[i] = console_ring_buf[IDX_OF_console_ring_buf(console_ring_buf_head, i)];
	}
	console_ring_buf_head += len;
	if (console_ring_buf_head >= BUFFER_SZ)
		console_ring_buf_head -= BUFFER_SZ;
	console_ring_buf_sz -= len;
	return len;
}

void console_tx_proc()
{
	int target_idx = (~DMAed_idx) & 0x01;
	if (!DMACC1Config_bit.E && (tx_bufs[target_idx].data_len > 0)) {
		console_ring_buf_put(tx_bufs[target_idx].data, tx_bufs[target_idx].data_len); // 在这里把输出的数据搬到console_ring_buf里，因为这个方法不会在中断里面调用
		DMACC1SrcAddr = (unsigned int)(tx_bufs[target_idx].data);
		DMACC1Control_bit.TRANSFERSIZE = tx_bufs[target_idx].data_len;
		DMACC1Config_bit.E = 1;
		tx_bufs[DMAed_idx].data_len = 0;
		DMAed_idx = target_idx;
	}
}

// 0成功，<0失败
static int put_char(unsigned char ch)
{
	int tx_buf_idx = (~DMAed_idx) & 0x01;
	if (tx_bufs[tx_buf_idx].data_len >= BUFFER_SZ) {
		if (!DMACC1Config_bit.E && tx_bufs[DMAed_idx].data_len < BUFFER_SZ) {	// 上一笔DMA已经完成了，尝试切换tx_buf
			tx_bufs[DMAed_idx].data[tx_bufs[DMAed_idx].data_len++] = ch;
			return 0;
		}
		return -1;
	} else {
		tx_bufs[tx_buf_idx].data[tx_bufs[tx_buf_idx].data_len++] = ch;
		return 0;
	}
}


/*
void debug_p(const char *fmt, ...)
{

#ifdef Debug
	printk( fmt, ...)
#endif

}

*/



// 打印一次不能太长，这里不处理一次printk超过BUFFER_SZ的情况
void printk(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	while (*fmt) {
		if (*fmt != '%') {
			if (!put_char(*fmt++))
				continue;
			else
				goto out;
		}

		switch (*++fmt) {		// 跳过百分号
			case 'c': {
				unsigned int ch = va_arg(ap, unsigned int);
				if (put_char(ch))
					goto out;
				break;
			}

			case 's': {
				const char *s = va_arg(ap, const char *);
				for (; *s; s++) {
					if (put_char(*s))
						goto out;
				}
				break;
			}

			case 'd': {
				unsigned char buf[16];
				int d = va_arg(ap, int);
				int count = itoa(d, buf);
				for (int i = 0; i < count; i++) {
					if (put_char(buf[i]))
						goto out;
				}
				break;
			}

			case 'x':
			case 'X':
			case 'p': {
				unsigned char buf[16];
				unsigned int d = va_arg(ap, unsigned int);
				int count = utohex(d, buf);
				for (int i = 0; i < count; i++) {
					if (put_char(buf[i]))
						goto out;
				}
				break;
			}

			default:
				if (put_char(*fmt))
					goto out;
				break;
		}
		fmt++;				// 跳过百分号后面的那个符号
	}
	va_end(ap);

out:
	console_tx_proc();
}

