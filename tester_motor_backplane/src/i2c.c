#include "config.h"
#include "backplane.h"


#define PHASE_MAX		64

#define I2C_BASE0 0x4001C000
#define I2C_BASE1 0x4005C000
#define I2C_BASE2 0x400A0000
static const unsigned int I2C_CONSET[NR_I2C_BUSES] = { I2C_BASE0, I2C_BASE1, I2C_BASE2 };
static const unsigned int I2C_STAT[NR_I2C_BUSES] = { I2C_BASE0 + 4, I2C_BASE1 + 4, I2C_BASE2 + 4 };
static const unsigned int I2C_DAT[NR_I2C_BUSES] = { I2C_BASE0 + 8, I2C_BASE1 + 8, I2C_BASE2 + 8 };
static const unsigned int I2C_CONCLR[NR_I2C_BUSES] = { I2C_BASE0 + 0x18, I2C_BASE1 + 0x18, I2C_BASE2 + 0x18 };

#define I2C_SESSION_STATE_IDLE		0
#define I2C_SESSION_STATE_BUSY		1
#define I2C_SESSION_STATE_SUCEEDED	2
#define I2C_SESSION_STATE_FAILED	3


typedef struct {
	// 用户配置的数据
	i2c_session_request_t *req_array;

	// 状态与统计
	unsigned char i2c_hw_status;
	volatile unsigned char session_state;			// I2C_SESSION_STATE_XXXX 状态
	unsigned int stats_session_error;
	unsigned int stats_session_ok;
	unsigned int stats_session_timeout;

	// 内部使用的数据
	int _data_idx_in_each_req;
	int _req_idx;
	int _nr_reqs;
	int _timeout_cnt_in_100us;
	unsigned int _tick_start;
	unsigned int _tick_succeeded;		// 成功结束的时候标记tick，用于处理延时

	// 特殊标记3933，有时候3933不应答需要重发几次START
	int _START_retry_count;

	// 支持延时（op=3），注意每个阶段的操作的i2c timeout时间是独立计算的
	i2c_session_request_t *phase_req_array[PHASE_MAX];	// 最多支持PHASE_MAX段，中间可以延时
	unsigned int phase_delay_100uS[PHASE_MAX];			// 注1:第一个和最后一个操作不能是延时 注2:个数为phase_count-1
	int phase_current_idx;
	int phase_count;									// 至少为1
} i2c_session_t;

static i2c_session_t i2c_session_array[NR_I2C_BUSES];

void i2c_print_stats() {
	for (int i = 0; i < NR_I2C_BUSES; i++) {
		i2c_session_t *d = i2c_session_array + i;
		printk("I[%d]err%d ok%d timeout%d\n", i, d->stats_session_error, d->stats_session_ok, d->stats_session_timeout);
	}
}

void i2c_init()
{
	memset(i2c_session_array, 0, sizeof(i2c_session_array));

	PCONP_bit.PCI2C0 = 1;
	I2C0SCLH = (_F_PCLK / CONFIG_I2C0_FREQUENCY) / 2;
	I2C0SCLL = (_F_PCLK / CONFIG_I2C0_FREQUENCY) / 2;
	I2C0CONCLR = 0x6C;

	PCONP_bit.PCI2C1 = 1;
	I2C1SCLH = (_F_PCLK / CONFIG_I2C1_FREQUENCY) / 2;
	I2C1SCLL = (_F_PCLK / CONFIG_I2C1_FREQUENCY) / 2;
	I2C1CONCLR = 0x6C;

	PCONP_bit.PCI2C2 = 1;
	I2C2SCLH = (_F_PCLK / CONFIG_I2C2_FREQUENCY) / 2;
	I2C2SCLL = (_F_PCLK / CONFIG_I2C2_FREQUENCY) / 2;
	I2C2CONCLR = 0x6C;


	extern void I2C0_IRQHandler(void);
	I2C0CONSET_bit.I2EN = 1;
	vicIrqFuncSet(NVIC_I2C0, 20, (unsigned int)I2C0_IRQHandler);	// 电源模块的优先级较高，均低于时钟中断
	vicIrqEnable(NVIC_I2C0);

	extern void I2C1_IRQHandler(void);
	I2C1CONSET_bit.I2EN = 1;
	vicIrqFuncSet(NVIC_I2C1, 19, (unsigned int)I2C1_IRQHandler);
	vicIrqEnable(NVIC_I2C1);

	extern void I2C2_IRQHandler(void);
	I2C2CONSET_bit.I2EN = 1;
	vicIrqFuncSet(NVIC_I2C2, 18, (unsigned int)I2C2_IRQHandler);
	vicIrqEnable(NVIC_I2C2);
}

// 内部使用，不处理phase逻辑，一笔交给中断一次做完
static void __i2c_start_session(unsigned char bus_id, i2c_session_request_t * req_array)
{
	i2c_session_t *d = i2c_session_array + bus_id;

	d->_nr_reqs = 0;
	while (req_array[d->_nr_reqs].data)
		d->_nr_reqs++;

	if (d->session_state != I2C_SESSION_STATE_IDLE) {
		printk("[%d]start i2c while busy count=%d %d %x %x\n", bus_id, d->_nr_reqs, req_array[0].op_mode, req_array[0].sla_addr, req_array[0].data[0]);
		return;
	}

	d->req_array = req_array;

	d->_data_idx_in_each_req = 0;
	d->_req_idx = 0;

	// 每一笔操作最多35ms，按这个时间设置超时保护
	d->_tick_start = g_tick_100us;
	d->_timeout_cnt_in_100us = 350 * (d->_nr_reqs + 1);

	d->session_state = I2C_SESSION_STATE_BUSY;
	IOWRITE8(I2C_CONSET[bus_id], 0x60); // set I2EN | START，session出现不可恢复的错误时总线会被关掉
}

// 用户应该保证在总线空闲的时候才启动新的i2c_session
void i2c_start_session(unsigned char bus_id, i2c_session_request_t * req_array)
{
	i2c_session_t *d = i2c_session_array + bus_id;
	int req_idx = 0;

	d->phase_req_array[0] = req_array;
	d->phase_delay_100uS[0] = 0;
	d->phase_current_idx = 0;
	d->phase_count = 1;
	while (req_array[req_idx].data || (req_array[req_idx].op_mode == 3)) {
		if (req_array[req_idx].op_mode == 3) {
			d->phase_req_array[d->phase_count] = req_array + (req_idx + 1);		// 约定延时不能是最后一笔操作
			d->phase_delay_100uS[d->phase_count - 1] = (req_array[req_idx].sla_addr << 8) | req_array[req_idx].data_len;	// 延时高字节再sla_addr，低字节在data_len，单位是100uS，0~6.5S
			d->phase_count++;
			if (req_array[req_idx].data) {	// 约定：延时data字段必须为0
				printk("why data!=NULl! %s %d\n", __FUNCTION__, __LINE__);
				req_array[req_idx].data = 0;	// 有点脏，本不该修改传进来的req_array。
			}
		}
		req_idx++;
	}
	__i2c_start_session(bus_id, d->phase_req_array[0]);
}

// -1 完成但错误，或超时，0 未完成，1 完成且正确
// AA:0x04 STOP:0x10 START:0x20
int i2c_session_is_finished(unsigned char bus_id)
{
	i2c_session_t *d = i2c_session_array + bus_id;
//	printk("[%d]state=%d\n", bus_id, d->session_state);

	switch (d->session_state) {
		case I2C_SESSION_STATE_IDLE:
			return 1;

		case I2C_SESSION_STATE_SUCEEDED: {
			if (IOREAD8(I2C_CONSET[bus_id]) & 0x10) {		// 必须等最后一笔STOP发送完成才能认为session完成了，否则返回busy
				return 0;
			}
			if (d->phase_current_idx >= d->phase_count - 1) {	// 没有多余的phase了，处理session结束逻辑
				d->session_state = I2C_SESSION_STATE_IDLE;
				return 1;
			} else {	// 还有phase没做完，延时之后继续做
				if (is_time_elapsed_100us(d->phase_delay_100uS[d->phase_current_idx], d->_tick_succeeded)) {
					// 这一阶段已经完了，开始下一阶段
					d->phase_current_idx++;
					d->session_state = I2C_SESSION_STATE_IDLE;
					__i2c_start_session(bus_id, d->phase_req_array[d->phase_current_idx]);
					return 0;
				} else {
					return 0;		// 延时中，让上面等
				}
			}
		}

		case I2C_SESSION_STATE_FAILED: {	// 这时候总线已经被停掉了，下一次start_session才会打开
#ifdef Debug
			unsigned char da_start = d->req_array[d->_req_idx].data[0];
			unsigned char da_end = d->req_array[d->_req_idx].data[d->req_array[d->_req_idx].data_len - 1];
			debug("I[%d]F chip%x req%d/%d req0=%x/%d rw%d sla%x data%x~%x|%d\n", bus_id, d->i2c_hw_status, d->_req_idx, d->_nr_reqs, d->req_array[0].data[0], d->req_array[0].data_len, d->req_array[d->_req_idx].op_mode, d->req_array[d->_req_idx].sla_addr, da_start, da_end, d->req_array[d->_req_idx].data_len);
#endif
			d->session_state = I2C_SESSION_STATE_IDLE;
			return -1;
		}

		case I2C_SESSION_STATE_BUSY:
			if (is_time_elapsed_100us(d->_timeout_cnt_in_100us, d->_tick_start)) {
#ifdef Debug
				unsigned char da_start = d->req_array[d->_req_idx].data[0];
				unsigned char da_end = d->req_array[d->_req_idx].data[d->req_array[d->_req_idx].data_len - 1];
				debug("I[%d]T chip%x req%d/%d req0=%x/%d rw%d sla%x data%x~%x|%d\n", bus_id, d->i2c_hw_status, d->_req_idx, d->_nr_reqs, d->req_array[0].data[0], d->req_array[0].data_len, d->req_array[d->_req_idx].op_mode, d->req_array[d->_req_idx].sla_addr, da_start, da_end, d->req_array[d->_req_idx].data_len);
#endif
				IOWRITE8(I2C_CONCLR[bus_id], 0x6C);			// 把总线停掉
				d->session_state = I2C_SESSION_STATE_IDLE;
				d->stats_session_timeout++;
				return -1;
			}
			return 0;

		default:
			printk("[%d]never be here state=%d\n", d->session_state);
			d->session_state = I2C_SESSION_STATE_IDLE;
			return -1;
	}
}

void i2c_finish_session_forcibly(unsigned char bus_id)
{
	IOWRITE8(I2C_CONCLR[bus_id], 0x6C);
	i2c_session_array[bus_id].session_state = I2C_SESSION_STATE_IDLE;
}

// 碰到错误，就把i2c停掉
void i2c_session_isr(const int bus_id) {
	i2c_session_t *d = i2c_session_array + bus_id;
	i2c_session_request_t *req = d->req_array + MIN(d->_req_idx, d->_nr_reqs - 1);
	d->i2c_hw_status = IOREAD8(I2C_STAT[bus_id]) & 0xF8; // 读I2C状态
    
	switch (d->i2c_hw_status) {
		case 0xF8:	// unknown error
			IOWRITE8(I2C_CONCLR[bus_id], 0x08);
			return;
		
		case 0x00:	// bus error
		case 0x30:	// MASTER_W data nack
		    printk("[%d]resend %x %d\n", bus_id, d->i2c_hw_status, d->_START_retry_count);
			goto transaction_failed;
		
		case 0x20:	// sla+w nack，对于3933要特殊处理，再发一次START
		case 0x48:	// sla+r nack
			if (d->_START_retry_count) {							// need to resend START
				IOWRITE8(I2C_CONSET[bus_id], 0x30); 				// STOP | START
				IOWRITE8(I2C_CONCLR[bus_id], 0x0C);
				printk("[%d]resend %x %d\n", bus_id, d->i2c_hw_status, d->_START_retry_count);
				d->_data_idx_in_each_req = 0;
				d->_START_retry_count--;
				return;
			}
			printk("[%d]resend %x %d\n", bus_id, d->i2c_hw_status, d->_START_retry_count);
			goto transaction_failed;

		case 0x08:		// start已发送, 发送sla+rw
			//printk("[%d]%d/%d r%d s%x d%x~%x|%d\n", bus_id, d->_req_idx, d->_nr_reqs, req->op_mode, req->sla_addr, req->data[0], req->data[req->data_len - 1], req->data_len);
            //IODELAY(50);
			IOWRITE8(I2C_DAT[bus_id], (req->sla_addr << 1) | ((req->op_mode==1) ? 1 : 0));	// opmode=2第一笔是写
			IOWRITE8(I2C_CONCLR[bus_id], 0x2C);					// clear START here
			return;
			
		case 0x10:		// repeat_start已发送，发送sla+r，现在只有在读3933的时候会到这里
			//printk("[%d]i%d %d/%d r%d s%x d%x~%x|%d\n", bus_id, d->_data_idx_in_each_req, d->_req_idx, d->_nr_reqs, req->op_mode, req->sla_addr, req->data[0], req->data[req->data_len - 1], req->data_len);
            //IODELAY(50);
			IOWRITE8(I2C_DAT[bus_id], (req->sla_addr << 1) | 1);
			IOWRITE8(I2C_CONCLR[bus_id], 0x2C);					// clear START here
			return;

		case 0x18:		// MASTER_W，发第一个字节
			IOWRITE8(I2C_DAT[bus_id], req->data[d->_data_idx_in_each_req++]);
			IOWRITE8(I2C_CONCLR[bus_id], 0x2C);
			return;

		case 0x28:		// MASTER_W，上一个字节发送成功
			if (req->op_mode != 2) {
				if (d->_data_idx_in_each_req >= req->data_len) {	// 上一个字节是最后一个字节，说明一个req完成了
					// 注意！！！这个和datasheet不一样，datasheet说发STOP|AA，但外设芯片资料都只要STOP
					IOWRITE8(I2C_CONSET[bus_id], 0x10);				// STOP
					IOWRITE8(I2C_CONCLR[bus_id], 0x28);
					goto transaction_succeeded;
				} else {											// 还有数据
					IOWRITE8(I2C_DAT[bus_id], req->data[d->_data_idx_in_each_req++]);
					IOWRITE8(I2C_CONCLR[bus_id], 0x2C);
					return;
				}
			} else {	// 对于op_mode=2，write完第一字节后，要ACK+repeat_start
				IOWRITE8(I2C_CONSET[bus_id], 0x20); 			// RESTART
				IOWRITE8(I2C_CONCLR[bus_id], 0x0C);
				return;
			}

		case 0x38:		// 错误，MASTER_W，发送时总线丢失进入了slave模式，等总线空闲的时候重新发一个START，切回master
			goto transaction_failed;

		case 0x40:		// sla+r发送成功
			if (req->data_len > 1) {		// 如果只有一个字节要读，就不ack - 不进0x50，直接进0x58
				IOWRITE8(I2C_CONSET[bus_id], 0x04);
			}
			IOWRITE8(I2C_CONCLR[bus_id], 0x08);
			return;

		case 0x50:		// SLAVE_R，收到1字节数据
			if (req->op_mode != 2) {
				if (d->_data_idx_in_each_req < req->data_len) req->data[d->_data_idx_in_each_req++] = IOREAD8(I2C_DAT[bus_id]);
				if (d->_data_idx_in_each_req >= (req->data_len - 1)) {	// 已经收完了，NACK(不回ACK)
					IOWRITE8(I2C_CONCLR[bus_id], 0x0C);				// clear AA | SI
				} else {											// 回ACK继续收下一个字节
					IOWRITE8(I2C_CONSET[bus_id], 0x04);
					IOWRITE8(I2C_CONCLR[bus_id], 0x08);
				}
				return;
			} else {
				if (d->_data_idx_in_each_req < req->data_len) req->data[d->_data_idx_in_each_req++] = IOREAD8(I2C_DAT[bus_id]);
				IOWRITE8(I2C_CONSET[bus_id], 0x10);					// STOP
				IOWRITE8(I2C_CONCLR[bus_id], 0x2C);
				goto transaction_succeeded;
			}

		case 0x58:		// SLAVE_R，最后一字节收到以后，NACK已经应答了，这表示这个req完成了
			req->data[d->_data_idx_in_each_req] = IOREAD8(I2C_DAT[bus_id]);
			IOWRITE8(I2C_CONSET[bus_id], 0x10); 				// STOP
			IOWRITE8(I2C_CONCLR[bus_id], 0x2C);
			goto transaction_succeeded;

		default:		// 其余状态是slave r/w，在这里不处理，尝试把总线切回master模式，应答SI
			printk("[%d]B c%x %d/%d r%d a%x d%x|%d\n", bus_id, d->i2c_hw_status, d->_req_idx, d->_nr_reqs, d->req_array[d->_req_idx].op_mode, d->req_array[d->_req_idx].sla_addr, d->req_array[d->_req_idx].data[0], d->req_array[d->_req_idx].data_len);
			goto transaction_failed;
	}

transaction_failed:
	IOWRITE8(I2C_CONCLR[bus_id], 0x6C);
	d->stats_session_error++;
	d->session_state = I2C_SESSION_STATE_FAILED;
	return;

transaction_succeeded:
	d->_req_idx++;
	if (d->_req_idx < d->_nr_reqs) {				// 还有下一个req
		d->_data_idx_in_each_req = 0;
		IOWRITE8(I2C_CONSET[bus_id], 0x20); 		// START，开始下一个req
	} else {										// 已经是最后一个req
		d->stats_session_ok++;
		d->session_state = I2C_SESSION_STATE_SUCEEDED;
		d->_tick_succeeded = g_tick_100us;
	}
	return;

}

//
// 下面blocking系列方法，都是返回-1表示失败，返回0表示成功
//
int i2c_do_session_blocking(unsigned char bus_id, i2c_session_request_t * req_array)
{
	i2c_start_session(bus_id, req_array);
	int rc;
	while ((rc = i2c_session_is_finished(bus_id)) == 0) /* nop */;
	return (rc == 1) ? 0 : rc;
}

// 输入bytes的第一个字节必须是addr，后面的是读取的输出buffer，因此输出参数len=期望读取的字节数+1
int i2c_write_1_byte_repeat_start_read_n_bytes_blocking(unsigned char bus_id, unsigned char sla, volatile unsigned char * buf_start_with_addr, int len)
{
	i2c_session_request_t req[] = {
		{ 2, sla, buf_start_with_addr, len },
		I2C_SESSION_REQUEST_END
	};
	return i2c_do_session_blocking(bus_id, req);
}

int i2c_read_n_bytes_blocking(unsigned char bus_id, unsigned char sla, volatile unsigned char * buf, int len)
{
	i2c_session_request_t req[] = {
		{ 1, sla, buf, len },
		I2C_SESSION_REQUEST_END
	};
	return i2c_do_session_blocking(bus_id, req);
}

int i2c_write_n_bytes_blocking(unsigned char bus_id, unsigned char sla, volatile unsigned char * buf, int len)
{
	i2c_session_request_t req[] = {
		{ 0, sla, buf, len },
		I2C_SESSION_REQUEST_END
	};
	return i2c_do_session_blocking(bus_id, req);
}

int i2c_read_n_bytes_with_addr_blocking(unsigned char bus_id, unsigned char sla, unsigned char addr, volatile unsigned char * buf, int len)
{
	unsigned char da[] = { addr };
	i2c_session_request_t req[] = {
		{ 0, sla, da, sizeof(da) },
		{ 1, sla, buf, len },
		I2C_SESSION_REQUEST_END
	};
	return i2c_do_session_blocking(bus_id, req);
}

// mux_channel_id可以是0~7表示选中某一路，也可以是8表示一个都不选
int i2c_set_PCA9548_blocking(unsigned char bus_id, unsigned char sla, unsigned char mux_channel_id)
{
	unsigned char da[] = { 1 << mux_channel_id };
	return i2c_write_n_bytes_blocking(bus_id, sla, da, 1);
}

// 设3933的时候把I2C频率改了
// 设置3933容易失败，调用者应该考虑在3933设置失败之后进行总线恢复，避免后续的i2c操作也失败
int i2c_set_NCT3933U_blocking(unsigned char bus_id, unsigned char sla, unsigned char addr, unsigned char data)
{
	static const unsigned int I2C_FREQ[NR_I2C_BUSES] = { CONFIG_I2C0_FREQUENCY, CONFIG_I2C1_FREQUENCY, CONFIG_I2C2_FREQUENCY };
	static const unsigned int I2C_SCLH[NR_I2C_BUSES] = { 0x4001C010, 0x4005C010, 0x400A0010 }; //   I2C FREQ Register  I2SCLH
	static const unsigned int I2C_SCLL[NR_I2C_BUSES] = { 0x4001C014, 0x4005C014, 0x400A0014 }; //   I2C FREQ Register  I2SCLL
	unsigned char da[] = { addr, data };
	int rc;

	// 连续错的话，就把频率降一半再试，降到<=20MHz再涨回来
	// 测试中频率变化一下能明显改善3933的写入错的问题
//	static unsigned int continues_error_count[NR_I2C_BUSES];
//	static unsigned int current_i2c_freq[NR_I2C_BUSES] = { CONFIG_I2C0_FREQUENCY, CONFIG_I2C1_FREQUENCY, CONFIG_I2C2_FREQUENCY };
//	if (continues_error_count[bus_id] > 16) {    //  当操作I2C连续出错超过16次时 
//		current_i2c_freq[bus_id] >>= 1;          //  当前频率除2 
//		if (current_i2c_freq[bus_id] <= (20 << 10)) current_i2c_freq[bus_id] = I2C_FREQ[bus_id];  
//		continues_error_count[bus_id] = 0;
//		printk("[%d]3933 freq switched to 1/%d\n", bus_id, I2C_FREQ[bus_id] / current_i2c_freq[bus_id]);
//	}

//	if (current_i2c_freq[bus_id] != I2C_FREQ[bus_id]) {// 设置当前操作频率 
//		IOWRITE16(I2C_SCLH[bus_id], _F_PCLK / current_i2c_freq[bus_id] / 2);
//		IOWRITE16(I2C_SCLL[bus_id], _F_PCLK / current_i2c_freq[bus_id] / 2);
//	}

	i2c_session_array[bus_id]._START_retry_count = CONFIG_3933_RESEND_START_COUNT;
	rc = i2c_write_n_bytes_blocking(bus_id, sla, da, 2);
	i2c_session_array[bus_id]._START_retry_count = 0;

//	if (current_i2c_freq[bus_id] != I2C_FREQ[bus_id]) {
//		IOWRITE16(I2C_SCLH[bus_id], _F_PCLK / I2C_FREQ[bus_id] / 2);
//		IOWRITE16(I2C_SCLL[bus_id], _F_PCLK / I2C_FREQ[bus_id] / 2);
//	}

//	if (rc == 0) {
//		continues_error_count[bus_id] = 0;
//	} else {
//		continues_error_count[bus_id]++;
//	}

	return rc;
}

int i2c_read_AT24C16A_blocking(unsigned char bus_id, unsigned char sla, unsigned char *p_data_208b)
{
	// 只用eeprom的第一个page，13个word，每个word 16byte，blocking
	unsigned char da[] = { 0 };
	i2c_session_request_t req[] = {
		{ 0, sla, da, sizeof(da) },		// 先送word addr
		{ 1, sla, p_data_208b, 208 },	// 不超过256，可以一次读完
		I2C_SESSION_REQUEST_END
	};
	return i2c_do_session_blocking(bus_id, req);
}

// 为了兼容8字节和16字节的两种不同尺寸的word_size，按8字节来写入，逻辑上word个数为26
int i2c_write_AT24C16A_blocking(unsigned char bus_id, unsigned char sla, unsigned char *p_data_208b)
{
#define WORD_SIZE	8
	// 只用eeprom的第一个page，13个word，每个word 16byte，blocking
	unsigned char da[WORD_SIZE + 1];
	i2c_session_request_t req[] = {
		{ 0, sla, da, sizeof(da) },
		I2C_SESSION_REQUEST_END
	};

	int rc;
	for (unsigned char word_addr = 0; word_addr < ((208 + WORD_SIZE - 1) / WORD_SIZE); word_addr++) {
		da[0] = word_addr * WORD_SIZE;
		memcpy(da + 1, p_data_208b + (word_addr * WORD_SIZE), WORD_SIZE);
		// Twr=5ms，手册规范，连续写期间，从停止P到内部写完成周期（或下一个开始）最大是5ms，实际测试至少需要1.5ms
		// 测试中QSFP+的老tray板上的eeprom，延时要超过10ms才行
		IODELAY(50000);
		i2c_start_session(bus_id, req);

		while ((rc = i2c_session_is_finished(bus_id)) == 0) /* nop */;
		if (rc < 0) {	// 失败
			return -1;
		} else {
			continue;	// 成功
		}
	}
	return 0;
#undef WORD_SIZE
}

