#include "config.h"
#include "backplane.h"

#define INA219_TIMEOUT_100uS	3000

#define ADDR_219_CONFIG			        0
#define ADDR_219_SHUNT_VOLTAGE	        1
#define ADDR_219_READY_VOLTAGE	        2
#define ADDR_219_CURRENT		        4
#define ADDR_219_CALIBRATION	        5
/***********************************
如果要检查状态然后读电流，和读电压然后读电流是一样的，因为状态位 在电压寄存器里面；
***********************************/

void ina219_init(ina219_t * chip219, unsigned char bus_id, unsigned char sla, 
					unsigned short regdata_config, unsigned char has_current, 
					unsigned short regdata_calibration)
{
	static /* const */ unsigned char addr_219_ready_voltage = ADDR_219_READY_VOLTAGE;
	static /* const */ unsigned char addr_219_current = ADDR_219_CURRENT;
	static /* const */ unsigned char addr_219_SHUNT = ADDR_219_SHUNT_VOLTAGE;

	memset(chip219, 0, sizeof(ina219_t));

	chip219->bus_id = bus_id;

	chip219->w_config_addr_data[0] = ADDR_219_CONFIG;
	chip219->w_config_addr_data[1] = (regdata_config & 0xFF00) >> 8;
	chip219->w_config_addr_data[2] = regdata_config & 0xFF;
	if (has_current) {
		chip219->w_calibration_addr_data[0] = ADDR_219_CALIBRATION;
		chip219->w_calibration_addr_data[1] = (regdata_calibration & 0xFF00) >> 8;
		chip219->w_calibration_addr_data[2] = regdata_calibration & 0xFF;
		
		chip219->req_trigger[0].op_mode = 0;
		chip219->req_trigger[0].sla_addr = sla;
		chip219->req_trigger[0].data = chip219->w_calibration_addr_data;
		chip219->req_trigger[0].data_len = 3;
		
		chip219->req_trigger[1].op_mode = 0;
		chip219->req_trigger[1].sla_addr = sla;
		chip219->req_trigger[1].data = chip219->w_config_addr_data;
		chip219->req_trigger[1].data_len = 3;
	} else {
		chip219->req_trigger[0].op_mode = 0;
		chip219->req_trigger[0].sla_addr = sla;
		chip219->req_trigger[0].data = chip219->w_config_addr_data;
		chip219->req_trigger[0].data_len = 3;
	}

	chip219->req_check_ready_and_read_voltage[0].op_mode = 0;
	chip219->req_check_ready_and_read_voltage[0].sla_addr = sla;
	chip219->req_check_ready_and_read_voltage[0].data = &addr_219_ready_voltage;
	chip219->req_check_ready_and_read_voltage[0].data_len = 1;
	
	chip219->req_check_ready_and_read_voltage[1].op_mode = 1;
	chip219->req_check_ready_and_read_voltage[1].sla_addr = sla;
	chip219->req_check_ready_and_read_voltage[1].data = (unsigned char *)&chip219->output_ready_voltage;
	chip219->req_check_ready_and_read_voltage[1].data_len = 2;

	chip219->req_read_current[0].op_mode = 0;
	chip219->req_read_current[0].sla_addr = sla;
	chip219->req_read_current[0].data = &addr_219_current;
	chip219->req_read_current[0].data_len = 1;
	
	chip219->req_read_current[1].op_mode = 1;
	chip219->req_read_current[1].sla_addr = sla;
	chip219->req_read_current[1].data = (unsigned char *)&chip219->output_current;
	chip219->req_read_current[1].data_len = 2;

	chip219->req_read_shunt[0].op_mode = 0;
	chip219->req_read_shunt[0].sla_addr = sla;
	chip219->req_read_shunt[0].data = &addr_219_SHUNT;
	chip219->req_read_shunt[0].data_len = 1;
	
	chip219->req_read_shunt[1].op_mode = 1;
	chip219->req_read_shunt[1].sla_addr = sla;
	chip219->req_read_shunt[1].data = (unsigned char *)&chip219->output_shunt;
	chip219->req_read_shunt[1].data_len = 2;	
}


void ina219_exec_trigger(ina219_t * chip219)
{
	chip219->output_ready_voltage = 0;
	chip219->output_current = 0;
	i2c_start_session(chip219->bus_id, chip219->req_trigger);
}

void ina219_exec_check_ready_and_read_voltage(ina219_t * chip219)
{
	chip219->output_ready_voltage = 0;
	i2c_start_session(chip219->bus_id, chip219->req_check_ready_and_read_voltage);
}

void ina219_exec_read_current(ina219_t * chip219)
{
	chip219->output_current = 0;
	i2c_start_session(chip219->bus_id, chip219->req_read_current);
}

void ina219_exec_read_shunt_volt(ina219_t * chip219)
{
	chip219->output_shunt = 0;
	i2c_start_session(chip219->bus_id, chip219->req_read_shunt);
}

int ina219_data_is_ready(ina219_t * chip219)
{
	return (chip219->output_ready_voltage & 0x200) ? 1 : 0;
}

unsigned short ina219_data_get_voltage(ina219_t * chip219)
{
	unsigned short v;
	v = (chip219->output_ready_voltage & 0xF800) >> 8;		// 丢弃低3位
	v |= (chip219->output_ready_voltage & 0xFF) << 8;
	return (v >> 1);		// >>1是1mV
}

static unsigned short _ina219_data_get_current(ina219_t * chip219)
{
	unsigned short v;
	v = (chip219->output_current & 0xFF00) >> 8;
	v |= (chip219->output_current & 0xFF) << 8;
	if (v & 0x8000) {		// 负值检测
		v = 0;
	}
	return v;
}


unsigned short ina219_data_get_current(ina219_t * chip219)
{
	if (chip219->output_ready_voltage & 0x100) {	// overflow，按理说电压也会overflow，但用硬件保证不会
		return 0xFFFF;
	} else {
		unsigned short v;
		unsigned short u;
		v = (chip219->output_current & 0xFF00) >> 8;
		v |= (chip219->output_current & 0xFF) << 8;
		if (v & 0x8000) {		// 负值检测
			v = 0;
		//	u = v & 0x7FFF;
		//	v = 0x8000 - u;
		//	v = -v;
		}
		return v;
	}
}


unsigned short ina219_data_get_shunt_volt(ina219_t * chip219)
{
	unsigned short u;
	short v;
	
	v = (chip219->output_shunt & 0xFF00) >> 8;
	v |= (chip219->output_shunt & 0xFF) << 8;
	if (v & 0x8000) {		// 负值检测
		u = v & 0x7FFF;
		v = 0x8000 - u;
		v = -v;
	//	v = 0;
	}
	return v;
}

// 每次i2c_session完成的时候调一次foo，不要在session中调用
void ina219_batch_read_voltage(NULLABLE poll_func_ptr foo, ina219_t * chip219, unsigned short * out_voltage)
{
	int rc;
	unsigned int trigger_start_tick;

retry_trigger:
	ina219_exec_trigger(chip219);
	while (!(rc = i2c_session_is_finished(chip219->bus_id))) /* nop */;
	if (rc < 0) {
		i2c_finish_session_forcibly(chip219->bus_id);
		printk("219 trigger failed i2c_bus=%d sla=%x\n", chip219->bus_id, chip219->req_trigger[0].sla_addr);
		goto retry_trigger;
	}
	trigger_start_tick = g_tick_100us;
	if (foo) (*foo)();

	while (1) {
		ina219_exec_check_ready_and_read_voltage(chip219);
		while (!(rc = i2c_session_is_finished(chip219->bus_id))) /* nop */;
		if (foo) (*foo)();

		if (rc < 0) {
			i2c_finish_session_forcibly(chip219->bus_id);
//			debug("219 check_ready failed i2c_bus=%d mux_idx=%d\n", chip219->bus_id, runtime.current_mux_idx);
//			continue;		// 不明原因不能直接重新检查ready，得从头开始避免老错
			goto retry_trigger;
		} else {
			if (ina219_data_is_ready(chip219)) {
				*out_voltage = ina219_data_get_voltage(chip219);
				return;
			} else {
				if (is_time_elapsed_100us(INA219_TIMEOUT_100uS, trigger_start_tick)) {
					i2c_finish_session_forcibly(chip219->bus_id);
					printk("219 why no data output after %d > %dms? i2c_bus=%d\n", g_tick_100us - trigger_start_tick, INA219_TIMEOUT_100uS / 10, chip219->bus_id); // g_data_runtime.hw_adc_mux_idx);
					goto retry_trigger;
				}
				continue;		// ad测量未完成，继续
			}
		}
	}

}

void ina219_batch_read_voltage_and_current(poll_func_ptr foo, ina219_t * chip219, unsigned short * out_voltage, unsigned short * out_current)
{
	int rc;
	unsigned short mV;

	ina219_batch_read_voltage(foo, chip219, &mV);

	while (1) {
		ina219_exec_read_current(chip219);
		while (!(rc = i2c_session_is_finished(chip219->bus_id))) /* nop */;
		(*foo)();
		if (rc < 0) {
			printk("219 read_current failed i2c_bus=%d\n", chip219->bus_id);
			continue;
		} else {
			*out_current = _ina219_data_get_current(chip219);		// 不处理溢出
			*out_voltage = mV;
			return;
		}
	}
}

void ina219_batch_read_bus_and_shunt_voltage(poll_func_ptr foo, ina219_t * chip219, NULLABLE unsigned short * out_bus_voltage, unsigned short * out_shunt_voltage)
{
	int rc;
	unsigned short mV;

	ina219_batch_read_voltage(foo, chip219, &mV);

	while (1) {
		ina219_exec_read_shunt_volt(chip219);
		while (!(rc = i2c_session_is_finished(chip219->bus_id))) /* nop */;
		(*foo)();
		if (rc < 0) {
			printk("219 read_shunt_voltage failed i2c_bus=%d\n", chip219->bus_id);
			continue;
		} else {
			*out_shunt_voltage = ina219_data_get_shunt_volt(chip219);		// 不处理溢出
			if (out_bus_voltage) *out_bus_voltage = mV;
			return;
		}
	}
}



/**************************************************** 
	 ADS122C04 用于采集PD电流
	 I2C从地址为:0x40~0x4F，24bit，4个单端或2对差分输入
	 single-shot和连续转换模式
	 可编程增益倍数/FSR为(1,2,4,8,16,32,64,128
	 寄存器为
	 00h：Configure Reg0: 7:4 MUX[3:0], 3:1:GAIN[2:0] 0:PGA_BYPASS
	 01h: Configure Reg1: 7:5 DR[2:0], 4:MODE， 3：CM， 2:1： VREF[1:0], 0:TS
	 02h: Configure Reg2: 7:DRDY, 6:DCNT, 5:4: CRC[1:0],3:BCS,2:0 IDAC[2:0]
	 03h: Configure Reg3: 7:5 I1MUX[2:0], 4:2:I2MUX[2:0], 1:0:RESERVED
	 操作：
	 上电复位 Power-up reset/复位命令06h，07h
	 转换模式：单个转换， 连续转换
	 操作模式：正常模式，Tubor模式，Powerdown模式
	 命令：
	 	RESET：		0000 011x
	 	START/SYNC:	0000 100x
	 	POWERDOWN:	0000 001x
	 	RDATA:		0001 xxxx
	 	RREG:		0010 rrxx  // rr为寄存器地址
	 	WREG:		0100 rrxx
	 操作流程为：上电复位（缺省设置）-->低功耗状态--->START/SYNC--->持续转换-->读DRDY-->读DATA0~3-->START/SYNC
****************************************************/
void ads122c_init(ads122c04_t *chip_ads122c, unsigned char bus_id, unsigned char sla, 
						unsigned char regdata00, unsigned char regdata01, unsigned char regdata02, unsigned char regdata03)
{
	unsigned char start_cmd = 0x08;
	unsigned char read_data = 0x10;
	unsigned char reset_cmd = 0x06;
	unsigned char powerdown_cmd = 0x02;
	unsigned char rreg_rdry = 0x28;
	unsigned char wreg_cmd  = 0x40;

	chip_ads122c->bus_id = bus_id;
	
	chip_ads122c->w_config_addr_data[0] = regdata00;
	chip_ads122c->w_config_addr_data[1] = regdata01;
	chip_ads122c->w_config_addr_data[2] = regdata02;
	chip_ads122c->w_config_addr_data[3] = regdata03;
	chip_ads122c->req_trigger[0].op_mode = 0;
	chip_ads122c->req_trigger[0].sla_addr = sla;
	chip_ads122c->req_trigger[0].data = &wreg_cmd;
	chip_ads122c->req_trigger[0].data_len = 1;
	chip_ads122c->req_trigger[1].op_mode = 0;
	chip_ads122c->req_trigger[1].sla_addr = sla;
	chip_ads122c->req_trigger[1].data = chip_ads122c->w_config_addr_data;
	chip_ads122c->req_trigger[1].data_len = 4;

	chip_ads122c->req_set_channel[0].op_mode = 0;
	chip_ads122c->req_set_channel[0].sla_addr = sla;
	chip_ads122c->req_set_channel[0].data = &wreg_cmd;
	chip_ads122c->req_set_channel[0].data_len = 1;
	chip_ads122c->req_set_channel[1].op_mode = 0;
	chip_ads122c->req_set_channel[1].sla_addr = sla;
	chip_ads122c->req_set_channel[1].data = chip_ads122c->w_config_addr_data;
	chip_ads122c->req_set_channel[1].data_len = 1;
	
	chip_ads122c->req_start.op_mode = 0;
	chip_ads122c->req_start.sla_addr = sla;
	chip_ads122c->req_start.data = &start_cmd;
	chip_ads122c->req_start.data_len = 1;

	chip_ads122c->req_powerdown.op_mode = 0;
	chip_ads122c->req_powerdown.sla_addr = sla;
	chip_ads122c->req_powerdown.data = &powerdown_cmd;
	chip_ads122c->req_powerdown.data_len = 1;
	
	chip_ads122c->req_reset.op_mode = 0;
	chip_ads122c->req_reset.sla_addr = sla;
	chip_ads122c->req_reset.data = &reset_cmd;
	chip_ads122c->req_reset.data_len = 1;

	chip_ads122c->req_check_ready[0].op_mode = 0;
	chip_ads122c->req_check_ready[0].sla_addr = sla;
	chip_ads122c->req_check_ready[0].data = &rreg_rdry;   // 读 RDRY
	chip_ads122c->req_check_ready[0].data_len = 1;
	chip_ads122c->req_check_ready[1].op_mode = 1;
	chip_ads122c->req_check_ready[1].sla_addr = sla;
	chip_ads122c->req_check_ready[1].data = &chip_ads122c->ready; // 
	chip_ads122c->req_check_ready[1].data_len = 1;
	
	chip_ads122c->req_read_voltage[0].op_mode = 0;
	chip_ads122c->req_read_voltage[0].sla_addr = sla;
	chip_ads122c->req_read_voltage[0].data = &read_data;   // rdata, 24bit
	chip_ads122c->req_read_voltage[0].data_len = 1;
	chip_ads122c->req_read_voltage[1].op_mode = 1;
	chip_ads122c->req_read_voltage[1].sla_addr = sla;
	chip_ads122c->req_read_voltage[1].data = chip_ads122c->output_data;   // rdata, 24bit
	chip_ads122c->req_read_voltage[1].data_len = 3;
	
}
#if 0
void ads122c_exec_trigger(ads122c04_t * chip_ads122c)
{
	i2c_start_session(chip_ads122c->bus_id, chip_ads122c->req_trigger);
}

void ads122c_exec_start(ads122c04_t * chip_ads122c)
{
	i2c_start_session(chip_ads122c->bus_id, &chip_ads122c->req_start);
}

void ads122c_exec_reset(ads122c04_t * chip_ads122c)
{
	i2c_start_session(chip_ads122c->bus_id, &chip_ads122c->req_reset);
}

void ads122c_exec_powerdown(ads122c04_t * chip_ads122c)
{
	i2c_start_session(chip_ads122c->bus_id, &chip_ads122c->req_powerdown);
}

void ads122c_set_mux(ads122c04_t * chip_ads122c, unsigned char channel_idx)
{
	unsigned char regdata0 = chip_ads122c->w_config_addr_data[0];
	regdata0 = (((channel_idx & 0x0F) + 8) << 4) | (regdata0 & 0x0F);
	chip_ads122c->w_config_addr_data[0] = regdata0;
	i2c_start_session(chip_ads122c->bus_id, chip_ads122c->req_set_channel);
}

void ads122c_exec_check_ready(ads122c04_t * chip_ads122c)
{
	i2c_start_session(chip_ads122c->bus_id, chip_ads122c->req_check_ready);
}

void ads122c_exec_rdata(ads122c04_t * chip_ads122c)
{
	chip_ads122c->ready = 0;
	memset(chip_ads122c->output_data, 0, 3);
	i2c_start_session(chip_ads122c->bus_id, chip_ads122c->req_read_voltage);
}

int ads122c_get_ready(ads122c04_t *chip_ads122c)
{
	return (chip_ads122c->ready & 0x80) ? 1 : 0;
}

int ads122c_get_voltage(ads122c04_t *chip_ads122c)
{
	int data = ((chip_ads122c->output_data[0] & 0x7F) << 16) | (chip_ads122c->output_data[1] << 8) | chip_ads122c->output_data[2];
	int signal = chip_ads122c->output_data[0] & 0x80; 
	if (chip_ads122c->output_data[0] & 0x80)  // 负数
		data = 0;
	
	return data;
}
#endif

void ads122c_exec_trigger(ads122c04_t * chip_ads122c)
{
//	i2c_start_session(chip_ads122c->bus_id, chip_ads122c->req_trigger);
//	unsigned char cmd = 0x40;
	unsigned char buf[5];

	buf[0] = 0x40;
	buf[1] = 0; //chip_ads122c->w_config_addr_data[0];
	buf[2] = 0x0; // chip_ads122c->w_config_addr_data[1];
	buf[3] = 0; // chip_ads122c->w_config_addr_data[2];
	buf[4] = 0; //chip_ads122c->w_config_addr_data[3];
		
	if (0 != i2c_write_n_bytes_blocking(0, 0x40, buf, 3)) printk("config ads122c04 error\n");
	else debug("config ads122c04 ok\n");

}

void ads122c_exec_start(ads122c04_t * chip_ads122c)
{
//	i2c_start_session(chip_ads122c->bus_id, &chip_ads122c->req_start);
	unsigned char cmd = 0x08;
	if (0 != i2c_write_n_bytes_blocking(0, 0x40, &cmd, 1)) printk("start ads122c04 error\n");
	else debug("start ads122c04 ok\n");
}

void ads122c_exec_reset(ads122c04_t * chip_ads122c)
{
//	i2c_start_session(chip_ads122c->bus_id, &chip_ads122c->req_reset);
	unsigned char cmd = 0x06;
	if (0 != i2c_write_n_bytes_blocking(0, 0x40, &cmd, 1)) printk("reset ads122c04 error\n");
	else debug("reset ads122c04 ok\n");
}

void ads122c_exec_powerdown(ads122c04_t * chip_ads122c)
{
//	i2c_start_session(chip_ads122c->bus_id, &chip_ads122c->req_powerdown);
	unsigned char cmd = 0x02;
	if (0 != i2c_write_n_bytes_blocking(0, 0x40, &cmd, 1)) printk("powerdown ads122c04 error\n");
	else debug("powerdown ads122c04 ok\n");
}

void ads122c_set_mux(ads122c04_t * chip_ads122c, unsigned char channel_idx)
{
	unsigned char regdata0 = chip_ads122c->w_config_addr_data[0];
	regdata0 = (((channel_idx & 0x03) + 8) << 4) | 0x0;
	chip_ads122c->w_config_addr_data[0] = regdata0;
	unsigned char buf[2] = { 0x40, regdata0};

	if (0 != i2c_write_n_bytes_blocking(0, 0x40, buf, 2)) printk("mux ads122c04 %d error\n", channel_idx);
	else debug("mux ads122c04 %d ok\n", channel_idx);
//	i2c_start_session(chip_ads122c->bus_id, chip_ads122c->req_set_channel);
}

int ads122c_exec_check_ready(ads122c04_t * chip_ads122c)
{
//	i2c_start_session(chip_ads122c->bus_id, chip_ads122c->req_check_ready);
//	unsigned char cmd = 0x28;
	unsigned char rdry = 0;
	if (0 != i2c_read_n_bytes_with_addr_blocking(0, 0x40, 0x28, &rdry, 1)) {
		printk("get ads122c04 rdry error\n");
		return -1;
	} else {
	//	printk("get ads122c04 rdry=%d ok\n", rdry);
		chip_ads122c->ready = rdry;
		return 0;
	}
}

void ads122c_exec_rdata(ads122c04_t * chip_ads122c)
{
/*
	chip_ads122c->ready = 0;
	memset(chip_ads122c->output_data, 0, 3);
	i2c_start_session(chip_ads122c->bus_id, chip_ads122c->req_read_voltage);
*/
//	unsigned char cmd = 0x10;
	unsigned char buf[3];
	if (0 != i2c_read_n_bytes_with_addr_blocking(0, 0x40, 0x10, buf, 3)) printk("rdata ads122c04  error\n");
	else debug("rdata ads122c04 %x %x %x ok\n", buf[0], buf[1], buf[2]);
	memcpy(chip_ads122c->output_data, buf, 3);	
}

int ads122c_get_ready(ads122c04_t *chip_ads122c)
{
	debug("rd_reg=%x\n", chip_ads122c->ready);
	return (chip_ads122c->ready & 0x80) ? 1 : 0;
}

int ads122c_get_voltage(ads122c04_t *chip_ads122c)
{
	int data = ((chip_ads122c->output_data[0] & 0x7F) << 16) | (chip_ads122c->output_data[1] << 8) | chip_ads122c->output_data[2];
	int signal = chip_ads122c->output_data[0] & 0x80; 
	if (chip_ads122c->output_data[0] & 0x80)  // 负数
		data = 0;
	debug("data=%d / %x \n", data, data);
	return data;
}




