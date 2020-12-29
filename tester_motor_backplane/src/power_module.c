#include "config.h"
#include "backplane.h"


// VCC_adj/RS+通过LM2596模块变换出来，输出RS+，可通过U46（INA219,0x4A,位于I2C1）读取
// VCC_adj/RS+输出通过3933调整，3933位于I2C1， 地址为0x2A

#define SLA_3933		0x15
#define	SLA_OUTPUT_219	0x4A

#define ADDR_3933				1	// 电源模块上的3933使用CR01


/******************************************
Configuration Register 00h (Read/Write)

Bit 15 RST: Reset Bit
Setting this bit to '1' generates a system reset that is the same as power-on reset. Resets all registers to default values; this bit self-clears.

Bit 13 BRNG: Bus Voltage Range
0 = 16V FSR 
1 = 32V FSR (default value)

Bits 11, 12 PG: PGA (Shunt Voltage Only)
Sets PGA gain and range. Note that the PGA defaults to ÷8 (320mV range). 
00 ： ±40mV
01 ： ±80mV
10 ： ±160mV
11 ： ±320mV

Bits 10–7 BADC4-1: Bus ADC Resolution/Averaging

Bits 6–3 ADC4-1 Shunt ADC Resolution/Averaging
ADC4-1 MODE/SAMPLES CONVERSION TIME
0 X 0 0 	9-bit 84μs
0 X 0 1 	10-bit 148μs
0 X 1 0 	11-bit 276μs
0 X 1 1 	12-bit 532μs
1 0 0 0 	12-bit 532μs
1 0 0 1 	2 1.06ms
1 0 1 0 	4 2.13ms
1 0 1 1 	8 4.26ms
1 1 0 0 	16 8.51ms
1 1 0 1 	32 17.02ms
1 1 1 0 	64 34.05ms
1 1 1 1 	128 68.10ms

Bits 2–0  MODE3-1 ：Operating Mode
0 0 0 Power-Down
0 0 1 Shunt Voltage, Triggered
0 1 0 Bus Voltage, Triggered
0 1 1 Shunt and Bus, Triggered
1 0 0 ADC Off (disabled)
1 0 1 Shunt Voltage, Continuous
1 1 0 Bus Voltage, Continuous
1 1 1 Shunt and Bus, Continuous

******************************************/
// 4x 采样平均，连续模式；
#define OUTPUT_219_CONFIG		((0 << 13) | (3 << 11) | (9 << 7) | (0x3 << 3) | 3)
#define OUTPUT_219_CALIBRATION	0xaaa

#define SLA_219_CONFIG2		((0 << 13) | (3 << 11) | (9 << 7) | (0x3 << 3) | 1)
//#define SLA_219_CONFIG2		((0 << 13) | (3 << 11) | (1 << 7) | (0x1 << 3) | 1)
#define SLA_219_CALIBRATION2	0xaaa // 1.5ohm 10 uA/bit

#define INA219_CONFIG_IV 1

#define PM_OUT_MV_MAX	4020
#define PM_OUT_MV_MIN	1220


/**************************************
外部部调用：
设定输出电压值；设定小于最小值则关断模块；
0xFF : 1921mV
0x00: 2629mV
0x7F: 4020mV
**************************************/
void power_module_set_mV(unsigned char power_module_idx, unsigned int mV)
{
	power_module_info_t *d = g_data_power_modules + power_module_idx;

	if (mV < PM_OUT_MV_MIN) {
		//gpio_power_module_onoff(power_module_idx, 0);
		gpio_ld_power_onoff(0);
		d->is_on = 0;
		d->dac_output = 0;
		d->target_output_mV = 0;
		d->power_good =0;
		d->delay_tick = 3; // 延迟 300mS 为了保证流程启动时，DUT OE 要先有效，然后才有VCC_adj

	} else {
		if(mV > PM_OUT_MV_MAX) 	mV = PM_OUT_MV_MAX;

		d->target_output_mV = mV;
		d->is_on = 1;

		unsigned int _delta = mV - PM_OUT_MV_MIN;
		d->dac_output = (unsigned char)((_delta *255)/(PM_OUT_MV_MAX - PM_OUT_MV_MIN));
	}

}

#define PM_FINE_TUNE_INTERVAL   100
/**************************************
内部调用：
微调输出电压到设定值；

**************************************/
int power_module_proc_review_constant_voltage(unsigned char power_module_idx)
{
	static unsigned int last_tick;
	static unsigned char retry = 5;
	unsigned char value_3933[6];

	power_module_info_t *d = g_data_power_modules + power_module_idx;

	if (d->is_on) {
		if (!is_time_elapsed_100us(1000, last_tick)) // 调整间隔大于 100mS
			return 1;
		
		last_tick = g_tick_100us;
		if (d->delay_tick > 0) {
			d->delay_tick--;
			if(0 == d->delay_tick){
				gpio_ld_power_onoff(1);
			}
			return 1;
		}

		int tune_up_threshold = d->target_output_mV;
		int tune_down_threshold = d->target_output_mV + 30;
		unsigned char _step = 0;

		if (d->output_mV < tune_up_threshold){
			if (d->dac_output < 254){
				_step = d->dac_output + 1;
				debug("P[%d]voltage review++ ->%d %d/%d\n", power_module_idx, d->dac_output,  d->output_mV, d->target_output_mV);
			}
		} 
		else if (d->output_mV > tune_down_threshold){
			if (d->dac_output > 1){
				_step = d->dac_output - 1;
				debug("P[%d]voltage review-- ->%d %d/%d\n", power_module_idx, d->dac_output,  d->output_mV, d->target_output_mV);
			}
		} else {
			d->power_good =1;
			retry = 5;
			return 0;
		}
	//	_step += d->dac_output;
		unsigned char value = (_step >= 128) ? (_step - 128) : (255 - _step); // range is 00h~FFh
	//	d->dac_output = _step;
		debug("dac_output = %d, %d\n", value, _step);
		// 上一笔完成之后设3933，这时候可以安全设置，总线和i2c_mux不会打架
		// 注意方便人类理解的-127~127的power_value与写入3933的值定义不一样，需要换算
		if (!i2c_set_NCT3933U_blocking(1, SLA_3933, ADDR_3933, value)) {
		//	printk("set nct3933 fail.\n");
			d->dac_output = _step;
		}
#if 0

		unsigned int _retry = 10;
		while (_retry) {
			_retry--;
			if (!i2c_set_NCT3933U_blocking(1, SLA_3933, ADDR_3933, value)) 
				continue;
			//	printk("set nct3933 fail.\n");
			d->dac_output = _step;
			
		}
		debug("mV=%d /%d\n", d->output_mV, d->target_output_mV);
		if (_retry == 0) printk("set nct3933 fail.\n");

		value_3933[0] = 0x01; // ADDR_3933;
		if (0 != i2c_write_1_byte_repeat_start_read_n_bytes_blocking(1, SLA_3933, value_3933, 2)) {
			printk("read 3933 error!\n");
		} else {
			printk("read 3933 value=%x %x\n", value_3933[1], value_3933[2]);
		}
#endif
		return 1;	
	}else{
		d->power_good = 0;
		retry = 5;
	}

	return 0;
}


// 每一步都要等到i2c_session_is_finished才依次到下一步，如果出错就从0开始
enum {
	STEP_IDLE = 0,									// 初始/出错/循环完成会到这里
//	STEP_INPUT_219_CONFIG_DONE,	
	STEP_OUTPUT_219_CONFIG_DONE,		
//	STEP_INPUT_219_READ_VOLTAGE_DONE,	// 这里会多次进入，等219 ready
//	STEP_INPUT_219_READ_CURRNET_DONE,	// 处理完值之后，要把去读OUTPUT_219
	STEP_OUTPUT_219_READ_VOLTAGE_DONE,	// 这里会多次进入，等219 ready
	STEP_OUTPUT_219_READ_CURRNET_DONE,				// 这一步完成之后需要把切mux把另一路的初始化做了
	STEP_MAX
};
/**************************************
外部部调用：
不断测量输入输出电压电流；
微调输出电压到设定值；
输入超压，过流关断；
输出超压，过流关断；

**************************************/
void poll_power_module(unsigned char power_module_idx)
//void poll_power_module( )
{
//	static unsigned char power_module_idx = 0;	// power_module_idx与mux_channel_id正好相同
	power_module_info_t *d;
	static unsigned int last_i2c_error_tick = 0;

	if (!is_time_elapsed_100us(1000, last_i2c_error_tick)) return;	// 总线访问错误的时候歇一下，好像能够明显改善3933的问题

	d = g_data_power_modules + power_module_idx;

	if ((d->step_idx == STEP_IDLE) || (d->step_idx >= STEP_MAX)) {		// 未开始、已完成或出错以后会到这里，这时候需要切换至下一路并开始
		i2c_finish_session_forcibly(d->bus_id);
	}

	int i2c_session_status = i2c_session_is_finished(d->bus_id);
	if (!i2c_session_status)	// 上一笔未完成直接返回
		goto continue_this_module;

	// 到这里，说明上一笔已完成
	if (i2c_session_status < 0) {
		last_i2c_error_tick = g_tick_100us;
		debug("P[%d]failed step=%d\n", power_module_idx, d->step_idx);
		d->step_idx = STEP_IDLE;
		goto handle_next_module;
	}

	switch (d->step_idx) {
		case STEP_IDLE:
			d->step_idx = STEP_OUTPUT_219_READ_VOLTAGE_DONE;
			ina219_exec_check_ready_and_read_voltage(&d->ina219_output);
			goto continue_this_module;

		case STEP_OUTPUT_219_READ_VOLTAGE_DONE:
			d->output_mV = ina219_data_get_voltage(&d->ina219_output);
			power_module_proc_review_constant_voltage(power_module_idx);
			goto handle_next_module;

		default:
			printk("P[%d]never be here, power_module_idx=%d step=%d\n", d->bus_id, power_module_idx, d->step_idx);
			goto handle_next_module;
	}

continue_this_module:
		return;
handle_next_module:

	// 如果需要切i2c_mux，在这里切
	d->step_idx = STEP_IDLE;
}



// 调用i2c0任务初始化的时候，电源模块是没上电的，但所有3v供电的部分（i2c_mux, 219, 3933）都是可以访问的，其中只有i2c_mux能够被p_reset复位
// 因此这个函数只上电的时候调用一次
// i2c_mux的初值为0，一路都不选，这正是我们需要的，不需要初始化
// 219每次访问的时候会完整的重新设置，因此不需要初始化
// 3933初值为0，初始化时需要设置为0xFF，最低电压输出
// 所以这个初始化函数只初始化一些数据和软件状态即可
void power_module_init()
{
	// 两路电源模块都关闭，并设置输出电压为最低
	for (int i = 0; i < NR_POWER_MODULES; i++) {
		power_module_info_t * d = g_data_power_modules + i;

		memset(d, 0, sizeof(g_data_power_modules));
		d->bus_id = BUS_ID1;
		i2c_finish_session_forcibly(d->bus_id);
		ina219_init(&d->ina219_output, d->bus_id, SLA_OUTPUT_219, OUTPUT_219_CONFIG, INA219_CONFIG_IV, OUTPUT_219_CALIBRATION);
		ina219_exec_trigger(&d->ina219_output);
		d->loops =0;

		power_module_set_mV(i, 0);// 关闭模块；	
	}
}

void power_module_print()
{
	for (int i = 0; i < NR_POWER_MODULES; i++) {
		power_module_info_t *d = g_data_power_modules + i;
		unsigned int efficent = d->output_mA;
		efficent *= d->output_mV;
		efficent *= 100;
		efficent /= (d->input_mV + 1);
		efficent *= 100;
		efficent /= (d->input_deci_mA + 1);
		printk("P[%d]good=%d	iV=%d iA=%d oV=%d oA=%d p=%d %d.%d%%\n",
				i, d->power_good,
				d->output_mV,  d->output_mA,
				d->dac_output, efficent / 10, efficent % 10);
	}
}

