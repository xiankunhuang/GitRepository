#include "config.h"
#include "backplane.h"

#define MCU_DAC_OUTPUT  1023
#define DAC_STEP  4095	// 10bit: 1023

#define DUT_ERROR_CNT_LIMIT	30

#define STEP_IDLE  0

dut_info_t dut_info;

/**************************************
1，校准 Vf 电压测量的4x 放大的放大倍数；
2，校准mpd 电流测量的各挡位的电阻值；


**************************************/

static inline void safe_set_little_endian_u16(unsigned char * p, unsigned short v)
{
	*p = v & 0xFF;
	*(p + 1) = (v & 0xFF00) >> 8;
}

static inline void safe_set_little_endian_u32(unsigned char * p, unsigned int v)
{
	*p = v & 0xFF;
	*(p + 1) = (v & 0xFF00) >> 8;
	*(p + 2) = (v & 0xFF0000) >> 16;
	*(p + 3) = (v & 0xFF000000) >> 24;
}

static inline unsigned int round(unsigned int value)
{
	int mod = value - value / 10 * 10;
	return (mod > 5) ? (value / 10 + 1) : (value / 10); 
}

/***********************************
// 由于mcu输出的是LVCMOS，通过运放转换成15v，可能需要30us或更多输出才能稳定；

MUX1: 输出接PD+
channel 0~3：TO1~TO4

MUX2：PD-
channel 0~3：TO1~TO4
channel 4： 33K

MUX3： PD- 和 PD_BUF
channel 0： 1.1M

channel 2： 100K

channel 3： 10K

channel 4： 910R

config: 
bit7~6	: mode
bit5~4	: amp
bit3~0	: channel


channel 0： 1.1M
channel 1： 330K
channel 2： 100K
channel 3： 51K
channel 4： 20K
channel 5： 10K
channel 6： 3.3K
channel 7： 910R

***********************************/



// 从64路DUT中选中一路；
/*
static unsigned char next_dut( unsigned char dut)
{
	++dut;
	if (dut > NR_CHANNELS)	// reset
		dut =0;

	while (dut < NR_CHANNELS) {
		if (g_data_tray.DUT_OE[dut]) {
			gpio_select_channel(dut);
			break;
		}
		++dut;
	}

	IODELAY(100); //todo 约25us

	return dut;
}
*/

// 从64路DUT中选中一路；
static unsigned char pick_up_dut( unsigned char dut)
{

	if(dut > NR_CHANNELS)	// reset
		dut = NR_CHANNELS;
	else {
		gpio_select_channel(dut);
	}

	IODELAY(100); //todo 约25us

	g_data_runtime.DUT_selected = dut;
	
	return dut;
}


void check_selected_DUT()
{
	unsigned char dut;
	unsigned int _drv;

	dut = g_data_runtime.DUT_selected;
	g_data_tray.DUT_online[dut] = 1;
	g_data_tray.DUT_OE[dut >> 4] = ~(1 << (dut | 0xFFFF));
	for (int i = 0; i < NR_CHANNELS; i++) g_data_runtime.is_channel_on_array[i] = 0;
	if (dut < NR_CHANNELS) {
		if ((dut_info.Vf[dut] > working_profile_req.Vf_up_limit) || (dut_info.Vf[dut] < working_profile_req.Vf_low_limit)) {
				g_data_tray.DUT_err[dut] = 1;
				g_data_tray.DUT_online[dut] = 0;
		} else {
			g_data_runtime.is_channel_on_array[dut] = 1;
			
		}
		debug("[%d]on=%x online=%d\n", dut, g_data_runtime.is_channel_on_array[dut], g_data_tray.DUT_online[dut]);
	}
///////////////////////////////////////	
	dut_info.status = DUT_DATA_INVALID;
}

// RSA 是200mA 挡位； RSB 是20mA 挡位；
static unsigned int update_RS_current( )
{
	return g_data_adc[ADC_ID_RS].value * 10;
}

// 计算并返回 Vf in mV
static unsigned int update_Vf( )
{
	return g_data_adc[ADC_ID_VF].value * 16 / 100;	
}

void update_mPD_gain()
{ // relay 切换时间典型值1.5ms，3 ms max.
static unsigned char old_v = 0xff;	

	if(old_v == dut_info.gain)
		return;
	else {//先开下一挡，再关上一档； 
	//	printk("gain %d->%d\n", old_v, dut_info.gain);
		if(0 == dut_info.gain) // 5uA
			FIO1SET_bit.P1_30 = 1;
		else if(1 == dut_info.gain) // 50uA
			FIO1SET_bit.P1_31 = 1;
		else if(2 == dut_info.gain) // 500uA
			FIO0SET_bit.P0_23 = 1;
		else // if(3 == dut_info.gain) // 5000uA
			FIO0SET_bit.P0_24 = 1;

		IODELAY(5000); // 每次循环大约0.25us 
		if(0 == old_v)
			FIO1CLR_bit.P1_30 = 1;
		else if(1 == old_v)
			FIO1CLR_bit.P1_31 = 1;
		else if(2 == old_v)
			FIO0CLR_bit.P0_23 = 1;
		else // if(3 == dut_info.gain)
			FIO0CLR_bit.P0_24 = 1;

		old_v = dut_info.gain;

		IODELAY(5000);  // 每次循环大约0.25us 
	}
}


/***********************************
IIC有2类，一类由多人使用，需要设置启动位，触发一次动一次；
使用前检测 g_data_runtime.iic_task_bitmap 对应bit，

另一类是输出（gpio，dac），程序外改预期值，poll函数检测到预期值和当前值不等自动设置；

#define BUS_ID0		0	// EPD; DAC; power module0; 
#define BUS_ID1		1	// 64:1 dut mux; rs200; rs200_neg;
#define BUS_ID2		2	// iic mux; amux0; rs_eml-;

***********************************/

#define IIC_TASK_IDLE 0

void poll_iic()
{
	static unsigned int iic_task[3];
	static unsigned char _adc_idx , iic_idx;	
	int iic_ret;
	
	iic_idx =0;

// IIC0: PD;  
	iic_ret =  i2c_session_is_finished(BUS_ID0);// 

	if (iic_ret > 0) {
	
		if (ADC_PD_SET == iic_task[iic_idx] ) {
			if (STEP_IDLE == g_data_adc[ADC_ID_PD]._step_idx) {
				g_data_runtime.iic_task_bitmap &= ~ADC_PD_SET; // 缺异常检测；
				iic_task[iic_idx] = IIC_TASK_IDLE; 
			}
			else
				adc_proc_pd_buf();
		}
 
 		if (STEP_IDLE == iic_task[iic_idx]) {
			if (g_data_runtime.iic_task_bitmap & ADC_PD_SET) {     //PD电压测量
				adc_proc_pd_buf();
				iic_task[iic_idx] = ADC_PD_SET;
			}
		}
	}
	else if (iic_ret < 0) {
		printk("poll_iic0 err %x\n", iic_task[0]);
	}
	
// IIC1	// 64:1 dut mux; rs
	iic_idx = 1;
	iic_ret = i2c_session_is_finished(BUS_ID1);

	if (iic_ret > 0) {
		if (ADC_RS_SET == iic_task[iic_idx]) {
			if (STEP_IDLE == g_data_adc[ADC_ID_RS]._step_idx) {
				g_data_runtime.iic_task_bitmap &= ~ADC_RS_SET; // 缺异常检测；
				iic_task[iic_idx] = IIC_TASK_IDLE; 
			} else
				adc_proc_RS(); 
		} else if (ADC_POWER_MODULE_SET == iic_task[iic_idx]) {
			if (STEP_IDLE == g_data_power_modules[0].step_idx) {
				g_data_runtime.iic_task_bitmap &= ~ADC_POWER_MODULE_SET; // 缺异常检测
				iic_task[iic_idx] = IIC_TASK_IDLE; 
			}
			else
				poll_power_module(0);
		}
	//	mcp47_poll();
		if (STEP_IDLE == iic_task[iic_idx]) {
			if (g_data_runtime.iic_task_bitmap & ADC_RS_SET) {
				adc_proc_RS();
				iic_task[iic_idx] = ADC_RS_SET;
			} else if(g_data_runtime.iic_task_bitmap & ADC_POWER_MODULE_SET ) {
				poll_power_module(0);
				iic_task[iic_idx] = ADC_POWER_MODULE_SET;
			}
		} 
	} else if(iic_ret < 0) {
		printk("poll_iic1 err %x\n", iic_task[iic_idx]);
	}
//	gpio_select_channel(g_data_runtime.DUT_selected);

// IIC2: iic mux; amux0; rs_eml-;
	 
	iic_idx =2;
	iic_ret = i2c_session_is_finished(BUS_ID2);

	if (iic_ret > 0) {
		if (ADC_AMUX_SET == iic_task[iic_idx]) {
			if (STEP_IDLE == g_data_adc[ADC_ID_AMUX]._step_idx) {
				adc_to_value( _adc_idx);
				g_data_runtime.iic_task_bitmap &= ~( 0x01<< _adc_idx); // 缺异常检测		
				iic_task[iic_idx] = IIC_TASK_IDLE; 
			} else
				adc_proc_mux();
		} else if (ADC_VF_SET == iic_task[iic_idx]) {
			if (STEP_IDLE == g_data_adc[ADC_ID_VF]._step_idx) {
				g_data_runtime.iic_task_bitmap &= ~ADC_VF_SET; // 缺异常检测；
				iic_task[iic_idx] = IIC_TASK_IDLE; 
			} else
				adc_proc_vf(); 
		}

		if (STEP_IDLE ==  iic_task[iic_idx]) {
			//	PCAL6416A_poll();
			if (g_data_runtime.iic_task_bitmap & ADC_AMUX_SET) { //通用电压测量
				while (_adc_idx < 16 /*NR_ADC_SENSORS*/) {
					if (g_data_runtime.iic_task_bitmap & (0x01 << _adc_idx)) {
						gpio_set_adc_mux(_adc_idx);
						adc_proc_mux();
						iic_task[iic_idx] = ADC_AMUX_SET; 
						break;
					}
					_adc_idx++;
				}
				if (_adc_idx >= 16 /*NR_ADC_SENSORS*/)
					_adc_idx = 0;
			} else if (g_data_runtime.iic_task_bitmap & ADC_VF_SET ) {
				adc_proc_vf(); 
				iic_task[iic_idx] = ADC_VF_SET;
			}
		}
	} else if (iic_ret < 0) {
		printk("poll_iic2 err %x\n", iic_task[iic_idx]);
	}
				
}

/*
static  void current2DAC(unsigned int curr)
{
	static	unsigned int last_cfg =0x1;
	unsigned int dac_out0, dac1; 

//	printk("curr=%d last_cfg=%d\n", curr, last_cfg);
	if(last_cfg == curr)
		return;
	
	last_cfg = curr;
	curr /= DUT_PARALLEL;
//	dac1 = (curr - 20000)  + DAC0_CURRENT;;
	if(curr > DAC0_CURRENT) curr =  DAC0_CURRENT;

	dac_out0 = (DAC_STEP * curr ) / DAC0_CURRENT;
//	dac_out1 = (DAC_STEP * curr * 10) / DAC0_CURRENT;
//	dac_out0 = ((dac_out1 - dac_out0 * 10) >= 5) ? (dac_out0 + 1) : dac_out0;
//	g_data_tray.DAC_out_req[0] = (unsigned short)dac_out0;
	g_data_tray.DAC_out_req[1] = (unsigned short)dac_out0;
	printk("DAC_out=%d\n", dac_out0);
	g_data_tray.DAC_need_update = 1;
	hivoltage_set_current_dac(dac_out0);
}

*/
static  void current2DAC(unsigned int curr)
{
	static	unsigned int last_cfg =0x1;
	unsigned int dac_out, dac_out0, mod;

//	printk("curr=%d last_cfg=%d\n", curr, last_cfg);
	if(last_cfg == curr)
		return;

	last_cfg = curr;
	if (curr >  215000) dac_out0 = 215000;
	dac_out0 = (curr * 1886 + 250000) / 10000;
	mod = dac_out0 - dac_out0 / 10 * 10;
	dac_out = (mod < 5) ? (dac_out0 / 10) : (dac_out0 / 10 + 1);
	dac_out = (curr == 0) ? 0 : dac_out;
	g_data_tray.DAC_out_req[1] = (unsigned short)dac_out;
	debug("DAC_out=%d\n", dac_out);
	g_data_tray.DAC_need_update = 1;
	hivoltage_set_current_dac(dac_out);	
}

//函数大约1秒调用100次；
#define CC_RAMP_SLEWRATE	150	// ~15mA/s

static unsigned int get_ramp_step( unsigned int I_set)
{
	unsigned int ramp_step;

	ramp_step = I_set >>8 ; //256 step	

	if(ramp_step < 10)
		ramp_step = 10;
	else if(ramp_step > CC_RAMP_SLEWRATE)
		ramp_step = CC_RAMP_SLEWRATE;
	
	return ramp_step;
}

/****************************************************************************
电流设置函数，
(CC_SET == req)：设定闭环目标值，先开环ramp 到目标值附近，再闭环微调；
闭环微调，基于 CC_INTERVAL_CLOSELOOP 次正常通道的电流测量平均值；
(CC_BYPASS == req)： 直接设定开环目标值；



****************************************************************************/

#define CC_INTERVAL_FINETUNE	30000	// 3 秒微调节一次

unsigned int current_review( unsigned char req, unsigned int I_set) //I_set 单位 uA
{
	static unsigned int I_set_adj, I_set_target, ramp_step, curr_sum; //uA， 输出电流反馈调节；		
	static unsigned short round, loops; 
	static unsigned int last_tick = 0;
	static unsigned char mode = CC_MODE_IDLE; 
	static unsigned char is_openloop = 0;

	g_data_runtime.CC_good = 0;

	if(MAX_DUT_CURRENT < I_set) I_set = MAX_DUT_CURRENT;

	switch (req) {
		case CC_SET_OPENLOOP:
		case CC_SET: {
			static unsigned int count = 0;
			if ((count++ < 10) && (g_data_runtime.running_state == R_STATE_CC || g_data_runtime.running_state == R_STATE_SCANNING)) printk("current review req=%x I_set=%d\n", req, I_set);
			is_openloop = (CC_SET == req) ? 0 : 1;
			mode = CC_MODE_IDLE;
			if( 0 < I_set_adj){
				mode = CC_MODE_RAMP_DOWN;
				ramp_step = get_ramp_step(I_set_adj);
			}
			I_set_target = I_set;	
		}
			break;

		case CC_BYPASS:
			I_set_adj = I_set;
			I_set_target = I_set;
			mode = CC_MODE_BYPASS;
			goto set_DAC;
			break;
		
		case CC_GET_MODE:
			return mode;
			break;
			
		case 0xff:
			printk("target=%d	adj=%d	ramp_step=%d	mode=%d\n" ,I_set_target, I_set_adj , ramp_step, mode);
			goto dont_adj;
			break;
	}

	if (CC_MODE_IDLE == mode) {
		I_set_adj = 0; 
		if(I_set_target >0){
			ramp_step = get_ramp_step(I_set_target);
			mode = CC_MODE_RAMP_UP;
		}
		goto dont_adj;
	} else if(CC_MODE_BYPASS == mode) {

		goto dont_adj;
	} else if(CC_MODE_RAMP_DOWN == mode) {
		if( g_data_tray.DAC_need_update)
			goto dont_adj;

		if(I_set_adj > ramp_step)
			I_set_adj -= ramp_step;
		else{
			I_set_adj = 0; 		
			mode = CC_MODE_IDLE;
			printk("CC_RAMP_DOWN done\n" );
		}
	}else if(CC_MODE_RAMP_UP == mode){
		I_set_adj += ramp_step;
	
		if(I_set_adj >= I_set_target){
			I_set_adj = I_set_target;
			printk("CC_RAMP_UP done= %x \n" , I_set_adj);	
			mode = CC_MODE_FINE_TUNE; // working mode：CC_MODE_FINE_TUNE 之后 开始采集数据；
			loops = dut_info.loops;
			last_tick = g_tick_100us;
			round =0;
			curr_sum =0;
		}
	}else if (CC_MODE_FINE_TUNE == mode) {
		if(loops == dut_info.loops) 
			goto dont_adj;
		
//		if(dut_info.status != DUT_DATA_VALID) goto dont_adj;
		if(is_openloop)
			goto dont_adj;
		loops = dut_info.loops;

		unsigned char _dut =0;
		unsigned int _delta,_limit_up;
		unsigned char drv_i, drv_j;
		
		_delta = I_set_target >> 3; // 超过12.5%的算电流异常； 	
		if(_delta < 1000) // 合理范围有最小值；
			_delta = 1000;
		while (_dut < NR_LOGIC_CHANNELS) {
			drv_i = _dut & 0x0f ;
			drv_j = (_dut >>4 )& 0x03 ;
			if((~g_data_tray.DUT_OE[drv_j]) & (0x01<<drv_i)){
				if((I_set_target > (dut_info.curr[_dut] + _delta))
					|| (dut_info.curr[_dut] < _delta)
					|| (I_set_target < (dut_info.curr[_dut] - _delta))){ // 电流异常；
printk("#");
					if (g_data_tray.DUT_err[_dut] < 0xff) g_data_tray.DUT_err[_dut]++;
				}
				else if( 0 == g_data_tray.DUT_err[_dut]){ // 正常才纳入统计；
					curr_sum += dut_info.curr[_dut];
					round++;
				}
			}
			_dut++;
		}

		if (!is_time_elapsed_100us(CC_INTERVAL_FINETUNE, last_tick)) // 调整间隔
			goto dont_adj;
		
		last_tick = g_tick_100us;
		
		if(0 == round)
			goto dont_adj;

		unsigned int curr_avg;	
		curr_avg = curr_sum / round; // printk("curr_AVG[%d]= \t%d \n",i,  _curr );
		round =0;
		curr_sum =0;
		unsigned char _dont_adj; 
	
		_dont_adj= I_set_target >>10 ; // 差异小于千分之一不调；
		if(_dont_adj<5) _dont_adj =5; // 最小粒度 5uA

		_limit_up = I_set_target >>2; // 最多调25%
		if(_limit_up < 5000) _limit_up = 5000;
		
		_limit_up += I_set_target;
	
		if(curr_avg > I_set_target)	// 调小
		{
			_delta = curr_avg - I_set_target;
			if(_delta <_dont_adj)
			{
				g_data_runtime.CC_good = 1;
				goto dont_adj;
			}	
 			if(_delta > ramp_step) _delta = ramp_step;

			if(I_set_adj > _delta)
				I_set_adj -= _delta;
			else
				I_set_adj =0;
		}
		else   // 调大
		{
			_delta = I_set_target - curr_avg ;
			if(_delta < _dont_adj)
			{
				g_data_runtime.CC_good = 1;
				goto dont_adj;
			}
			if(_delta > ramp_step) _delta = ramp_step;

			I_set_adj += _delta;
			if(I_set_adj > _limit_up ) I_set_adj = _limit_up;
		}
	
		if(I_set_adj > MAX_DUT_CURRENT) I_set_adj = MAX_DUT_CURRENT;

		printk("adj_curr_to: \t%d curr_avg: \t%d \n", I_set_adj , curr_avg);
	
	}

	set_DAC:
	current2DAC( I_set_adj );			
	
	dont_adj:
	g_data_runtime.CC_good = 1;
	return I_set_adj;
}





#define SWITCH_SAFE_VOLTAGE  300   // in mv

/*
#define R_STATE_IDLE_OUTSIDE	1	// idle，夹具底座在机箱外
#define R_STATE_IDLE_INSIDE		2	// idle，夹具底座在机箱内
#define R_STATE_PLUGIN_SCAN		3	// 插入检查，通过Vf 上下限；
#define R_STATE_FOCUS			4	// PD 正在对准激光器发光点
#define R_STATE_LIV				5	// LIV 测试进行中
#define R_STATE_CC				6	// 横流中，比如进行光谱测试；
#define R_STATE_STEP_MOTOR		7	// STEP MOTOR正在运行中；


当夹具在 outside 时候，除了 plugin scan 时候电源模块打开，其余时间关闭；
当夹具在 inside 时候，电源模块一直打开，dut 通道切换的时候，先切入下一个，再切出；
*/



// 只在这一个地方改变g_data_runtime.running_state的值
#define MPD_UPPER_THRESHOLD 0xe00	// 12bit max 0xfff;
#define MPD_LOWER_THRESHOLD 0x100
#define MOVE_UNIT 50
#define MPD_DIFF 10
#define MOVE_STEPS 20

void update_work_state()
{
	static unsigned char sub_step = 0;
	static char gain_adj;
	static int alignment;

	if((g_data_runtime.running_state != g_data_runtime.running_state_req) ){// 切换状态的准备工作；
		if(g_data_runtime.fail_safe_tick_down > 0)
			return;
	
		switch (g_data_runtime.running_state_req) {
			case R_STATE_IDLE:
				
// 开电源模块，恒流到0，
				power_module_set_mV(0,	working_profile_req.V_set_mV);// power on 电源模块
				pick_up_dut(0);
			    gpio_set_all_channel_off();

				current_review( CC_SET_OPENLOOP, 0);
				current2DAC(0);
				gpio_ld_power_onoff(0);

				printk("switch to IDLE\n" );
				g_data_runtime.running_state = R_STATE_IDLE;
				break;


			case R_STATE_SCANNING:	
				power_module_set_mV(0,	working_profile_req.V_set_mV);
				printk("switch to PLUGIN_SCAN\n" );
				
				
				gpio_ld_power_onoff(1);
				gpio_set_reverse_leakage_mode(g_data_tray.DUT_rev_mode);
				if (gpio_set_pin_config(g_data_tray.DUT_type, g_data_tray.pin_config) < 0) {
					g_data_runtime.running_state = R_STATE_IDLE;
					g_data_runtime.running_state_req = R_STATE_IDLE;
					break;
				}
				g_data_runtime.DUT_selected = 0; // working_profile_req.DUT_idx;
				pick_up_dut(g_data_runtime.DUT_selected); 
				
				for (int i = 0; i < NR_CHANNELS; i++) g_data_tray.DUT_err[i] = 0;
				g_data_tray.DUT_pd_mode = 0;// working_profile_req.DUT_pd_mode;
				gpio_set_pd_mode(g_data_tray.DUT_pd_mode);
				current2DAC(working_profile_req.I_set_uA);
			//	current_review(CC_SET, working_profile_req.I_set_uA); // 配置扫描电流
				dut_info.gain = 0;
				update_mPD_gain();
				IODELAY(80000);
				sub_step = 0;
				g_data_runtime.running_state = R_STATE_SCANNING;
				break;

			case R_STATE_CC:
				gpio_ld_power_onoff(1);
				gpio_set_reverse_leakage_mode(g_data_tray.DUT_rev_mode);
				if (gpio_set_pin_config(g_data_tray.DUT_type, g_data_tray.pin_config) < 0) {
					g_data_runtime.running_state = R_STATE_IDLE;
					g_data_runtime.running_state_req = R_STATE_IDLE;
					break;
				}
				power_module_set_mV(0,	working_profile_req.V_set_mV);
				g_data_runtime.DUT_selected = working_profile_req.DUT_idx;
				pick_up_dut(g_data_runtime.DUT_selected);
				current2DAC(working_profile_req.I_set_uA);
			//	current_review(CC_SET, working_profile_req.I_set_uA); // 配置扫描电流
				g_data_tray.DUT_pd_mode = working_profile_req.DUT_pd_mode;
				gpio_set_pd_mode(g_data_tray.DUT_pd_mode);
				dut_info.gain = 0;
				update_mPD_gain();
				
				sub_step = 0;
				g_data_runtime.running_state = R_STATE_CC;
				printk("switch to CC\n" );
				break;

			case R_STATE_LIV:
			//	memset(ASYNC_OP_MMAP_ADDR, 0x00, 32);
				printk("switch to LIV %d\n", g_data_runtime.running_state);
				if (0 == g_data_runtime.LIV_nr_step) {
					printk("switch to LIV %d\n", g_data_runtime.running_state);
					g_data_runtime.LIV_info->error_code = ASYNC_OP_STATE_FINISHED_FAILED;
					return;
				}

				//g_data_runtime.LIV_I_max = g_data_runtime.LIV_I_step * g_data_runtime.LIV_nr_step;
				g_data_runtime.async_op_state = ASYNC_OP_STATE_ONGOING;
				g_data_runtime.LIV_wait = 0;
				g_data_runtime.LIV_step = 0; 
				g_data_runtime.LIV_timeout = 0;
				g_data_runtime.LIV_I = 0;
				gpio_ld_power_onoff(1);
				power_module_set_mV(0,	working_profile_req.V_set_mV);
				gpio_set_reverse_leakage_mode(g_data_tray.DUT_rev_mode);
				if (gpio_set_pin_config(g_data_tray.DUT_type, g_data_tray.pin_config) < 0) {
					g_data_runtime.running_state = R_STATE_IDLE;
					g_data_runtime.running_state_req = R_STATE_IDLE;
					break;
				}
				g_data_tray.DUT_pd_mode = 1; // working_profile_req.DUT_pd_mode;
				gpio_set_pd_mode(g_data_tray.DUT_pd_mode);
				dut_info.gain = 0;
				update_mPD_gain();
				g_data_runtime.DUT_selected = g_data_runtime.LIV_DUT;
				printk("DUT_selected=%d \n", g_data_runtime.DUT_selected);
				pick_up_dut(g_data_runtime.DUT_selected);
				current2DAC(0);
			//	current_review( CC_SET, 0);
				dut_info.LIV_curr = 0;
				alignment = 0;
				sub_step = 0;
				printk("switch to LIV nr_steps=	%d/%d\n", g_data_runtime.LIV_nr_step, g_data_runtime.LIV_info->nr_steps);
				g_data_runtime.running_state = R_STATE_LIV;
				g_data_runtime.running_state_req = R_STATE_LIV;
				break;

			case R_STATE_FOCUS:
			//	focus_init();
			//	power_module_set_mV(0,	g_data_runtime.focus_info->voltage_mV);
				gpio_ld_power_onoff(1);
				gpio_set_reverse_leakage_mode(g_data_tray.DUT_rev_mode);
				if (gpio_set_pin_config(g_data_tray.DUT_type, g_data_tray.pin_config) < 0) {
					g_data_runtime.running_state = R_STATE_IDLE;
					g_data_runtime.running_state_req = R_STATE_IDLE;
					break;
				}
				g_data_runtime.DUT_selected = working_profile_req.DUT_idx;
				pick_up_dut(g_data_runtime.DUT_selected);
				current2DAC(working_profile_req.I_set_uA);
				current_review( CC_SET_OPENLOOP, g_data_runtime.focus_info->current_uA);
				g_data_tray.DUT_pd_mode = working_profile_req.DUT_pd_mode;
				gpio_set_pd_mode(g_data_tray.DUT_pd_mode);
				dut_info.gain = 1;
				update_mPD_gain();
				g_data_runtime.running_state = R_STATE_FOCUS;
				g_data_runtime.running_state_req = R_STATE_FOCUS;
				g_data_runtime.iic_task_bitmap = ADC_PD_SET;
				sub_step = 0;
				printk("switch to FOCUS\n" );
				move_to_dut(g_data_runtime.DUT_selected, g_data_runtime.focus_info->position[0], g_data_runtime.focus_info->position[1], g_data_runtime.focus_info->position[2]);
				
				break;
		}	
	}
	
	switch (g_data_runtime.running_state) {
		case R_STATE_IDLE:
		//	power_module_set_mV(0,	working_profile_req.V_set_mV);
			current2DAC(0);
			current_review( CC_SET_OPENLOOP, 0);
		//	current2DAC(working_profile_req.I_set_uA);

			if (0 == sub_step) {
				if(CC_MODE_IDLE != current_review( CC_GET_MODE, 0))
					return;
				
				gpio_set_all_channel_off();
				gpio_ld_power_onoff(0);
				sub_step = 1;
			}

			tray_update_state(); // idle状态iic2都是空闲的；
			break;

			
		case R_STATE_SCANNING:
			if (0 == sub_step) {
			//	if (g_data_runtime.CC_good == 0)
			//		return;
				if (g_data_tray.DAC_need_update) // 等待恒流更新完成；
					return;
				IODELAY(480000);
			//	if ((CC_MODE_IDLE == current_review(CC_GET_MODE, 0)) || (CC_MODE_RAMP_UP == current_review(CC_GET_MODE, 0))) // || (g_data_runtime.CC_good == 0))
			//		return;
				g_data_runtime.iic_task_bitmap |= ADC_PD_SET | ADC_RS_SET | ADC_VF_SET; // | ADC_POWER_MODULE_SET;
				alignment = 0;
				sub_step = 1;
			} else if (1 == sub_step){
				if(g_data_runtime.iic_task_bitmap & ( ADC_PD_SET | ADC_RS_SET | ADC_VF_SET)) // | ADC_POWER_MODULE_SET))
					return;
				unsigned int mpd = g_data_adc[ADC_ID_PD].value * g_data_runtime.gain_x[dut_info.gain]; // 单位nA

				if(1 == gain_adj){ // 要放前面，避免连续2次换挡；
					alignment -= mpd; // 新的校准值
					gain_adj = 0;
				}

				if((g_data_adc[ADC_ID_PD].value > MPD_UPPER_THRESHOLD) && (dut_info.gain < 3)){ // 升档
					dut_info.gain++;
					alignment += mpd;
					update_mPD_gain();
					g_data_runtime.iic_task_bitmap |= ADC_PD_SET;
					gain_adj = 1;
					return;
				} else if((g_data_adc[ADC_ID_PD].value < MPD_LOWER_THRESHOLD) && (dut_info.gain > 0)) { //降挡
					dut_info.gain--;
					alignment += mpd;
					update_mPD_gain();
					g_data_runtime.iic_task_bitmap |= ADC_PD_SET;
					gain_adj = 1;
					return;
				}
				mpd += alignment;
				
					
				unsigned int curr = update_RS_current();
				unsigned short vf = update_Vf();
				dut_info.curr[g_data_runtime.DUT_selected] = curr;
				dut_info.Vf[g_data_runtime.DUT_selected] = vf;
				dut_info.mPD[g_data_runtime.DUT_selected] = mpd;
				
				printk("SCANNING[%d] curr=%d Vf=%d mPD=%d gain=%d\n", g_data_runtime.DUT_selected, curr, vf, mpd, dut_info.gain);
				check_selected_DUT(); // 根据 Vf 范围判断dut online；置位online 标记

				g_data_runtime.DUT_selected++;
				current2DAC(0);
				if(g_data_runtime.DUT_selected >= NR_CHANNELS){
					check_selected_DUT(); // 根据 Vf 范围判断dut online；置位online 标记
					current2DAC(0);
				//	current_review( CC_SET, 0);
					power_module_set_mV(0, 0);// power off 电源模块
				//	gpio_ld_power_onoff(0);
					g_data_runtime.running_state = R_STATE_IDLE;
					g_data_runtime.running_state_req = R_STATE_NULL;
					g_data_runtime.DUT_selected = 0;
				} else {
					pick_up_dut(g_data_runtime.DUT_selected);
					current2DAC(working_profile_req.I_set_uA);
				//	current_review(CC_SET_OPENLOOP, working_profile_req.I_set_uA);
				}
				sub_step = 0;
			}
			break;
			
		case R_STATE_CC: {	// constant current;
				if(0 == sub_step){//触发测量
				//	if ((CC_MODE_IDLE == current_review(CC_GET_MODE, 0)) || (CC_MODE_RAMP_UP == current_review(CC_GET_MODE, 0)))
				//		return;
					current2DAC(working_profile_req.I_set_uA);
					if (g_data_runtime.CC_good == 0)
						return;
				//	if (g_data_tray.DAC_need_update) // 等待恒流更新完成；
				//		return;
					g_data_runtime.iic_task_bitmap = ADC_PD_SET | ADC_RS_SET | ADC_VF_SET | ADC_POWER_MODULE_SET;
					sub_step = 1;

				} else if(1 == sub_step) {
					if(!(g_data_runtime.iic_task_bitmap & (ADC_PD_SET |  ADC_RS_SET | ADC_VF_SET | ADC_POWER_MODULE_SET))){
						// update 测量值；如果需要换挡就换挡再测一次；
						static unsigned int count1 = 0;
						unsigned int mpd = g_data_adc[ADC_ID_PD].value * g_data_runtime.gain_x[dut_info.gain]; // 单位nA

						if(1 == gain_adj){ // 要放前面，避免连续2次换挡；
							alignment -= mpd; // 新的校准值
							gain_adj = 0;
						}

						if((g_data_adc[ADC_ID_PD].value > MPD_UPPER_THRESHOLD) && (dut_info.gain < 3)){ // 升档
							dut_info.gain++;
							alignment += mpd;
							update_mPD_gain();
							g_data_runtime.iic_task_bitmap |=  (ADC_PD_SET);
							gain_adj = 1;
							return;
						}
						else if((g_data_adc[ADC_ID_PD].value < MPD_LOWER_THRESHOLD) && (dut_info.gain > 0)){ //降挡
							dut_info.gain--;
							alignment += mpd;
							update_mPD_gain();
							g_data_runtime.iic_task_bitmap |=  (ADC_PD_SET);
							gain_adj = 1;
							return;
						}
		
						mpd += alignment; 	
					
					//	g_data_runtime.step_motor_move_done = 1;
						unsigned int curr = update_RS_current();
						unsigned short vf = update_Vf();
						dut_info.curr[g_data_runtime.DUT_selected] = curr;
						dut_info.Vf[g_data_runtime.DUT_selected] = vf;
						dut_info.mPD[g_data_runtime.DUT_selected] = mpd;
						g_data_runtime.record[0] = sm[0].position;
						g_data_runtime.record[1] = sm[1].position;
						g_data_runtime.record[2] = sm[2].position;
						g_data_runtime.record[3] = mpd;
						dut_info.mPD[g_data_runtime.DUT_selected] = mpd;
						check_selected_DUT();
						if (count1++ % 10000 == 0) 
							printk("[%d, %d, %d]mpd=%d, vf=%d cur=%d\n", sm[0].position, sm[1].position, sm[2].position, dut_info.mPD[g_data_runtime.DUT_selected], vf, curr);
						
						g_data_runtime.iic_task_bitmap = ADC_PD_SET | ADC_RS_SET | ADC_VF_SET;
						alignment = 0;
						sub_step = 0;

					}
				}else if(2 == sub_step) {
					if (!(g_data_runtime.iic_task_bitmap & (ADC_PD_SET | ADC_RS_SET | ADC_VF_SET))) {
						// update 测量值；如果需要换挡就换挡再测一次；
						unsigned int pd = g_data_adc[ADC_ID_PD].value * g_data_runtime.gain_x[dut_info.gain]; // 单位nA
						static unsigned int count2;
					
						if(1 == gain_adj){ // 要放前面，避免连续2次换挡；
							alignment -= pd; // 新的校准值
							gain_adj = 0;
						}

						if((g_data_adc[ADC_ID_PD].value > MPD_UPPER_THRESHOLD) && (dut_info.gain < 3)){ // 升档
							dut_info.gain++;
							alignment += pd;
							update_mPD_gain();
							g_data_runtime.iic_task_bitmap |=  (ADC_PD_SET);
							gain_adj = 1;
							return;
						}else if((g_data_adc[ADC_ID_PD].value < MPD_LOWER_THRESHOLD) && (dut_info.gain > 0)){ //降挡
							dut_info.gain--;
							alignment += pd;
							update_mPD_gain();
							g_data_runtime.iic_task_bitmap |=  (ADC_PD_SET);
							gain_adj = 1;
							return;
						}

						pd += alignment;	
					
					//	g_data_runtime.step_motor_move_done = 1;
						unsigned int curr = update_RS_current();
						unsigned short vf = update_Vf();
						dut_info.curr[g_data_runtime.DUT_selected] = curr;
						dut_info.Vf[g_data_runtime.DUT_selected] = vf;
						dut_info.mPD[g_data_runtime.DUT_selected] = pd;
						g_data_runtime.record[0] = sm[0].position;
						g_data_runtime.record[1] = sm[1].position;
						g_data_runtime.record[2] = sm[2].position;
						g_data_runtime.record[3] = pd;
						dut_info.PD[g_data_runtime.DUT_selected] = pd; 
						check_selected_DUT();
					if (count2++ < 5)	printk("[%d, %d, %d]large_pd=%d, Vf=%d, cur=%d\n", sm[0].position, sm[1].position, sm[2].position, dut_info.PD[g_data_runtime.DUT_selected], vf, curr);
						sub_step = 0;
					}
				}
			}
			break;

		case R_STATE_LIV:
			
			if(0 == sub_step){
			//	if(CC_MODE_IDLE != current_review( CC_GET_MODE, 0)) // 等恒流ramp down 到0；
			//		return;
				if (g_data_runtime.set_current_finished == 0)
					return;
				alignment = 0;
				sub_step = 1;
				printk("LIV: START @%d\n",g_tick_100us);
			} else if (1 == sub_step) {//等待电压稳定
				dut_info.LIV_curr = dut_info.LIV_curr + g_data_runtime.LIV_I_step;
				if(dut_info.LIV_curr > g_data_runtime.LIV_I_max ) // LIV done;
					dut_info.LIV_curr = g_data_runtime.LIV_I_max;
	
			//	current_review( CC_BYPASS, dut_info.LIV_curr); //
				debug("LIV_curr=%d @%d ", dut_info.LIV_curr, g_tick_100us);
				current2DAC(dut_info.LIV_curr);
			//	dut_info.gain = 0;
			//	update_mPD_gain();
			//	alignment = 0;
				sub_step = 2;
				break;

			} else if(2 == sub_step){// 更新恒流值
				if (g_data_tray.DAC_need_update) // 等待恒流更新完成；
					return;
			//	if ((CC_MODE_IDLE == current_review(CC_GET_MODE, 0)) || (CC_MODE_RAMP_UP == current_review(CC_GET_MODE, 0)))
				if (g_data_runtime.set_current_finished == 0)
					return;
				g_data_runtime.set_current_finished = 0;

				g_data_runtime.iic_task_bitmap |=  (ADC_PD_SET | ADC_RS_SET | ADC_VF_SET); // | ADC_POWER_MODULE_SET);
				sub_step = 3;
			}
			else if(3 == sub_step){
				if(!(g_data_runtime.iic_task_bitmap & (ADC_PD_SET |  ADC_RS_SET | ADC_VF_SET))){
					// update 测量值；如果需要换挡就换挡再测一次；
					unsigned int mpd = g_data_adc[ADC_ID_PD].value * g_data_runtime.gain_x[dut_info.gain]; // 单位nA

					if(1 == gain_adj){ // 要放前面，避免连续2次换挡；
						alignment -= mpd; // 新的校准值
						gain_adj = 0;
					} else if (0 == gain_adj)
						alignment = 0;
					if((g_data_adc[ADC_ID_PD].value > MPD_UPPER_THRESHOLD) && (dut_info.gain < 3)){ // 升档
						dut_info.gain++;
						alignment += mpd;
						update_mPD_gain();
						g_data_runtime.iic_task_bitmap |=  (ADC_PD_SET);
						gain_adj = 1;
						return;
					}else if((g_data_adc[ADC_ID_PD].value < MPD_LOWER_THRESHOLD) && (dut_info.gain > 0)) { //降挡
						dut_info.gain--;
						alignment += mpd;
						update_mPD_gain();
						g_data_runtime.iic_task_bitmap |=  (ADC_PD_SET);
						gain_adj = 1;
						return;
					}
					mpd += alignment;

					unsigned int curr = update_RS_current();
					unsigned short vf = update_Vf();

					unsigned char *record;
					record = g_data_runtime.LIV_info->records[g_data_runtime.LIV_step];
					safe_set_little_endian_u32(record, curr);
					safe_set_little_endian_u16(record + 4, vf);
					safe_set_little_endian_u32(record + 6, mpd);
				//	*(record + 10) = dut_info.gain;
				//	check_selected_DUT();
					printk("[%d, %d/%d] curr=%d vf=%d mpd=%d gain=%d\n", g_data_runtime.DUT_selected,  g_data_runtime.LIV_info->step_idx,  g_data_runtime.LIV_info->nr_steps,  curr, vf, mpd, dut_info.gain);

					g_data_runtime.LIV_step++;
					g_data_runtime.LIV_info->step_idx = g_data_runtime.LIV_step;
					if (g_data_runtime.LIV_step >= g_data_runtime.LIV_nr_step){ // phase done;
//							g_data_runtime.LIV_wait =1;
					//	current_review( CC_SET, g_data_runtime.I_set);
					//	current2DAC(g_data_runtime.I_set);
						sub_step = 0;
						g_data_runtime.async_op_state = ASYNC_OP_STATE_FINISHED_OK; 
						printk("LIV: DONE @%d\n",g_tick_100us);
						g_data_runtime.running_state_req = R_STATE_IDLE;
						return;
					}
					g_data_runtime.LIV_timeout = 0;
					sub_step = 1;
				}
			}
			break;


		case R_STATE_FOCUS:	{// move to find the posion to get the maxim pd current;
				static int count = 0;
				if(0 == sub_step){//触发测量
					if (g_data_runtime.CC_good == 0)
						return;
					current2DAC(working_profile_req.I_set_uA);
					g_data_runtime.iic_task_bitmap = ADC_PD_SET | ADC_RS_SET | ADC_VF_SET;
					alignment = 0;
					g_data_adc[ADC_ID_PD].value = 0;
					sub_step = 1;
				} else if (1 == sub_step) {
					if ((sm[0].target_position != sm[0].position) || (sm[1].target_position != sm[1].position) || (sm[2].target_position != sm[2].position)) return;
					move_to_dut(g_data_runtime.DUT_selected, g_data_runtime.focus_info->target_position[0], g_data_runtime.focus_info->target_position[1], g_data_runtime.focus_info->target_position[2]);
					sub_step = 2;
				} else if (2 == sub_step) {
					if(!(g_data_runtime.iic_task_bitmap & (ADC_PD_SET |  ADC_RS_SET | ADC_VF_SET))){

						// update 测量值；如果需要换挡就换挡再测一次；
						if (g_data_adc[ADC_ID_PD].value > 150) g_data_adc[ADC_ID_PD].value = g_data_adc[ADC_ID_PD].value - 150;
						else g_data_adc[ADC_ID_PD].value = 0;
	
						unsigned int mpd = g_data_adc[ADC_ID_PD].value * g_data_runtime.gain_x[dut_info.gain]; // 单位nA
						
						if (count == 0) mpd = 0;
						count++;
						dut_info.mPD[g_data_runtime.DUT_selected] = mpd;
						unsigned int curr = update_RS_current();
						unsigned short vf = update_Vf();
						unsigned char *record;
						record = g_data_runtime.focus_info->records[g_data_runtime.focus_info->step_idx];
						safe_set_little_endian_u32(record, sm[0].position);
						safe_set_little_endian_u32(record + 4, sm[1].position);
						safe_set_little_endian_u32(record + 8, sm[2].position);
						safe_set_little_endian_u32(record + 12, mpd);
						check_selected_DUT();
						debug("[%d] mpd=%d vf=%d curr=%d @[%d, %d, %d]\n", g_data_runtime.focus_info->step_idx, mpd,  vf, curr, sm[0].position, sm[1].position, sm[2].position);
						g_data_runtime.focus_info->step_idx++;
						
						if ((g_data_runtime.focus_info->target_position[0] == sm[0].position) && (g_data_runtime.focus_info->target_position[1] == sm[1].position) && (g_data_runtime.focus_info->target_position[2] == sm[2].position)) {
							sub_step = 0;
							count = 0;
							g_data_runtime.async_op_state = ASYNC_OP_STATE_FINISHED_OK; 
							g_data_runtime.running_state = R_STATE_CC;
							g_data_runtime.running_state_req = R_STATE_CC;
							printk("FOCUS: FINISHED @%d @%d @%d\n", g_tick_100us, g_data_runtime.running_state, g_data_runtime.focus_info->step_idx);
							alignment = 0;	
							return;
						
						} else {
							g_data_runtime.iic_task_bitmap = ADC_PD_SET;
							sub_step = 2;
						}
					}
				}
			}
			break;
		default:
			printk("bad working state, never be here\n");
			break;
	}
}






