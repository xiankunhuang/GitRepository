#ifndef _BACKPLANE_H_
#define _BACKPLANE_H_

#include "config.h"

// longjmp到main主循环之前，用于异常或异步事件处理
#define EXCEPTION_FAIL_SAFE							1
#define EXCEPTION_TRAY_UNPLUGGED					2
#define EXCEPTION_NEW_RUNNING_STATE_EXCEPT_TESTING	3		// 注意不能用来设置进入testing
#define EXCEPTION_TESTING_ENTER						4		// 进入testing
__noreturn void trigger_exception(int ex);

// 起调度器的作用，内部会用longjmp来处理异常
// 对于耗时操作，需要在***安全***的地方反复调用该方法
// 例如对于耗时的iic操作，在每一个iic session完成的时候调，避免把总线搞乱
typedef void (*poll_func_ptr)(void);

//
// gpio.c
//
/* 模块用到以下GPIO：
 * 	1.led：board_led：p3[25], tray_led p3[26]
 * 	2.power_en: pwr_en p2[0]
 * 	3.p_reset#：p0[29]，这个是三个复用器的全复位，用于灾难恢复，注意这个脚需要同时把p0[30]的方向一致才能正常工作
 *  4.tray_plug:p2[12]和P2[11], input，两个都被拉高才被识别为tray插入了
 *  5.rs485_tx_en: p1[0]
 *  6.MUX_A~D分别在P2.3~P2.6，MUX_OE0#和OE1在P2.1和P2.2
 * 所有的gpio方向都不变，只需要初始化一次，以后就不用管方向了
*/
extern void gpio_init();
extern void gpio_led(int led_idx, int led_cmd);
extern void gpio_set_main_controller_onoff(int is_on);
extern void gpio_set_adc_mux(int adc_mux_idx);						// 用来切adc_mux，注意0~15是用来量通道的电流和电压的
extern void gpio_set_motor_mode(unsigned char motor_idx, unsigned char mode);
extern void gpio_motor_pulse_x(unsigned char  start_pulse);
extern void gpio_motor_pulse_y(unsigned char start_pulse);
extern void gpio_motor_pulse_z(unsigned char start_pulse);
extern void gpio_set_motor_onoff(int is_on);
extern void gpio_board_led(int led_cmd);
extern void gpio_rs485_tx_onoff(int is_on);
extern void gpio_set_door_lock_onoff(int is_on);
extern void gpio_electromagnet_onoff(int is_on);
extern int gpio_top_x();
extern int gpio_bottom_x();
extern int gpio_top_y();
extern int gpio_bottom_y();
extern int gpio_top_z();
extern int gpio_bottom_z();


//
// i2c.c
//
extern void i2c_print_stats();
extern void i2c_init();

extern void i2c_finish_session_forcibly(unsigned char bus_id);
extern int i2c_session_is_finished(unsigned char bus_id);

typedef struct {	// 这里面的顺序不能调整，大量的代码用这个顺序初始化req数组
	unsigned char op_mode;			 // 0:write 1:read 2:先写地址、repeat_start、再读值
	unsigned char sla_addr;          // 地址
	volatile unsigned char *data;    // 数据
	unsigned char data_len;          // 数据长度
} i2c_session_request_t;
#define I2C_SESSION_REQUEST_END		{0, 0, 0, 0}		// 结束的判定是 i2c_session_request_t.data == NULL

extern void i2c_start_session(unsigned char bus_id, i2c_session_request_t *req_array);		// req_array需要以I2C_SESSION_REQUEST_END结尾

extern int i2c_do_session_blocking(unsigned char bus_id, i2c_session_request_t * req_array);
extern int i2c_write_1_byte_repeat_start_read_n_bytes_blocking(unsigned char bus_id, unsigned char sla, volatile unsigned char * buf_start_with_addr, int len);
extern int i2c_write_n_bytes_blocking(unsigned char bus_id, unsigned char sla, volatile unsigned char * buf, int len);
extern int i2c_read_n_bytes_blocking(unsigned char bus_id, unsigned char sla, volatile unsigned char * buf, int len);
extern int i2c_read_n_bytes_with_addr_blocking(unsigned char bus_id, unsigned char sla, unsigned char addr, volatile unsigned char * buf, int len);

extern int i2c_set_PCA9548_blocking(unsigned char bus_id, unsigned char sla, unsigned char mux_channel_id);
extern int i2c_set_NCT3933U_blocking(unsigned char bus_id, unsigned char sla, unsigned char addr, unsigned char data);
extern int i2c_read_AT24C16A_blocking(unsigned char bus_id, unsigned char sla, unsigned char *p_data_208b);
extern int i2c_write_AT24C16A_blocking(unsigned char bus_id, unsigned char sla, unsigned char *p_data_208b);


/*
//
// mcp47fvb,  8/10/12 bit dual DAC
//

ADDRESS 00h
DAC0 Output value

ADDRESS 01h
DAC1 Output value

ADDRESS 08h
bit 3:2 DAC1
bit 1:0 DAC0
DAC Voltage Reference Control bits
11 = VREF pin (Buffered); VREF buffer enabled.
10 = VREF pin (Unbuffered); VREF buffer disabled.
01 = Internal Band Gap (1.22V typical); VREF buffer enabled. VREF voltage driven when powered-down.
00 = VDD (Unbuffered); VREF buffer disabled. Use this state with Power-down bits for lowest current.

ADDRESS 09h
bit 3:2 DAC1
bit 1:0 DAC0
DAC Power-Down Control bits(2)
11 = Powered Down - VOUT is open circuit.
10 = Powered Down - VOUT is loaded with a 100 k resistor to ground.
01 = Powered Down - VOUT is loaded with a 1 k resistor to ground.
00 = Normal Operation (Not powered-down).

ADDRESS 0Ah
bit 9 : DAC1 Output Driver Gain control bits
bit 8 : DAC0 Output Driver Gain control bits
1 = 2x Gain.
0 = 1x Gain.
*/
typedef struct {
	unsigned char bus_id;

	unsigned short w0_dac0;
	unsigned short w1_dac1;	
	unsigned short w08_vref;
	unsigned short w09_power_down;
	unsigned short w0a_gain;


	unsigned char w_dac0_addr_data[3];
	unsigned char w_dac1_addr_data[3];
	unsigned char w_set_Vref_addr_data[3];

	i2c_session_request_t req_set_dac0;
	i2c_session_request_t req_set_dac1;	
	i2c_session_request_t req_set_Vref;		

	i2c_session_request_t req_trigger[3];
	i2c_session_request_t req_check_ready_and_read_voltage[3];
	i2c_session_request_t req_read_current[3];

	volatile unsigned short output_ready_voltage;			// 注意isr直接输出到这里，是big endian
	volatile unsigned short output_current;					// 注意isr直接输出到这里，是big endian
} mcp47_t;



//
// ina219.c
//
typedef struct {
	unsigned char bus_id;

	unsigned char w_config_addr_data[3];
	unsigned char w_calibration_addr_data[3];
	i2c_session_request_t req_trigger[3];
	i2c_session_request_t req_check_ready_and_read_voltage[3];
	i2c_session_request_t req_read_current[3];
	i2c_session_request_t req_read_shunt[3];	

	volatile unsigned short output_ready_voltage;			// 注意isr直接输出到这里，是big endian
	volatile unsigned short output_current;					// 注意isr直接输出到这里，是big endian
	volatile unsigned short output_shunt;					// 注意isr直接输出到这里，是big endian

} ina219_t;
extern void ina219_init(ina219_t * chip219, unsigned char bus_id, unsigned char sla, unsigned short regdata_config, unsigned char has_current, unsigned short regdata_calibration);
extern void ina219_exec_trigger(ina219_t * chip219);
extern void ina219_exec_check_ready_and_read_voltage(ina219_t * chip219);
extern void ina219_exec_read_current(ina219_t * chip219);
extern void ina219_exec_read_shunt_volt(ina219_t * chip219);
extern int ina219_data_is_ready(ina219_t * chip219);
extern unsigned short ina219_data_get_voltage(ina219_t * chip219);
extern unsigned short ina219_data_get_current(ina219_t * chip219);
extern unsigned short ina219_data_get_shunt_volt(ina219_t * chip219);
extern void ina219_batch_read_voltage(NULLABLE poll_func_ptr foo, ina219_t * chip219, unsigned short * out_voltage);
extern void ina219_batch_read_voltage_and_current(poll_func_ptr foo, ina219_t * chip219, unsigned short * out_voltage, unsigned short * out_current);
extern void ina219_batch_read_bus_and_shunt_voltage(poll_func_ptr foo, ina219_t * chip219, NULLABLE unsigned short * out_bus_voltage, unsigned short * out_shunt_voltage);


typedef struct {
	unsigned char bus_id;

	unsigned char w_config_addr_data[4];
	i2c_session_request_t req_trigger[2];
	i2c_session_request_t req_set_channel[2];
	i2c_session_request_t req_start;
	i2c_session_request_t req_reset;
	i2c_session_request_t req_powerdown;
	i2c_session_request_t req_read_reg;
	i2c_session_request_t req_read_data[2];
	i2c_session_request_t req_check_ready[2];
	i2c_session_request_t req_read_voltage[2];

	volatile unsigned char ready;
	volatile unsigned char reg_data;
	volatile unsigned char output_data[3];	
} ads122c04_t;
extern void ads122c_init(ads122c04_t *chip_ads122c, unsigned char bus_id, unsigned char sla, unsigned char regdata00, unsigned char regdata01, unsigned char regdata02, unsigned char regdata03);
extern void ads122c_exec_trigger(ads122c04_t * chip_ads122c);
extern void ads122c_exec_start(ads122c04_t * chip_ads122c);
extern void ads122c_exec_reset(ads122c04_t * chip_ads122c);
extern int ads122c_exec_check_ready(ads122c04_t * chip_ads122c);
extern void ads122c_exec_rdata(ads122c04_t * chip_ads122c);
extern int ads122c_get_ready(ads122c04_t *chip_ads122c);
extern int ads122c_get_voltage(ads122c04_t *chip_ads122c);
extern void ads122c_set_mux(ads122c04_t * chip_ads122c, unsigned char channel_idx);


//
// flash.c
//
typedef __packed struct {			// 定义成__packed，顺序与协议一致，字段没有boundary问题，可以直接memcpy进来
	unsigned int magic;
	unsigned char addr;
	unsigned char hw_rev;
	unsigned char sn[16];
	unsigned char oem_info[16];
	unsigned short boot_loader_rev;		// 从firmware中最后一次更新boot_loader的版本号
	unsigned short boot_loader_sz;
	unsigned short firmware_rev;		// 当前运行的firmware版本号，需要flash中这个值总是FIRMWARE_VER_CODE，以便boot_loader做升级判定
	unsigned short upgrade_sector_sz[3];
	unsigned short upgrade_sector_rev[3];
	unsigned short pad;
	unsigned int upgrade_total_len_expected;	// 总长度	// 当作升级标记用；0 表示已经升级；
} backplane_eeprom_t;
extern void flash_print_backplane_eeprom();
extern void flash_write_backplane_eeprom();
extern void flash_read_backplane_eeprom();
extern void flash_write_boot_loader_from_sram_if_necessary(unsigned short rev, unsigned int length, unsigned int total);
extern int flash_write_upgrade_sector_from_sram(unsigned char upgrade_sector_idx, unsigned short rev, unsigned int length, unsigned int total);
extern void flash_write_firmware_in_boot_loader_if_necessary();
__noreturn extern void flash_go(unsigned char is_boot_loader);
extern int flash_is_keeping_main_controller_on();
extern void flash_clear_keeping_main_controller_on_flag();
__noreturn extern void flash_go_while_keeping_main_controller_on(unsigned char is_boot_loader);


//
// rs485.c
//
extern void rs485_print_stats();
extern void rs485_init();
extern int rs485_crc_and_tx(unsigned char *frame, unsigned short frame_len);
extern void rs485_rx_proc();
extern int rs485_is_busy();
extern unsigned short crc16(const unsigned char * data, unsigned short data_len);


//
// protocol.c
// 4byte 对齐；
typedef struct {
	unsigned char running_state;
	unsigned char DUT_type; // DUT_comm_negative;	// 共阴或者共阳， 引脚配置；
	unsigned char DUT_rev_mode; // reverse leakage mode
	unsigned char DUT_pd_mode;
	unsigned char DUT_idx;
	short 	temp_req; //温度设定，0.1度，为0 表示不管温度；

	unsigned short V_set_mV;	// 输出电压设定	
	unsigned int I_set_uA;	//uA，光谱测试电流设定；	

	unsigned short Vf_low_limit;// 插入扫描Vf 上限；	
	unsigned short Vf_up_limit;	// 插入扫描Vf 下限；

	unsigned short LIV_nr_steps; // liv总步数
	unsigned short LIV_I_step_uA; // liv 测试电流步长

	unsigned int DUT0_position[3]; // 第一颗dut 的三维坐标
	unsigned int DUT31_position[3]; // 最后一颗dut 的三维坐标	
	unsigned int DUT_pitch; // dut 的间距

} working_profile_expected_t;

extern void protocol_exec(const unsigned char *frame, unsigned short frame_len);
extern unsigned short protocol_frame_len(const unsigned char func_id);
extern void safe_set_little_endian_u16(unsigned char * p, unsigned short v);
extern void safe_set_little_endian_u32(unsigned char * p, unsigned int v);
extern unsigned short parse_little_endian_u16(const unsigned char * p);
extern unsigned short parse_little_endian_u16(const unsigned char * p);
//extern void LIV_data_copy(  );

//
// console.c
//
extern void printk(const char *fmt, ...);
extern void console_tx_proc();
extern void console_rx_proc();
extern void console_init();
extern int console_ring_buf_get(unsigned char * buf, unsigned short max_length);
extern void console_cmd(const unsigned char cmd_id, const unsigned char * arg);


//
// init.c
//
extern unsigned char is_time_elapsed_100us(int cnt, unsigned int prev_tick);
extern void boot_jump(unsigned int firmwareStartAddress);
extern void init_all_hw();


//
// adc.c
// 0,1 为RS，2为 pd_buf
typedef struct {
	// 状态
	unsigned short mV;
//	unsigned int deci_mA[NR_ADC_SENSORS];

	// 计算值
//	unsigned short protocol_value_output[NR_ADC_SENSORS];	// 对于CH，是滤波后的电流；对于VCC是滤波后的电压；对于TEMP是换算及滤波后的温度
//	int k_value[NR_CHANNELS];		                        // 在插入检测的时候把每个CH K值，只在pcb_test模式有效，正常模式保持0

	// 内部使用
	unsigned char _step_idx;
	unsigned char _channel_idx;
//	unsigned char sum_count[NR_ADC_SENSORS];
//	unsigned int sum_of_16[NR_ADC_SENSORS];
//	unsigned char continuous_channel_current_failure[NR_CHANNELS];

	// 状态
	unsigned short op_down_cnt;
	unsigned short target_mV;
	short          value;          // 暂存的测量值；
	unsigned char is_valid;		   // 标记为1表示值已经稳定可用了

	signed char measure_config;    // 测电压还是电流；

	ina219_t ina219_adc;
} adc_info_t;
extern void adc_print();
extern void adc_init();
extern void adc_proc();
extern void adc_to_value(unsigned char _adc_idx);
extern unsigned short adc_read(int mux_idx);
extern void adc_init();
extern int adc_proc_mux();
extern void adc_pd_proc();
extern void adc_to_temp();
extern int adc_proc_k_temp();

//
// power_module.c
// RS+接vcc_adj, 通过该ina219可以测试vcc_adj和RS电流
//
typedef struct {
	unsigned short target_output_mV; // 设定值；
	// 实时状态
	unsigned short input_mV;
	unsigned short input_deci_mA;
	unsigned short output_mV;
	unsigned short output_mA;
	
	// 工作状态
	unsigned char is_on;
	unsigned int power_good;//电压值稳定后的tick；
	unsigned char delay_tick;	

	// 内部使用
	unsigned char dac_output;	// 当前值, 注意这里保存的值最高位是符号位，后面7位是值
	unsigned char step_idx;
	unsigned char loops;
	unsigned char bus_id;	

//	unsigned short output_mV_filtered;
//	unsigned char dac_output_expected;
//	signed char output_volt_expected;	// 配置这个值的时候需要把is_value_valid置0，待配置值稳定后这个标记才会为1
//	unsigned char is_going_to_target_output;
//	unsigned char _value_valid_times;	// 打开电源模块后，需要采集若干次数据以后才准确

	ina219_t ina219_input;
	ina219_t ina219_output;
} power_module_info_t;
extern void power_module_print();
extern void power_module_set_mV(unsigned char power_module_idx, unsigned int mV);
extern void power_module_init();
extern void poll_power_module(unsigned char power_module_idx);
extern int power_module_proc_review_constant_voltage(unsigned char power_module_idx);


//
// driver_module.c
//

extern unsigned int current_review( unsigned char req, unsigned int I_set);
extern void update_work_state();
extern void poll_iic();
extern void pd_buf_matrix(unsigned char config);
extern void update_mPD_gain();
//
// temp_control.c
//

typedef struct{  
    int SetValue; // 设定目标 Desired Value  
    unsigned int pValue; // 设定目标 present Value  
    
    int K_Proportion; // 比例常数 Proportional Const  
    int K_Integral; // 积分常数 Integral Const  
    int K_Derivative; // 微分常数 Derivative Const 
    
//    unsigned int LastError; // Error[-1]  
    int Error; // Error[0]   // 偏差   
    int PrevError; // Error[-2]  
    
    int Error_Integral; // Integral of Errors  
    int Error_Derivative;

    int Error_I;
    int CurrError_D;
    int PrevError_D; 


    int p; // debug  
    int i;
    int d;
    int pid;


    int Sum; // 测量值的和 ；
    short SumCount; // 测量的次数；      

	unsigned char duty_cycle; // 输出

}PID;  

extern int temp_control(unsigned int target,  unsigned char op_code);

//
// step_motor.c
//

typedef struct{  

	volatile int position;		
	int target_position;
	int stampe;	            // 连续运行时间，用作超时异常监测
	int first_tick_origin;
	int dir;
//	unsigned char idx;	    // 电机编号；


	unsigned int tick;	    // 调速周期；
	unsigned int interval;	// 定时器周期，40ns为单位，取反表示 当前速度；	

	unsigned int max_v;	    // 最大速度；
	unsigned int max_step;	// 加速过程最大的步数，最大速度/加速步数 = 加速度；
	unsigned int step;	    // 加速过程当前的步数

//	unsigned char accl_table_idx;
	volatile unsigned int	total_pulse; // 本次任务总的步数；非0表示运行中；
	volatile unsigned int	current_pulse;	// 当前已经走的pulse 数；
	volatile unsigned int	accl_pulse; // 加速过程总共用的步数，用作 减速 起点计算；
	
	int limit_up;

	char emergency_step;    // 是否触发急停 1: 触发， 0没有触发
	int step_motor_init_done;
}s_motor;    
extern void motor_goto_mA(unsigned char idx, int mA);
extern void step_motor_set_current(U8 idx, unsigned int mA);
extern void step_motor_goto(unsigned char idx, unsigned int x, unsigned int y, unsigned int z);
extern void step_motor_init();
extern void step_motor_routine();
extern void print_step_motor_postion();

typedef __packed struct {
	unsigned short step_idx; // 有效记录条数
	unsigned char records[1500][16];    // sm_postion[3], pd
} focus_info_t;

//
// tray.c
//

typedef struct {
//	unsigned char status;
	unsigned char is_new_tray;	// 置1则通知主控板更新tray信息，包括插入检测的状况
	unsigned char is_plugged_in;

	unsigned char DUT_type;
	unsigned char DUT_rev_mode;
	unsigned char DUT_pd_mode;

	unsigned short pin_config;	    // 引脚配置；
	unsigned char pin_config_ldp;	// LD+引脚配置
	unsigned char pin_config_ldn;	// LD-引脚配置
	unsigned char pin_config_pd;	// PD引脚配置
	unsigned char pin_config_pd_gnd;
	unsigned char pin_config_buf;
	unsigned char pin_config_eml;  
	unsigned char pd_type;
//	unsigned char config_valid;	// 配置是否有效；

//	on=1 fault=0是正常
//	on=1 fault=1是警告
//	on=0 fault=1是过流关闭
//	上述逻辑有online=1的前提；如果online=0应该永远保持后面都是0
	unsigned short DUT_online[NR_CHANNELS];

	unsigned char DUT_err[NR_CHANNELS];
	unsigned short DUT_OE[6];	

	unsigned short DAC_out_req[2]; //DAC0: 200mA, DAC1: 20mA
	unsigned char DAC_need_update;

	// tray eeprom总长度208字节，包括64字节soliware字段和144字节甲方字段
	unsigned char *eeprom;
	unsigned char is_eeprom_valid;
	unsigned char type;
	unsigned char need_to_update_eeprom;
} tray_info_t;
extern void tray_print();
extern void tray_init();
extern void tray_update_state();
extern void tray_soft_unplug();

// tray eeprom总长度208字节，包括64字节soliware字段和144字节甲方字段
typedef struct {
	unsigned char reversion[16]; // 0x00-0x0F
	unsigned char sn[16];		//0x20-0x2F
	unsigned char oem[16];		//0x30-0x3F
	unsigned char sign[16];		//0x40-0x5F
	unsigned char customer[144];	//用户私有
} tray_eeprom_t;

#define DUT_DATA_INVALID  0
#define DUT_DATA_VALID  1
#define DUT_DATA_DIRTY  2

#define DUT_DATA_ACQING  3

typedef struct {
	unsigned int curr[NR_CHANNELS];	// current;
	unsigned short Vf[NR_CHANNELS];	// Vf；
	unsigned int mPD[NR_CHANNELS];	// nA ;
	unsigned int PD[NR_CHANNELS];	// nA ;
	unsigned int fPD[NR_CHANNELS];	// nA ;
	unsigned char mPD_gain[NR_CHANNELS];	// 挡位
	unsigned char gain; // 挡位
	unsigned char loops; // 测量的轮数；
	unsigned int LIV_curr;	// liv 测试的当前值；

	unsigned char status;
} dut_info_t;

typedef __packed struct {
	unsigned int curr;
	unsigned short Vf;
	unsigned int mPD;
} DUT_LIV_info_t;

typedef __packed struct {

	unsigned short step_idx;
	unsigned short nr_steps;
	unsigned short liv_voltage_mV;
	unsigned char channel_idx;
	unsigned char current_step_uA;
	unsigned char error_code; 		 // 无错误则置0，有错误则置错误码
	unsigned char pin_config;
	unsigned char pad[22];			 // 补齐至32字节
	unsigned char records[2976][10]; // 去掉 mpd 的挡位
} LIV_info_t;


//
// main.c
//
// 不定义宏了，这些值除会在main()里直接赋值，并可以被控制台及485更改
typedef __packed struct {		// 定义成__packed，没有跨内存边界的问题，方便在功能号7中直接拷贝进来
	unsigned int rs485_offline_timeout_100uS;
	unsigned short vcc3_mcu_lower_limit_mV;
	unsigned short vcc3_mcu_upper_limit_mV;
	unsigned short vcc3_lower_limit_mV;
	unsigned short vcc3_upper_limit_mV;
	unsigned int watchdog_interval_100uS;	    // 这个用作watchdog复位的时长，这么长时间没反应，就判定为关机逻辑成立
	unsigned int power_on_delay_100uS;			// 开机逻辑成立之后，距离上次关机后这么长时间再开机
	unsigned int power_off_delay_100uS;			// 主控板通知关机后，延时这么长时间再关机
} global_config_t;

typedef __packed struct {		                                // 定义成__packed，没有跨内存边界的问题，方便在功能号7中直接拷贝进来
	unsigned int motor_max_step[NR_MOTORS];				        // 步进电机运行
	unsigned int motor_max_v[NR_MOTORS];				        // 步进电机最大速度
	unsigned int motor_accel_start_step[NR_MOTORS];		        // 步进电机最大加速度
	unsigned int motor_total_step[NR_MOTORS];			        // 步进电机在滑竿移动最大距离
	unsigned int motor_border_margin[NR_MOTORS];		        // 步进电机零点弹回的步数
	unsigned short motor_current[NR_MOTORS];			        // 步进电机运转时的电流
	unsigned char motor_micro_step[NR_MOTORS];			        // 步进电机微步，可设置为32、16、8、4、2、1分之一步
	unsigned char motor_direction[NR_MOTORS];			        // 步进电机方向
} motor_config_t;




// 实际使用的背板地址，注意高4bit是row地址，低4bit是column地址
// 如果行地址或者列地址是0xF，则表示需要自动获取
// 如果backplane_info里配置的地址的行地址或者列地址为0xF，则是自动获取，否则就使用其中指定的值
// 如果自动获取失败（adc读到行或列的非法电压），则设置为0xFF
// 自动获取失败后，以后还会重试
// 目前行地址和列地址的定义都是0~3200mV，每200一档，行列各16档（4bit）
typedef struct {
	unsigned char backplane_addr;
	unsigned char backplane_type;
	unsigned char is_channel_selected_array[NR_CHANNELS];
	unsigned char is_channel_fault_array[NR_CHANNELS];
	unsigned char is_channel_on_array[NR_CHANNELS];	
	unsigned char is_channel_focus_finished_array[NR_CHANNELS];

	unsigned char is_main_controller_watchdog_started;
	unsigned char is_main_controller_on;
	unsigned char need_to_set_main_controller_onoff;		// 0:不设置 1:打开(若上次主动关机，则有延时逻辑) 2:延时关闭

	unsigned char status;  // 自检，等待温度稳定，光谱测试，liv 测试，等待进料，idle，测试完成，异常
	unsigned char door_lock;
	unsigned char electromagnet_state;
	unsigned char tested;

	unsigned int SM_position[3]; // xyz 三轴 当前位置
//	unsigned short motor_accl_table[100];
//	unsigned short motor_accl,	
//	unsigned char motor_accl_table_idx;
	unsigned char motor_idx;
	unsigned int nr_motor_step;         // 本次一共要走的 步数；
	unsigned int motor_accl_steps;      //加速阶段的总步数；
	unsigned int motor_current_steps;	//当前已走 步数；
	unsigned int need_to_move;
	unsigned int need_to_update_motors;
	unsigned int current_position[3];
	
	signed char moter_3933_step[3];
	
    unsigned int temp_value;	    // 夹具底座温度
    unsigned int temp_TEC;	        // tec 制冷端的温度

	unsigned char heater_duty; // 加热器出力
//	unsigned char DUT_idx; // 当前正在测试的dut  

	unsigned char fixture_plugin; // 夹具插入状态
	unsigned char has_fixture_eeprom;
	unsigned char heater_good; // 加热器电阻正常


	unsigned int loops_per_second;
	unsigned int aligned_1s_tick;
	unsigned int tick_1s;

	unsigned char running_state;
	unsigned char running_state_req;
	unsigned char state_changing;	
//	unsigned char delay_tick_100ms;	
	unsigned char async_op_state;
	unsigned char has_tray_eeprom;
	
	unsigned char fail_safe_tick_down;
	
	short		  temp_set;
	unsigned int  I_set;
	unsigned char CC_good;	
	unsigned char set_current_finished;

	unsigned char 	DUT_selected; // 当前选通的 dut；

	unsigned char 	LIV_DUT;
	unsigned short 	LIV_V_set;
	unsigned short 	LIV_step;
	unsigned short 	LIV_nr_step;	
	unsigned int 	LIV_I_max;	
	unsigned int 	LIV_I;
	unsigned short 	LIV_I_step;
	
	unsigned char 	LIV_timeout;
	unsigned char   LIV_mpd_range;
	unsigned char   new_drawer;
	unsigned int    mon_data[NR_ADC_SENSORS];
	unsigned int    k_temp;
	
//	LIV_info_t *LIV_info;  //(LIV_info_t *)ASYNC_OP_MMAP_ADDR ; 0x2007C000
	focus_info_t *focus_info; // (focus_info_t *)ASYNC_OP_MMAP_ADDR ; 0x2007C000


	unsigned char LIV_wait;// phase之间等待数据被host取走；
	unsigned char LIV_ongoing;

	unsigned char sub_status;
	unsigned char DAC0_ison;

	unsigned short channel_good[4]; // 驱动硬件正常；

	unsigned int last_rs485_frame_received_tick;
	unsigned int start_move_tick;
	unsigned short sync_tick;
	unsigned char start_record;
	unsigned char end_record;

	unsigned int iic_task_bitmap;
	unsigned char amp_gain;	

	unsigned int gain_x[4];	// 8个挡位对应的增益	

	unsigned int step_motor_position[3];	// 步进电机当前坐标，0/1/2 ： x/y/z		
	unsigned int step_motor_target_position[3];	
	unsigned char step_motor_init_done;
	unsigned char step_motor_init_pending;
	unsigned char step_motor_move_done;
	unsigned char step_motor_step;
	unsigned char step_motor_to_switch[3];
	unsigned int  step_motor_nr_step;
	unsigned char read_pd_done;
	unsigned char error_code;
	unsigned int record[4];

} runtime_info_t;

extern volatile unsigned int g_tick_100us;

extern global_config_t g_global_config;
extern motor_config_t g_motor_config;

extern backplane_eeprom_t g_data_backplane_eeprom;
extern adc_info_t g_data_adc[NR_ADC_INA219];
extern runtime_info_t g_data_runtime;
extern focus_info_t focus_info;

extern working_profile_expected_t working_profile_req;

extern s_motor sm[3]; 
extern s_motor * smotor; 


extern PID spid;

extern tray_eeprom_t tray_eeprom;


extern  void debug_print();




#endif				// _BACKPLANE_H_

