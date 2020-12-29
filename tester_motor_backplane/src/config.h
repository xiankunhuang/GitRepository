// 这个文件放影响程序流程的全局宏和常量

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "autover.h"

//#ifdef Debug
//#define debug(...)	printk(__VA_ARGS__)
//#else
#define debug(...)	do {} while (0)
//#endif

#define NULLABLE					// 用于描述可为NULL的参数，不用NULLABLE描述的一般均不能为NULL

#define ABS(v)   ((v >= 0) ? (v) : (-v))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define DIMOFARRAY(array)		(sizeof(array) / sizeof(array[0]))		// 计算数组长度
#define memset(dest, val, len)			do { for (int _i_ = 0; _i_ < (len); _i_++) { ((unsigned char *)(dest))[_i_] = (val); } } while (0)
#define memcpy(dest, src, len)			do { for (int _i_ = 0; _i_ < (len); _i_++) { ((unsigned char *)(dest))[_i_] = ((unsigned char *)(src))[_i_]; } } while (0)
#define GET_BIG_ENDIAN_U16(p)			((unsigned short)((*((unsigned char *)(p)) << 8) | (*((unsigned char *)(p) + 1))))
#define GET_BIG_ENDIAN_U32(p)			((unsigned int)((*((unsigned char *)(p)) << 24) | (*((unsigned char *)(p) + 1) << 16) | (*((unsigned char *)(p) + 2) << 8) | (*((unsigned char *)(p) + 3))))
#define GET_LITTLE_ENDIAN_U16(p)		((unsigned short)((*((unsigned char *)(p) + 1) << 8) | (*((unsigned char *)(p)))))
#define GET_LITTLE_ENDIAN_U32(p)		((unsigned int)((*((unsigned char *)(p) + 3) << 24) | (*((unsigned char *)(p) + 2) << 16) | (*((unsigned char *)(p) + 1) << 8) | (*((unsigned char *)(p)))))

// 经测试，设置为100M的cpu，在各种优化选项下，每次循环大约0.25us；也就是4000次差不多1ms
#define IODELAY(loop_cnt)	do { for (volatile unsigned int _i_ = 0; _i_ < loop_cnt; _i_++) ; } while (0)
#define IOWRITE32(addr, data)	(*(volatile unsigned int *)(addr) = (data))
#define IOREAD32(addr)			((unsigned int)(*((volatile unsigned int *)(addr))))
#define IOWRITE16(addr, data)	(*(volatile unsigned short *)(addr) = (data))
#define IOREAD16(addr)			((unsigned short)(*((volatile unsigned short *)(addr))))
#define IOWRITE8(addr, data)	(*(volatile unsigned char *)(addr) = (data))
#define IOREAD8(addr)			((unsigned char)(*((volatile unsigned char *)(addr))))

/*some type definitions*/
typedef unsigned char    U8;
typedef unsigned short   U16;
typedef unsigned long    U32;
typedef int              S32;
typedef short            S16;
typedef char             S8;


#define CONFIG_UART0_BAUDRATE			500000
#define CONFIG_UART1_BAUDRATE			256000
#define CONFIG_UART3_BAUDRATE			500000 //115200
#define CONFIG_I2C0_FREQUENCY			200000
#define CONFIG_I2C1_FREQUENCY			400000
#define CONFIG_I2C2_FREQUENCY			400000			// 注意有3933的时候，推荐配置为300K/5次重试
#define CONFIG_3933_FREQUENCY			300000
#define CONFIG_3933_RESEND_START_COUNT	5				// 0表示关闭，不对3933做特殊处理

#define SRAM_ADDR						0x2007C000		// 共32KB，不被编译器使用
#define SRAM_SZ							0x8000
#define SRAM_CONTROL_BLOCK_ADDR			0x20083FF0
#define SOLIWARE_MAGIC	0xFACEC0DE		// 传统功能号9升级方法，第一个4字节是0xFACEC0DE，第二个4字节是(长度/256)向上取整，注意不能超过32K

#define FW_START_ADDR					0x00004000   // loader up to 16KB
#define FW_START_SECTOR_NUM				4

#define ASYNC_OP_MMAP_ADDR				0x2007C000		// 让上位机能够按物理地址访问的mmap
#define ASYNC_OP_MMAP_SZ				0x8000			// 大小为32K，这一区域与0x2007C000一样，不被编译器使用


#define FW_UPDATE_SIGN_ADDR   0x20083FF0
#define FW_UPDATE_SIGN        0xFACEC0DE


#define CONFIG_FRAME_BUF_SZ				0x400

#define PROTOCOL_VER_CODE				0x0001

#define PROTOCOL_CMD_FUNC_ID_MAX		0x09
#define PROTOCOL_CMD_FUNC_08_88_LEN_MIN	0x0C
#define PROTOCOL_CMD_FUNC_08_88_LEN_MAX	0x10C

#define NR_I2C_BUSES				3
#define NR_POWER_MODULES			1					// 电源模块的个数
#define NR_DRIVER_MODULES			1					// 驱动模块的个数
#define DUT_PARALLEL 1 // 1 2 4
#define NR_CHANNELS_PER_DRIVER		64 //32
#define NR_CHANNELS					64 //32
#define NR_LOGIC_CHANNELS			(NR_CHANNELS / DUT_PARALLEL)

#define TRAY_EEPROM_SLA				0x50

// motor
#define ACCEL_START_STEP	 800 // 3	//10	//10

// adc

#define BUS_ID0		0	// EPD; DAC; power module0; 
#define BUS_ID1		1	// 64:1 dut mux; rs200; rs200_neg;
#define BUS_ID2		2	// iic mux; amux0; rs_eml-;






	// 电压测量的各个通道
#define NR_ADC_SENSORS		16

#define ADC_VCC_MCU         0
#define ADC_VCC_3V3			1
#define ADC_VCC_5V2			2
#define ADC_DAC_OUT1		3
#define ADC_DAC_OUT2		4
#define ADC_DAC_OUT3		5
#define ADC_LOCK_VCC		6
#define ADC_MGND			7
#define ADC_SAMPLE_MCU		8
#define ADC_SAMPLE_TEMP		9
#define ADC_VCC_16V			12
#define ADC_PCB_VER			15
#define ADC_VCC_TEMP		16

#define ADC_ID_AMUX			0
#define ADC_ID_TEMP			1
	
#define NR_ADC_INA219		2 // 几颗 ina219

	// IIC task
#define ADC_AMUX_SET		0xFFFF
#define ADC_AMUX_BITMAP		(1<<ADC_VCC_MCU)|(1<<ADC_VCC_3V3)|(1<<ADC_VCC_5V2)|(1<<ADC_VCC_16V)|(1<<ADC_DAC_OUT1)|(1<<ADC_DAC_OUT2)|(1<<ADC_DAC_OUT3)|(1<<ADC_LOCK_VCC)|(1<<ADC_SAMPLE_MCU)|(1<<ADC_SAMPLE_TEMP)
#define ADC_TEMP_SET		0x10000

#define SLA_219				0x4A	// 3颗219 的地址都是一样




#define BOARDER_LIMIT				15000


#define TEMPERATURE_NOT_AVALIABLE	-3000





//温控配置
#define TC_TEMP_UP    	1000
#define TC_TEMP_LOW      100
#define TC_TEMP_STOP	-400


// 可配置参数；
#define DUT_RAMP_UP_INTERVAL	1000	 // 100uS
#define CURRENT_REVIEW_INTERVAL	5000	 // 100uS

#define LIV_TEST_STEP_MAX		32       // 暂定待改；
#define MAX_DUT_CURRENT         220000   // 200mA
#define DAC0_CURRENT            217000   // 216000 // 1041000 // 252800 // 212000 // uA
#define DAC1_CURRENT            21300    // uA

#define LIV_GAIN_1K		        1000	//1000
#define LIV_GAIN_3p3K	        330	    //3300
#define LIV_GAIN_10K	        100	    //10000
#define LIV_GAIN_20K	        50	    //20000
#define LIV_GAIN_51K	        20	    //51000
#define LIV_GAIN_100K	        10	    //100000
#define LIV_GAIN_330K	        3	    //330000
#define LIV_GAIN_1000K	        1	    //1000000


// ASYNC_OP的四种状态
#define ASYNC_OP_STATE_IDLE				0
#define ASYNC_OP_STATE_ONGOING			1
#define ASYNC_OP_STATE_FINISHED_OK		2
#define ASYNC_OP_STATE_FINISHED_FAILED	3
#define ASYNC_OP_STATE_PHASE_FINISH_OK  4
//#define ASYNC_OP_STATE_WAIT_CONFIRM		4


// LED的三种命令
#define LED_CMD_OFF			0
#define LED_CMD_ON			1
#define LED_CMD_TOGGLE		2

// BITMAP的六种命令
#define BITMAP_CMD_SET_BITMAP						0
#define BITMAP_CMD_TURN_ON_ALL_CHANNELS				1
#define BITMAP_CMD_TURN_OFF_ALL_CHANNELS			2
#define BITMAP_CMD_TURN_ON_CHANNEL					3
#define BITMAP_CMD_TURN_OFF_CHANNEL					4
#define BITMAP_CMD_TURN_OFF_ALL_CHANNELS_EXCEPT_ONE	5

//
// 板子的独特定义放在这里
//





#define PD_BUF_AMP_1X		(0x00<<4)
#define PD_BUF_AMP_4X		(0x01<<4)


#define P_RESET_N_DISABLE()	do { FIO0SET_bit.P0_29 = 1; FIO0SET_bit.P0_30 = 1; } while (0)
#define P_RESET_N_ENABLE() 	do { FIO0CLR_bit.P0_29 = 1; FIO0CLR_bit.P0_30 = 1; } while (0)


// pcal6416
#define PCAL6416A_BUS_ID 	2
#define NR_PCAL6416A	 	6	// 32 dut, 2 PCAL6416A
#define PCAL6416A_ADDR_L	0x20
#define PCAL6416A_ADDR_H	0x21
#define ADDR_6416_P0		2
#define ADDR_6416_P1		3
#define ADDR_6416_DIR		6

#define I2C_MUX_SLA		    0x70


// adc

//#define BUS_ID0		0	
//#define BUS_ID1		1		// 电源模块和两个电流采样 219 在IIC1上
#define TEMPERATURE_NOT_AVALIABLE	-3000
//#define SLA_ADC_219			0x48

//#define BUS_ID_BUF		0		// PD_BUF 的219 在IIC0上

#define SLA_219_RS1			0x48	// 1 ohm 采样电阻，200mA 挡位
#define SLA_219_RS10		0x49	// 10 ohm 采样电阻，20mA 挡位
#define SLA_219_RS_DIFF		0x4B	// 电流差分输入
#define SLA_219_BUF			0x4A

#define SOLIWARE_MODEL_NAME			"TESTER-MOTOR-CONTROLLER"


#define DUT_POWER_ON_OVERLOAD_DETECTION_DURATION	1000		// dut上电后，会有时间持续读取电流，用于判定短路
#define DUT_POWER_ON_I2C_DELAY_100uS				10000		// dut上电之后，等待一段时间再访问i2c
#define DUT_POWER_ON_I2C_RETRY_COUNT				3			// 插入上电检测阶段，访问I2C失败则重试


#define TRAY_EEPROM_CMD_OFFSET				0x30

#define ASYNC_OP_ITH_NR_IBIAS	              81		// 81可以保证mmap不超过16KB
#define ASYNC_OP_ITH_ERROR_MAX	              20


// power module
#define INPUT_24V_THRESHOLD_LOW		       16000
#define INPUT_24V_THRESHOLD_HIGH	       28000
#define OUTPUT_V_THRESHOLD_OV			     800	// 输出过压保护；
#define OUTPUT_I_THRESHOLD_HIGH		       15000


// temp_control.c

#define TC_RESET 0
#define TC_SET 	1
#define TC_UPDATE	2 	



// COC 类型，共阴 或者 共阳
#define DUT_TYPE_COMM_NEG	 		0
#define DUT_REVERSE_LEAKAGE_MODE	1

//step motor
#define NR_MOTORS		                3
#define MOTOR_DRIVER_MICRO_STEP		    8                            

#define MOTOR_0_DRIVER_MICRO_STEP		32               // Z轴方向电机微步
#define MOTOR_1_DRIVER_MICRO_STEP		32                


#define ACCEL_START_INTERVAL	    8000	             // 2000	1875 800 // 3	//10	//10

#define STEP_MOTOR_TOTAL_STEP_X	    250000 * MOTOR_DRIVER_MICRO_STEP // 600000 STEP @ 32 mciro step
#define STEP_MOTOR_TOTAL_STEP_Y	    10000 * MOTOR_DRIVER_MICRO_STEP  // 60000// 105000 // 122383
#define STEP_MOTOR_TOTAL_STEP_Z	    15300 * MOTOR_DRIVER_MICRO_STEP  // 70000 // 7200 // 1223 Z轴没有top开关，必须定义一个比较合适的值
#define BOARDER_MARGIN	            6000

#define STEP_MOTOR_MAX_CURRENT	        5080    // 电路限制；
#define STEP_MOTOR_CURRENT_HIGH_mA      3000    // 4000
#define STEP_MOTOR_CURRENT_mA	        3000
#define STEP_MOTOR_CURRENT_LOW_mA	    800

//MOTOR0 Z 
//MOTOR1 tray
#define MOTOR0_MAX_STEP	         2000	                    //1800	 
#define MOTOR0_MAX_V	         (MOTOR0_MAX_STEP * 300)    //300 最大速度150有机会达到‘380 350	//130
#define MOTOR1_MAX_STEP	         1000	                    //5000
#define MOTOR1_MAX_V	         (MOTOR1_MAX_STEP * 2500)   //8000000   400 380 //150 
#define MOTOR2_MAX_STEP	         3000	                    //1500	//5000
#define MOTOR2_MAX_V	         (MOTOR2_MAX_STEP * 250)	//200



// 电机默认方向；
#define MOTOR0_DIR      0   // z轴方向
#define MOTOR1_DIR	    0   // y轴方向
#define MOTOR2_DIR	    1


#define NR_STEP_MOTOR	3


#define STEP_MOTOR_TIMEOUT_100US 	10 * 10000	// 超时 10s

//
// PLL settings
// 设置M和N值
// M = 75，N = 2
// 需要设定为100M Hz
// Fcco取300M = 100M x 3
// Fin使用内部的4M的振荡器
// 那么最后Fcco输出的频率还应该被分频器（CCLKCFG）除以3
//
#define _F_OSC				4000000
#define _CCLK_DIV_VALUE		3
#define _F_CCLK				100000000		// Frequence of core clock
#define _F_CCO				300000000
#define _F_PCLK				(_F_CCLK / 4)	// 25Mhz, 40ns; Frequence of peripheral clock
#define _F_ADC_CLK			12000000        // 12MHZ
#define _F_ADC_CLKDIV		_F_PCLK/_F_ADC_CLK   
#define _PLL_N_VALUE		1                      // 使用内部rc时为2，这里用于外部晶振
#define _PLL_M_VALUE		6                      // 使用内部rc时75


// 框架头文件放在这
#include "LPC1700.h"
#include "vicControl.h"
#include "IAP.h"
#include "LPC1700PinCfg.h"


#endif // _CONFIG_H_

