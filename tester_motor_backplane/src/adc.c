#include "config.h"
#include "backplane.h"
#include "lpc1700.h"

#include <stdarg.h>



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
// Configuration Register address： 0x00；
// shunt Voltage Register address： 0x01；
// Bus Voltage Register address： 	0x02；
// Current Register address： 		0x04
// Calibration Register address： 	0x05

/*****************************************
模块通过AD0.2端口采集监控24V输入电源
iic2, 0x4A, 模拟采集，shunt模式，16倍衰减
*****************************************/



//  多路电压测量mux0；        shunt 模式，输入电压最高3.3V，衰减16倍；即206mV， Triggered
#define SLA_219_MUX0            0x4A
#define SLA_219_CONFIG0		((0 << 13) | (3 << 11) | (9 << 7) | (0x3 << 3) | 1)
#define SLA_219_CALIBRATION0	0xaaa

#define SLA_219_MUX1            0x4F
#define SLA_219_CONFIG1		((0 << 13) | (1 << 11) | (0xC << 7) | (0xC << 3) | 1)
#define SLA_219_CALIBRATION1	0x1000 // 0xaaa



// 关于温度测量
// power_module是热敏电阻LMT84，219读到的是器件输出的电压，可以直接用来查表
// tray是热敏电阻NTCG10，电路为VCC3--R_10K--V219--R_NTCG10--GND，219读到的值是热敏电阻的分压
// 为了取到0.1度的精度，取16个mV相加得到sum_of_16，再(>>4)在表中查到整数部分，小数部分再配合与左边的值来计算
 short mV_to_deci_Celsius_LMT84(unsigned int mVx16)
{
#define TEMP_COUNT	201
#define TEMP_LOW	(-500)
#define TEMP_HIGH	1500
#define TEMP_STEP	10
	// -50~150，精度为1度
	const static unsigned short tab[] = {
		1299, 1294, 1289, 1284, 1278, 1273, 1268, 1263, 1257, 1252, // -50
		1247, 1242, 1236, 1231, 1226, 1221, 1215, 1210, 1205, 1200,
		1194, 1189, 1184, 1178, 1173, 1168, 1162, 1157, 1152, 1146, // -30
		1141, 1136, 1130, 1125, 1120, 1114, 1109, 1104, 1098, 1093,
		1088, 1082, 1077, 1072, 1066, 1061, 1055, 1050, 1044, 1039, // -10
		1034, 1028, 1023, 1017, 1012, 1007, 1001,  996,  990,  985,
		 980,  974,  969,  963,  958,  952,  947,  941,  936,  931, // 10
		 925,  920,  914,  909,  903,  898,  892,  887,  882,  876,
		 871,  865,  860,  854,  849,  843,  838,  832,  827,  821, // 30
		 816,  810,  804,  799,  793,  788,  782,  777,  771,  766,
		 760,  754,  749,  743,  738,  732,  726,  721,  715,  710, // 50
		 704,  698,  693,  687,  681,  676,  670,  664,  659,  653,
		 647,  642,  636,  630,  625,  619,  613,  608,  602,  596, // 70
		 591,  585,  579,  574,  568,  562,  557,  551,  545,  539,
		 534,  528,  522,  517,  511,  505,  499,  494,  488,  482, // 90
		 476,  471,  465,  459,  453,  448,  442,  436,  430,  425,
		 419,  413,  407,  401,  396,  390,  384,  378,  372,  367, // 110
		 361,  355,  349,  343,  337,  332,  326,  320,  314,  308,
		 302,  296,  291,  285,  279,  273,  267,  261,  255,  249, // 130
		 243,  237,  231,  225,  219,  213,  207,  201,  195,  189,
		 183														// 150
	};

	unsigned int mV = mVx16 >> 4;
	
	int i;

	if ((mV > tab[0]) || (mV < tab[TEMP_COUNT - 1])) return TEMPERATURE_NOT_AVALIABLE;

	for (i = 0; i < TEMP_COUNT; i++) {
		if (mV >= tab[i])
			break;
	}
	// 循环完了以后，mV会落在区间 ( tab[i-1]，tab[i] ]，所以0需要单独处理
	if (i == 0) {
		return TEMP_LOW;
	} else {
		int temp = TEMP_STEP * i + TEMP_LOW;
		temp -= (TEMP_STEP * (mVx16 - (tab[i] << 4))) / ((tab[i - 1] - tab[i]) << 4);
		return (short)temp;
	}

#undef TEMP_COUNT
#undef TEMP_LOW
#undef TEMP_HIGH
#undef TEMP_STEP
}

short uV10_to_deci_Celsius_LMT84(unsigned int uV10)
{
#define TEMP_COUNT	201
#define TEMP_LOW	(-500)
#define TEMP_HIGH	1500
#define TEMP_STEP	10
	// -50~150，精度为1度
	const static unsigned short tab[] = {
		1299, 1294, 1289, 1284, 1278, 1273, 1268, 1263, 1257, 1252, // -50
		1247, 1242, 1236, 1231, 1226, 1221, 1215, 1210, 1205, 1200,
		1194, 1189, 1184, 1178, 1173, 1168, 1162, 1157, 1152, 1146, // -30
		1141, 1136, 1130, 1125, 1120, 1114, 1109, 1104, 1098, 1093,
		1088, 1082, 1077, 1072, 1066, 1061, 1055, 1050, 1044, 1039, // -10
		1034, 1028, 1023, 1017, 1012, 1007, 1001,  996,  990,  985,
		 980,  974,  969,  963,  958,  952,  947,  941,  936,  931, // 10
		 925,  920,  914,  909,  903,  898,  892,  887,  882,  876,
		 871,  865,  860,  854,  849,  843,  838,  832,  827,  821, // 30
		 816,  810,  804,  799,  793,  788,  782,  777,  771,  766,
		 760,  754,  749,  743,  738,  732,  726,  721,  715,  710, // 50
		 704,  698,  693,  687,  681,  676,  670,  664,  659,  653,
		 647,  642,  636,  630,  625,  619,  613,  608,  602,  596, // 70
		 591,  585,  579,  574,  568,  562,  557,  551,  545,  539,
		 534,  528,  522,  517,  511,  505,  499,  494,  488,  482, // 90
		 476,  471,  465,  459,  453,  448,  442,  436,  430,  425,
		 419,  413,  407,  401,  396,  390,  384,  378,  372,  367, // 110
		 361,  355,  349,  343,  337,  332,  326,  320,  314,  308,
		 302,  296,  291,  285,  279,  273,  267,  261,  255,  249, // 130
		 243,  237,  231,  225,  219,  213,  207,  201,  195,  189,
		 183														// 150
	};

	unsigned int mV = uV10 / 100;
	
	int i;

	if ((mV > tab[0]) || (mV < tab[TEMP_COUNT - 1])) return TEMPERATURE_NOT_AVALIABLE;

	for (i = 0; i < TEMP_COUNT; i++) {
		if (mV >= tab[i])
			break;
	}
	// 循环完了以后，mV会落在区间 ( tab[i-1]，tab[i] ]，所以0需要单独处理
	if (i == 0) {
		return TEMP_LOW;
	} else {
		int temp = TEMP_STEP * i + TEMP_LOW;
		int tempa= tab[i - 1] * 100;
		int tempb= tab[i] * 100;
			
		temp -= (TEMP_STEP * (uV10 - tempb)) / (tempa - tempb);
		return (short)temp;
	}

#undef TEMP_COUNT
#undef TEMP_LOW
#undef TEMP_HIGH
#undef TEMP_STEP
}



#if 0
static short uV10_to_deci_Celsius_NTCG10(unsigned int uV10)
{
#define TEMP_COUNT	34
#define TEMP_LOW	(-400)
#define TEMP_HIGH	1250
#define TEMP_STEP	50
	// -40~125，精度为5度
	const static unsigned short tab[] = {
		50140, 49378, 48448, 47332, 46012, 44480, 42733, 40781, 38637, 36336,   // -40
		33916, 31423, 28898, 26400, 23963, 21631, 19446, 17388, 15512, 13804,   // 10
		12247, 10862,  9627,  8505,  7556,  6686,  5950,  5275,  4691,  4181,   // 60
		 3729,  3334,  2984,  2677                                              // 110
	};
	int i, mVx16;

	mVx16 = (uV10<<4) / 100;

	if ((mVx16 > tab[0]) || (mVx16 < tab[TEMP_COUNT - 1])) return TEMPERATURE_NOT_AVALIABLE;

	for (i = 0; i < TEMP_COUNT; i++) {
		if (mVx16 >= tab[i])
			break;
	}
	// 循环完了以后，会落在区间 ( tab[i-1]，tab[i] ]，所以0需要单独处理
	if (i == 0) {
		return TEMP_LOW;
	} else {
		int temp = TEMP_STEP * i + TEMP_LOW;
		temp -= (TEMP_STEP * (mVx16 - tab[i])) / (tab[i - 1] - tab[i]);
		return (short)temp;
	}
#undef TEMP_COUNT
#undef TEMP_LOW
#undef TEMP_HIGH
#undef TEMP_STEP
}


static short mV_to_deci_Celsius_NTCG10(unsigned int mVx16)
{
#define TEMP_COUNT	34
#define TEMP_LOW	(-400)
#define TEMP_HIGH	1250
#define TEMP_STEP	50
	// -40~125，精度为5度
	const static unsigned short tab[] = {
		50140, 49378, 48448, 47332, 46012, 44480, 42733, 40781, 38637, 36336,   // -40
		33916, 31423, 28898, 26400, 23963, 21631, 19446, 17388, 15512, 13804,   // 10
		12247, 10862,  9627,  8505,  7556,  6686,  5950,  5275,  4691,  4181,   // 60
		 3729,  3334,  2984,  2677                                              // 110
	};
	int i;

	if ((mVx16 > tab[0]) || (mVx16 < tab[TEMP_COUNT - 1])) return TEMPERATURE_NOT_AVALIABLE;

	for (i = 0; i < TEMP_COUNT; i++) {
		if (mVx16 >= tab[i])
			break;
	}
	// 循环完了以后，会落在区间 ( tab[i-1]，tab[i] ]，所以0需要单独处理
	if (i == 0) {
		return TEMP_LOW;
	} else {
		int temp = TEMP_STEP * i + TEMP_LOW;
		temp -= (TEMP_STEP * (mVx16 - tab[i])) / (tab[i - 1] - tab[i]);
		return (short)temp;
	}
#undef TEMP_COUNT
#undef TEMP_LOW
#undef TEMP_HIGH
#undef TEMP_STEP
}
#endif

static short deci_Celsius_Ktype_thermocouple_to_uV(short deci_Celsius)
{
#define KTEMP_COUNT	261
#define KTEMP_LOW	(-600)
#define KTEMP_HIGH	2000
#define KTEMP_STEP	10
		// -50~150，精度为1度
		const static short tab[] = {
			-2243, -2208, -2173, -2138, -2103, -2067, -2032, -1996, -1961, -1925,	// -60~-51
			-1889, -1854, -1818, -1782, -1745, -1709, -1673, -1637, -1600, -1564,	// -50~-41
			-1527, -1490, -1453, -1417, -1380, -1343, -1305, -1268, -1231, -1194,	// -40~-31
			-1156, -1119, -1081, -1043, -1006,	-968,  -930,  -892,  -854,	-816,	// -30~-21
			 -778,	-739,  -701,  -663,  -624,	-586,  -547,  -508,  -407,	-431,	// -20~-11
			 -392,	-353,  -314,  -275,  -236,	-197,  -157,  -118,   -79,	 -39,	// -10~-1
				0,	  39,	 79,   119,   158,	 198,	238,   277,   317,	 237,	//	0~9
			  397,	 437,	477,   517,   557,	 597,	637,   677,   718,	 758,	 // 10~19
			  798,	 838,	879,   919,   960,	1000,  1041,  1081,  1122,	1163,	 // 20
			 1203,	1244,  1285,  1326,  1366,	1407,  1448,  1489,  1540,	1571,	// 30
			 1612,	1653,  1694,  1735,  1776,	1817,  1858,  1899,  1941,	1982,	// 40
			 2023,	2064,  2106,  2147,  2188,	2230,  2271,  2312,  2354,	2395,	// 50
			 2436,	2478,  2519,  2561,  2602,	2644,  2685,  2727,  2768,	2810,	// 60~69
			 2851,	2893,  2934,  2976,  3017,	3059,  3100,  3142,  3184,	3225,	// 70
			 3267,	3308,  3350,  3391,  3433,	3474,  3516,  3557,  3599,	3640,	// 80
			 3682,	3723,  3765,  3806,  3848,	3889,  3931,  3972,  4013,	4055,	// 90
			 4096,	4138,  4179,  4220,  4262,	4303,  4344,  4385,  4427,	4468,	// 100
			 4509,	4550,  4591,  4633,  4674,	4715,  4756,  4797,  4838,	4879,	// 110
			 4920,	4961,  5002,  5043,  5084,	5124,  5165,  5206,  5247,	5288,	// 120
			 5328,	5369,  5410,  5450,  5491,	5532,  5572,  5613,  5653,	5694,	// 130
			 5735,	5775,  5815,  5856,  5896,	5937,  5977,  6017,  6058,	6098,	// 140
			 6138,	6179,  6219,  6259,  6299,	6339,  6380,  6420,  6460,	6500,	// 150
			 6540,	6580,  6620,  6660,  6701,	6741,  6781,  6821,  6861,	6901,	// 160
			 6941,	6981,  7021,  7060,  7100,	7140,  7180,  7220,  7260,	7300,	// 170
			 7340,	7380,  7420,  7460,  7500,	7540,  7579,  7619,  7659,	7699,	// 180
			 7739,	7779,  7819,  7859,  7899,	7939,  7979,  8109,  8059,	8099,	// 190
			 8138
		};
	short degree = (deci_Celsius - KTEMP_LOW) / 10;
	if (degree < 0) return tab[0];
	else if (degree > KTEMP_COUNT) return tab[KTEMP_COUNT - 1];
	
	short uV = tab[degree];
	uV += (deci_Celsius - (degree * 10 + KTEMP_LOW)) * (tab[degree + 1] - tab[degree]) / 10;
	debug("deci_Celsius=%d, degree=%d, tab=%d, uV=%d\n", deci_Celsius, degree, tab[degree], uV);
	return uV;
		
#undef KTEMP_COUNT
#undef KTEMP_LOW
#undef KTEMP_HIGH
#undef KTEMP_STEP

	
}

static short uV_to_deci_Celsius_Ktype_thermocouple(short uV)
{
#define KTEMP_COUNT	261
#define KTEMP_LOW	(-600)
#define KTEMP_HIGH	2000
#define KTEMP_STEP	10
	// -50~150，精度为1度
	const static short tab[] = {
		-2243, -2208, -2173, -2138, -2103, -2067, -2032, -1996, -1961, -1925,   // -60~-51
		-1889, -1854, -1818, -1782, -1745, -1709, -1673, -1637, -1600, -1564,   // -50~-41
		-1527, -1490, -1453, -1417, -1380, -1343, -1305, -1268, -1231, -1194,   // -40~-31
		-1156, -1119, -1081, -1043, -1006,  -968,  -930,  -892,  -854,  -816,   // -30~-21
		 -778,  -739,  -701,  -663,  -624,  -586,  -547,  -508,  -407,  -431,   // -20~-11
		 -392,  -353,  -314,  -275,  -236,  -197,  -157,  -118,   -79,   -39,   // -10~-1
			0, 	  39, 	 79,   119,   158,   198,   238,   277,   317,   237,   //  0~9
		  397,   437,   477,   517,   557,   597,   637,   677,   718,   758,    // 10~19
		  798,   838,   879,   919,   960,  1000,  1041,  1081,  1122,  1163,    // 20
		 1203,  1244,  1285,  1326,  1366,  1407,  1448,  1489,  1540,  1571,	// 30
		 1612,  1653,  1694,  1735,  1776,  1817,  1858,  1899,  1941,  1982,	// 40
		 2023,  2064,  2106,  2147,  2188,  2230,  2271,  2312,  2354,  2395,	// 50
		 2436,  2478,  2519,  2561,  2602,  2644,  2685,  2727,  2768,  2810,   // 60~69
		 2851,  2893,  2934,  2976,  3017,  3059,  3100,  3142,  3184,  3225,	// 70
		 3267,  3308,  3350,  3391,  3433,  3474,  3516,  3557,  3599,  3640,	// 80
		 3682,  3723,  3765,  3806,  3848,  3889,  3931,  3972,  4013,  4055,	// 90
		 4096,  4138,  4179,  4220,  4262,  4303,  4344,  4385,  4427,  4468,	// 100
		 4509,  4550,  4591,  4633,  4674,  4715,  4756,  4797,  4838,  4879,	// 110
		 4920,  4961,  5002,  5043,  5084,  5124,  5165,  5206,  5247,  5288,	// 120
		 5328,  5369,  5410,  5450,  5491,  5532,  5572,  5613,  5653,  5694,	// 130
		 5735,  5775,  5815,  5856,  5896,  5937,  5977,  6017,  6058,  6098,  	// 140
		 6138,  6179,  6219,  6259,  6299,  6339,  6380,  6420,  6460,  6500,	// 150
		 6540,  6580,  6620,  6660,  6701,  6741,  6781,  6821,  6861,  6901,	// 160
		 6941,  6981,  7021,  7060,  7100,  7140,  7180,  7220,  7260,  7300,	// 170
		 7340,  7380,  7420,  7460,  7500,  7540,  7579,  7619,  7659,  7699,	// 180
		 7739,  7779,  7819,  7859,  7899,  7939,  7979,  8109,  8059,  8099,	// 190
		 8138
	};

	
	int i;

	if ((uV < tab[0]) || (uV > tab[KTEMP_COUNT - 1])) return TEMPERATURE_NOT_AVALIABLE;

	for (i = 0; i < KTEMP_COUNT; i++) {
		if (uV <= tab[i])
			break;
	}
	
	// 循环完了以后，uV会落在区间 ( tab[i-1]，tab[i] ]，所以0需要单独处理
	if (i == 0) {
		return KTEMP_LOW;
	} else {
		int temp = KTEMP_STEP * i + KTEMP_LOW;
		int tempa= tab[i - 1];
		int tempb= tab[i];	
		temp -= (KTEMP_STEP * (tempb -uV)) / (tempb - tempa);
		return (short)temp;
	}

#undef KTEMP_COUNT
#undef KTEMP_LOW
#undef KTEMP_HIGH
#undef KTEMP_STEP
}


// 每一步都要等到i2c_session_is_finished才依次到下一步，如果出错就从0开始
enum {
	STEP_IDLE = 0,									// 初始/出错/循环完成会到这里
	STEP_ADC_219_CHECK_READY,		// 这里会多次进入，等219 ready
	STEP_ADC_219_READ_CURRNET,
	STEP_ADC_219_READ_SHUNT_VOLT,
	STEP_MAX
};

#define MUX5_GAIN	16
/***********************************
将adc量出来的raw data 转换成有意义的值；
***********************************/
void adc_to_value(unsigned char _adc_idx)
{
	int dac_10uv = (int)g_data_adc[ADC_ID_AMUX].value;

	dac_10uv <<= 4; //mux5 的gain 是 16；
//	printk("[%d]shunt_10uV=%d\n", _adc_idx, dac_10uv);

	switch (_adc_idx){
		case ADC_VCC_5V2:
			g_data_runtime.mon_data[_adc_idx] = dac_10uv * 2 / 100; // 衰减 2x
			break;
		case ADC_VCC_16V:
			g_data_runtime.mon_data[_adc_idx] = dac_10uv * 4 / 100; // 衰减 2x
			break;
		
		case ADC_VCC_3V3:
		case ADC_VCC_MCU:
		case ADC_PCB_VER:
		case ADC_LOCK_VCC:
			g_data_runtime.mon_data[_adc_idx] = dac_10uv / 100; // 电压，mV
			break;
		
		case ADC_MGND:
			g_data_runtime.mon_data[_adc_idx] =  dac_10uv ; // mA, 采样电阻100 mOHM；
			break;

		case ADC_DAC_OUT1:
		case ADC_DAC_OUT2:
		case ADC_DAC_OUT3:
			g_data_runtime.mon_data[_adc_idx] =  dac_10uv * 2 / 100 ;
			break;

		case ADC_SAMPLE_TEMP:
			g_data_runtime.mon_data[_adc_idx] =  uV10_to_deci_Celsius_LMT84(dac_10uv);
			break;
		default:
			g_data_runtime.mon_data[_adc_idx] = dac_10uv / 100; // 电压，mV
			break;
	}

}


// -1 完成但错误，或超时，0 未完成，1 完成且正确
int adc_proc_mux()
{
	adc_info_t *d = g_data_adc + ADC_ID_AMUX;
	static unsigned char _tick;

	switch (d->_step_idx) {
		case STEP_IDLE:
			ina219_exec_trigger(&d->ina219_adc);
			d->_step_idx = STEP_ADC_219_CHECK_READY;
			_tick = 0xff;
			return 0;

		case STEP_ADC_219_CHECK_READY:
			if (ina219_data_is_ready(&d->ina219_adc)) {
				ina219_exec_read_shunt_volt(&d->ina219_adc);
				d->_step_idx = STEP_ADC_219_READ_SHUNT_VOLT;
			} else {
				_tick--;
				if (0 == _tick) {
					d->_step_idx =STEP_IDLE;
					printk("adc_proc_mux timeout\n" );
					return -1;
				}
				ina219_exec_check_ready_and_read_voltage(&d->ina219_adc);
			}
			return 0;
			
		case STEP_ADC_219_READ_SHUNT_VOLT:
			d->value = ina219_data_get_shunt_volt(&d->ina219_adc);
			d->_step_idx = STEP_IDLE;
			return 1;

		default:
			d->_step_idx = STEP_IDLE;
			printk("adc_proc_mux err step_idx=%d\n", d->_step_idx);
			return -1;
		}
}

void adc_to_temp()
{
	int dac_uv = g_data_adc[ADC_ID_TEMP].value * 10;
	short base_uv = deci_Celsius_Ktype_thermocouple_to_uV(g_data_runtime.mon_data[ADC_SAMPLE_TEMP]);

	dac_uv >>= 4; //mux5 的gain 是 16；
//	dac_uv += base_uv;

	g_data_runtime.k_temp = uV_to_deci_Celsius_Ktype_thermocouple(dac_uv + base_uv);
	debug("temp=%d/%d/base=%d/k=%d\n", g_data_runtime.k_temp, dac_uv + base_uv, base_uv, dac_uv);
	
}


int adc_proc_k_temp()
{
	adc_info_t *d = g_data_adc + ADC_ID_TEMP;
	static unsigned char _tick;

	switch (d->_step_idx) {
		case STEP_IDLE:
			ina219_exec_trigger(&d->ina219_adc);
			d->_step_idx =STEP_ADC_219_CHECK_READY;
			_tick = 0xff;
			return 0;

		case STEP_ADC_219_CHECK_READY:
			if (ina219_data_is_ready(&d->ina219_adc)) {
				ina219_exec_read_shunt_volt(&d->ina219_adc);
				d->_step_idx = STEP_ADC_219_READ_SHUNT_VOLT;
			} else {
				_tick--;
				if (0 ==  _tick) {
					d->_step_idx =STEP_IDLE;
					printk("adc_proc_mux timeout\n" );
					return -1;
				}
				ina219_exec_check_ready_and_read_voltage(&d->ina219_adc);
			}
			return 0;
			
		case STEP_ADC_219_READ_SHUNT_VOLT:
			d->value = ina219_data_get_shunt_volt(&d->ina219_adc);
			debug("uV=%d/%x\n", d->value, d->ina219_adc.output_shunt);
			d->_step_idx =STEP_IDLE;
			return 1;

		default:
			d->_step_idx =STEP_IDLE;
			printk("adc_proc_mux err step_idx=%d\n", d->_step_idx);
			return -1;
		}
}


// 每一步都要等到i2c_session_is_finished才依次到下一步，如果出错就从0开始
enum {
	STEP_ADC_IDLE = 0,									// 初始/出错/循环完成会到这里
	STEP_ADC_219_TRIG1,		                            // 触发低量程的采样；
	STEP_ADC_219_CHECK_READY0,
	STEP_ADC_219_READ_CURRNET0,
	STEP_ADC_219_READ_VOLTAGE,	
	STEP_ADC_MAX
};


void adc_init()
{

// 设置MCU 的 adc 输出作为外接DAC的 Vref；
	for (unsigned char i = 0; i < NR_ADC_INA219; i++)
		memset(&g_data_adc[i], 0, sizeof(g_data_adc[0]));


// 多路电压测量mux0；         shunt 模式，输入电压最高3.3V，衰减16倍；即206mV，
	ina219_init(&g_data_adc[ADC_ID_AMUX].ina219_adc, BUS_ID0, SLA_219_MUX0, SLA_219_CONFIG0, 1, SLA_219_CALIBRATION0);
	ina219_init(&g_data_adc[ADC_ID_TEMP].ina219_adc, BUS_ID0, SLA_219_MUX1, SLA_219_CONFIG1, 1, SLA_219_CALIBRATION1);
	i2c_finish_session_forcibly(BUS_ID0);


// mcu 的 AD 用作 24V 掉电监测；	
// enable AD0.2; PCLK 是 25M，实际可以填2或者1； Burst mode;	Turn ON; 
	AD0CR_bit.SEL = 4 ;
//	AD0CR_bit.CLKDIV = _F_ADC_CLKDIV ;
	AD0CR_bit.CLKDIV = 3 ;
	AD0CR_bit.BURST = 1 ;
	AD0CR_bit.PDN = 1 ;
}

void adc_print()
{
	printk("ADC:\n");
	printk("VCC: MCU=%dmV ", g_data_runtime.mon_data[ADC_VCC_MCU]);
	printk("3V=%dmV ", g_data_runtime.mon_data[ADC_VCC_3V3]);
	printk("5V=%dmV ", g_data_runtime.mon_data[ADC_VCC_5V2]);
	printk("EML=%dmV ", g_data_runtime.mon_data[ADC_VCC_16V]);
	printk("PCB_VER=%dmV ", g_data_runtime.mon_data[ADC_PCB_VER]);
	printk("M_GND=%dmV ", g_data_runtime.mon_data[ADC_MGND]);
	printk("DAC1=%d ", g_data_runtime.mon_data[ADC_DAC_OUT1]);
	printk("DAC2=%d ", g_data_runtime.mon_data[ADC_DAC_OUT2]);
	printk("DAC3=%d ", g_data_runtime.mon_data[ADC_DAC_OUT3]);
	printk("\n");
	printk("temp=%d/%d\n", g_data_runtime.k_temp, g_data_adc[ADC_ID_TEMP].value);
	
}





