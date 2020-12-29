#include "config.h"
#include "backplane.h"


void gpio_init()
{
	FIO3DIR_bit.P3_25 = 1;	// cpu onboard led
	FIO3DIR_bit.P3_26 = 1;	// led red
	FIO1DIR_bit.P1_18 = 1;	// led orange
	FIO1DIR_bit.P1_21 = 1;	// led white
	FIO1DIR_bit.P1_22 = 1;	// led blue

	FIO1DIR_bit.P1_0 = 1;	// rs485 tx enable
	FIO0DIR_bit.P0_1 = 1;	// UART2 tx en bit
	FIO0DIR_bit.P0_4 = 1;	// UART3 tx en bit
	FIO1SET_bit.P1_0 = 1;	// 都初始化成tx_en=1
	FIO0SET_bit.P0_1 = 1;
	FIO0SET_bit.P0_4 = 1;
	FIO0DIR_bit.P0_0 = 1;	// main_controller enable
	
// step motor XYZ
	FIO1DIR_bit.P1_17 = 1;	// step motor X dir
	FIO1DIR_bit.P1_16 = 1;	// step motor X pulse
	FIO1DIR_bit.P1_15 = 0;	// 升降机原点// input, step motor X position top 
	FIO1DIR_bit.P1_14 = 0;	// 升降机上限 // input, step motor X position  bottom	
	FIO0DIR_bit.P0_7 = 1;   // step motor X enable
	FIO0SET_bit.P0_7 = 1;

	FIO1DIR_bit.P1_10 = 1;	// step motor Y dir
	FIO1DIR_bit.P1_9 = 1;	// step motor Y pulse
	FIO1DIR_bit.P1_8 = 0;	// 取盘限位 // input, step motor Y position top 
	FIO1DIR_bit.P1_4 = 0;	// input, step motor Y position bottom 
	FIO1DIR_bit.P1_1 = 1;   // step motor Y enable
	FIO1SET_bit.P1_1 = 1;

	FIO1DIR_bit.P1_25 = 1;	// step motor Z dir
	FIO1DIR_bit.P1_26 = 1;	// step motor Z pulse
	FIO1DIR_bit.P1_27 = 0;	// input, step motor Z position top 
	FIO1DIR_bit.P1_28 = 0;	// 升降机下限// input, step motor Z position bottom 
	FIO1DIR_bit.P1_29 = 1;  // step motor Z enable
	FIO1SET_bit.P1_29 = 1;

	FIO2DIR_bit.P2_2 = 1;   // MS_X_0
	FIO2DIR_bit.P2_3 = 1;   // MS_X_1
	FIO2DIR_bit.P2_4 = 1;   // MS_X_2
	FIO2DIR_bit.P2_5 = 1;   // MS_Y_0
	FIO2DIR_bit.P2_6 = 1;   // MS_Y_1
	FIO2DIR_bit.P2_7 = 1;   // MS_Y_2
	FIO2DIR_bit.P2_13 = 1;  // MS_Z_0
	FIO2DIR_bit.P2_12 = 1;  // MS_Z_1
	FIO2DIR_bit.P2_11 = 1;  // MS_Z_2

	FIO0DIR_bit.P0_17 = 0;	// Lock FV in
	FIO0DIR_bit.P0_18 = 0;	// 光电开关     Door Lock Enable
	FIO0DIR_bit.P0_8 = 1;  	// 电磁铁      FAN Enable
	FIO0CLR_bit.P0_8 = 1;

// analog MUX1 
	FIO0DIR_bit.P0_24 = 1;	// _MUX1_A
	FIO0DIR_bit.P0_23 = 1;	// _MUX1_B
	FIO1DIR_bit.P1_31 = 1;	// _MUX1_C
	FIO1DIR_bit.P1_30 = 1;	// _MUX1_D

	// 如果有标志是在升级固件，保持主控板开电
	if (flash_is_keeping_main_controller_on()) {
#ifdef boot_loader
		printk("keep_on_flag in boot_loader\n");
#else
		printk("keep_on_flag in firmware\n");
#endif
		gpio_set_main_controller_onoff(1);
#ifndef boot_loader		// 在boot_loader中不碰这个标记
		flash_clear_keeping_main_controller_on_flag();
#endif
	} else {
		gpio_set_main_controller_onoff(1);
	}

//	gpio_set_main_controller_onoff(1);
	gpio_board_led(LED_CMD_OFF);
	gpio_led(0, LED_CMD_OFF);
	gpio_led(1, LED_CMD_OFF);
	gpio_led(2, LED_CMD_OFF);
	gpio_led(3, LED_CMD_OFF);
	gpio_rs485_tx_onoff(1);
	

//	FIO0DIR_bit.P0_25 = 0; // input, 24V 供电输入，衰减11x
	DACR = (1023)<<6 ;//参考电压是3V，一般我们输出~3000mV；

}

void gpio_led(int led_idx, int led_cmd)
{
	static unsigned char led_is_on[4];		// 低电平点亮
	switch (led_idx) {
		case 0:		// P1_18, orange led
			switch (led_cmd) {
				case LED_CMD_OFF:
					led_is_on[led_idx] = 0;
					FIO1SET_bit.P1_18 = 1;
					break;
				case LED_CMD_ON:
					led_is_on[led_idx] = 1;
					FIO1CLR_bit.P1_18 = 1;
					break;
				case LED_CMD_TOGGLE:
					if (led_is_on[led_idx]) {
						led_is_on[led_idx] = 0;
						FIO1SET_bit.P1_18 = 1;
					} else {
						led_is_on[led_idx] = 1;
						FIO1CLR_bit.P1_18 = 1;
					}
					break;
			}
			break;

		case 1:		// P1_22, blue led
			switch (led_cmd) {
				case LED_CMD_OFF:
					led_is_on[led_idx] = 0;
					FIO1SET_bit.P1_22 = 1;
					break;
				case LED_CMD_ON:
					led_is_on[led_idx] = 1;
					FIO1CLR_bit.P1_22 = 1;
					break;
				case LED_CMD_TOGGLE:
					if (led_is_on[led_idx]) {
						led_is_on[led_idx] = 0;
						FIO1SET_bit.P1_22 = 1;
					} else {
						led_is_on[led_idx] = 1;
						FIO1CLR_bit.P1_22 = 1;
					}
					break;
			}
			break;

		case 2:		// P1_21, white led
			switch (led_cmd) {
				case LED_CMD_OFF:
					led_is_on[led_idx] = 0;
					FIO1SET_bit.P1_21 = 1;
					break;
				case LED_CMD_ON:
					led_is_on[led_idx] = 1;
					FIO1CLR_bit.P1_21 = 1;
					break;
				case LED_CMD_TOGGLE:
					if (led_is_on[led_idx]) {
						led_is_on[led_idx] = 0;
						FIO1SET_bit.P1_21 = 1;
					} else {
						led_is_on[led_idx] = 1;
						FIO1CLR_bit.P1_21 = 1;
					}
					break;
			}
			break;

		case 3:		// P3_26, red led
			switch (led_cmd) {
				case LED_CMD_OFF:
					led_is_on[led_idx] = 0;
					FIO3SET_bit.P3_26 = 1;
					break;
				case LED_CMD_ON:
					led_is_on[led_idx] = 1;
					FIO3CLR_bit.P3_26 = 1;
					break;
				case LED_CMD_TOGGLE:
					if (led_is_on[led_idx]) {
						led_is_on[led_idx] = 0;
						FIO3SET_bit.P3_26 = 1;
					} else {
						led_is_on[led_idx] = 1;
						FIO3CLR_bit.P3_26 = 1;
					}
					break;
			}
			break;
		}
}

void gpio_set_main_controller_onoff(int is_on)
{
	if (is_on) {
		g_data_runtime.is_main_controller_on = 1;
		g_data_runtime.is_main_controller_watchdog_started = 0;
		FIO0SET_bit.P0_0 = 1;
	} else {
		FIO0CLR_bit.P0_0 = 1;
		g_data_runtime.is_main_controller_on = 0;
		g_data_runtime.is_main_controller_watchdog_started = 0;

		unsigned int prev_tick = g_tick_100us;

		while(is_time_elapsed_100us(3000, prev_tick)) 	// 关电后延迟300ms, todo: 清除来自RK3288 的接受缓冲；
		{
			WDFEED = 0xAA;
			WDFEED = 0x55;
		}
	}
}


void gpio_set_adc_mux(int adc_mux_idx)
{
	if (adc_mux_idx & 0x01) FIO0CLR_bit.P0_24 = 1; else FIO0SET_bit.P0_24 = 1;
	if (adc_mux_idx & 0x02) FIO0CLR_bit.P0_23 = 1; else FIO0SET_bit.P0_23 = 1;
	if (adc_mux_idx & 0x04) FIO1CLR_bit.P1_31 = 1; else FIO1SET_bit.P1_31 = 1;
	if (adc_mux_idx & 0x08) FIO1CLR_bit.P1_30 = 1; else FIO1SET_bit.P1_30 = 1;
}

/*******
	  DMODE0  DMODE1  DMODE2
		L		L		L		待机模式
		L		L		H		全步分辨率
		L		H		L		半步分辨率（类型A）
		L		H		H		1/4步分辨率
		H		L		L		半步分辨率（类型B）
		H		L		H		1/8步分辨率
		H		H		L		1/16步分辨率
		H		H		H		1/32步分辨率
*/
void gpio_set_motor_mode(unsigned char motor_idx, unsigned char mode)
{
printk("idx=%d mode=%d\n", motor_idx, mode);
	if (motor_idx == 0) {
		switch (mode) {
			case 0:
				FIO2CLR_bit.P2_2 = 1;   // MS_X_0:0
				FIO2CLR_bit.P2_3 = 1;   // MS_X_1:0
				FIO2CLR_bit.P2_4 = 1;   // MS_X_2:0
				break;
			case 1:
				FIO2CLR_bit.P2_2 = 1;   // MS_X_0:0
				FIO2CLR_bit.P2_3 = 1;   // MS_X_1:0
				FIO2SET_bit.P2_4 = 1;   // MS_X_2:1
				break;
			case 2:
				FIO2CLR_bit.P2_2 = 1;   // MS_X_0:0
				FIO2SET_bit.P2_3 = 1;   // MS_X_1:1
				FIO2CLR_bit.P2_4 = 1;   // MS_X_2:0
				break;
			case 3:
				FIO2CLR_bit.P2_2 = 1;   // MS_X_0:0
				FIO2SET_bit.P2_3 = 1;   // MS_X_1:1
				FIO2SET_bit.P2_4 = 1;   // MS_X_2:1
				break;
			case 4:
				FIO2SET_bit.P2_2 = 1;   // MS_X_0:1
				FIO2CLR_bit.P2_3 = 1;   // MS_X_1:0
				FIO2CLR_bit.P2_4 = 1;   // MS_X_2:0
				break;
			case 5:
				FIO2SET_bit.P2_2 = 1;   // MS_X_0:1
				FIO2CLR_bit.P2_3 = 1;   // MS_X_1:0
				FIO2SET_bit.P2_4 = 1;   // MS_X_2:1
				break;
			case 6:
				FIO2SET_bit.P2_2 = 1;   // MS_X_2:1
				FIO2SET_bit.P2_3 = 1;   // MS_X_1:1
				FIO2CLR_bit.P2_4 = 1;   // MS_X_0:0
				break;
			case 7:
				FIO2SET_bit.P2_2 = 1;   // MS_X_0:1
				FIO2SET_bit.P2_3 = 1;   // MS_X_1:1
				FIO2SET_bit.P2_4 = 1;   // MS_X_2:1
				break;
		}
	//	g_data_runtime.motor_mode[0] = mode;
	} else if (motor_idx == 1) {
		switch (mode) {
			case 0:
				FIO2CLR_bit.P2_5 = 1;   // MS_Y_0
				FIO2CLR_bit.P2_6 = 1;   // MS_Y_1
				FIO2CLR_bit.P2_7 = 1;   // MS_Y_2
				break;
			case 1:
				FIO2CLR_bit.P2_5 = 1;   // MS_Y_0
				FIO2CLR_bit.P2_6 = 1;   // MS_Y_1
				FIO2SET_bit.P2_7 = 1;   // MS_Y_2
				break;
			case 2:
				FIO2CLR_bit.P2_5 = 1;   // MS_Y_0
				FIO2SET_bit.P2_6 = 1;   // MS_Y_1
				FIO2CLR_bit.P2_7 = 1;   // MS_Y_2
				break;
			case 3:
				FIO2CLR_bit.P2_5 = 1;   // MS_Y_0
				FIO2SET_bit.P2_6 = 1;   // MS_Y_1
				FIO2SET_bit.P2_7 = 1;   // MS_Y_2
				break;
			case 4:
				FIO2SET_bit.P2_5 = 1;   // MS_Y_0
				FIO2CLR_bit.P2_6 = 1;   // MS_Y_1
				FIO2CLR_bit.P2_7 = 1;   // MS_Y_2
				break;
			case 5:
				FIO2SET_bit.P2_5 = 1;   // MS_Y_0
				FIO2CLR_bit.P2_6 = 1;   // MS_Y_1
				FIO2SET_bit.P2_7 = 1;   // MS_Y_2
				break;
			case 6:
				FIO2SET_bit.P2_5 = 1;   // MS_Y_0
				FIO2SET_bit.P2_6 = 1;   // MS_Y_1
				FIO2CLR_bit.P2_7 = 1;   // MS_Y_2
				break;
			case 7:
				FIO2SET_bit.P2_5 = 1;   // MS_Y_0
				FIO2SET_bit.P2_6 = 1;   // MS_Y_1
				FIO2SET_bit.P2_7 = 1;   // MS_Y_2
				break;
		}
	//	g_data_runtime.motor_mode[1] = mode;
	} else if (motor_idx == 2) {
		switch (mode) {
			case 0:
				FIO2CLR_bit.P2_13 = 1;	// MS_Z_0
				FIO2CLR_bit.P2_12 = 1;	// MS_Z_1
				FIO2CLR_bit.P2_11 = 1;	// MS_Z_2
				break;
			case 1:
				FIO2CLR_bit.P2_13 = 1;	// MS_Z_0
				FIO2CLR_bit.P2_12 = 1;	// MS_Z_1
				FIO2SET_bit.P2_11 = 1;	// MS_Z_2
				break;
			case 2:
				FIO2CLR_bit.P2_13 = 1;	// MS_Z_0
				FIO2SET_bit.P2_12 = 1;	// MS_Z_1
				FIO2CLR_bit.P2_11 = 1;	// MS_Z_2
				break;
			case 3:
				FIO2CLR_bit.P2_13 = 1;	// MS_Z_0
				FIO2SET_bit.P2_12 = 1;	// MS_Z_1
				FIO2SET_bit.P2_11 = 1;	// MS_Z_2
				break;
			case 4:
				FIO2SET_bit.P2_13 = 1;	// MS_Z_0
				FIO2CLR_bit.P2_12 = 1;	// MS_Z_1
				FIO2CLR_bit.P2_11 = 1;	// MS_Z_2
				break;
			case 5:
				FIO2SET_bit.P2_13 = 1;	// MS_Z_0
				FIO2CLR_bit.P2_12 = 1;	// MS_Z_1
				FIO2SET_bit.P2_11 = 1;	// MS_Z_2
				break;
			case 6:
				FIO2SET_bit.P2_13 = 1;	// MS_Z_0
				FIO2SET_bit.P2_12 = 1;	// MS_Z_1
				FIO2CLR_bit.P2_11 = 1;	// MS_Z_2
				break;
			case 7:
				FIO2SET_bit.P2_13 = 1;	// MS_Z_0
				FIO2SET_bit.P2_12 = 1;	// MS_Z_1
				FIO2SET_bit.P2_11 = 1;	// MS_Z_2
				break;
		}
	//	g_data_runtime.motor_mode[2] = mode;
	}
}

void gpio_motor_pulse_x(unsigned char start_pluse)
{
	static unsigned char px;

	if(start_pluse){
		FIO1SET_bit.P1_16 = 1;
		px = 1;
	}else if (px) {
		FIO1CLR_bit.P1_16 = 1;
		px = 0;
	} else {
		FIO1SET_bit.P1_16 = 1;
		px = 1;
	}
}

void gpio_motor_pulse_y(unsigned char  start_pluse)
{
	static unsigned char py;

	if(start_pluse){
		FIO1SET_bit.P1_9 = 1;
		py = 1;
	}else if (py) {
		FIO1CLR_bit.P1_9 = 1;
		py = 0;
	} else {
		FIO1SET_bit.P1_9 = 1;
		py = 1;
	}
}

void gpio_motor_pulse_z(unsigned char start_pluse)
{
	static unsigned char pz;

	if(start_pluse){
		FIO1SET_bit.P1_26 = 1;
		pz = 1;
	}else if (pz) {
		FIO1CLR_bit.P1_26 = 1;
		pz = 0;
	} else {
		FIO1SET_bit.P1_26 = 1;
		pz = 1;
	}
}


void gpio_set_motor_onoff(int is_on)
{
	static int old_state = 0;
	if (is_on == old_state) return;
	if (is_on) {
		FIO0SET_bit.P0_7 = 1;
		FIO1SET_bit.P1_1 = 1;
		FIO1SET_bit.P1_29 = 1;
		printk("start motor @%d\n", g_tick_100us);
	} else {
		FIO0CLR_bit.P0_7 = 1;
		FIO1CLR_bit.P1_1 = 1;
		FIO1CLR_bit.P1_29 = 1;
		printk("stop motor @%d\n", g_tick_100us);
	}
	old_state = is_on;
}

void gpio_board_led(int led_cmd)
{
	static unsigned char led_is_on = 0;

	if(LED_CMD_TOGGLE == led_cmd)
		led_cmd = led_is_on ? LED_CMD_OFF : LED_CMD_ON;

	switch (led_cmd) {
		case LED_CMD_OFF:
			led_is_on = 0;
			FIO3CLR_bit.P3_25 = 1;
			break;
		case LED_CMD_ON:
			led_is_on = 1;
			FIO3SET_bit.P3_25 = 1;
			break;
	}
}


void gpio_rs485_tx_onoff(int is_on)
{
	if (is_on)
		FIO1SET_bit.P1_0 = 1;
	else
		FIO1CLR_bit.P1_0 = 1;
}


void gpio_set_door_lock_onoff(int is_on)
{
//	if (is_on) {
//		FIO0SET_bit.P0_18 = 1;
//		g_data_runtime.door_lock = 1;
//	 } else {
//		FIO0CLR_bit.P0_18 = 1;
//		g_data_runtime.door_lock = 0;
//	 }
}

void gpio_electromagnet_onoff(int is_on)
{
	if (is_on) {
		FIO0SET_bit.P0_8 = 1;
		g_data_runtime.electromagnet_state = 1;
	 } else { 
		FIO0CLR_bit.P0_8 = 1;
		g_data_runtime.electromagnet_state = 0;
	 }
}

int gpio_top_x()
{
	return FIO1PIN_bit.P1_15;
}

int gpio_bottom_x()
{
	return FIO1PIN_bit.P1_14;
}

int gpio_top_y()
{
	return FIO1PIN_bit.P1_8;
}

int gpio_bottom_y()
{
	return FIO1PIN_bit.P1_4;
}

int gpio_top_z()
{
	return FIO1PIN_bit.P1_27;
}

int gpio_bottom_z()
{
	return FIO1PIN_bit.P1_28;
}




