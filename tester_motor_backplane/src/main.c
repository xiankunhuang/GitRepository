#include "config.h"
#include "backplane.h"

// 所有的全局变量都在这里定义
#define U8 unsigned char
#define U16 unsigned long
#define U32 unsigned int

#pragma data_alignment=4
backplane_eeprom_t g_data_backplane_eeprom;
adc_info_t g_data_adc[NR_ADC_INA219];
tray_info_t g_data_tray;
runtime_info_t g_data_runtime;
power_module_info_t g_data_power_modules[NR_POWER_MODULES];
volatile unsigned int g_tick_100us;

//s_motor * smotor; 


global_config_t g_global_config;
motor_config_t g_motor_config;


PID spid;


#pragma data_alignment=4
working_profile_expected_t working_profile_req;
#pragma data_alignment=4
tray_eeprom_t tray_eeprom;

#ifdef Debug
unsigned char *g_high_stack;
volatile unsigned char *g_low_stack;
#endif

volatile unsigned int loop_count,_test_tick1, _test_tick2,_test_tick3,_test_tick4,_test_tick5,_test_tick6;
volatile	int sm_position, sm_dir; 

void self_calibrate()
{
	printk("self_calibrate...\n"); 
}


#define IIC_TASK_IDLE 	        0
#define STEP_IDLE 		        0

void poll_iic()
{
	static unsigned int iic_task[3];        //  I2C0, I2C1, I2C2
	static unsigned char _adc_idx , iic_idx;	
	int iic_ret;
	
	iic_idx = 0;

	// IIC0: PD;  
	iic_ret =  i2c_session_is_finished(BUS_ID0);//
	if (iic_ret > 0) {
	//	printk("iic_task_bitmap=%x, iic_task=%x\n", g_data_runtime.iic_task_bitmap, iic_task[iic_idx]);
		if (ADC_AMUX_SET == iic_task[iic_idx]) {  // I2C任务
			if (STEP_IDLE == g_data_adc[ADC_ID_AMUX]._step_idx) {
				adc_to_value( _adc_idx);
				g_data_runtime.iic_task_bitmap &= ~(0x01<< _adc_idx); // 缺异常检测		
				iic_task[iic_idx] = IIC_TASK_IDLE; 
			} 
			else
				adc_proc_mux();
		} else if (ADC_TEMP_SET == iic_task[iic_idx]) {
			if (STEP_IDLE == g_data_adc[ADC_ID_TEMP]._step_idx) {
				adc_to_temp();
				g_data_runtime.iic_task_bitmap &= ~ADC_TEMP_SET;      // 缺异常检测；
				iic_task[iic_idx] = IIC_TASK_IDLE; 
			} 
			else
				adc_proc_k_temp(); 
		}

		if (STEP_IDLE ==  iic_task[iic_idx]) {
			if (g_data_runtime.iic_task_bitmap & ADC_TEMP_SET) {
				adc_proc_k_temp(); 
				iic_task[iic_idx] = ADC_TEMP_SET;
			}else if (g_data_runtime.iic_task_bitmap & ADC_AMUX_BITMAP) { //通用电压测量
				while (_adc_idx < 16 /*NR_ADC_SENSORS*/) {
					if (g_data_runtime.iic_task_bitmap & (0x01 << _adc_idx)) {
						gpio_set_adc_mux(_adc_idx);                   //  设置到指定通道 
						adc_proc_mux();
						iic_task[iic_idx] = ADC_AMUX_SET; 
						break;
					}
					_adc_idx++;
				}
				if (_adc_idx >= NR_ADC_SENSORS)  
					_adc_idx = 0;                                     // 通道循环
			} 
		}
	} else if (iic_ret < 0) {
		printk("poll_iic0 err %x\n", iic_task[iic_idx]);
	}	
}



#if 0
// 需要快速判断马上处理的事件；
static void fail_safe() 
{
	unsigned char err_cnt = 0;
	volatile unsigned int vcc24 = 0;	//定义成易失型，不然循环中不更新； 先来一次假异常；
	unsigned int now;

	now = g_tick_100us;
	vcc24 = 0;	// 先来一次假异常；


	// 如果异常就连续采样；
	while ((vcc24 <  g_global_config.power_module_input_voltage_lower_limit_mV)
		||(vcc24 > g_global_config.power_module_input_voltage_upper_limit_mV)) {// 供电电压异常；
		if ((g_tick_100us > (now + 10)) && (g_data_power_modules[0].is_on)) {
			printk("failsafe: Vin=%d\n", vcc24);			
			now = g_tick_100us;
			now = g_tick_100us - now;
			printk("Done tick=%d\n", now);
			return;
		}
		vcc24 = ADDR2_bit.RESULT;
		vcc24 = (vcc24 * 3000 * 11) >> 12 ;
	}
}
#endif

#define POWER_GOOD_THRESHOLD   16000

void RK3288_pwr_control()
{
	static unsigned short vcc24_mv =0;
	static unsigned int prev_tick = 0;
	static unsigned int pwr_good_tick=0;
	static unsigned int delayed_poweroff_start_tick = 0;

	static unsigned int tmp_tick = 0;

	
	if (is_time_elapsed_100us(100, prev_tick)) { // 10mS interval;
		prev_tick = g_tick_100us;
		vcc24_mv = ADDR2_bit.RESULT;
		vcc24_mv = (vcc24_mv * 3000 * 11) >> 12 ;
	//	printk("VCC24V:%d\n", vcc24_mv);
	//	vcc24_mv = 24000;

		if (++tmp_tick > 100) tmp_tick=0;
		
		if ((vcc24_mv > POWER_GOOD_THRESHOLD) && (!g_data_runtime.is_main_controller_on)) { // 上电逻辑
			if( ++pwr_good_tick > 500) // 延迟秒上电；
			{
				gpio_set_main_controller_onoff(1);
				pwr_good_tick = 0;
			}
		} else {
			pwr_good_tick =0;

			if((g_tick_100us > 20000) && (vcc24_mv < 12000) && (g_data_runtime.is_main_controller_on)
				&& (g_data_runtime.need_to_set_main_controller_onoff == 0))
			{
				g_data_runtime.need_to_set_main_controller_onoff = 2;

				printk("warning: vcc 24 low mV=%d tick=%d\n", vcc24_mv, g_tick_100us);
			}
		}

		if (g_data_runtime.need_to_set_main_controller_onoff == 2) {// 下电逻辑 
			if (!delayed_poweroff_start_tick) {
				delayed_poweroff_start_tick = g_tick_100us;
				printk("delayed to power off ARM\n");
			}
			if (is_time_elapsed_100us(g_global_config.power_off_delay_100uS, delayed_poweroff_start_tick)) {
				printk("power off ARM, tick=%d\n", g_tick_100us);
				gpio_set_main_controller_onoff(0);
				g_data_runtime.need_to_set_main_controller_onoff = 0;
				delayed_poweroff_start_tick = 0;
			}
		} else if ((g_data_runtime.need_to_set_main_controller_onoff == 0) 
			   && g_data_runtime.is_main_controller_watchdog_started 
			   && is_time_elapsed_100us(g_global_config.watchdog_interval_100uS, g_data_runtime.last_rs485_frame_received_tick)) {
			printk("watchdog: poweroff ARM\n");
			gpio_set_main_controller_onoff(0);
			delayed_poweroff_start_tick = 0;
		}
	}

}


#define LIV_TIMEOUT_TICK 30

#define THERMAL_TEST_TICK 50

void worker_power_cycle()
{

	printk("worker powercycle\n");

	WDFEED = 0xAA;
	WDFEED = 0x55;
	
	while(1) ;

}

#define   PULL_TRAY_NONE          0  // 要求没有拽到托盘 
#define   PULL_TRAY_DONE          1  // 要求有托拽到托盘 
/*  
 *  code：  0或1， 0表示托盘上没物料， 1表示托盘上有物料
 *  return:  0 表示满足条件正常， 1表示不满足条件  
 **/
static short Trigger_Test_Error(unsigned char code)
{
    unsigned char e = 0, i = 0;
    while (FIO0PIN_bit.P0_18 == 0){   // 检测到有托盘
		if(++i > 10){ e = 1; break;}
	}
    if(e != code){                    // 与要求条件不符，
     	if(3 != g_data_runtime.tested){
			g_data_runtime.tested = 3; 
			printk("g_data_runtime.tested = %d\n", g_data_runtime.tested);
    	}
		return 1;
    }
   
    return 0;
}

#define  DELAY_TICK                   3
#define  TEST_WAIT_TICK               1
#define  TEST_DONE_WAIT_TICK          2
// 当限位开关变化位置时要改变该数值(以下数值是基于微步为32时)
#define  BOTTOM_LAYER               1000         

#define  FIRST_LAYER                -19000       // 原来第一层不用。
#define  ELEVEN_LAYER               1671000      // 1671000+19000 = 1690000

#define  TOP_LAYER                  1760000

#define  MAX_LAYER_NUM               11
#define  EVERY_LAYER_STEP        ( (ELEVEN_LAYER - FIRST_LAYER) / (MAX_LAYER_NUM - 1) )         // 169000  每层间隔     


#define  SUBDIVISION_Z            MOTOR_0_DRIVER_MICRO_STEP               //  X 驱动 Z轴
#define  SUBDIVISION_Y            MOTOR_1_DRIVER_MICRO_STEP

#define  PULL_LOCK               (40000 * SUBDIVISION_Z / 32)             // 挂扣要向上移的步数           
#define  EACAPE_LOCK             (40000 * SUBDIVISION_Z / 32)             // 退扣时要向下移动的步数
// Y 轴
#define  PULL_BOUND              (1720 * SUBDIVISION_Y / 32)               // 向外拉的边界步数                
#define  PUSH_BOUND              (171000 * SUBDIVISION_Y / 32)             // 向里推的位置，          171000      175000  
#define  LOCK_BOUND              (171000 * SUBDIVISION_Y / 32)             // 向上挂托盘时位置          168000     175000
#define  PUSH_SAFE_BOUND         (152000 * SUBDIVISION_Y / 32)             // 扣退回安全位置

// Z 轴
#define  EVERY_LAYER(idx)        ( ( (idx) * EVERY_LAYER_STEP + FIRST_LAYER )  * SUBDIVISION_Z / 32 - EACAPE_LOCK)  // 每一层的位置, 0~10 表示1~11层高度  
#define  LAYER_TOP               (1740000 * SUBDIVISION_Y / 32)                       // 上方最多允许的位置


#define  LAYER_ONE_TO_TWO        (EVERY_LAYER_STEP * SUBDIVISION_Z / 32)              //  第一层到第二层需要的步数       
#define  LAYER_TWO_TO_THREE      (EVERY_LAYER_STEP * SUBDIVISION_Z / 32)              //  290000 - 120000
#define  LAYER_THREE_TO_FOUR     (EVERY_LAYER_STEP * SUBDIVISION_Z / 32)              //  458000 - 290000
#define  LAYER_FOUR_TO_FIVE      (EVERY_LAYER_STEP * SUBDIVISION_Z / 32)              //  630000 - 458000
#define  LAYER_FIVE_TO_SIX       (EVERY_LAYER_STEP * SUBDIVISION_Z / 32)              //  797000 - 630000
#define  LAYER_SIX_TO_SEVEN      (EVERY_LAYER_STEP * SUBDIVISION_Z / 32)              //  967000 - 797000
#define  LAYER_SEVEN_TO_EIGHT    (EVERY_LAYER_STEP * SUBDIVISION_Z / 32)              //  1138000 - 967000
#define  LAYER_EIGHT_TO_NINE     (EVERY_LAYER_STEP * SUBDIVISION_Z / 32)              //  1303000 - 1138000
#define  LAYER_NINE_TO_TEN       (EVERY_LAYER_STEP * SUBDIVISION_Z / 32)              //  1473000 - 1303000
#define  LAYER_TEN_TO_ELEVEN     (EVERY_LAYER_STEP * SUBDIVISION_Z / 32)              //  1643000 - 1473000 


#define  X_LAYER_ESCAPE_1                 (EVERY_LAYER(0))    // -59000 
#define  X_LAYER_ESCAPE_2                 (EVERY_LAYER(1))    // 110000
#define  X_LAYER_ESCAPE_3                 (EVERY_LAYER(2))    // 279000
#define  X_LAYER_ESCAPE_4                 (EVERY_LAYER(3))    // 448000
#define  X_LAYER_ESCAPE_5                 (EVERY_LAYER(4))    // 617000
#define  X_LAYER_ESCAPE_6                 (EVERY_LAYER(5))    // 786000
#define  X_LAYER_ESCAPE_7                 (EVERY_LAYER(6))    // 955000
#define  X_LAYER_ESCAPE_8                 (EVERY_LAYER(7))    // 1124000
#define  X_LAYER_ESCAPE_9                 (EVERY_LAYER(8))    // 1293000
#define  X_LAYER_ESCAPE_10                (EVERY_LAYER(9))    // 1462000
#define  X_LAYER_ESCAPE_11                (EVERY_LAYER(10))   // 1631000


/* 0~7 对应2,3,4,5,6,7,8,9层    */
unsigned int X_LAYER_ESCAPE[] = {// X_LAYER_ESCAPE_1, 
                                  X_LAYER_ESCAPE_2,
                                  X_LAYER_ESCAPE_3, 
                                  X_LAYER_ESCAPE_4, 
                                  X_LAYER_ESCAPE_5,  
                                  X_LAYER_ESCAPE_6,    
                                  X_LAYER_ESCAPE_7,  
                                  X_LAYER_ESCAPE_8,   
                                  X_LAYER_ESCAPE_9,
                                  X_LAYER_ESCAPE_10,
                                  X_LAYER_ESCAPE_11};

unsigned int X_LAYER_LOCK[] = {// X_LAYER_ESCAPE_1 + PULL_LOCK, 
	                           X_LAYER_ESCAPE_2 + PULL_LOCK,      // 150000
	                           X_LAYER_ESCAPE_3 + PULL_LOCK,      // 319000
	                           X_LAYER_ESCAPE_4 + PULL_LOCK,      // 488000
	                           X_LAYER_ESCAPE_5 + PULL_LOCK,      // 657000
	                           X_LAYER_ESCAPE_6 + PULL_LOCK,      // 826000
	                           X_LAYER_ESCAPE_7 + PULL_LOCK,      // 995000
	                           X_LAYER_ESCAPE_8 + PULL_LOCK,      // 1164000
	                           X_LAYER_ESCAPE_9 + PULL_LOCK,      // 1333000
	                           X_LAYER_ESCAPE_10 + PULL_LOCK,     // 1502000
	                           X_LAYER_ESCAPE_11 + PULL_LOCK};    // 1671000

unsigned int X_LAYER_PUSH[] = {// X_LAYER_ESCAPE_1 + EACAPE_LOCK,  
	                           X_LAYER_ESCAPE_2 + EACAPE_LOCK,  
	                           X_LAYER_ESCAPE_3 + EACAPE_LOCK, 
	                           X_LAYER_ESCAPE_4 + EACAPE_LOCK, 
	                           X_LAYER_ESCAPE_5 + EACAPE_LOCK, 
	                           X_LAYER_ESCAPE_6 + EACAPE_LOCK, 
	                           X_LAYER_ESCAPE_7 + EACAPE_LOCK, 
	                           X_LAYER_ESCAPE_8 + EACAPE_LOCK, 
	                           X_LAYER_ESCAPE_9 + EACAPE_LOCK,
	                           X_LAYER_ESCAPE_10 + EACAPE_LOCK,
	                           X_LAYER_ESCAPE_11 + EACAPE_LOCK};

#define  ARRAY_MAX_MEMBER                          10
#define  SPEED_X_FACTOR                            5               // 速度除 2 降一半 
#define  SPEED_Y_FACTOR                            1               // 速度除 2 降一半 

#define  AUTO_LOCK                                               // 自动锁定托盘

/**
 *  idx:        当前操作第几层 
 *  ready_step: 当前准备操作那一步 
 *  return：     返回下次控制托架应对应那一步
 **/
unsigned char pull_push_tray_(unsigned char idx,  unsigned char ready_step)
{
	// x_layer => 推进托盘槽位置高度，   x_top => 送往测试层，  x_escape = 退出测试层高度	
	unsigned int x_layer = EVERY_LAYER(idx),  x_top = LAYER_TOP,  x_escape = EVERY_LAYER(idx) - EACAPE_LOCK, x_lock = EVERY_LAYER(idx) - 10000, idx_revise; 

	static unsigned int tick = 0;
	static char step = 0;                             // 以后 ready_step 赋值给它

    // printk("pull_tray[%d]: step_motor_init_done =%d step_motor_init_pending =%d\n", idx, g_data_runtime.step_motor_init_done, g_data_runtime.step_motor_init_pending);
	if (!g_data_runtime.step_motor_init_done || g_data_runtime.step_motor_init_pending)	     return step;          //  如果没完成电机运行就退出

    // printk("pull_tray[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);
	if ((sm[0].target_position != sm[0].position) || (sm[1].target_position != sm[1].position))  return step;     //  
        

	// printk("pull_tray target_x = %d: targete_y = %d\n", sm[0].target_position, sm[1].target_position);

    if( idx < ARRAY_MAX_MEMBER ){ 	
		x_escape = X_LAYER_ESCAPE[idx];			
	    x_lock = X_LAYER_LOCK[idx];
		x_layer = X_LAYER_PUSH[idx]; 
	}
    printk("pull_tray step =%d\n", step);
    
    switch(step){
    	case 0:  // gpio_electromagnet_onoff(0);      //  y轴拉到安全位置， 
				if( Trigger_Test_Error(PULL_TRAY_NONE) ) {/*step = 0; */ break; }
				step_motor_goto(idx, BOTTOM_LAYER, PUSH_SAFE_BOUND, 0); 
				step = 1; 
				break;
    	case 1:  // gpio_electromagnet_onoff(0);      //  升降轴运动到指定层料盘处       idx的位置低8mm
				if ( (BOTTOM_LAYER>>3<<1 != sm[0].target_position) || (PUSH_SAFE_BOUND>>3<<1 != sm[1].target_position) ) {  // 异常  与上一步要求的位置不同时
					printk("pull_tray err[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);                    // 当前位置
					printk("pull_tray err[%d]: target_x =%d target_y =%d step =%d\n", idx, x_escape>>3<<1, PUSH_SAFE_BOUND>>3<<1, step);  // 期望位置
					/** 发现异常, 要处理一下 **/
					step_motor_goto(idx, BOTTOM_LAYER, PUSH_SAFE_BOUND, 0);                                                               // 重新设置到期望位置
					step = 1;  
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_NONE) ) {/*step = 0; */ break; }
			#ifndef AUTO_LOCK
				step_motor_goto(idx, x_escape, PUSH_SAFE_BOUND, 0); 
			#else
				step_motor_goto(idx, x_lock, PUSH_SAFE_BOUND, 0); 
			#endif
				step = 2;
    	     	break;
    	case 2:  // gpio_electromagnet_onoff(0);      //  移动托盘到对应托架位置
    	     #ifndef AUTO_LOCK
				if ( (x_escape>>3<<1 != sm[0].target_position) || (PUSH_SAFE_BOUND>>3<<1 != sm[1].target_position) ) {      // 异常  与上一步要求的位置不同时
					printk("pull_tray err[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);                    // 当前位置
					printk("pull_tray err[%d]: target_x =%d target_y =%d step =%d\n", idx, x_escape>>3<<1, PUSH_SAFE_BOUND>>3<<1, step);  // 期望位置
					// 发现异常, 要处理一下
					step_motor_goto(idx, x_escape, PUSH_SAFE_BOUND, 0);                                                                   // 重新设置到期望位置 
			 #else
				if ( (x_lock>>3<<1 != sm[0].target_position) || (PUSH_SAFE_BOUND>>3<<1 != sm[1].target_position) ) {        // 异常  与上一步要求的位置不同时
					printk("pull_tray err[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);                    // 当前位置
					printk("pull_tray err[%d]: target_x =%d target_y =%d step =%d\n", idx, x_lock>>3<<1, PUSH_SAFE_BOUND>>3<<1, step);    // 期望位置
					// 发现异常, 要处理一下
					step_motor_goto(idx, x_lock, PUSH_SAFE_BOUND, 0);                                                                     // 重新设置到期望位置 
             #endif
					step = 2;
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_NONE) ) {/*step = 0; */ break; }
			#ifndef AUTO_LOCK
				step_motor_goto(idx, x_escape, LOCK_BOUND, 0);	
			#else
				step_motor_goto(idx, x_lock, LOCK_BOUND, 0);	
			#endif
				step = 3;
				break;
        case 3:  // gpio_electromagnet_onoff(0);      //  向上移动8mm, 扣住托盘   /直接向前推扣住托盘
             #ifndef AUTO_LOCK
				if ( (x_escape>>3<<1 != sm[0].target_position) || (LOCK_BOUND>>3<<1 != sm[1].target_position) ) {           // 异常  与上一步要求的位置不同时
					printk("pull_tray err[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);                    // 当前位置
					printk("pull_tray err[%d]: target_x =%d target_y =%d step =%d\n", idx, x_escape>>3<<1, LOCK_BOUND>>3<<1, step);       // 期望位置
					// 发现异常, 要处理一下 
					step_motor_goto(idx, x_escape, LOCK_BOUND, 0);			                                                // 重新设置到期望位置 
			 #else  
				if ( (x_lock>>3<<1 != sm[0].target_position) || (LOCK_BOUND>>3<<1 != sm[1].target_position) ) {             // 异常  与上一步要求的位置不同时
					printk("pull_tray err[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);                    // 当前位置
					printk("pull_tray err[%d]: target_x =%d target_y =%d step =%d\n", idx, x_lock>>3<<1, LOCK_BOUND>>3<<1, step);         // 期望位置
					// 发现异常, 要处理一下 
					step_motor_goto(idx, x_lock, LOCK_BOUND, 0);			                                                // 重新设置到期望位置 
             #endif
					step = 3;
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_NONE) ) {/*step = 0; */ break; }
				step_motor_goto(idx, x_lock, LOCK_BOUND, 0); 
				step = 4;
				break;
    	case 4:  // gpio_electromagnet_onoff(1);      //  拉出托盘到当前层最边缘位置
				tick++;
				if (tick < TEST_WAIT_TICK) return step;
				tick = 0;
				if ( (x_lock>>3<<1 != sm[0].target_position) || (LOCK_BOUND>>3<<1 != sm[1].target_position) ) {              // 异常  与上一步要求的位置不同时
					printk("pull_tray err[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);                    // 当前位置
					printk("pull_tray err[%d]: target_x =%d target_y =%d step =%d\n", idx, x_lock>>3<<1, LOCK_BOUND>>3<<1, step);         // 期望位置
					/** 发现异常, 要处理一下 **/
					step_motor_goto(idx, x_lock, LOCK_BOUND, 0);                                                                          // 重新设置到期望位置 
					step = 4;
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_NONE) ) {/*step = 0; */ break; }
				sm[1].max_v = g_motor_config.motor_max_v[1] * SPEED_Y_FACTOR;                    // 拉托盘速度减半
				step_motor_goto(idx, x_lock, PULL_BOUND, 0);
				step = 5;
				break;
        case 5:  tick++;                           //  上升运动至检测位置 
				if (tick < TEST_WAIT_TICK) return step;
				tick = 0;
				if ( (x_lock>>3<<1 != sm[0].target_position) || (PULL_BOUND>>3<<1 != sm[1].target_position) ) {              // 异常  与上一步要求的位置不同时
					printk("pull_tray err[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);                    // 当前位置
					printk("pull_tray err[%d]: target_x =%d target_y =%d step =%d\n", idx, x_lock>>3<<1, PULL_BOUND>>3<<1, step);         // 期望位置
					/** 发现异常, 要处理一下 **/
					step_motor_goto(idx, x_lock, PULL_BOUND, 0);                                                                          // 重新设置到期望位置
					step = 5;                          
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_DONE) ) {/*step = 0; */ break; }
				step_motor_goto(idx, x_top, PULL_BOUND, 0);
				step = 6;
    	     	break;
        case 6: tick++;                                        //  等待完成器件检测, 并向移至原来托架层位置
				if (tick < TEST_DONE_WAIT_TICK) return step;   //  等待检测时间 
				tick = 0;
				// step_motor_goto(idx, x_layer, PULL_BOUND, 0);

				if ( (x_top>>3<<1 != sm[0].target_position) || (PULL_BOUND>>3<<1 != sm[1].target_position) ) {               // 异常  与上一步要求的位置不同时
					printk("pull_tray err[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);                   // 当前位置
					printk("pull_tray err[%d]: target_x =%d target_y =%d step =%d\n", idx, x_top>>3<<1, PULL_BOUND>>3<<1, step);         // 期望位置
					/** 发现异常, 要处理一下 **/
					step_motor_goto(idx, x_top, PULL_BOUND, 0);                                                                          // 重新设置到期望位置
					step = 6;                         
					break;
				}

				idx_revise = idx;
				idx_revise += 1;
				if(idx_revise >= ARRAY_MAX_MEMBER)  idx_revise = 0;                             // 当到最高层时，改为最低层 
				if( Trigger_Test_Error(PULL_TRAY_DONE) ) {/*step = 0; */ break; }
				step_motor_goto(idx, X_LAYER_PUSH[idx_revise], PULL_BOUND, 0);   // 放下一层
				// gpio_electromagnet_onoff(0);

				step = 7;
				break;
    	case 7: tick++;                           //  将托盘推放到原来位置
				if (tick < TEST_WAIT_TICK) return step;
				tick = 0;
				// gpio_electromagnet_onoff(0);
				// step_motor_goto(0, x_layer, PUSH_BOUND, 0); 

				idx_revise = idx;
				idx_revise += 1;
				if(idx_revise >= ARRAY_MAX_MEMBER)  idx_revise = 0;                             // 当到最高层时，改为最低层 

				if ( (X_LAYER_PUSH[idx_revise]>>3<<1 != sm[0].target_position) || (PULL_BOUND>>3<<1 != sm[1].target_position) ) {  // 异常  与上一步要求的位置不同时
					printk("pull_tray err[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);                               // 当前位置
					printk("pull_tray err[%d]: target_x =%d target_y =%d step =%d\n", idx, X_LAYER_PUSH[idx_revise]>>3<<1, PULL_BOUND>>3<<1, step);  // 期望位置
					/** 发现异常, 要处理一下 **/
					step_motor_goto(idx, X_LAYER_PUSH[idx_revise], PULL_BOUND, 0);                                                            // 重新设置到期望位置
					step = 7;                         
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_DONE) ) {/*step = 0; */ break; }
				step_motor_goto(0, X_LAYER_PUSH[idx_revise], PUSH_BOUND, 0); 
				step = 8;
				break;
    	case 8: tick++;                           //  退出拉托架的扣，向下移动8mm
				if (tick < TEST_WAIT_TICK) return step;
				tick = 0;
				// gpio_electromagnet_onoff(0);
				// step_motor_goto(0, x_escape, PUSH_BOUND, 0);

				idx_revise = idx;
				idx_revise += 1;
				if(idx_revise >= ARRAY_MAX_MEMBER)  idx_revise = 0;                             // 当到最高层时，改为最低层 

				if ( (X_LAYER_PUSH[idx_revise]>>3<<1 != sm[0].target_position) || (PUSH_BOUND>>3<<1 != sm[1].target_position) ) {  // 异常  与上一步要求的位置不同时
					printk("pull_tray err[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);			                     // 当前位置
					printk("pull_tray err[%d]: target_x =%d target_y =%d step =%d\n", idx, X_LAYER_PUSH[idx_revise]>>3<<1, PUSH_BOUND>>3<<1, step);  // 期望位置
					/** 发现异常, 要处理一下 **/
					step_motor_goto(idx, X_LAYER_PUSH[idx_revise], PUSH_BOUND, 0);							         // 重新设置到期望位置
					step = 8;			   
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_NONE) ) {/*step = 0; */ break; }
				sm[1].max_v = g_motor_config.motor_max_v[1];                    // 拉托盘速度恢复
				step_motor_goto(0, X_LAYER_ESCAPE[idx_revise], PUSH_BOUND, 0); 
				step = 9;

				break;
    	case 9: tick++;                           //  退出回边界位置
				if (tick < TEST_WAIT_TICK) return step;
				tick = 0;
				// gpio_electromagnet_onoff(0);
				// step_motor_goto(0, x_escape, PULL_BOUND, 0);


				idx_revise = idx;
				idx_revise += 1;
				if(idx_revise >= ARRAY_MAX_MEMBER)  idx_revise = 0;		              // 当到最高层时，改为最低层 

				if ( (X_LAYER_ESCAPE[idx_revise]>>3<<1 != sm[0].target_position) || (PUSH_BOUND>>3<<1 != sm[1].target_position) ) {  // 异常  与上一步要求的位置不同时
					printk("pull_tray err[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);			            // 当前位置
					printk("pull_tray err[%d]: target_x =%d target_y =%d step =%d\n", idx, X_LAYER_ESCAPE[idx_revise]>>3<<1, PUSH_BOUND>>3<<1, step);  // 期望位置
					/** 发现异常, 要处理一下 **/
					step_motor_goto(idx, X_LAYER_ESCAPE[idx_revise], PUSH_BOUND, 0);							         // 重新设置到期望位置
					step = 9;			   
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_NONE) ) {/*step = 0; */ break; }
				step_motor_goto(0, X_LAYER_ESCAPE[idx_revise], PULL_BOUND, 0); 

				step = 10;
				break;
    	case 10: tick++;                           //  确认是否退出回边界位置
				if (tick < TEST_WAIT_TICK) return step;               
				tick = 0;
				//gpio_electromagnet_onoff(0);

				idx_revise = idx;
				idx_revise += 1;
				if(idx_revise >= ARRAY_MAX_MEMBER)  idx_revise = 0;		              // 当到最高层时，改为最低层 

				if ( (X_LAYER_ESCAPE[idx_revise]>>3<<1 != sm[0].target_position) || (PULL_BOUND>>3<<1 != sm[1].target_position) ) {  // 异常  与上一步要求的位置不同时
					printk("pull_tray err[%d]: position_x =%d position_y =%d\n", idx, sm[0].position, sm[1].position);			            // 当前位置
					printk("pull_tray err[%d]: target_x =%d target_y =%d step =%d\n", idx, X_LAYER_ESCAPE[idx_revise]>>3<<1, PULL_BOUND>>3<<1, step);  // 期望位置
					/** 发现异常, 要处理一下 **/
					step_motor_goto(idx, X_LAYER_ESCAPE[idx_revise], PULL_BOUND, 0);							            // 重新设置到期望位置
					step = 10;			   
					break;
				}
				g_data_runtime.tested = 1;       // 完成测量 
				step = 0;
				break;
    	default: 
             	break;
             

	}
	return step;
}


/**
 *  idx:        当前操作第几层 
 *  ready_step: 当前准备操作那一步 
 *  return：     返回下次控制托架应对应那一步, 
 **/
unsigned char pull_push_tray(unsigned char idx,  unsigned char ready_step)
{
	// x_layer => 推进托盘槽位置高度，   x_top => 送往测试层，  x_escape = 退出测试层高度	
	unsigned int  x_top = LAYER_TOP,  x_escape = EVERY_LAYER(idx) - EACAPE_LOCK, x_lock = EVERY_LAYER(idx) - 10000, idx_revise; 

	static unsigned int tick = 0;
	static char step = 0;                             // 以后 ready_step 赋值给它

	if (!g_data_runtime.step_motor_init_done || g_data_runtime.step_motor_init_pending)	     return step;          

	if ((sm[0].target_position != sm[0].position) || (sm[1].target_position != sm[1].position))  return step;       

//	if ( (ready_step != 0) && ((ready_step > (step + 1) ) || (ready_step < step )  return step;                 // ready_step 只允许在非零时做限制，要求只能是与上一步相同或下一步
//	step = ready_step;                    // 更新当前准备操作

    if( idx < ARRAY_MAX_MEMBER ){ 	      //  有效层   0 ~ (ARRAY_MAX_MEMBER - 1)
		x_escape = X_LAYER_ESCAPE[idx];			
	    x_lock = X_LAYER_LOCK[idx];
	}
    printk("pull tray step = %d\n", step);
    			
    switch(step){
    	case 0: //  y轴拉到最边位置，z轴到当前要操作层 
				if( Trigger_Test_Error(PULL_TRAY_NONE) ) {/*step = 0; */ break; }                                           // 刚开始时如果检测到有托盘时什么都不做 
				if ( (x_lock>>3<<1 != sm[0].target_position) || (PULL_BOUND>>3<<1 != sm[1].target_position) ) {             // 与要求的位置不同时
					step_motor_goto(idx, x_lock, PULL_BOUND, 0);												            // 设置到期望位置
					break;
				}				
				step = 1; 
				break;
    	case 1: //  Z轴到当前要操作层，y轴扣住托盘
    	        if ( (x_lock>>3<<1 != sm[0].target_position) || (LOCK_BOUND>>3<<1 != sm[1].target_position) ) {             // 与要求的位置不同时
					step_motor_goto(idx, x_lock, LOCK_BOUND, 0);												            // 设置到期望位置 
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_NONE) ) {/*step = 0; */ break; }
				step = 2;
    	    	break;
    	case 2: //  向外拉托盘到最边界位置
				if ( (x_lock>>3<<1 != sm[0].target_position) || (PULL_BOUND>>3<<1 != sm[1].target_position) ) {             // 与要求的位置不同时
					step_motor_goto(idx, x_lock, PULL_BOUND, 0);			                                                // 设置到期望位置 
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_DONE) ) { step = 1;  break; }   // 当检测到没有挂住托盘时，重新再尝试挂托盘   退回到上一步位置     step = 1;
				step = 3;
				break;
        case 3: //  托盘向上送到测量位置，先送到最高层位置, 再送到测量位置 
				if ( (X_LAYER_LOCK[ARRAY_MAX_MEMBER - 1]>>3<<1 != sm[0].target_position) || (PULL_BOUND>>3<<1 != sm[1].target_position) ) {              // 与要求的位置不同时
					step_motor_goto(idx, X_LAYER_LOCK[ARRAY_MAX_MEMBER - 1], PULL_BOUND, 0);                                                              // 设置到期望位置 
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_DONE) ) {/* step = 0; */  break; }            
				sm[0].max_v = g_motor_config.motor_max_v[0] * SPEED_X_FACTOR;                     // Z轴速度变慢
				step = 4;
				break;
    	case 4: //  将Z轴速度降速后，再上升至测量层位置 
				if ( (x_top>>3<<1 != sm[0].target_position) || (PULL_BOUND>>3<<1 != sm[1].target_position) ) {                                 // 与要求的位置不同时
					step_motor_goto(idx, x_top, PULL_BOUND, 0);                                                                               // 设置到期望位置
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_DONE) ) {/*step = 0; */ break; }
				sm[0].max_v = g_motor_config.motor_max_v[0];                                     // Z轴速度恢复
				step = 5;
				break;
        case 5: //  送回到原来层位置(当前测试是回到原来层的下一层)
				idx_revise = idx + 1;
				if(idx_revise >= ARRAY_MAX_MEMBER)	idx_revise = 0; 							 // 当到最高层时，改为最低层 
        		if ( (X_LAYER_PUSH[idx_revise]>>3<<1 != sm[0].target_position) || (PULL_BOUND>>3<<1 != sm[1].target_position) ) {              // 与要求的位置不同时
					step_motor_goto(idx, X_LAYER_PUSH[idx_revise], PULL_BOUND, 0);                                                             // 设置到期望位置
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_DONE) ) {/*step = 0; */ break; }
				step = 6;
    	     	break;
        case 6: // 推回原来层托架位置
				idx_revise = idx + 1;
				if(idx_revise >= ARRAY_MAX_MEMBER)	idx_revise = 0; 							 // 当到最高层时，改为最低层 
				if ( (X_LAYER_PUSH[idx_revise]>>3<<1 != sm[0].target_position) || (PUSH_BOUND>>3<<1 != sm[1].target_position) ) {             // 与要求的位置不同时
					step_motor_goto(idx, X_LAYER_PUSH[idx_revise], PUSH_BOUND, 0);															  // 设置到期望位置
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_NONE) ) {step = 5;  break; }                   // 当发现没有推回, 退回到上一步 step = 5;
				step = 7;
				break;
    	case 7: //  退挂托盘的扣
				idx_revise = idx + 1;
				if(idx_revise >= ARRAY_MAX_MEMBER)  idx_revise = 0;                             // 当到最高层时，改为最低层 
				if ( (X_LAYER_ESCAPE[idx_revise]>>3<<1 != sm[0].target_position) || (PUSH_BOUND>>3<<1 != sm[1].target_position) ) {          // 与要求的位置不同时
					step_motor_goto(idx, X_LAYER_ESCAPE[idx_revise], PUSH_BOUND, 0);												          // 设置到期望位置
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_NONE) ) {/*step = 0;*/  break; }                 
				step = 8;
				break;
    	case 8: //  退出回边界位置
				idx_revise = idx + 1;
				if(idx_revise >= ARRAY_MAX_MEMBER)  idx_revise = 0;                             // 当到最高层时，改为最低层 
				if ( (X_LAYER_ESCAPE[idx_revise]>>3<<1 != sm[0].target_position) || (PULL_BOUND>>3<<1 != sm[1].target_position) ) {          // 与要求的位置不同时
					step_motor_goto(idx, X_LAYER_ESCAPE[idx_revise], PULL_BOUND, 0);										                  // 设置到期望位置
					break;
				}
				if( Trigger_Test_Error(PULL_TRAY_NONE) ) {step = 5;  break; }                   // 如果发现托盘没推回去，退回到 step = 5;        
				g_data_runtime.tested = 1;       // 完成测量 
				step = 0;
				break;    	
    	default: 
             	break;
             

	}
	return step;
}



void debug_print()
{

//      printk("iic_task_bitmap=%x\n",	g_data_runtime.iic_task_bitmap);
//	unsigned char _idx;

//	printk("\n-------------%d: %d-------------\n", g_data_runtime.tick_1s , g_data_runtime.loops_per_second );


	if (0) {
		s_motor *m;
		m = sm;
		printk("M0 T/C = %d/%d	",m->target_position *4, m->position *4);
		m = sm +1;	
		printk("M1 T/C = %d/%d	",m->target_position *4, m->position *4);
		m = sm +2;
		printk("M2 T/C = %d/%d	\n",m->target_position *4, m->position *4);

	}





	if (0) {
		s_motor *m;
		m = sm;
		printk("M0 T/C = %d/%d	tick=%d	step=%d	interval=%d	accl_pulse=%d	current_pulse=%d\n",m->target_position, m->position, m->tick, m->step, m->interval, m->accl_pulse, m->current_pulse);
		m = sm +1;	
		printk("M1 T/C = %d/%d	tick=%d	step=%d	interval=%d	accl_pulse=%d	current_pulse=%d\n",m->target_position, m->position, m->tick, m->step, m->interval, m->accl_pulse, m->current_pulse);
		m = sm +2;
		printk("M2 T/C = %d/%d	tick=%d	step=%d	interval=%d	accl_pulse=%d	current_pulse=%d\n",m->target_position, m->position, m->tick, m->step, m->interval, m->accl_pulse, m->current_pulse);
	}

	if (0) {
//		printk("P2_2=%d P2_3=%d P2_4=%d\n", FIO2PIN_bit.P2_2, FIO2PIN_bit.P2_3, FIO2PIN_bit.P2_4);
//		printk("P2_5=%d P2_6=%d P2_7=%d\n", FIO2PIN_bit.P2_5, FIO2PIN_bit.P2_6, FIO2PIN_bit.P2_7);
//		printk("P2_13=%d P2_12=%d P2_11=%d\n", FIO2PIN_bit.P2_13, FIO2PIN_bit.P2_12, FIO2PIN_bit.P2_11);
		printk("P1_14=%d P1_15=%d P1_28=%d P1_8=%d P0_18=%d\n", FIO1PIN_bit.P1_14, FIO1PIN_bit.P1_15, FIO1PIN_bit.P1_28, FIO1PIN_bit.P1_8, FIO0PIN_bit.P0_18);
	}
}


void tray_ctl()
{
	static unsigned char idx = 0, self_start_times = 0;	
	//  第一层会碰到限位开关，从1~9做测试  
 	if( idx >= 10 ) idx = 0;  	 											  // 0~9有效
 	if (!g_data_runtime.tested)  pull_push_tray(idx, ((idx + 3) & 0x7));     // 只有当 g_data_runtime.tested 为0时测做测试 
	
	if (1 == g_data_runtime.tested) {                                         //  测完成后，先将位置回到底部位置 
        if (!g_data_runtime.step_motor_init_done || g_data_runtime.step_motor_init_pending)	return;     
		if ((sm[0].target_position != sm[0].position) || (sm[1].target_position != sm[1].position))  return;
		g_data_runtime.tested = 2;
 		step_motor_goto(0, BOTTOM_LAYER, PULL_BOUND, 0);
		self_start_times  = 0;                                  // 完成后清零
	}
	if (2 == g_data_runtime.tested){                            //
		if (!g_data_runtime.step_motor_init_done || g_data_runtime.step_motor_init_pending)	return;     
		if ((sm[0].target_position != sm[0].position) || (sm[1].target_position != sm[1].position))  return;
		g_data_runtime.tested = 0;
		g_data_runtime.step_motor_init_done = 0;               // 归原点
		idx = idx + 1;
	}
	/* g_data_runtime.tested == 3 时自动测试流程出现错误， 测试时通过串口16指令激活                           */
	
	if (g_data_runtime.tested == 3) { 
		if(self_start_times < 5){        // 20 * 100ms  2秒内如果还不能正常运行就停止
			self_start_times++;
			g_data_runtime.tested = 0;    // 重新偿试 
 		}
	}
}

void routine_1000ms()
{
#ifdef Debug
	if ((g_high_stack - g_low_stack) > 0x300)	// 栈溢出告警
	printk("t=%x stack=%x|%d\n", g_tick_100us, (unsigned int)g_high_stack, g_high_stack - g_low_stack);
#endif

	if(++g_data_runtime.tick_1s > 0xf0000000)
		g_data_runtime.tick_1s = 0;
	
	if(g_data_runtime.fail_safe_tick_down > 0)
		g_data_runtime.fail_safe_tick_down--;

	if (g_data_runtime.sync_tick > 0) worker_power_cycle();
	if(0 == g_data_runtime.tick_1s %3)
		debug_print();

	gpio_board_led(LED_CMD_TOGGLE); 	
}


void routine_100ms()
{
	static unsigned int tick_100ms = 0;
	static unsigned char led_idx = 0;
	tick_100ms++;

//	static unsigned int temp_sum = 0;
		
#ifndef Debug
		WDFEED = 0xAA; // reset watchdog，初次reset会让watchdog启动;
		WDFEED = 0x55;
#endif

	if ((g_data_runtime.aligned_1s_tick + 10000) < g_tick_100us) {// led 闪灯对齐；
		g_data_runtime.aligned_1s_tick += 10000;
		gpio_led(led_idx, LED_CMD_TOGGLE);
		led_idx++;
		if (led_idx >= 4) led_idx = 0;
	}

	tray_ctl();
	
}



void routine_10ms()
{
	static unsigned int tick_10ms = 0;
	tick_10ms++;
}

void routine_1ms()
{

	static unsigned int tick_1ms = 0;

	tick_1ms++;

	if ((sm[0].target_position == sm[0].position) && (sm[1].target_position == sm[1].position) && (sm[2].target_position == sm[2].position)) {
		g_data_runtime.step_motor_move_done = 1;
		g_data_runtime.start_record = 0;
	} 
}



#define MOTOR_MAX_STEP 5000
void parameter_init()
{

	g_data_runtime.last_rs485_frame_received_tick = g_tick_100us;
	g_global_config.rs485_offline_timeout_100uS = 0xffffffff;      //1200000;	 

	g_global_config.vcc3_mcu_lower_limit_mV = 3000;
	g_global_config.vcc3_mcu_upper_limit_mV = 3500;
	g_global_config.vcc3_lower_limit_mV = 3000;
	g_global_config.vcc3_upper_limit_mV = 3500;
	g_global_config.power_on_delay_100uS = 100000;
	g_global_config.power_off_delay_100uS = 100000;
	g_data_runtime.need_to_set_main_controller_onoff = 1;
	printk("global_config size=%d\n", sizeof(g_global_config));
	
	g_data_runtime.aligned_1s_tick = g_tick_100us;

	g_data_runtime.fail_safe_tick_down = 0;

	g_data_runtime.iic_task_bitmap = 0x00;
	
	
	g_motor_config.motor_max_step[0] = MOTOR0_MAX_STEP;            // 2000;  // 5000
	g_motor_config.motor_max_step[1] = MOTOR1_MAX_STEP;            // MOTOR1_MAX_STEP;   // 2200;
	g_motor_config.motor_max_step[2] = MOTOR2_MAX_STEP;            // 3000;
	g_motor_config.motor_max_v[0] = MOTOR0_MAX_V;                  // 600000; // g_motor_config.motor_max_step[0] * 300;//380;
	g_motor_config.motor_max_v[1] = MOTOR1_MAX_V;                  // MOTOR1_MAX_V;       // 660000;// g_motor_config.motor_max_step[1] * 300;//400;
	g_motor_config.motor_max_v[2] = MOTOR2_MAX_V;                  // 750000;       // g_motor_config.motor_max_step[2] * 250;//400
	g_motor_config.motor_accel_start_step[0] = ACCEL_START_INTERVAL;  // 8000;
	g_motor_config.motor_accel_start_step[1] = ACCEL_START_INTERVAL;  // 8000;
	g_motor_config.motor_accel_start_step[2] = ACCEL_START_INTERVAL;  // 8000;
	g_motor_config.motor_total_step[0] = 1600000;                  // STEP_MOTOR_TOTAL_STEP_X; // 20000 * MOTOR_DRIVER_MICRO_STEP; // MOTOR_DRIVER_MICRO_STEP = 8
	g_motor_config.motor_total_step[1] = 45000;                    // STEP_MOTOR_TOTAL_STEP_Y; // 20000 * MOTOR_DRIVER_MICRO_STEP;
	g_motor_config.motor_total_step[2] = STEP_MOTOR_TOTAL_STEP_Z;  // 15300 * MOTOR_DRIVER_MICRO_STEP;
	g_motor_config.motor_current[0] = STEP_MOTOR_CURRENT_HIGH_mA;  // STEP_MOTOR_CURRENT_mA;   // 3000; //初始设置1A
	g_motor_config.motor_current[1] = STEP_MOTOR_CURRENT_mA;       //  3000;
	g_motor_config.motor_current[2] = 0;                           // STEP_MOTOR_CURRENT_mA; //  3000;
	g_motor_config.motor_micro_step[0] = MOTOR_0_DRIVER_MICRO_STEP;
	g_motor_config.motor_micro_step[1] = MOTOR_1_DRIVER_MICRO_STEP;                      
	g_motor_config.motor_micro_step[2] = 8;
	g_motor_config.motor_border_margin[0] = 10;                    // BOARDER_MARGIN; //  6000;
	g_motor_config.motor_border_margin[1] = 10;                    // BOARDER_MARGIN; //  6000;
	g_motor_config.motor_border_margin[2] = 0;                     // BOARDER_MARGIN * 2; //  6000;
	g_motor_config.motor_direction[0] = MOTOR0_DIR;                // MOTOR0_DIR; 
	g_motor_config.motor_direction[1] = MOTOR1_DIR;                // MOTOR1_DIR; 
	g_motor_config.motor_direction[2] = MOTOR2_DIR;                
	g_data_runtime.need_to_update_motors = 0;

		
	g_data_runtime.need_to_move = 0;
	g_data_runtime.step_motor_init_done = 0;
	g_data_runtime.step_motor_move_done = 0;

	g_data_runtime.step_motor_to_switch[0] = 0;
	g_data_runtime.step_motor_to_switch[1] = 0;
	g_data_runtime.step_motor_to_switch[2] = 0;
	g_data_runtime.sync_tick = 0;                                  
	g_data_runtime.focus_info = (focus_info_t *)ASYNC_OP_MMAP_ADDR;

	printk("check_ADC...\n");	

	for (int i = 0; i < NR_ADC_SENSORS; i++) {
		g_data_runtime.iic_task_bitmap = 1<<i;
		if ((g_data_runtime.iic_task_bitmap & ADC_AMUX_BITMAP) == 0) continue;;
		while (g_data_runtime.iic_task_bitmap){
			poll_iic();
			console_tx_proc();
			IODELAY(1000);
		}
		IODELAY(10000);

		printk("#%d = %d\n",i, g_data_runtime.mon_data[i]); 
	}
	
	g_data_runtime.iic_task_bitmap = ADC_TEMP_SET;
	while (g_data_runtime.iic_task_bitmap)
		poll_iic();
	adc_to_temp();

	printk("#kt = %d\n", g_data_runtime.k_temp); 

}
#ifdef Debug
unsigned char *g_high_stack;
volatile unsigned char *g_low_stack;
#endif

#define WAIT_DM882S_INIT_DONE           2                     // 2 秒

int main()
{
	unsigned int loop_count = 0, current_add_ = 0;            // 必须放在第一个，来标记栈底的位置
    static unsigned char wait_1s = 0, motor_first_init = 1;
	
	static unsigned int tick_1ms = 0;
//	static unsigned int tick_10ms = 0;
	static unsigned int _tick_100us = 0;
//	static unsigned int _tick_1ms;

#ifdef Debug
	g_high_stack = (unsigned char *)&loop_count;
	g_low_stack = (unsigned char *)&loop_count;                
#endif
	
	extern int flash_is_keeping_main_controller_on();
	int keep_main_controller_on_flag = flash_is_keeping_main_controller_on();
	init_all_hw();
	flash_read_backplane_eeprom();
	g_data_runtime.backplane_addr = 1;	// ChamberController总用地址1
	IRQEnable();
#ifdef boot_loader
	printk(SOLIWARE_MODEL_NAME"_bootloader: v%d %s %s\n", FIRMWARE_VER_CODE, __DATE__, __TIME__);
	flash_print_backplane_eeprom();
	console_tx_proc();
	
	flash_write_firmware_in_boot_loader_if_necessary();

	printk("JUMP\n");
	IODELAY(40000);

	IRQDisable();
	flash_go(0);

#else // !boot_loader
#ifdef Debug
	printk(SOLIWARE_MODEL_NAME"_BACKPLANE Debug: %s, %s\n", __DATE__, __TIME__);
#else
	printk(SOLIWARE_MODEL_NAME"_BACKPLANE Release: v%d %s, %s\n", FIRMWARE_VER_CODE, __DATE__, __TIME__);
#endif
	gpio_init(); // 不同PCBA 需要重新调整；
	i2c_init();
	adc_init();
	parameter_init();
//	gpio_set_main_controller_onoff(1);
	self_calibrate();
	_tick_100us = g_tick_100us;
//	_tick_1ms = 0;
	while (1) {
		if (is_time_elapsed_100us(10, _tick_100us)) {
			_tick_100us += 10;
			tick_1ms++;
			routine_1ms();
			if (0 == tick_1ms % 10) {  
				routine_10ms();
				if (0 == tick_1ms % 100) { 
					routine_100ms();
					if(0 == tick_1ms % 1000) { 	
					    routine_1000ms();
						if (wait_1s < WAIT_DM882S_INIT_DONE)  wait_1s++;					
						g_data_runtime.loops_per_second = loop_count;
						loop_count = 0;				 
					}
				}
			}
		}
		if (g_data_runtime.step_motor_init_pending){ // init pending
			if ((sm[0].target_position == sm[0].position) 
				&& (sm[1].target_position == sm[1].position)){
					g_data_runtime.step_motor_init_done = 0;
					g_data_runtime.step_motor_init_pending = 0;
					printk("ready to init...\n");
				}
		} else if (!g_data_runtime.step_motor_init_done) {
			if( (!Trigger_Test_Error(PULL_TRAY_NONE)) || (motor_first_init == 1) ) {    // 当检测到有托盘时 
		    //	printk("go to init...\n");
				if (wait_1s >= WAIT_DM882S_INIT_DONE) {
					motor_first_init = 0;
					step_motor_init();                                            // 等待DM882S驱动器初始化完成后再执行
				}
			}
		}
		if (wait_1s >= WAIT_DM882S_INIT_DONE)  step_motor_routine();              // 等待DM882S驱动器初始化完成  后再执行
		RK3288_pwr_control();
		poll_iic();
		console_rx_proc();
		console_tx_proc();
        rs485_rx_proc();

		loop_count++;
	}
#endif	// boot_loader undefined

}

/*************************************************************
181030：
1，修正上电第一次检测dut 异常的问题，是由于amux 的初始值造成的；
2，修改mpd 的测量方法，加入修正值。 避免溢出的处理有点confuse；
3，修改启动 amux 的电源电压门槛由 500mv 到 300mv，希望能够消除毛刺；





*************************************************************/

