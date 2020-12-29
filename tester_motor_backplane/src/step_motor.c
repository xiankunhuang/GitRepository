#include "config.h"
#include "backplane.h"

s_motor sm[3]; 
U32 sm_step[2000]; 


// 共约 122383 step
#define STEP_MOTOR_PULSE_MIN	400		//1.5A 驱动电路，250 开始有风险；
#define STEP_MOTOR_PULSE_MAX	10000	//20000		// 6000 起步开始有风险；
#define STEP_MOTOR_PULSE_MIN_INIT	2000	// 初始化 或者 慢慢走阶段 的 限速；

#define STEP_MOTOR_DIR_BOT2TOP	1	
#define STEP_MOTOR_DIR_TOP2BOT	0

// #define MOTOR_TIMING_BELT	1  // 同步带 或者 滚珠丝杠 



/*  同步带电机       timing belt
#define STEP_MOTOR_TOTAL_STEP_X	1000000 // 100000 // 42238
#define STEP_MOTOR_TOTAL_STEP_Y	1050000 //60000// 105000 // 122383
#define STEP_MOTOR_TOTAL_STEP_Z	70000 //70000 // 7200 // 1223 Z轴没有top开关，必须定义一个比较合适的值
#define BOARDER_MARGIN				1500

#define MOTOR0_MAX_STEP	20
#define MOTOR0_MAX_V	30000
#define MOTOR1_MAX_STEP	20
#define MOTOR1_MAX_V	30000
#define MOTOR2_MAX_STEP	10
#define MOTOR2_MAX_V	40000
*/

/**************************************
计算周期固定，计算的粒度，100us 计算一次？
设置加速的步数  和最大速度，加速度就是 smotor->max_v / smotor->max_step;
急停：根据加速一共走过的步数，设定为剩余步数，实际上由于摩擦力存在，减速应该可以更快；
假设 最大速度 100us 100次脉冲，即1us/次，最小速度 100us/次， 加速度为 1次/us；
总脉冲数为： 100*（1+100）/2 = 5050次；

T1MR0：
第1us 填入	2500/1 = 2500
第2us 填入	2500/2 = 1250
第3us 填入	2500/3 = 833
第4us 填入	2500/4 = 625
第50us 填入	2500/50 = 50
第60us 填入	2500/60 = 42
第70us 填入	2500/70 = 36
第80us 填入	2500/80 = 31
第90us 填入	2500/90 = 28
第99us 填入	2500/99 = 25
第100us 填入	2500/100 = 25
第1us 填入	25
T1MR0 = _F_PCLK/10000;		// 100us. 填入的值是 25 000 000 /10 000 = 2500

1，从0开始加速；每100us整数倍 更新一次速度 寄存器 的值，假设加速100 次 达成，需要 10ms 多一点，刚起步时候更新比较慢；
2，每次TIMER1 更新 位移值，如果位移值 达到目标距离的一半，就开始降速，否则加到最大速度后匀速；
3，根据加速阶段的 step 数量，计算降速点，到达降速点 开始降速；

最快速度： 250（10us） 脉冲开始有风险；
撞到限位开关之后，急停，延时一段时间之后，等急停完成，然后再延时一段时间 等回退完成，这个过程都忽略限位开关；
**************************************/

/*
smotor->min_cycle = 50;
smotor->max_step = 2000;
2.0A driver current; 32 micro-step;		// 偶然有风险；


*/


/* 丝杠电机 */
//#define MOTOR_DRIVER_MICRO_STEP		8

//#define STEP_MOTOR_TOTAL_STEP_X	2000 * MOTOR_DRIVER_MICRO_STEP // 600000 STEP @ 32 mciro step
//#define STEP_MOTOR_TOTAL_STEP_Y	2000000 * MOTOR_DRIVER_MICRO_STEP //60000// 105000 // 122383
//#define STEP_MOTOR_TOTAL_STEP_Z	15300  * MOTOR_DRIVER_MICRO_STEP//70000 // 7200 // 1223 Z轴没有top开关，必须定义一个比较合适的值
#define BOARDER_MARGIN	               6000

// 驱动芯片最大输入频率 100k = 10us = 250 x 40ns 
// 定时器转载的最小值 只能是 125 （单个边压） ；



/*
最大速度（周期） = MOTOR0_MAX_V / MOTOR0_MAX_STEP ， 值越小 最大速度越高；

最大加速度： MOTOR0_MAX_STEP	值越小 加速度越高；

电机驱动 手册最大 电流4A，推荐小于 70%

motor->interval = motor->max_v / motor->step ;


*/

/*********************************************************************************************************
返回值为定时器设定值，为0 表示到终点
843	100 step  
1832	1000	steps
6110	10000 steps

最大速度调高容易死掉；一般在加减速阶段挂掉，不进timer 中断

IRQDisable(); 
				 // 靠近电机一头的限位开关定义为 零点；
				motor->position = -BOARDER_MARGIN;	motor->target_position = 0;					
				motor->total_pulse = (motor->accl_pulse/2);
				motor->total_pulse += 1; // 避免为0
				motor->current_pulse = 0; 
				motor->emergency_step = 1;
				IRQEnable();

				
//#define ACCEL_INTERVAL_CPU_CYCLE	 (100000000 / 4)/10000		
// 100us. 填入的值是 25 000 000 /10 000 = 2500 个 40ns 的周期，合100us；
// 10us. 250 个 40ns 的周期，合10us；
2次定时器翻转一次，所以 10uS 周期 需要写入 125 = 250/2

输出脉冲频率 ： 定时器写入值
100K		125
50K			250
25K			500

步进电机控制器 TB67S109 输入脉冲频率最高 100K (10uS)

对于 microstep   1/8 

步 距 角-----1.8度

timer 设定250 = 250* 

电机最大转速 = 100k * 60S /( 8 * (360/1.8) ) = 6000 000 /1600 = 3750 RPM


电机转速 = 100k * 60S /( 8 * (360/1.8) ) = 6000 000 /1600 = 3750 RPM
电机转速 = 50k * 60S /( 8 * (360/1.8) ) = 1875 RPM


ACCEL_START_INTERVAL  为起步频率，对最大速度 和加速度 都没有影响； 之前用的 800

MOTOR0_MAX_V 中后面常数为 最大速度

MOTOR0_MAX_STEP 加速度，

电流3A，Y 轴 900/200/8000 基本成功；

以下基本可用
#define MOTOR0_MAX_STEP	1400	//5000
#define MOTOR0_MAX_V	(MOTOR0_MAX_STEP * 180)// 最大速度150有机会达到‘380 350	//130
#define MOTOR1_MAX_STEP	1600	//5000
#define MOTOR1_MAX_V	(MOTOR1_MAX_STEP * 200) //400 380 //150
#define MOTOR2_MAX_STEP	1200	//5000
#define MOTOR2_MAX_V	(MOTOR2_MAX_STEP * 300)	//200

*********************************************************************************************************/


// #define STEP_MOTOR_MAX_CURRENT	        5080 // 电路限制；
// #define STEP_MOTOR_CURRENT_mA	        3000
// #define STEP_MOTOR_CURRENT_LOW_mA	    800


//#define MOTOR0_MAX_STEP	2000	//1800	//5000
//#define MOTOR0_MAX_V	(MOTOR0_MAX_STEP * 300)//250 最大速度150有机会达到‘380 350	//130
//#define MOTOR1_MAX_STEP	2200	//5000
//#define MOTOR1_MAX_V	(MOTOR1_MAX_STEP * 300) //400 380 //150
//#define MOTOR2_MAX_STEP	2000 // 3000	//1500	//5000
//#define MOTOR2_MAX_V	(MOTOR2_MAX_STEP * 250)	//200



// 电机默认方向；
// #define MOTOR0_DIR   0
// #define MOTOR1_DIR	0
#define MOTOR2_DIR	1


#define NR_STEP_MOTOR	3


// #define STEP_MOTOR_TIMEOUT_100US 	10* 8000	// 超时

#define IIC_3933_RETRY	200

U8 is_step_motor_idle()
{	

	return ((sm[0].target_position == sm[0].position) && (sm[1].target_position == sm[1].position)); // && (sm[2].target_position == sm[2].position));
}

#define SLA_3933_MOTOR	0x10	//地址是0x28，右移一位，最低位是 R/W bit

#define SLA_3933_0		0x15
#define SLA_3933_1		0x10
#define SLA_3933_2		0x14

#define ADDR_3933				1	// 3933使用CR01


void step_motor_set_current(U8 idx, unsigned int mA)
{

	if(mA > STEP_MOTOR_MAX_CURRENT)
		mA = STEP_MOTOR_MAX_CURRENT;

	U8 NTC3933_sddr;
	if(0 == idx)
		NTC3933_sddr = SLA_3933_0;
	else if(1 == idx)
		NTC3933_sddr = SLA_3933_1;
	else 
		NTC3933_sddr = SLA_3933_2;
	
	U8 dac_output = (mA * 128 ) / STEP_MOTOR_MAX_CURRENT;
	dac_output |= 0x80 ;        //   bit7 : source =1; sink =0;

	U8 loops = 0;
	while(++loops < IIC_3933_RETRY){
		if (!i2c_set_NCT3933U_blocking(BUS_ID0, NTC3933_sddr, 1, dac_output)){     // BUS_ID0
			break;
		}	
	}

	if(loops < IIC_3933_RETRY)
		printk("M[%d] set_current %d done- %d, I2C%d\n", idx, mA, loops, BUS_ID0);
	else
		printk("	!!!	error: M[%d] set_current %d fail\n", idx, mA);	
	
//	IODELAY(1000);  // 修改了值稍微延迟；

}


void step_motor_set_micro_step(U8 idx, U8 micro_step)
{
//	static U16 old_value[3] = {0};
	
	if(idx >2)
		return;

	if(micro_step == 0) // standby,off;
		micro_step = 0;
	else if(micro_step == 16)
		micro_step = 6;
	else if(micro_step == 8)
		micro_step = 5;
	else if(micro_step == 4)
		micro_step = 3;
	else if(micro_step == 2)
		micro_step = 2;
	else if(micro_step == 1)
		micro_step = 1;
	else
		micro_step = 7;

//	old_value[idx] = micro_step << (idx * 3);
	
//	U16 set_value = old_value[0] | old_value[1] | old_value[2];

	gpio_set_motor_mode(idx, micro_step);
/*
	unsigned char value_3[] = { 2, set_value & 0xFF, (set_value & 0xFF00) >> 8 };
	i2c_finish_session_forcibly(1); // iic1
	
	if (i2c_write_n_bytes_blocking(1, 0x20, value_3, 3) ) {
		printk("!!!	step_motor_set_micro_step failed\n");
	}
*/
	IODELAY(1000);  // 修改了值稍微延迟；
}


/**
 * idx:  托架索引号  
 * x  
 * y  
 * z 
 ******/
void step_motor_goto(unsigned char idx, unsigned int x, unsigned int y, unsigned int z)
{
//	static U8 zero_loops = 0;

	// 除8是为了坐标变换，
	// 乘2 是为了避免丢步，因为单数脉冲在换向时候可能被丢了；
	x >>= 3;	x <<= 1;
	y >>= 3;	y <<= 1;
	z >>= 3;	z <<= 1;

	if (x > g_motor_config.motor_total_step[0]) x = g_motor_config.motor_total_step[0];
	if (y > g_motor_config.motor_total_step[1]) y = g_motor_config.motor_total_step[1];
	if (z > g_motor_config.motor_total_step[2]) z = g_motor_config.motor_total_step[2];

	if ((!x) && (!y)){
		g_data_runtime.step_motor_init_pending = 1;
		printk("need init...\n");
	}
		
	sm[0].target_position = x; // (x < BOARDER_LIMIT) ? BOARDER_LIMIT : x; 
	sm[1].target_position = y; // (y < BOARDER_LIMIT) ? BOARDER_LIMIT : y; 
	sm[2].target_position = z; // (z < BOARDER_LIMIT) ? BOARDER_LIMIT : z; 

}

// #define  MOTOR_INIT_MODE_1

// 顶点为靠近 step motor 的那个限位开关；底点为远离的；        
void step_motor_init()
{
	static char step = 0;
        /* 初始化时，先将拉托盘电机推到0点位置，再将拉托盘电机拉到最边 */
#ifdef MOTOR_INIT_MODE_1
	if (0 == step) {

		for (unsigned char i = 0; i < 2; i++ ) {
			printk("[%d]set micro_step=%d\n ", i, g_motor_config.motor_micro_step[i]);
			if (i == 1) step_motor_set_micro_step(1, g_motor_config.motor_micro_step[i]);
			memset(&sm[i], 0, sizeof(sm[0]));
		}
		
		IODELAY(40000);// 延迟等残留的中断；

		for(unsigned char i = 0; i < 2; i++ ) {                             
			sm[i].limit_up = g_motor_config.motor_total_step[i];               
			sm[i].max_step = g_motor_config.motor_max_step[i];
			sm[i].max_v = g_motor_config.motor_max_v[i] * (i == 0 ? 3 : 2);     
			sm[i].position = g_motor_config.motor_total_step[i] + (g_motor_config.motor_total_step[i]>>2);
		   	sm[i].target_position = 0;                              // 目标位置变0
			debug("[%d]limit_up=%d max_step=%d; max_v=%d, position=%d target_position=%d\n", i, 
				sm[i].limit_up, sm[i].max_step, sm[i].max_v, sm[i].position, sm[i].target_position);
		}
		step = 3;	
	} else if (3 == step) {
		if (sm[0].position != 0)
			return;

		if (sm[1].position != 0)
			return;
		
		// 修改成正常运行速度
		for(unsigned char i = 0; i < 2; i++ ) sm[i].max_v = g_motor_config.motor_max_v[i];

		printk("step_motor_init done\n");
		g_data_runtime.step_motor_init_done = 1;
		g_data_runtime.need_to_update_motors = 0;
		step = 0;
	}
#else
        switch(step)
        {
	    case 0:   for (unsigned char i = 0; i < 2; i++ ) {// 
	                  printk("[%d]set micro_step=%d\n ", i, g_motor_config.motor_micro_step[i]);
	                  step_motor_set_micro_step(i, g_motor_config.motor_micro_step[i]);
	                  memset(&sm[i], 0, sizeof(sm[0]));                          // 将 sm[] 结构体清空  
                  }
	              IODELAY(40000);        // 延迟等残留的中断；
	              for(unsigned char i = 0; i < 2; i++ ) {                            
	                  sm[i].limit_up = g_motor_config.motor_total_step[i];
	                  sm[i].max_step = g_motor_config.motor_max_step[i];
	                  sm[i].max_v = g_motor_config.motor_max_v[i] * (i == 0 ? 3 : 2);
	                  sm[i].position = g_motor_config.motor_total_step[i] + (g_motor_config.motor_total_step[i]>>2);
				
					  if (1 == i){						                         
			             sm[i].target_position =  sm[i].position;                // 上下轴先不移动 
					  }else{
					     sm[i].target_position = 0;                              // z 轴先动 (x轴电机控制)
					  }
				      debug("[%d]limit_up=%d max_step=%d; max_v=%d, position=%d target_position=%d\n", i, 
					  sm[i].limit_up, sm[i].max_step, sm[i].max_v, sm[i].position, sm[i].target_position);
			      }
			      printk("step_motor_init step %d\n", step);
			      step = 1;		     
	    	      break;
	    case 1:   if (sm[0].position != 0)      
				  	 return;
	              sm[1].target_position = 0;                                       
	              printk("M[1]step_motor_init step %d\n", step);
		          step = 4;
	    	      break;
	/*			  
	    case 2:   if (sm[1].position != 0)                                      
			         return;	
	              sm[1].target_position = (163000>>3<<1);                           
 	              printk("M[1]step_motor_init step %d\n", step);
		          step = 4;
	    	      break;	     
            case 3:  if (sm[0].position != 0)
			            return;
                     sm[1].target_position = 0;
	                 printk("step_motor_init step %d\n", step);
		             step = 4;
	    	         break;
	*/	     
	    default: if (sm[0].position != 0)
			         return;

		         //if (sm[1].position != (163000>>3<<1))
		     	 if (sm[1].position != 0)
		     	     return;
		     
	    	     for(unsigned char i = 0; i < 2; i++ ) sm[i].max_v = g_motor_config.motor_max_v[i];
			     printk("step_motor_init done\n");
			     g_data_runtime.step_motor_init_done = 1;
			     g_data_runtime.need_to_update_motors = 0;
			     printk("step_motor_init step %d\n", step);
			     step = 0;
	             break;
	}
#endif	
    /* P0_7 DISENABLE X   */	 
 	// FIO0SET_bit.P0_7 = 1;  
}




void step_motor_start(char motor_idx)
{
    if(motor_idx > 2 /*NR_MOTORS*/){
		printk("motor_idx error: %d\n", motor_idx);
		return;
	}

  	step_motor_set_current(motor_idx, g_motor_config.motor_current[motor_idx]);	
	s_motor *motor = sm + motor_idx;	
    printk("---M[%d] Target_pos: %d\n", motor_idx,  motor->target_position);

	if (motor->position == motor->target_position) {       
		return;
	}

	if (motor->position >= motor->target_position) {
		motor->total_pulse = motor->position - motor->target_position;
		motor->dir = 0;
	} else {
		motor->total_pulse = motor->target_position - motor->position;
		motor->dir = 1;
	}

//  printk("M[%d]start %d -> %d [%d]\n" ,motor_idx, motor->position, motor->target_position,motor->total_pulse * 4);

/*
    if(g_data_runtime.step_motor_init_done){
        U32 max_v;
        max_v = g_motor_config.motor_max_v[motor_idx];
    }
*/
	motor->tick = (motor->max_v / g_motor_config.motor_accel_start_step[motor_idx])  << 11;	         // 150<<11;
	motor->accl_pulse = 0;
	motor->current_pulse = 0;
//	motor->step = (motor->max_v / ACCEL_START_INTERVAL);
	motor->interval = g_motor_config.motor_accel_start_step[motor_idx];                              // 起始加速度    40ns * 8000 = 320,000ns 一秒可以走3125步， 3125/6400 约 0.5转/S;
	motor->stampe = g_tick_100us;                // 电机开始时间

	if (0 == motor_idx) {				         // 升降机
		if(motor->dir == g_motor_config.motor_direction[0])
			FIO1SET_bit.P1_17 = 1;
		else
			FIO1CLR_bit.P1_17 = 1;

		T1TC = 0;	
		T1IR_bit.MR0INT = 1; 
		T1MR0 = motor->interval; 
		T1TCR_bit.CE = 1;		
	} else if (1 == motor_idx) {		         // 拖拉托架
		if(motor->dir == g_motor_config.motor_direction[1])
			FIO1SET_bit.P1_10 = 1;
		else
			FIO1CLR_bit.P1_10 = 1;

		T2TC = 0;	
		T2IR_bit.MR0INT = 1; 
		T2MR0 = motor->interval; 
		T2TCR_bit.CE = 1;
	} 
}

/*
回原点的过程，当碰到开关，开始减速到停止，然后往回再走一个安全边界，这个过程开关可能一致触发，需要注意；

*/
void step_motor_routine( )
{
//    static U32 stamp = 0;   
	volatile s_motor *motor;
	for(U8 motor_idx = 0; motor_idx < 2; motor_idx++){
		char e = 0;
		motor = sm + motor_idx;
		if (0 == motor->total_pulse) { // 空闲
			if(motor->step == 0){   
				if(motor->stampe > 0){
//					printk("M[%d]Done. time elapses:	%d\n",motor_idx, (g_tick_100us - motor->stampe) );
					step_motor_set_current(motor_idx, STEP_MOTOR_CURRENT_LOW_mA);
					motor->stampe = 0;
				}
			}

			if (motor->position != motor->target_position) {      // 有pending的任务
				if(motor->target_position > motor->limit_up + 10) // 范围检测；范围异常是不是都不该动？
					motor->target_position = motor->limit_up;
				else if(motor->target_position < 0){              
					printk("---M[%d] T_position: %d\n", motor_idx, motor->target_position);
					motor->target_position = 0;
				}
			
				step_motor_start(motor_idx);
			} 
	    } else {
			if (is_time_elapsed_100us(STEP_MOTOR_TIMEOUT_100US, motor->stampe)) {  // timeout;
				e = 5; 				
				g_data_runtime.error_code = 0x10 + motor_idx;                      // 0x10, 0x11,0x12 timeout err code 
				printk("M[%d] timeout, stampe=%d tick=%d\n", motor_idx, motor->stampe, g_tick_100us);
				motor->stampe = g_tick_100us;
//				g_data_runtime.step_motor_init_done = 0;
				if (motor_idx == 0) T1TCR_bit.CE = 1;
				else if (motor_idx == 1) T2TCR_bit.CE = 1;
                                   
				return;
			}
			
			if( motor->emergency_step ){	                          // 延时等回退； 原点限位时
			    
				if (motor->position != motor->target_position)  {
					continue;
				}
				motor->emergency_step = 0;
			}

			// 急停检测；光电开关低有效，机械开关高有效； 靠近电机一头的限位开关定义为 零点；
			U8 i = 0;
			if (0 == motor_idx) {                             // 拖曳托架
				while (FIO1PIN_bit.P1_15 == 0){               // 升降机上端   到最上方时为0， 其它为 1
					 if(++i > 5) {e = 2; printk("[0]top\n"); break; }
						
				}
				
				i = 0;
				while (FIO1PIN_bit.P1_28 == 0){   	          // 升降机下端       到最下方时为 0， 其它为 1
					if(++i > 10){ e = 1; printk("[0]bottom\n");break;}						
				}
			} else if (1 == motor_idx) {	                  //  拖拉托架电机
				
				while (FIO1PIN_bit.P1_8 == 1){                //  向里推托架，到位限位
					if(++i > 10) { e = 3;  printk("[1]top\n"); break;}
				}

				i = 0;
				while (FIO1PIN_bit.P1_14 == 0){               //  向外拉托架，也是0限位
					 if(++i > 5) {e = 1; printk("[1]bottom\n"); break; }
						
				}
	/*   托盘检是否到位时用
				while (FIO0PIN_bit.P0_18 == 0){
					if(++i > 10) { e = 5; break;}
				}
	*/
			} 
		
			if  (e == 1) {
				 if( !motor->accl_pulse )
					 continue;
				 
				 IRQDisable(); 
				 // 靠近电机一头的限位开关定义为 零点；
				 motor->position = -g_motor_config.motor_border_margin[motor_idx];	// 当前限位触发时当做 1990
				 motor->target_position = 0;
				 motor->total_pulse = motor->accl_pulse;
				 motor->total_pulse += 1;		                                    // 避免为0
				 motor->current_pulse = 0; 
				 motor->emergency_step = 1;      
				 IRQEnable();
			
				 if (motor_idx == 0) T1TCR_bit.CE = 1;
				 else if (motor_idx == 1) T2TCR_bit.CE = 1;
			
			} else if (e == 2) {
				if( !motor->accl_pulse )
					continue;
				// 靠近该限位时，不做处理
			//	IRQDisable(); 				
			//	motor->limit_up = motor->position - g_motor_config.motor_border_margin[motor_idx];
			//	motor->position = -g_motor_config.motor_border_margin[motor_idx];
		    //    if(motor->target_position > (motor->position + g_motor_config.motor_border_margin[motor_idx]) ){
		    //        motor->target_position = motor->position + g_motor_config.motor_border_margin[motor_idx]; 
            //        motor->total_pulse = (motor->accl_pulse/2);     // 
            //        motor->total_pulse += 1;                        // 避免为0
            //        motor->current_pulse = 0; 
            //        motor->emergency_step = 1;
            //    }
			//	IRQEnable();
			//	if (motor_idx == 0) T1TCR_bit.CE = 1;
			//	else if (motor_idx == 1) T2TCR_bit.CE = 1;				 
			}else if (e == 3){                                      // 托盘向里推已到位 
            /*    if( !motor->accl_pulse )
                    continue; 
				
				IRQDisable(); 
				 // 靠近电机一头的限位开关定义为 零点；
				if (motor->position < motor->target_position)
					motor->position = motor->target_position + 1;				

				// if (motor->target_position > 1000)
				//		motor->position = motor->target_position;    
				motor->total_pulse = 1;       
				motor->total_pulse += 1;                             // 避免为0
				motor->current_pulse = 0; 
				// motor->emergency_step = 1;
				IRQEnable();
			    if (motor_idx == 0) T1TCR_bit.CE = 1;
				else if (motor_idx == 1) T2TCR_bit.CE = 1;
			*/				  
             }
                     
		}
		
	}

}


void print_step_motor_postion()
{
	int position[2], target_postion[2];
	
	printk("Postion: ");
	for (int i = 0; i < 2; i++) {
		position[i] = sm[i].position;
		target_postion[i] = sm[i].target_position;
		printk("[%d] %d->%d, ", i, position[i], target_postion[i]);
	}
	printk("\n");
}



 
