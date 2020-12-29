#include "lpc1700.h"
#include "backplane.h"



/*********************************************************************************************************
** Function name:	     SysTick_Handler
** Descriptions:	     系统定时器中断函数
** input parameters:     无
** output parameters:    无
** Returned value:       无
*********************************************************************************************************/
void SysTick_Handler(void)
{
    volatile unsigned int Dummy = STCTRL;                                     /* 清中断标志                   */
}


/*********************************************************************************************************
* Function Name  : NMIException
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void Nmi_Handler(void)
{
    while (1);
}
/*********************************************************************************************************
* Function Name  : DebugMonitor
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void DebugMon_Handler(void)
{
}

/*********************************************************************************************************
* Function Name  : PendSVC
* Description    : This function handles PendSVC exception.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void PendSV_Handler(void)
{
}

/*********************************************************************************************************
* Function Name  : WWDG_IRQHandler
* Description    : This function handles WWDG interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void WWDG_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : TIMER0_IRQHandler
* Description    : This function handles TIMER0 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void TIMER0_IRQHandler(void)
{
	extern volatile unsigned int g_tick_100us;
#ifdef Debug
	// 看看栈到哪儿了
	extern volatile unsigned char *g_low_stack;
	unsigned char low_stack_is_here;
	if (g_low_stack > &low_stack_is_here)
		g_low_stack = &low_stack_is_here;
#endif
	T0IR_bit.MR0INT = 1;
	g_tick_100us++;
	T0TCR_bit.CE = 1;
}

//#define ACCEL_INTERVAL_CPU_CYCLE	 (100000000 / 4)/10000		
// 40ns 周期，32bit 可以 160S 不溢出；
// 100us. 填入的值是 25 000 000 /10 000 = 2500 个 40ns 的周期，合100us；
#define ACCEL_INTERVAL_CPU_CYCLE	 250

#define MOTOR_ACCELERATION	400

#define MOTOR_MAX_V		400	//350

#define MOTOR_START_V	(5000 * MOTOR_MAX_V)



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

步进电机控制器 TB67S109 输入脉冲频率最高 100K (10uS)




*********************************************************************************************************/
//U32 motor_ISR3(char m_idx)
//{
//IRQDisable(); 
//
//
//	s_motor *motor = sm + m_idx;
//
//	if (motor->target_position != motor->position) {
//		if (motor->dir)
//			motor->position++;
//		else
//			motor->position--;
//	}else
//		printk("--------$\n");
//
//	motor->current_pulse++;
//		
//	if (motor->current_pulse >= motor->total_pulse) { //到终点；	
//		motor->total_pulse = 0;
//		motor->accl_pulse = 0;
//		motor->current_pulse = 0;
//		motor->step = 0;
//		motor->interval = 0;
//	} else if ((motor->current_pulse + (motor->accl_pulse/2 )) >= motor->total_pulse) {	//减速
//
//	if(motor->tick > motor->interval)
//		motor->tick -= motor->interval;
//
//		motor->step = (motor->tick / (MOTOR_ACCELERATION /2));
//	
//		if(motor->step < 1)
//			motor->step = 1;
//
//		motor->interval = MOTOR_START_V / motor->step;
//
//	} else if(motor->interval > MOTOR_MAX_V){ // 加速
//		motor->accl_pulse++;
//
//		motor->tick += motor->interval;
//
//		motor->step = (motor->tick / MOTOR_ACCELERATION);
//		
//		if(motor->step < 10)
//			motor->step = 10;
//
////		if(motor->step >= motor->max_step){
////			printk("			accl_pulse =  %d\n", motor->accl_pulse);
////			motor->step = motor->max_step;
////		}
//
//
//		motor->interval = MOTOR_START_V / motor->step ;
//
//
//		
//		if(motor->interval <= MOTOR_MAX_V){
//			printk("			accl_pulse =  %d\n", motor->accl_pulse);
//			motor->interval = MOTOR_MAX_V;
//		}
//
//	}// 否则匀速；
//
//IRQEnable();
//
//
//	return motor->interval ;
//
//
//
////	motor->interval = MOTOR_START_V * MOTOR_ACCELERATION/ motor->tick ;
//
//
//
//
//}
//
///*
//4870	100 step  
//7998	1000	steps
//11503	10000 steps
//
//*/
//
//U32 motor_ISR0(char m_idx)
//{
//	s_motor *motor = sm + m_idx;
//
//	if (motor->target_position != motor->position) {
//		if (motor->dir)
//			motor->position++;
//		else
//			motor->position--;
//	}
//
//	motor->current_pulse++;
//		
//	if (motor->current_pulse >= motor->total_pulse) { //到终点；	
//		motor->total_pulse = 0;
//		motor->accl_pulse = 0;
//		motor->current_pulse = 0;
//		motor->step = 0;
//		return 0;
//	} else if ((motor->current_pulse + (motor->accl_pulse )) >= motor->total_pulse) {	//减速
//		motor->tick += motor->interval;
//		if(motor->tick > ACCEL_INTERVAL_CPU_CYCLE){ // 计算周期；2500
//			if(motor->step > ACCEL_START_INTERVAL){
//				motor->step --;
//				motor->interval = motor->max_v / motor->step;
//				motor->tick = 0;
//			}
//		}
//	} else if(motor->step < motor->max_step){ // 加速
//		motor->accl_pulse++;
//
//		motor->tick += motor->interval;		
//		if(motor->tick > ACCEL_INTERVAL_CPU_CYCLE){ // 计算周期；
//			if(motor->step < motor->max_step){
//				motor->step++;
//				motor->interval = motor->max_v / motor->step ;
//				motor->tick = 0;
//			}
//		}
//	}// 否则匀速；
//
//	return motor->interval ;
//}

// 每角度对应一个值 值为乘10000
U32 motor_ISR(char m_idx)
{
	s_motor *motor = sm + m_idx;
	 
	if (motor->target_position != motor->position) {                                      // 计算当前位置点
		if (motor->dir)
			motor->position++;
		else
			motor->position--;
	}

	motor->current_pulse++;
		
	if (motor->current_pulse >= motor->total_pulse) {                                     // 到终点；	
		motor->total_pulse = 0;
		motor->accl_pulse = 0;
		motor->current_pulse = 0;
		motor->step = 0;
		return 0;
	} else if ((motor->current_pulse + (motor->accl_pulse /2)) >= motor->total_pulse) {  // 减速

		if(motor->tick > motor->interval)
			motor->tick -= (motor->interval *2);

		motor->step = (motor->tick >> 11);

		if(motor->step < 100)                                                              // 避免被0除；
			motor->step = 100; 		

		if(motor->step > motor->max_v)
			motor->step = motor->max_v;
 
		motor->interval = motor->max_v / motor->step;
/*
		if(motor->interval > ACCEL_START_INTERVAL)
			motor->interval = ACCEL_START_INTERVAL;
*/		
	} else if(motor->step < motor->max_step){                                             // 加速
		motor->accl_pulse++;

		motor->tick += motor->interval;                                 // 加速用了多少tick 
		motor->step = (motor->tick >> 11);                              // 最小 step  

		if(motor->step < 100)                   						// 避免被0除；
			motor->step = 100; 

		if(motor->step >= motor->max_step){
			motor->step = motor->max_step;
		}			

		if(motor->step > ACCEL_START_INTERVAL)
			motor->step = ACCEL_START_INTERVAL;

		motor->interval = motor->max_v / motor->step;
		
	}// 否则匀速；

	if(!motor->interval){
		printk("M[%d] motor->max_v = %d,  motor->step = %d", m_idx, motor->max_v , motor->step);
		motor->total_pulse = 0;
		motor->accl_pulse = 0;
		motor->current_pulse = 0;
		motor->step = 0;
		return 0;
	}
	
	return motor->interval ;
}

/*********************************************************************************************************
* Function Name  : TIMER1_IRQHandler
* Description    : This function handles TIMER1 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/


void TIMER1_IRQHandler(void)
{
	T1IR_bit.MR0INT = 1;   // Writing a logic one to the corresponding IR bit will reset the interrupt
	gpio_motor_pulse_x(0);
	U32 v = motor_ISR(0);
	if( 0 == v){            //到终点；
            T1TCR_bit.CE = 0;	// 这个一定要；
            return;
	}

	//Match Register 0. MR0 can be enabled through the MCR to reset the TC, stop both the TC and PC, and/or generate an interrupt every time MR0 matches the TC.	   // 100us
	T1MR0 = v; // 单位为 40ns
	T1TCR_bit.CE = 1;
}


/*********************************************************************************************************
* Function Name  : TIMER2_IRQHandler
* Description    : This function handles TIMER2 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void TIMER2_IRQHandler(void)
{
	T2IR_bit.MR0INT = 1;   // Writing a logic one to the corresponding IR bit will reset the interrupt
	gpio_motor_pulse_y(0);
	U32 v = motor_ISR(1);
	if( 0 == v){                    // 到终点；
		T2TCR_bit.CE = 0;	        // 这个一定要；
		return;
	}		

	//Match Register 0. MR0 can be enabled through the MCR to reset the TC, stop both the TC and PC, and/or generate an interrupt every time MR0 matches the TC.	   // 100us
	T2MR0 = v; // 单位为 40ns
	T2TCR_bit.CE = 1;
}

/*********************************************************************************************************
* Function Name  : TIMER3_IRQHandler
* Description    : This function handles TIMER3 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void TIMER3_IRQHandler(void)
{
	T3IR_bit.MR0INT = 1; // Writing a logic one to the corresponding IR bit will reset the interrupt
	gpio_motor_pulse_z(0);
	U32 v = motor_ISR(2);
	
	if( 0 == v){ //到终点；
		T3TCR_bit.CE = 0;	// 这个一定要；
		return;
	}

	//Match Register 0. MR0 can be enabled through the MCR to reset the TC, stop both the TC and PC, and/or generate an interrupt every time MR0 matches the TC.	   // 100us
	T3MR0 = v; // 单位为 40ns
	T3TCR_bit.CE = 1;
}

/*********************************************************************************************************
* Function Name  : UART0_IRQHandler
* Description    : This function handles UART0 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void UART0_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : UART1_IRQHandler
* Description    : This function handles UART1 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void UART1_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : UART2_IRQHandler
* Description    : This function handles UART2 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void UART2_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : UART3_IRQHandler
* Description    : This function handles UART3 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void UART3_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : PWM1_IRQHandler
* Description    : This function handles PWM1 interrupt request. 
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void PWM1_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : I2C0_IRQHandler
* Description    : This function handles I2C0 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
extern void i2c_session_isr(const int bus_id);
void I2C0_IRQHandler(void)
{
	i2c_session_isr(0);
}

/*********************************************************************************************************
* Function Name  : I2C1_IRQHandler
* Description    : This function handles I2C1 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void I2C1_IRQHandler(void)
{
	i2c_session_isr(1);
}

/*********************************************************************************************************
* Function Name  : I2C2_IRQHandler
* Description    : This function handles I2C2 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void I2C2_IRQHandler(void)
{
	i2c_session_isr(2);
}

/*********************************************************************************************************
* Function Name  : I2C3_IRQHandler
* Description    : This function handles I2C3 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void I2C3_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : SPI_IRQHandler
* Description    : This function handles SPI interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void SPI_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : SSP0_IRQHandler
* Description    : This function handles SSP0 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void SSP0_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : SSP1_IRQHandler
* Description    : This function handles SSP1 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void SSP1_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : PLL0_IRQHandler
* Description    : This function handles PLL0 interrupt request
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void PLL0_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : RTC_IRQHandler
* Description    : This function handles RTC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void RTC_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : EXTI0_IRQHandler
* Description    : This function handles External interrupt Line 0 request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void EINT0_IRQHandler(void)
{

}

/*********************************************************************************************************
* Function Name  : EXTI1_IRQHandler
* Description    : This function handles External interrupt Line 1 request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void EINT1_IRQHandler(void)
{  

}

/*********************************************************************************************************
* Function Name  : EINT2_IRQHandler
* Description    : This function handles External interrupt Line 2 request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void EINT2_IRQHandler(void)
{
}

/*********************************************************************************************************
** Function name:	     EINT3_GPIO_Handler
** Descriptions:	      
** input parameters:     无
** output parameters:    无
** Returned value:       无
**                       Note：EINT3与GPIO共用同一个中断号
*********************************************************************************************************/
void  EINT3_GPIO_Handler(void)
{

}

/*********************************************************************************************************
* Function Name  : ADC_IRQHandler
* Description    : This function handles ADC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void ADC_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : BOD_IRQHandler
* Description    : This function handles BOD interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void BOD_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : USB_IRQHandler
* Description    : This function handles USB interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void USB_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : CAN_IRQHandler
* Description    : This function handles CANinterrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void CAN_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : GPDMA_IRQHandler
* Description    : This function handles DMA interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void GPDMA_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : I2S_IRQHandler
* Description    : This function handles I2S interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void I2S_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : Ethernet_IRQHandler
* Description    : This function handles Ethernet interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void Ethernet_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : RIT_IRQHandler
* Description    : This function handles RIT interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void RIT_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : MC_IRQHandler
* Description    : This function handles MC interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void MC_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : QE_IRQHandler
* Description    : This function handles QE interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void QE_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : PLL1_IRQHandler
* Description    : This function handles PLL1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void PLL1_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : USBAct_IRQHandler
* Description    : This function handles USBAct interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void USBAct_IRQHandler(void)
{
}

/*********************************************************************************************************
* Function Name  : CANAct_IRQHandler
* Description    : This function handles CANAct interrupt request.
* Input          : None
* Output         : None
* Return         : None
*********************************************************************************************************/
void CANAct_IRQHandler(void)
{
}

/*********************************************************************************************************
**                                      End Of File
*********************************************************************************************************/


