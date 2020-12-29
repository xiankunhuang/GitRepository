#include "config.h"
#include "backplane.h"


// 所有硬件初始化都在这里完成，启动时被调用一次
void init_all_hw(void)
{
	IRQDisable();
	PinInit();
	console_init();

	// 初始化watchdog之后狗是关的，第一次reset狗会让狗启动
	WDTC = 2000000;		// 2s，默认时钟源频率为_F_CCLK
	WDMOD = 0x3;		// WDEN | WRESET 

	rs485_init();

	// timer0 for tick ISR
	extern void TIMER0_IRQHandler(void);
	PCONP_bit.PCTIM0 = 1;
	T0TC = 0;
	T0PR = 0;
	T0MR0 = _F_PCLK/10000;		//  100us. 填入的值是 25 000 000 /10 000 = 2500 个 40ns 的周期，合100us；
	T0MCR = 0x07;
	T0TCR = 0x01;
	vicIrqFuncSet(NVIC_TIMER0, 21, (unsigned int)TIMER0_IRQHandler);	// 30 把时钟中断设为高优先级
	vicIrqEnable(NVIC_TIMER0);

	// timer1 for step motor ISR
	extern void TIMER1_IRQHandler(void);
	PCONP_bit.PCTIM1 = 1;
	T1TC = 0; //Timer Counter. The 32-bit TC is incremented every PR+1 cycles of PCLK. The TC is controlled through the TCR.
	T1PR = 0; //Prescale Register. When the Prescale Counter (below) is equal to this value, the next clock increments the TC and clears the PC.
	T1MR0 = _F_PCLK/100; //Match Register 0. MR0 can be enabled through the MCR to reset the TC, stop both the TC and PC, and/or generate an interrupt every time MR0 matches the TC.		// 100us

// 有待商榷：T1MCR.bit3	Stop on MR0: the TC and PC will be stopped and TCR[0] will be set to 0 if MR0 matches	the TC.
	T1MCR = 0x03;

//	T1MCR = 0x07;// Match Control Register. The MCR is used to control if an interrupt is generated and if the TC is reset when a Match occurs
	T1TCR = 0x01;//Timer Control Register. bit0:Counter Enable //The Timer Counter can be disabled or reset through the TCR.
	vicIrqFuncSet(NVIC_TIMER1, 23, (unsigned int)TIMER1_IRQHandler);//25	// 设为较低优先级
	vicIrqEnable(NVIC_TIMER1);

	extern void TIMER2_IRQHandler(void);
	PCONP_bit.PCTIM2 = 1;
	T2TC = 0;
	T2PR = 0;
	T2MR0 = _F_PCLK / 10;
	T2MCR = 0x03;
	T2TCR = 0x01;
	vicIrqFuncSet(NVIC_TIMER2, 24, (unsigned int)TIMER2_IRQHandler);	//24
	vicIrqEnable(NVIC_TIMER2);

	extern void TIMER3_IRQHandler(void);
	PCONP_bit.PCTIM3 = 1;
	T3TC = 0;
	T3PR = 0;
	T3MR0 = _F_PCLK / 100;
	T3MCR = 0x03;
	T3TCR = 0x01;
	vicIrqFuncSet(NVIC_TIMER3, 22, (unsigned int)TIMER3_IRQHandler);	//26
	vicIrqEnable(NVIC_TIMER3);

// enable ADC;
	PCONP_bit.PCAD =1;

	// 注意DAC不需要初始化，默认配置直接可以用，默认没有DMA和INT
}

// 这个会被Startup.s调，所以不能是static
// 不知道为什么要这样实现，明明用最初的框架代码也可以工作得很好
// - Flash program frequence
// - PLL setings
// - Peripheral clock settings
 void targetResetInit (void)
{  
	FLASHCFG_bit.FLASHTIM = 0x04;  /*  Flash访问使用4个CPU时钟     */

	/* main oscillator enable */
	SCS_bit.OSCRANGE = 1;
	SCS_bit.OSCEN = 1;
	
	/* set PLL0 */
    if (PLL0STAT_bit.PLLE){
		/* 1. Disconnect PLL0 with one feed sequence if PLL0 is already connected. */
		PLL0CON = 1;
		PLL0FEED = 0xAA;
		PLL0FEED = 0x55;
    }
	/* 2. Disable PLL0 with one feed sequence. */
    PLL0CON = 0;
    PLL0FEED = 0xAA;
    PLL0FEED = 0x55;
	/* 等待锁相环0的状态标志归位 */
    while (PLL0STAT & (1 << 25));	

	/* 3. Change the CPU Clock Divider setting to speed up operation without PLL0, if desired. */
	// CCLKCFG = (CCLKDivValue - 1);

	/* 4. Write to the Clock Source Selection Control register to change the clock source if needed. */
	CLKSRCSEL = 0x01;  // main oscillator

	/* 5. Write to the PLL0CFG and make it effective with one feed sequence. The PLL0CFG can only be updated when PLL0 is disabled. */
    PLL0CFG  = (((_PLL_N_VALUE - 1) << 16)|(_PLL_M_VALUE - 1));    
    PLL0FEED = 0xAA;                                                    /* Enable but disconnect the PLL*/
    PLL0FEED = 0x55;	

	/* 6. Enable PLL0 with one feed sequence. */
	PLL0CON = 1;
    PLL0FEED = 0xAA;                                                    /* Enable but disconnect the PLL*/
    PLL0FEED = 0x55;	
    while (PLL0STAT_bit.PLLE == 0);								        /* Wait until the PLL is usable */

	/* 7. Change the CPU Clock Divider setting for the operation with PLL0. It is critical to do this before connecting PLL0. */
	CCLKCFG = (_CCLK_DIV_VALUE - 1);
	
	/* 8. Wait for PLL0 to achieve lock by monitoring the PLOCK0 bit in the PLL0STAT register */
	while ( ((PLL0STAT & (1 << 26)) == 0) );						    /* Check lock bit status 		*/
	while (((PLL0STAT & 0x00007FFF) != (_PLL_M_VALUE - 1)) && 
           (((PLL0STAT & 0x00FF0000) >> 16) != (_PLL_N_VALUE - 1)));

	/* 9. Connect PLL0 with one feed sequence. */
 	PLL0CON  = 3;														/* connect the PLL 				*/
    PLL0FEED = 0xAA;
    PLL0FEED = 0x55;	    						
	while (((PLL0STAT & (1 << 25))!= (1 << 25)));						/* Wait until the PLL is 		*/ 
																		/* connected and locked 		*/	
	/*  
	 *  Set system timers for each component  
	 */
	/* PCLK is 1/4 CCLK */
	PCLKSEL0 = 0x00000000;												
    PCLKSEL1 = 0x00000000;

    CLKOUTCFG = 0;

	return;
}

// g_tick_100us大约每5天溢出一次，因此用这个值的时候必须做溢出检查
// !0 则已经流逝了超过cnt*100us，==0则还没流逝那么久
unsigned char is_time_elapsed_100us(int cnt, unsigned int prev_tick)
{
	unsigned int now = g_tick_100us;
	if (now >= prev_tick) {
		return ((now - prev_tick) > cnt);
	} else {		// 溢出了
		return ((0xFFFFFFFF - prev_tick + now) > cnt);
	}
}

__noreturn void boot_jump(unsigned int firmwareStartAddress)
{
	VTOR = firmwareStartAddress;
	__asm("  LDR SP, [R0]   \n" " LDR PC, [R0, #4]  ");
	// LDR SP, [R0]			;Load new stack pointer address
	// LDR PC, [R0, #4]		;Load new program counter address
}

