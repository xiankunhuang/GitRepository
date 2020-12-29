;/****************************************Copyright (c)**************************************************
;**                               Guangzou ZLG-MCU Development Co.,LTD.
;**                                      graduate school
;**                                 http://www.zlgmcu.com
;**
;**--------------File Info-------------------------------------------------------------------------------
;** File name: 	          Startup.s
;** Last modified Date:   2009-06-10
;** Last Version:         V1.00
;** Descriptions:         The start up codes for LPC1700, including the initializing codes for the entry 
;**                       point of exceptions and the stacks of user tasks.ry project should have a 
;**                       independent copy of this file for related modifications
;**------------------------------------------------------------------------------------------------------
;** Created by:           LinEnqiang	
;** Created date:   	  2009-06-10
;** Version:              V1.00
;** Descriptions:         The original version
;**
;**------------------------------------------------------------------------------------------------------
;** Modified by:          
;** Modified date:        
;** Version:              
;** Descriptions:         
;********************************************************************************************************/

;/********************************************************************************************************
; The imported labels    
; 引入的外部标号在这声明
;********************************************************************************************************/     
        EXTERN  __RunFirst
        EXTERN  __iar_program_start 
        EXTERN  __vicControl
        EXTERN  __SoftWareInterrupt        
        EXTERN  Nmi_Handler
        EXTERN  PendSV_Handler
        EXTERN  SysTick_Handler
        EXTERN  DebugMon_Handler
        EXTERN  targetResetInit
        EXTERN  WWDG_IRQHandler
        EXTERN  TIMER0_IRQHandler
        EXTERN  TIMER1_IRQHandler
        EXTERN  TIMER2_IRQHandler
        EXTERN  TIMER3_IRQHandler
        EXTERN  UART0_IRQHandler
        EXTERN  UART1_IRQHandler
        EXTERN  UART2_IRQHandler
        EXTERN  UART3_IRQHandler
        EXTERN  PWM1_IRQHandler
        EXTERN  I2C0_IRQHandler
        EXTERN  I2C1_IRQHandler
        EXTERN  I2C2_IRQHandler
        EXTERN  SPI_IRQHandler
        EXTERN  SSP0_IRQHandler
        EXTERN  SSP1_IRQHandler
        EXTERN  PLL0_IRQHandler
        EXTERN  RTC_IRQHandler
        EXTERN  EINT0_IRQHandler
        EXTERN  EINT1_IRQHandler
        EXTERN  EINT2_IRQHandler
        EXTERN  EINT3_GPIO_Handler
        EXTERN  ADC_IRQHandler
        EXTERN  BOD_IRQHandler
        EXTERN  USB_IRQHandler
        EXTERN  CAN_IRQHandler
        EXTERN  GPDMA_IRQHandler
        EXTERN  I2S_IRQHandler
        EXTERN  Ethernet_IRQHandler
        EXTERN  RIT_IRQHandler
        EXTERN  MC_IRQHandler
        EXTERN  QE_IRQHandler
        EXTERN  PLL1_IRQHandler
        EXTERN  USBAct_IRQHandler
        EXTERN  CANAct_IRQHandler   
;/********************************************************************************************************
; The emported labels    
; 给外部使用的标号在这声明
;********************************************************************************************************/
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c        
        
        MODULE  ?Startup   
        RSEG CSTACK:DATA:NOROOT(3)     
;/********************************************************************************************************
; The vectors 
; 异常向量表
;********************************************************************************************************/ 
        ASEGN .intvec:CODE:NOROOT(2)  		
        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     __ResetInit         
        DCD     Nmi_Handler
        DCD     __HardFault_Handler
        DCD     __MemManage_Handler
        DCD     __BusFault_Handler
        DCD     __UsageFault_Handler
__vector_table_0x1c
        DCD     0 
        DCD     0 
        DCD     0 
        DCD     0 
        DCD     __Svc_Handler 
        DCD     DebugMon_Handler
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler
;/********************************************************************************************************
;   外设中断向量表
;********************************************************************************************************/         
        DCD     WWDG_IRQHandler                                         ;/* 16 WDT                      */
        DCD     TIMER0_IRQHandler                                       ;/* 17 TIMER0                   */
        DCD     TIMER1_IRQHandler                                       ;/* 18 TIMER1                   */
        DCD     TIMER2_IRQHandler                                       ;/* 19 TIMER2                   */
        DCD     TIMER3_IRQHandler                                       ;/* 20 TIMER3                   */
        DCD     UART0_IRQHandler                                        ;/* 21 UART0                    */
        DCD     UART1_IRQHandler                                        ;/* 22 UART1                    */
        DCD     UART2_IRQHandler                                        ;/* 23 UART2                    */
        DCD     UART3_IRQHandler                                        ;/* 24 UART3                    */
        DCD     PWM1_IRQHandler                                         ;/* 25 PWM1                     */
        DCD     I2C0_IRQHandler                                         ;/* 26 I2C0                     */
        DCD     I2C1_IRQHandler                                         ;/* 27 I2C1                     */
        DCD     I2C2_IRQHandler                                         ;/* 28 I2C2                     */
        DCD     SPI_IRQHandler                                          ;/* 29 SPI                      */
        DCD     SSP0_IRQHandler                                         ;/* 30 SSP0                     */
        DCD     SSP1_IRQHandler                                         ;/* 31 SSP1                     */
        DCD     PLL0_IRQHandler                                         ;/* 32 PLL0                     */
        DCD     RTC_IRQHandler                                          ;/* 33 RTC                      */
        DCD     EINT0_IRQHandler                                        ;/* 34 EINT0                    */
        DCD     EINT1_IRQHandler                                        ;/* 35 EINT1                    */
        DCD     EINT2_IRQHandler                                        ;/* 36 EINT2                    */
        DCD     EINT3_GPIO_Handler                                      ;/* 37 EINT3                    */
        DCD     ADC_IRQHandler                                          ;/* 38 ADC                      */
        DCD     BOD_IRQHandler                                          ;/* 39 BOD                      */
        DCD     USB_IRQHandler                                          ;/* 40 USB                      */
        DCD     CAN_IRQHandler                                          ;/* 41 CAN                      */
        DCD     GPDMA_IRQHandler                                        ;/* 42 GP DMA                   */
        DCD     I2S_IRQHandler                                          ;/* 43 I2S                      */
        DCD     Ethernet_IRQHandler                                     ;/* 44 Ethernet                 */
        DCD     RIT_IRQHandler                                          ;/* 45 Repete Interrupt Timer   */
        DCD     MC_IRQHandler                                           ;/* 46 Motor Control PWM        */
        DCD     QE_IRQHandler                                           ;/* 47 Quadrature Encoder       */
        DCD     PLL1_IRQHandler                                         ;/* 48 PLL1                     */ 
        DCD     USBAct_IRQHandler                                       ;/* 49 USB_NEED_CLK             */
        DCD     CANAct_IRQHandler                                       ;/* 50 CAN1WAKE, CAN2WAKE       */ 
        
        RSEG  HANDER:CODE:REORDER(2)
        THUMB        
__HardFault_Handler
        B .              
__MemManage_Handler
        B .
__BusFault_Handler
        B .
__UsageFault_Handler
        B .       
__IntDefault_Handler
        B .         
;/********************************************************************************************************
;** Function name:          __SvcHandler
;**
;** Descriptions:           SVC处理
;**
;** input parameters:       None
;** Returned value:         None
;**
;** Created by:             LinEnqiang
;** Created Date:           2009-06-10
;**-------------------------------------------------------------------------------------------------------
;** Modified by:            
;** Modified date:          
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/  
__Svc_Handler 
        TST     LR, #4                                                     
        ITE     EQ 
        MRSEQ   R2, MSP 
        MRSNE   R2, PSP 
        LDR     R0, [R2, #0]        
        LDR     R1, [R2, #24]
        LDRB    R1, [R1, #-2] 
        CMP     R0, #0x100 
        IT      LO
        BLO     __SoftWareInterrupt
        B       __vicControl  
;/********************************************************************************************************
;** Function name:          __ResetInit
;**
;** Descriptions:           复位处理
;**
;** input parameters:       None
;** Returned value:         None
;**
;** Created by:             LinEnqiang
;** Created Date:           2009-06-10
;**-------------------------------------------------------------------------------------------------------
;** Modified by:            
;** Modified date:          
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/  
__ResetInit 
        BL      __RunFirst
        BL      targetResetInit         		                        ;/*  目标板基本初始化           */         
        B       __iar_program_start          
        
;/********************************************************************************************************
;** Function name:          CrpData
;**
;** Descriptions:           加密芯片
;**
;** input parameters:       NONE
;** Returned value:         NONE
;**
;** Created by:             LinEnqiang
;** Created Date:           2009-06-23    
;**-------------------------------------------------------------------------------------------------------
;** Modified by:            
;** Modified date:          
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
        ASEGN CRP_DATA:CODE:NOROOT(2)            
        END
/********************************************************************************************************
**                                      End Of File
********************************************************************************************************/
