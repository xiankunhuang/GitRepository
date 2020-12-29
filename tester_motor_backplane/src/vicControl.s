;/****************************************Copyright (c)***************************************************
;**                         Guangzhou ZHIYUAN electronics Co.,LTD.                               
;**                                     
;**                               http://www.embedtools.com
;**
;**--------------File Info-------------------------------------------------------------------------------
;** File name: 			vicControl.s
;** Last modified Date: 2009-06-12
;** Last Version: 		1.0
;** Descriptions: 		Provide NVIC Control in prerogative mode
;**------------------------------------------------------------------------------------------------------
;** Created by: 		LinEnqiang
;** Created date:       2009-06-12
;** Version:			1.0
;** Descriptions: 		The original version
;**
;**------------------------------------------------------------------------------------------------------
;********************************************************************************************************/

;/********************************************************************************************************
; 宏定义
;********************************************************************************************************/
__VALID_PRI_LBIT    EQU     0x03                                        ;/* 优先级最低有效位,与芯片相关 */
__SYSTICK_PRI       EQU     0xFF                                        ;/* SysTick的优先级，默认为最低 */
__INT_BASE_PRI      EQU     (0x01 << __VALID_PRI_LBIT)                  ;/* 屏蔽优先级阈值-非零，默认为2*/
__INT_MAX_ID        EQU     0x33                                        ;/* 芯片支持的最大中断号        */
__VECT_KEY          EQU     (0x05FA << 16)                              ;/* 访问复位控制寄存器钥匙      */

__CTRL_MSP_SYS      EQU     0x00                                        ;/* 特权模式，使用主堆栈        */
__CTRL_MSP_USR      EQU     0x01                                        ;/* 用户模式，使用主堆栈        */
;__CTRL_SP_MODE      EQU     __CTRL_MSP_USR                              ;/* 当前使用模式                */
__CTRL_SP_MODE      EQU     __CTRL_MSP_SYS                              ;/* ??????                */


__SYSTICK_BASE_RATI EQU     1000

__PCON_ADDR         EQU     0x400FC0C0
__SYSTICK_CTR_ADDR  EQU     0xE000E010
__SETENA0           EQU     0xE000E100
__CLRENA0           EQU     0xE000E180
__CLRPEND0          EQU     0xE000E280
__INT0_PRI_ADDR     EQU     0xE000E400
__ICSR              EQU     0xE000ED04
__VTOR              EQU     0xE000ED08
__AIRCR             EQU     0xE000ED0C
__SYSCR             EQU     0xE000ED10
__PENDSVC_PRI_ADDR  EQU     0xE000ED22
__SYSTICK_PRI_ADDR  EQU     0xE000ED23
__STIR_ADDR         EQU     0xE000EF00

;/********************************************************************************************************
; The imported labels    
; 引入的外部标号在这声明
;********************************************************************************************************/     
        EXTERN  __vector_table
;/********************************************************************************************************
; The emported labels    
; 给外部使用的标号在这声明
;********************************************************************************************************/  
        PUBLIC  __RunFirst
        PUBLIC  __vicControl
        PUBLIC  __SoftWareInterrupt 		 
        PUBLIC SoftwareReset
            
        RSEG  RUNFIRST:CODE:REORDER(2)
        THUMB 
;/********************************************************************************************************
;** Function name:           RunFirst
;** Descriptions:            NVIC管理初始化
;** input parameters:        none
;** output parameters:       none
;** Returned value:          none
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/		
__RunFirst			
        CPSID   I              
        MVN     R3, #0x00    
        LDR.W   R0, =__SETENA0
        LDR.W   R1, =__CLRENA0 
        LDR.W   R2, =__CLRPEND0
LOOP_0        
        STR     R3, [R0]
        LDMIA   R0!,{R3}
        CMP     R3, #0
        IT      EQ
        BEQ     LOOP_END
        STMIA   R1!,{R3}
        STMIA   R2!,{R3} 
        B       LOOP_0
LOOP_END	
        MOV     R1, #__INT_BASE_PRI
        MSR     BASEPRI, R1         
        LDR.W   R0, =__PENDSVC_PRI_ADDR
        MOV     R1, #0xFF        
        STRB    R1, [R0]
        
        LDR.W   R0, =__SYSTICK_PRI_ADDR
        MOV     R1, #__SYSTICK_PRI        
        STRB    R1, [R0]  
        
        CPSIE   I
        MOV     R1, #0x03
        MRS     R0, CONTROL
        BIC     R0, R0, R1
        ORR     R0, R0, #__CTRL_SP_MODE
        MSR     CONTROL, R0         
        BX      LR     

;/********************************************************************************************************
;** Function name:           __SoftWareInterrupt
;** Descriptions:            软件中断，用于提供VIC管理服务
;** input parameters:        依功能而定
;** output parameters:       依功能而定
;** Returned value:          依功能而定
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/        
__SoftWareInterrupt
        CMP     R0, #3
        IT      HI
        BXHI    LR
        TBB.W   [PC, R0]                                                    
        DATA
__SwiFunction
        DCB     ( (DisableIRQ - __SwiFunction) / 2 )                    ;0
        DCB     ( (EnableIRQ  - __SwiFunction) / 2 )                    ;1
        DCB     ( (DisableFIQ - __SwiFunction) / 2 )                    ;2
        DCB     ( (EnableFIQ  - __SwiFunction) / 2 )                    ;3
        THUMB
;/*********************************************************************************************************
;** Function name:           DisableIRQ
;** Descriptions:            关闭全局 IRQ 中断
;** input parameters:        none
;** output parameters:       none
;** Returned value:          none                          
;*********************************************************************************************************/
DisableIRQ
        MOV     R0, #__INT_BASE_PRI
        MSR     BASEPRI, R0
        BX      LR

;/*********************************************************************************************************
;** Function name:           EnableIRQ
;** Descriptions:            打开全局 IRQ 中断
;** input parameters:        none
;** output parameters:       none
;** Returned value:          none                          
;*********************************************************************************************************/
EnableIRQ
        MOV     R0, #0
        MSR     BASEPRI, R0 
        BX      LR
;/*********************************************************************************************************
;** Function name:           DisableFIQ
;** Descriptions:            关闭全局 Fault 中断
;** input parameters:        none
;** output parameters:       none
;** Returned value:          none                          
;*********************************************************************************************************/
DisableFIQ                
        CPSID   F 
        BX      LR

;/*********************************************************************************************************
;** Function name:           EnableFIQ
;** Descriptions:            打开全局 Fault 中断
;** input parameters:        none
;** output parameters:       none
;*;* Returned value:         none                          
;*********************************************************************************************************/
EnableFIQ
        CPSIE   F 
        BX      LR
        
;/********************************************************************************************************
;** Function name:           __vicControl
;** Descriptions:            软件中断，用于提供VIC管理服务
;** input parameters:        依功能而定
;** output parameters:       依功能而定
;** Returned value:          依功能而定
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
__vicControl 
        SUB     R0, R0, #0x100
        CMP     R0, #10
        IT      HI
        BXHI    LR
        TBH.W   [PC, R0,LSL #1]                                                    
        DATA
SwiFunctionAdd       
        DC16    ( (SetvicIrqFunc    - SwiFunctionAdd) / 2 )             ; 0           
        DC16    ( (ClrvicIrqFunc    - SwiFunctionAdd) / 2 )             ; 1                     
        DC16    ( (EnablevicIrq     - SwiFunctionAdd) / 2 )             ; 2                       
        DC16    ( (DisablevicIrq    - SwiFunctionAdd) / 2 )             ; 3                     
        DC16    ( (GetvicIrqStatus  - SwiFunctionAdd) / 2 )             ; 4
        DC16    ( (EnableSoftInt    - SwiFunctionAdd) / 2 )             ; 5
        DC16    ( (DisableSoftInt   - SwiFunctionAdd) / 2 )             ; 6        
        DC16    ( (InitvicSysTick   - SwiFunctionAdd) / 2 )             ; 7
        DC16    ( (RemapvicVector   - SwiFunctionAdd) / 2 )             ; 8
        DC16    ( (ResetvicSys      - SwiFunctionAdd) / 2 )             ; 9
        DC16    ( (SetvicSysPcon    - SwiFunctionAdd) / 2 )             ; 10
        THUMB
;/********************************************************************************************************
;** Function name:           SetvicIrqFunc
;** Descriptions:            设置所选外设的中断优先级、中断服务函数地址，并使能中断
;** input parameters:        none
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
SetvicIrqFunc 
        LDR     R0, [R2, #4]                         
        CMP     R0, #14                                                 ;/* 判断通道号的合法性          */
        ITT     LO
        MOVLO   R0, #0
        BLO.W   vicControl_END
        CMP     R0, #__INT_MAX_ID
        ITT     HI
        MOVHI   R0, #0
        BHI.W   vicControl_END
        
        LDR     R1, =__VTOR
        LDR     R1, [R1]
        CMP     R1, #0 
        IT      EQ
        BEQ     SetvicIrqFunc_j 
        
        LDR     R3, [R1, R0, LSL #2]
        CMP     R3, #0
        ITT     NE
        MOVNE   R0, #0
        BNE.W   vicControl_END  
        
        LDR     R3, [R2, #12]
        CMP     R3, #0 
        ITTE    EQ
        MOVEQ   R0, #0
        BEQ.W   vicControl_END        
        STRNE   R3, [R1, R0, LSL #2]  
        
SetvicIrqFunc_j        
        LDR     R1, [R2, #8] 
        LSL.W   R1, R1, #__VALID_PRI_LBIT        
          
        CMP     R0, #14
        ITTT    EQ
        LDREQ   R0, =__PENDSVC_PRI_ADDR
        STRBEQ  R1, [R0]
        MOVEQ   R0, #0x01        
        BEQ.W   vicControl_END 
        
        CMP     R0, #15
        IT      HI
        BHI     SetvicIrqFunc_jj        
        LDR     R3, =__SYSTICK_PRI_ADDR
        STRB    R1, [R3]        
        LDR     R3, =__SYSTICK_CTR_ADDR
        LDR     R1, [R3]       
        ORR     R1, R1, #3
        STRB    R1, [R3]        
        MOV     R0, #0x01
        B.W     vicControl_END 
SetvicIrqFunc_jj       
        SUB     R0, R0, #16        
        LDR     R3, =__INT0_PRI_ADDR
        STRB    R1, [R3, R0]
        
        MOV     R1, #1
        LSR.W   R3, R0, #5
        AND     R0, R0, #0x1F
        LSL.W   R0, R1, R0
        LDR     R1, =__SETENA0
        STR.W   R0, [R1, R3, LSL #2]
        
        MOV     R0, #0x01
        B.W     vicControl_END
;/*********************************************************************************************************
;** Function name:           ClrvicIrqFunc
;** Descriptions:            清除所选外设的IRQ资源
;** input parameters:        none
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
ClrvicIrqFunc    
        LDR     R0, [R2, #4]                         
        CMP     R0, #14                                                 ;/* 判断通道号的合法性          */
        ITT     LO
        MOVLO   R0, #0
        BLO.W   vicControl_END
        CMP     R0, #__INT_MAX_ID
        ITT     HI
        MOVHI   R0, #0
        BHI.W   vicControl_END  
        
        CMP     R0, #14
        IT      HI
        BHI     ClrvicIrqFunc_j      
        B.W     ClrvicIrqFunc_jjj 
        
ClrvicIrqFunc_j       
        CMP     R0, #15
        IT      HI
        BHI     ClrvicIrqFunc_jj         
        
        LDR     R3, =__SYSTICK_CTR_ADDR
        LDR     R0, [R3]
        BIC     R0, R0, #3
        STRB    R0, [R3] 
        B.W     ClrvicIrqFunc_jjj 
        
ClrvicIrqFunc_jj        
        SUB     R0, R0, #16  
        MOV     R1, #1
        LSR.W   R3, R0, #5
        AND     R0, R0, #0x1F
        LSL.W   R0, R1, R0
        LDR     R1, =__CLRENA0
        STR.W   R0, [R1, R3, LSL #2]
        
ClrvicIrqFunc_jjj        
        LDR     R0, [R2, #4]  
        LDR     R1, =__VTOR
        LDR     R1, [R1]
        CMP     R1, #0 
        ITT     EQ
        MOVEQ   R0, #0
        BEQ.W   vicControl_END 
        
        MOV     R3, #0
        STR     R3, [R1, R0, LSL #2] 
        MOV     R0, #1
        B.W     vicControl_END 
;/*********************************************************************************************************
;** Function name:           EnablevicIrq
;** Descriptions:            使能相应外设的中断
;** input parameters:        none
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/      
EnablevicIrq    
        LDR     R0, [R2, #4]                         
        CMP     R0, #15                                                 ;/* 判断通道号的合法性          */
        ITT     LO
        MOVLO   R0, #0
        BLO.W   vicControl_END
        CMP     R0, #__INT_MAX_ID
        ITT     HI
        MOVHI   R0, #0
        BHI.W   vicControl_END
        
        CMP     R0, #15
        IT      HI
        BHI     EnablevicIrq_j         
        LDR     R3, =__SYSTICK_CTR_ADDR
        LDR     R1, [R3]
        ORR     R1, R1, #1
        STRB    R1, [R3]         
        MOV     R0, #0x01
        B.W     vicControl_END 
EnablevicIrq_j
        SUB     R0, R0, #16  
        MOV     R1, #1
        LSR.W   R3, R0, #5
        AND     R0, R0, #0x1F
        LSL.W   R0, R1, R0
        LDR     R1, =__SETENA0
        STR.W   R0, [R1, R3, LSL #2]
        
        MOV     R0, #0x01
        B.W     vicControl_END
;/*********************************************************************************************************
;** Function name:           DisablevicIrq
;** Descriptions:            禁止相应外设的中断
;** input parameters:        none
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/            
DisablevicIrq
        LDR     R0, [R2, #4]                         
        CMP     R0, #15                                                 ;/* 判断通道号的合法性          */
        ITT     LO
        MOVLO   R0, #0
        BLO.W   vicControl_END
        CMP     R0, #__INT_MAX_ID
        ITT     HI
        MOVHI   R0, #0
        BHI.W   vicControl_END
        
        CMP     R0, #15
        IT      HI
        BHI     DisablevicIrq_j         
        LDR     R3, =__SYSTICK_CTR_ADDR
        LDR     R1, [R3]
        BIC     R1, R1, #1
        STRB    R1, [R3]         
        MOV     R0, #0x01
        B.W     vicControl_END 
DisablevicIrq_j
        SUB     R0, R0, #16  
        MOV     R1, #1
        LSR.W   R3, R0, #5
        AND     R0, R0, #0x1F
        LSL.W   R0, R1, R0
        LDR     R1, =__CLRENA0
        STR.W   R0, [R1, R3, LSL #2]
        
        MOV     R0, #0x01
        B.W     vicControl_END
        
;/*********************************************************************************************************
;** Function name:           GetvicIrqStatus
;** Descriptions:            获取所选外设的中断通道号、优先级、中断服务函数地址及中断使能状态
;** input parameters:        
;** output parameters:       
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
GetvicIrqStatus
        LDR     R0, [R2, #4]                         
        CMP     R0, #14                                                 ;/* 判断通道号的合法性          */
        ITT     LO
        MOVLO   R0, #0
        BLO.W   vicControl_END
        CMP     R0, #__INT_MAX_ID
        ITT     HI
        MOVHI   R0, #0
        BHI.W   vicControl_END
        
        LDR     R1, [R2, #8]
        STMIA   R1!,{R0}                

        LDR     R3, =__VTOR
        LDR     R3, [R3]
        LDR.W   R3, [R3, R0, LSL #2] 
        STR.W   R3, [R1, #4]   
        MOV     R3, #1
        STR     R3, [R1, #8] 
        
        CMP     R0, #14        
        IT      HI
        BHI     GetvicIrqStatus_j        
        LDR     R0, =__PENDSVC_PRI_ADDR
        LDRB    R0, [R0]
        LSR     R0, R0, #__VALID_PRI_LBIT 
        STR     R0, [R1]        
        MOV     R0, #0x01        
        B.W     vicControl_END 
GetvicIrqStatus_j        
        CMP     R0, #15
        IT      HI
        BHI     GetvicIrqStatus_jj    
        
        LDR     R0, =__SYSTICK_PRI_ADDR
        LDRB    R0, [R0]
        LSR     R0, R0, #__VALID_PRI_LBIT 
        STR     R0, [R1]  
        
        LDR     R0, =__SYSTICK_CTR_ADDR
        LDR     R0, [R0]       
        AND     R0, R0, #3
        STR     R0, [R1, #8]  
        
        MOV     R0, #0x01
        B.W     vicControl_END 
GetvicIrqStatus_jj       
        SUB     R0, R0, #16        
        LDR     R3, =__INT0_PRI_ADDR
        LDRB    R3, [R3, R0]
        LSR     R3, R3, #__VALID_PRI_LBIT 
        STR     R3, [R1]
        
        LSR.W   R3, R0, #5
        AND     R0, R0, #0x1F
        MOV     R1, #1
        LSL.W   R0, R1, R0        
        LDR     R1, =__SETENA0
        LDR.W   R3, [R1, R3, LSL #2]
        LDR     R1, [R2, #8]
        TST.W   R3, R0
        ITE     EQ
        MOVEQ   R0, #0
        MOVNE   R0, #1
        STR     R0, [R1,#12]
        
        MOV     R0, #0x01
        B.W     vicControl_END
        
;/*********************************************************************************************************
;** Function name:           EnableSoftInt
;** Descriptions:           能所选中断通道号的向量软中断
;** input parameters:        
;** output parameters:       
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
EnableSoftInt
        LDR     R0, [R2, #4]                         
        CMP     R0, #14                                                 ;/* 判断通道号的合法性          */
        ITT     LO
        MOVLO   R0, #0
        BLO.W   vicControl_END
        CMP     R0, #__INT_MAX_ID
        ITT     HI
        MOVHI   R0, #0
        BHI.W   vicControl_END    
        
        CMP     R0, #15        
        ITEE    HI
        BHI     EnableSoftInt_j
        LDRLS   R1, =__ICSR
        LDRLS.W R3, [R1]
        
        MOV     R1, #1        
        CMP     R0, #14        
        ITE     EQ        
        ORREQ.W R3, R3, R1, LSL #28
        ORRNE.W R3, R3, R1, LSL #26     
        
        LDR     R1, =__ICSR       
        STR.W   R3, [R1]
        MOV     R0, #0x01
        B.W     vicControl_END   
        
EnableSoftInt_j        
        SUB     R0, R0,#16
        LDR     R1, =__STIR_ADDR
        STR     R0, [R1]  
        MOV     R0, #1
        B.W     vicControl_END
;/*********************************************************************************************************
;** Function name:           DisableSoftInt
;** Descriptions:           禁止所选中断通道号的向量软中断
;** input parameters:        
;** output parameters:       
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
DisableSoftInt
        LDR     R0, [R2, #4]                         
        CMP     R0, #14                                                 ;/* 判断通道号的合法性          */
        ITT     LO
        MOVLO   R0, #0
        BLO.W   vicControl_END
        CMP     R0, #__INT_MAX_ID
        ITT     HI
        MOVHI   R0, #0
        BHI.W   vicControl_END
        
        CMP     R0, #15        
        ITEE    HI
        BHI     DisableSoftInt_j
        LDRLS   R1, =__ICSR
        LDRLS.W R3, [R1]
        
        MOV     R1, #1        
        CMP     R0, #14        
        ITE     EQ        
        ORREQ.W R3, R3, R1, LSL #27
        ORRNE.W R3, R3, R1, LSL #25     
        
        LDR     R1, =__ICSR       
        STR.W   R3, [R1]
        MOV     R0, #0x01
        B.W     vicControl_END   
        
DisableSoftInt_j 
        SUB     R0, R0,#16        
        MOV     R1, #1
        LSR.W   R3, R0, #5
        AND     R0, R0, #0x1F
        LSL.W   R0, R1, R0
        LDR     R1, =__CLRPEND0
        STR.W   R0, [R1, R3, LSL #2]
        MOV     R0, #1
        B.W     vicControl_END        

;/*********************************************************************************************************
;** Function name:           InitvicSysTick
;** Descriptions:            初始化系统时钟
;** input parameters:        
;** output parameters:       
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/      
InitvicSysTick
        LDR     R1, [R2, #8]
        CMP     R1, #0                                                 
        ITT     EQ
        MOVEQ   R0, #0
        BEQ.W   vicControl_END
        
        LDR.W   R3, [R2, #12]
        CMP     R3, #0                                                 
        ITT     EQ
        MOVEQ   R0, #0
        BEQ.W   vicControl_END        
        
        MOV     R0, #__SYSTICK_BASE_RATI
        UDIV    R3, R3, R0
        MUL.W   R1, R1, R3
        MOV     R3, #1
        CMP     R1, R3, LSL #24
        ITT     GE
        MOVGE   R0, #0x00
        BGE.W   vicControl_END
        
        SUB.W   R1, R1, #1
        LDR     R3, =__SYSTICK_CTR_ADDR
        STR     R1, [R3, #4]
        
        LDR     R0, [R2, #4]
        AND     R0, R0,#1
        MOV     R1, #2
        ORR     R1, R1, R0, LSL #2
        STR     R1, [R3]
        
        MOV     R0, #0x01
        B.W     vicControl_END          

;/*********************************************************************************************************
;** Function name:           RemapvicVector
;** Descriptions:            异常向量表重映射到 RAM
;** input parameters:        
;** output parameters:       
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/ 
RemapvicVector         
        MOV     R0, #0x01
        STR.W   R0, [R2]  
        BX    LR
        
;/*********************************************************************************************************
;** Function name:           ResetvicSys
;** Descriptions:            请求芯片产生一次复位
;** input parameters:        
;** output parameters:       
;** Returned value:          none
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/ 
ResetvicSys 
        LDR.W   R1, =__AIRCR
        LDR.W   R3, [R1]
        LDR     R0, =0xFFFF0000
        BIC.W   R3, R3, R0
        LDR.W   R0, =__VECT_KEY
        ORR.W   R3, R3, R0
        
        LDR     R0, [R2, #4]
        CMP     R0, #0  
        ITE     EQ
        ORREQ.W R3, R3, #4
        ORRNE.W R3, R3, #1
        
        STR     R3, [R1]
        BX      LR        
;/*********************************************************************************************************
;** Function name:           SetvicSysPcon 
;** Descriptions:            设置系统功率控制模式
;** input parameters:        
;** output parameters:       
;** Returned value:          none
;** Created by:              LinEnqiang
;** Created Date:            2009-06-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/ 
SetvicSysPcon 
        LDR.W   R1, [R2, #4]
        CMP     R1, #3
        IT      HI       
        BXHI  	LR

        LDR     R0, =__PCON_ADDR
        LDR     R3, [R0]
        BIC     R3, R3, #3
        CMP     R1, #2
        IT      LO
        STRLO   R3, [R0]
        CMP     R1, #2
        ITT     EQ
        ORREQ   R3, R3, #1
        STREQ   R3, [R0]
        
        CMP     R1, #3
        ITT     EQ
        ORREQ   R3, R3, #3
        STREQ   R3, [R0]
 
        LDR     R0, =__SYSCR
        MOV     R3, #0x10 
        CMP     R1, #0
        ITEE    EQ
        STREQ   R3, [R0]
        MOVNE   R3, #0x14
        STRNE   R3, [R0]       
        BX      LR       
vicControl_END   
        STR.W   R0, [R2]    
        BX      LR 

SoftwareReset
	LDR.W		R0, =0x00000000
	LDR.W		R0, [R0]
	MOV		SP, R0
	
	LDR.W	R0, =0x00000000
	LDR.W	R0, [R0, #4]
	BX		R0
      END
;/********************************************************************************************************
;	End Of File
;********************************************************************************************************/
