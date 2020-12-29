/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**                                      
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               vicControl.h
** Latest modified Date:    2009-06-12
** Latest Version:          1.0
** Descriptions:            VIC控制代码
**
**--------------------------------------------------------------------------------------------------------
** Created by:              LinEnqiang
** Created date:            2009-06-12
** Version:                 1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#ifndef _VIC_CONTROL_H_
#define _VIC_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif                                                                  /*  __cplusplus                 */
/*********************************************************************************************************
 *                Hardware Abstraction Layer
*********************************************************************************************************/
#if defined ( __CC_ARM   )
  #define __ASM            __asm                                        
  #define __INLINE         __inline                                    

#elif defined ( __ICCARM__ )
  #define __ASM           __asm                                         
  #define __INLINE        inline                                       
  #define __NOP           __no_operation                               

#elif defined   (  __GNUC__  )
  #define __ASM            asm                                          
  #define __INLINE         inline                                       

#endif

/*********************************************************************************************************
  VIC配置信息定义
*********************************************************************************************************/
struct vic_irq_cfg {
    unsigned int ulChannel;                                              /*  通道                        */
    unsigned int ulPri;                                                  /*  优先级                      */
    unsigned int ulFunctionAddr;                                         /*  ISR地址                     */
    unsigned int ulEnable;                                               /*  使能标识                    */
};

typedef struct vic_irq_cfg      VIC_IRQ_CFG;
typedef struct vic_irq_cfg     *PVIC_IRQ_CFG;

/*********************************************************************************************************
** Function name:           OsSwiHandle1
** Descriptions:            SWI函数声明
** input parameters:        iHandle: 用于区分功能
**                          其他:    根据功能决定
** output parameters:       根据功能决定
** Returned value:          根据功能决定
*********************************************************************************************************/
#pragma swi_number = 0
 __swi void swiHandle1(int iHandle);
#pragma swi_number = 1 
__swi  unsigned int swiHandle (int iHandle, ...);

 /*********************************************************************************************************
** Function name:           IRQDisable
** Descriptions:            关闭全局 IRQ 中断
** input parameters:        none
** output parameters:       none
** Returned value:          none                          
*********************************************************************************************************/
__INLINE void IRQDisable(void) 
{
	swiHandle1(0);
}
/*********************************************************************************************************
** Function name:           IRQEnable
** Descriptions:            打开全局 IRQ 中断
** input parameters:        none
** output parameters:       none
** Returned value:          none                          
*********************************************************************************************************/
__INLINE void IRQEnable(void) 
{
	swiHandle1(1);
}
/*********************************************************************************************************
** Function name:           FIQDisable
** Descriptions:            关闭全局 Fault 中断(不建议使用)
** input parameters:        none
** output parameters:       none
** Returned value:          none                          
*********************************************************************************************************/
__INLINE void FIQDisable(void) 
{
	swiHandle1(2);
}
/*********************************************************************************************************
** Function name:           FIQEnable
** Descriptions:            打开全局 Fault 中断(不建议使用)
** input parameters:        none
** output parameters:       none
** Returned value:          none                          
*********************************************************************************************************/
__INLINE void FIQEnable(void) 
{
	swiHandle1(3);
}

/*********************************************************************************************************
** Function name:           vicIrqFuncSet
** Descriptions:            设置所选外设的中断优先级、中断服务函数地址，并使能中断
** input parameters:        uiChannel:  外设对应的中断通道号 >= 14
**                          uiPri:      中断优先级, 0 is the highest priority,32 priorities
**                          uiFuncAddr: 中断服务函数地址(异常表未映射到RAM时，可以为NULL)
** output parameters:       none
** Returned value:          1:  成功
**                          0:  失败
*********************************************************************************************************/
__INLINE unsigned int vicIrqFuncSet (   unsigned int uiChannel,
                                        unsigned int uiPri,
                                        unsigned int uiFuncAddr)
{
    return swiHandle(0x100, uiChannel, uiPri, uiFuncAddr);
}

/*********************************************************************************************************
** Function name:           vicIrqFuncClr
** Descriptions:            清除所选外设的IRQ资源
** input parameters:        uiChannel:  外设对应的中断通道号 >= 14
** output parameters:       none
** Returned value:          1:  成功
**                          0:  失败
*********************************************************************************************************/
__INLINE unsigned int vicIrqFuncClr (unsigned int uiChannel)
{
    return swiHandle(0x101, uiChannel);
}

/*********************************************************************************************************
** Function name:           vicIrqEnable
** Descriptions:            使能相应外设的中断
** input parameters:        uiChannel:  外设对应的中断通道号 >= 14
** output parameters:       none
** Returned value:          1:  成功
**                          0:  失败
*********************************************************************************************************/
__INLINE unsigned int vicIrqEnable (unsigned int uiChannel)
{
    return swiHandle(0x102, uiChannel);
}

/*********************************************************************************************************
** Function name:           vicIrqDisable
** Descriptions:            禁止相应外设的中断
** input parameters:        uiChannel:  外设对应的中断通道号 >= 14
** output parameters:       none
** Returned value:          1:  成功
**                          0:  失败
*********************************************************************************************************/
__INLINE unsigned int vicIrqDisable (unsigned int uiChannel)
{
    return swiHandle(0x103, uiChannel);
}

/*********************************************************************************************************
** Function name:           vicIrqStatusGet
** Descriptions:            获取所选外设的中断通道号、优先级、中断服务函数地址及中断使能状态
** input parameters:        uiChannel:  外设对应的中断通道号 >= 14
** output parameters:       pvicInfo:   配置信息
** Returned value:          1:  成功
**                          0:  失败
*********************************************************************************************************/
__INLINE unsigned int vicIrqStatusGet (unsigned int uiChannel, PVIC_IRQ_CFG pvicInfo)
{
    return swiHandle(0x104, uiChannel, (unsigned int)pvicInfo);
}

/*********************************************************************************************************
** Function name:           SoftIntEnable
** Descriptions:            使能所选中断通道号的向量软中断
** input parameters:        uiChannel   ：外设对应的中断通道号 >= 14
** output parameters:       none
** Returned value:          1:  成功
**                          0:  失败
*********************************************************************************************************/
__INLINE unsigned int  SoftIntEnable(unsigned int uiChannel)
{
	return swiHandle(0x105, uiChannel);
}
/*********************************************************************************************************
** Function name:           SoftIntDisable
** Descriptions:            禁止所选中断通道号的向量软中断
** input parameters:        uiChannel   ：外设对应的中断通道号 >= 14
** output parameters:       none
** Returned value:          1:  成功
**                          0:  失败
*********************************************************************************************************/
__INLINE unsigned int  SoftIntDisable(unsigned int uiChannel)
{
	return swiHandle(0x106, uiChannel);
}


/*********************************************************************************************************
** Function name:           vicSysTickInit
** Descriptions:            初始化系统时钟
** input parameters:        uiSrcMode：0--选择外部时钟源； 1--内部时钟源
**                          uiMsTime: 定时时间（基本单位：毫秒）
**                          uiSrcFreq：时钟频率
** output parameters:       none
** Returned value:          1:  成功
**                          0:  失败
*********************************************************************************************************/
__INLINE unsigned int vicSysTickInit (unsigned int uiSrcMode,
                                      unsigned int uiMsTime, 
                                      unsigned int uiSrcFreq)
{
	return swiHandle(0x107, uiSrcMode, uiMsTime, uiSrcFreq);
}

/*********************************************************************************************************
** Function name:           vicVectorRemap
** Descriptions:            异常向量表重映射到 RAM
** input parameters:        uiDestVectAddr: 映射向量表的目的地址
**                          uiSrcVectAddr:  向量表的源地址
**                          uiVectLen:      向量表长度(字)
** output parameters:       none
** Returned value:          1:  成功
**                          0:  失败
*********************************************************************************************************/
__INLINE unsigned int vicVectorRemap (  unsigned int uiDestVectAddr, 
                                        unsigned int uiSrcVectAddr, 
                                        unsigned int uiVectLen)
{
	return swiHandle(0x108, uiDestVectAddr, uiSrcVectAddr, uiVectLen);
}
/*********************************************************************************************************
** Function name:           vicSysReset
** Descriptions:            请求芯片产生一次复位
** input parameters:        uiRestMode: 0 - 控制逻辑产生一次复位; 
**                                      1 - 处理器内核（调试逻辑除外），不影响芯片上内核以外的电路.
** output parameters:       none
** Returned value:          1:  成功
**                          0:  失败
*********************************************************************************************************/
__INLINE void vicSysReset (unsigned int uiRestMode)
{
    swiHandle(0x109, uiRestMode);
}

/*********************************************************************************************************
** Function name:           vicSysPconSet 
** Descriptions:            设置系统功率控制模式
** input parameters:        uiPconMode: 0 - 使系统进入普通睡眠模式; 
**                                      1 - 使系统进入深度睡眠模式;
**                                      2 - 使系统进入普通掉电模式;
**                                      3 - 使系统进入深度掉电模式。 
** output parameters:       none
** Returned value:          1:  成功
**                          0:  失败
*********************************************************************************************************/
__INLINE void vicSysPconSet (unsigned int uiPconMode)
{
    swiHandle(0x10A, uiPconMode);
    asm("WFI");
}

void SoftwareReset();

#ifdef __cplusplus
}
#endif                                                                  /*  __cplusplus                 */

#endif                                                                  /*  __TARGET_H                  */
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

