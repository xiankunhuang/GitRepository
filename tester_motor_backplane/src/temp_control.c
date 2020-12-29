#include "config.h"
#include "backplane.h"


#define MAX_ERROR_INTEGRAL 10000

#define DERIVATIVE_TIME 6
#define INTEGRAL_TIME (DERIVATIVE_TIME *5)



void P_Calc(  )  
{  

	spid.p = (spid.K_Proportion * spid.Error);

	if(spid.p > MAX_ERROR_INTEGRAL)
		spid.p = MAX_ERROR_INTEGRAL;
	else if(spid.p < -MAX_ERROR_INTEGRAL)
		spid.p = -MAX_ERROR_INTEGRAL;

}  

void I_Calc( )  
{  

	spid.i += (spid.Error * spid.K_Integral);

	if(spid.i > MAX_ERROR_INTEGRAL)
		spid.i = MAX_ERROR_INTEGRAL;
	else if(spid.i < 0)
		spid.i = 0;

}  

void D_Calc( )  
{  

	spid.d = (spid.Error - spid.PrevError_D) * spid.K_Derivative;
	spid.PrevError_D = spid.Error;

	if(spid.d > MAX_ERROR_INTEGRAL)
		spid.d = MAX_ERROR_INTEGRAL;
	else if(spid.d < -MAX_ERROR_INTEGRAL)
		spid.d = -MAX_ERROR_INTEGRAL;

}  


unsigned int PIDCalc( unsigned char op_code)  
{  
    int result;  
// 比例项      // 积分项   // 微分项 // 结果应该小于 1000

	P_Calc();
	I_Calc();
	D_Calc();

	result = spid.p + spid.i;

	if(result > 10000) // 结果应该小于 100
		result = 10000;

	result +=  spid.d;

  	spid.pid = result;  

	if(result < 0)
		result = 0;
		 
	result /= 100;

	if(result > 100) // 结果应该小于 100
		result = 100;

	return result;
}  



/**************************************
op_code：TC_RESET/  TC_SET / TC_UPDATE
//  SetValue 设定目标 Desired Value 


**************************************/

int temp_control( unsigned int op_data, unsigned char op_code)
{
unsigned int rout; // PID Response (Output)  //占空比调节参数  
static int set_value_old = -500;

	if(spid.SetValue > TC_TEMP_UP)
		spid.SetValue = TC_TEMP_UP;
	else if(spid.SetValue < TC_TEMP_STOP)
		spid.SetValue = TC_TEMP_STOP;

	if(set_value_old != spid.SetValue)	// 初始化 温控
	{
		printk("TC_RESET");
		set_value_old = spid.SetValue;
		spid.PrevError_D = 0;
		spid.i =0;
		spid.K_Proportion = 35; // Set PID Coefficients  比例常数 Proportional Const  
		spid.K_Integral =  2;  //积分常数 Integral Const  	
		spid.K_Derivative = 5;   //微分常数 Derivative Const  
	}


	if(spid.SumCount > 0)
		spid.pValue = spid.Sum / spid.SumCount;

	spid.Error = (spid.SetValue * spid.SumCount) - spid.Sum;		   // 偏差    
	rout = PIDCalc (TC_UPDATE); // Perform PID Interation 
	spid.Sum =0;
	spid.SumCount =0;

	if(spid.Error > 200)  //设置的温度比实际的温度是否是大于3度//如果是，则全速加热  
	{
		rout = 100;
		spid.i =0;
	}
	else if(spid.Error < -200)  // 超过设定温度 2度；
	{  
		spid.i =0;
		rout = 0;  
	}  

	spid.duty_cycle = (unsigned char)rout;

	if((0 == spid.duty_cycle) && (set_value_old > TC_TEMP_STOP)){
		PWM1MR2 = 60; // 需要的占空比0~100；
	}
	else
		PWM1MR2 = 30; // 需要的占空比0~100；

	PWM1LER_bit.EM2L =1; // 生效；

	return 0;

}


