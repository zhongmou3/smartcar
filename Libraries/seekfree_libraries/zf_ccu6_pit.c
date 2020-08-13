/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		ccu6_pit
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		tasking v6.3r1
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/
 
#include "IfxCcu6_Timer.h"
#include "SysSe/Bsp/Bsp.h"
#include "isr_config.h"
#include "zf_ccu6_pit.h"




//-------------------------------------------------------------------------------------------------------------------
//  @brief      pit初始化
//  @param      ccu6n           选择CCU6模块(CCU6_0、CCU6_1)
//  @param      pit_ch          选择通道(PIT_CH0、PIT_CH1)
//  @param      time            周期时间
//  @return     void
//  @note						请使用.h文件中 带时间单位的宏定义函数
//  Sample usage:				pit_init(CCU6_0, PIT_CH0, 5000);	//设置周期中断5000us
//-------------------------------------------------------------------------------------------------------------------
void pit_init(CCU6N_enum ccu6n, CCU6_CHN_enum pit_ch, uint32 time)
{
	uint8 i;
	volatile Ifx_CCU6 *module;
	uint64 timer_input_clk;
	IfxCcu6_Timer g_Ccu6Timer;
	IfxCcu6_TimerId timer_id;
	IfxCcu6_Timer_Config timerConfig;
	uint32 timer_period;

	boolean interrupt_state = disableInterrupts();

	module = IfxCcu6_getAddress((IfxCcu6_Index)ccu6n);

	IfxCcu6_Timer_initModuleConfig(&timerConfig, module);



	timer_input_clk = IfxScuCcu_getSpbFrequency();
	i = 0;
	while(i<16)
	{
		timer_period = (uint32)(timer_input_clk * time / 1000000);
		if(timer_period < 0xffff)	break;
		timer_input_clk >>= 1;
	}
	if(16 <= i)	IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);


	switch(ccu6n)
	{
		case CCU6_0:
		{
			if(PIT_CH0 == pit_ch)
			{
				timerConfig.interrupt1.typeOfService  = CCU6_0_CH0_INT_SERVICE;
				timerConfig.interrupt1.priority       = CCU6_0_CH0_ISR_PRIORITY;
			}
			else
			{

				timerConfig.interrupt2.typeOfService  = CCU6_0_CH1_INT_SERVICE;
				timerConfig.interrupt2.priority       = CCU6_0_CH1_ISR_PRIORITY;
			}
		}break;

		case CCU6_1:
		{
			if(PIT_CH0 == pit_ch)
			{
				timerConfig.interrupt1.typeOfService  = CCU6_1_CH0_INT_SERVICE;
				timerConfig.interrupt1.priority       = CCU6_1_CH0_ISR_PRIORITY;
			}
			else
			{
				timerConfig.interrupt1.typeOfService  = CCU6_1_CH1_INT_SERVICE;
				timerConfig.interrupt1.priority       = CCU6_1_CH1_ISR_PRIORITY;
			}
		}break;
	}

	if(PIT_CH0 == pit_ch)
	{
		timer_id = IfxCcu6_TimerId_t12;
		timerConfig.interrupt1.source         = IfxCcu6_InterruptSource_t12PeriodMatch;
		timerConfig.interrupt1.serviceRequest = IfxCcu6_ServiceRequest_1;
		timerConfig.base.t12Period 			  = timer_period;
		timerConfig.base.t12Frequency 		  = (float)timer_input_clk;
		timerConfig.clock.t12countingInputMode = IfxCcu6_CountingInputMode_internal;
	}
	else
	{
		timer_id = IfxCcu6_TimerId_t13;
		timerConfig.interrupt2.source         = IfxCcu6_InterruptSource_t13PeriodMatch;
		timerConfig.interrupt2.serviceRequest = IfxCcu6_ServiceRequest_2;
		timerConfig.base.t13Period 			  = timer_period;
		timerConfig.base.t13Frequency 		  = (float)timer_input_clk;
		timerConfig.clock.t13countingInputMode = IfxCcu6_CountingInputMode_internal;
	}


    timerConfig.timer = timer_id;

    timerConfig.timer12.counterValue = 0;
    timerConfig.timer13.counterValue = 0;
    timerConfig.trigger.t13InSyncWithT12 = FALSE;

    IfxCcu6_Timer_initModule(&g_Ccu6Timer, &timerConfig);

	restoreInterrupts(interrupt_state);

	IfxCcu6_Timer_start(&g_Ccu6Timer);
}
