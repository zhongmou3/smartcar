/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		tasking v6.3r1
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/


#include "isr_config.h"
#include "isr.h"
#include "control.h"

//PIT中断函数  示例
IFX_INTERRUPT(cc60_pit_ch0_isr, CCU6_0_CH0_INT_SERVICE, CCU6_0_CH0_ISR_PRIORITY)
{
	double xishu;
	int16 temp_speed;
	int encoder_counter;
	int encoder_counter2;
	PIT_CLEAR_FLAG(CCU6_0, PIT_CH0);
	encoder_counter = gpt12_get(GPT12_T5);					//右后轮速度(编码器)
	gpt12_clear(GPT12_T5);
	encoder_counter2 = gpt12_get(GPT12_T2);					//右后轮速度(编码器)
	gpt12_clear(GPT12_T2);
	//encoder_counter2 = encoder_counter2;
	encoder_counter = - encoder_counter;
	float Ratio_Encoder = 200 / (1175 * 0.01);  			//右后轮速度=counter*左轮周长(mm)/(左轮转一圈对应的脉冲数*程序周期)
	speed = (encoder_counter+encoder_counter2)/2*Ratio_Encoder;
	//printf("speed: %d\n", speed);
	/*if(cardegree<0)
	{
		//后轮差速部分
		int d;
		d=-cardegree;
		xishu=-B*(-0.000147*d*d+0.016*d-0.00493)/(2*H);
	}
	//右转弯
	else
   	{
   	    xishu=B*(-0.000147*cardegree*cardegree+0.016*cardegree-0.00493)/(2*H);
	}
	speed = temp_speed/(1-0.9*xishu);*/
}


IFX_INTERRUPT(cc60_pit_ch1_isr, CCU6_0_CH1_INT_SERVICE, CCU6_0_CH1_ISR_PRIORITY)
{
	PIT_CLEAR_FLAG(CCU6_0, PIT_CH1);

}

IFX_INTERRUPT(cc61_pit_ch0_isr, CCU6_1_CH0_INT_SERVICE, CCU6_1_CH0_ISR_PRIORITY)
{
	PIT_CLEAR_FLAG(CCU6_1, PIT_CH0);

}

IFX_INTERRUPT(cc61_pit_ch1_isr, CCU6_1_CH1_INT_SERVICE, CCU6_1_CH1_ISR_PRIORITY)
{
	PIT_CLEAR_FLAG(CCU6_1, PIT_CH1);

}




IFX_INTERRUPT(eru_ch0_ch4_isr, ERU_CH0_CH4_INT_SERVICE, ERU_CH0_CH4_INT_PRIO)
{
	if(GET_GPIO_FLAG(ERU_CH0_REQ4_P10_7))//通道0中断
	{
		CLEAR_GPIO_FLAG(ERU_CH0_REQ4_P10_7);
	}

	if(GET_GPIO_FLAG(ERU_CH4_REQ13_P15_5))//通道4中断
	{
		CLEAR_GPIO_FLAG(ERU_CH4_REQ13_P15_5);
	}
}

IFX_INTERRUPT(eru_ch1_ch5_isr, ERU_CH1_CH5_INT_SERVICE, ERU_CH1_CH5_INT_PRIO)
{
	if(GET_GPIO_FLAG(ERU_CH1_REQ5_P10_8))//通道1中断
	{
		CLEAR_GPIO_FLAG(ERU_CH1_REQ5_P10_8);
	}

	if(GET_GPIO_FLAG(ERU_CH5_REQ1_P15_8))//通道5中断
	{
		CLEAR_GPIO_FLAG(ERU_CH5_REQ1_P15_8);
	}
}

//由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
//IFX_INTERRUPT(eru_ch2_ch6_isr, ERU_CH2_CH6_INT_SERVICE, ERU_CH2_CH6_INT_PRIO)
//{
//	if(GET_GPIO_FLAG(ERU_CH2_REQ7_P00_4))//通道2中断
//	{
//		CLEAR_GPIO_FLAG(ERU_CH2_REQ7_P00_4);
//
//	}
//	if(GET_GPIO_FLAG(ERU_CH6_REQ9_P20_0))//通道6中断
//	{
//		CLEAR_GPIO_FLAG(ERU_CH6_REQ9_P20_0);
//
//	}
//}



IFX_INTERRUPT(eru_ch3_ch7_isr, ERU_CH3_CH7_INT_SERVICE, ERU_CH3_CH7_INT_PRIO)//摄像头场中断
{
	if(GET_GPIO_FLAG(ERU_CH3_REQ6_P02_0))//通道3中断
	{
		CLEAR_GPIO_FLAG(ERU_CH3_REQ6_P02_0);
		if		(1 == camera_type)	mt9v03x_vsync();
		else if	(3 == camera_type)	ov7725_vsync();

	}
	if(GET_GPIO_FLAG(ERU_CH7_REQ16_P15_1))//通道7中断
	{
		CLEAR_GPIO_FLAG(ERU_CH7_REQ16_P15_1);

	}
}



IFX_INTERRUPT(dma_ch5_isr, ERU_DMA_INT_SERVICE, ERU_DMA_INT_PRIO)//摄像头DMA中断
{

	if		(1 == camera_type)	mt9v03x_dma();
	else if	(3 == camera_type)	ov7725_dma();
}


//串口中断函数  示例
IFX_INTERRUPT(uart0_tx_isr, UART0_INT_SERVICE, UART0_TX_INT_PRIO)
{
    IfxAsclin_Asc_isrTransmit(&uart0_handle);
}
IFX_INTERRUPT(uart0_rx_isr, UART0_INT_SERVICE, UART0_RX_INT_PRIO)
{
    IfxAsclin_Asc_isrReceive(&uart0_handle);
}
IFX_INTERRUPT(uart0_er_isr, UART0_INT_SERVICE, UART0_ER_INT_PRIO)
{
    IfxAsclin_Asc_isrError(&uart0_handle);
}

//串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, UART1_INT_SERVICE, UART1_TX_INT_PRIO)
{
    IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, UART1_INT_SERVICE, UART1_RX_INT_PRIO)
{
    IfxAsclin_Asc_isrReceive(&uart1_handle);
    mt9v03x_uart_callback();
}
IFX_INTERRUPT(uart1_er_isr, UART1_INT_SERVICE, UART1_ER_INT_PRIO)
{
    IfxAsclin_Asc_isrError(&uart1_handle);
}


//串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, UART2_INT_SERVICE, UART2_TX_INT_PRIO)
{
    IfxAsclin_Asc_isrTransmit(&uart2_handle);
}
IFX_INTERRUPT(uart2_rx_isr, UART2_INT_SERVICE, UART2_RX_INT_PRIO)
{
    IfxAsclin_Asc_isrReceive(&uart2_handle);
    wireless_uart_callback();
}
IFX_INTERRUPT(uart2_er_isr, UART2_INT_SERVICE, UART2_ER_INT_PRIO)
{
    IfxAsclin_Asc_isrError(&uart2_handle);
}



IFX_INTERRUPT(uart3_tx_isr, UART3_INT_SERVICE, UART3_TX_INT_PRIO)
{
    IfxAsclin_Asc_isrTransmit(&uart3_handle);
}
IFX_INTERRUPT(uart3_rx_isr, UART3_INT_SERVICE, UART3_RX_INT_PRIO)
{
    IfxAsclin_Asc_isrReceive(&uart3_handle);
}
IFX_INTERRUPT(uart3_er_isr, UART3_INT_SERVICE, UART3_ER_INT_PRIO)
{
    IfxAsclin_Asc_isrError(&uart3_handle);
}
