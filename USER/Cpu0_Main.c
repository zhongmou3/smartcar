/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		tasking v6.3r1
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-8-13
 ********************************************************************************************************************/


#include "headfile.h"
#include "control.h"

void core0_main(void)
{
	disableInterrupts();
	uint8 start_count=0;
	uint8 startpre_count=0;
	uint8 start_prepare_flag=0;
	uint8 start_flag=0;
	uint8 menu_flag=1;
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    //��������ʼ��
	gpt12_init(GPT12_T5, GPT12_T5INB_P10_3, GPT12_T5EUDB_P10_1);
	gpt12_init(GPT12_T2, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6);
	pit_interrupt_ms(CCU6_0, PIT_CH0, 10);

    //�û��ڴ˴����ø��ֳ�ʼ��������
    //�����ռ�ձȡ�Ƶ����ɳ���ͬ
	gtm_pwm_init(ATOM1_CH2_P10_5, 50, MID_STEER);


	uart_init(UART_0, 9600, UART0_TX_P14_0, UART0_RX_P14_1);
	//PWM��ʼ��
	//�ܽž��Ѿ��복�Ͻӿڶ�Ӧ
	//�����ռ�ձȡ�Ƶ����ɳ���ͬ
	gtm_pwm_init(ATOM0_CH4_P02_4, 10000, 0);//��ǰ//ATOM 0ģ���ͨ��4 ʹ��P02_4�������PWM  PWMƵ��10000HZ  ռ�ձȰٷ�֮0/GTM_ATOM0_PWM_DUTY_MAX*100  GTM_ATOM0_PWM_DUTY_MAX�궨����zf_gtm_pwm.h
	gtm_pwm_init(ATOM0_CH5_P02_5, 10000, 0);//��ǰ
	gtm_pwm_init(ATOM0_CH6_P02_6, 10000, 0);//�Һ�
	gtm_pwm_init(ATOM0_CH7_P02_7, 10000, 0);//���

	pwm_duty(ATOM1_CH2_P10_5, MID_STEER);	//waiting for the ball
			//�ı���ռ�ձ�
	pwm_duty(ATOM0_CH4_P02_4, 0);	//��ǰ
	pwm_duty(ATOM0_CH5_P02_5, 0);	//��ǰ
	pwm_duty(ATOM0_CH6_P02_6, 0);		//�Һ�
	pwm_duty(ATOM0_CH7_P02_7, 0);		//���

	gpio_init(P20_8, GPO, 1, PUSHPULL);//����P20_8Ϊ��� Ĭ������͵�ƽ  PUSHPULL���������
	gpio_init(P20_9, GPO, 1, PUSHPULL);
	gpio_init(P21_4, GPO, 1, PUSHPULL);
	gpio_init(P21_5, GPO, 1, PUSHPULL);

	gtm_pwm_init(ATOM3_CH1_P33_5, 500, 0);//���
	//gpio_init(P33_5, GPO, 0, PUSHPULL);//����P20_8Ϊ��� Ĭ������͵�ƽ  PUSHPULL���������
    uint16 lowcount;
    uint16 highcount;
    uint16 gpio_proc_flag;
	uint16 left_lowcount;
	uint16 right_lowcount;
	gpio_init(P33_8, GPI, 1, PULLUP);		//up
	gpio_init(P33_9, GPI, 1, PULLUP);		//right
	gpio_init(P33_11, GPI, 1, PULLUP);		//left
	gpio_init(P33_12, GPI, 1, PULLUP);		//mid
	//gpio_init(P02_8, GPO, 1, PUSHPULL);
	enableInterrupts();
	uint8 temp[9]={0};
	int data;
	while(menu_flag==1)
	{
		if(gpio_get(P33_8))//�ߵ�ƽ,û����
		{
			highcount++;
			if(highcount>200)
			{
				lowcount=0;
				gpio_proc_flag=0;
			}
		}
		else
		{
			highcount=0;
			if(gpio_proc_flag==0)//�͵�ƽ,����
				lowcount++;
		}

		if(lowcount>200&&gpio_proc_flag==0)
		{
			speed_gear++;
			if(speed_gear>3) speed_gear=0;
			gpio_proc_flag=1;
		}
		if(speed_gear==0)
		{
			gpio_set(P21_5, 1);
			gpio_set(P20_8, 1);
			gpio_set(P20_9, 1);
		}
		if(speed_gear==1)
		{
			gpio_set(P21_5, 1);
			gpio_set(P20_8, 1);
			gpio_set(P20_9, 0);
		}
		if(speed_gear==2)
		{
			gpio_set(P21_5, 1);
			gpio_set(P20_8, 0);
		    gpio_set(P20_9, 1);
		}
		if(speed_gear==3)
		{
			gpio_set(P21_5, 1);
			gpio_set(P20_8, 0);
		    gpio_set(P20_9, 0);
		}
		if (gpio_get(P33_9))
		{
			right_lowcount=0;
		}
		else 
		{

			if(right_lowcount>100)
			{
				gpio_set(P21_4, 1);		//��ת�ư�
				garage_flag=1;
			}
			else
			{
				right_lowcount++;
			}
		}
		if (gpio_get(P33_11))
		{
			left_lowcount=0;
		}
		else 
		{
			if(left_lowcount>100)
			{
				gpio_set(P21_4, 0);		//��ת����
				garage_flag=2;
			}
			else
			{
				left_lowcount++;
			}
		}
		if(gpio_get(P33_12)==0)
		{
			menu_flag=0;
			gpio_init(P20_8, GPO, 0, PUSHPULL);//����P20_8Ϊ��� Ĭ������͵�ƽ  PUSHPULL���������
			gpio_init(P20_9, GPO, 0, PUSHPULL);
			gpio_init(P21_4, GPO, 0, PUSHPULL);
			gpio_init(P21_5, GPO, 0, PUSHPULL);
		}
	
	}
	while(start_prepare_flag == 0)
	{
		cardegree = degree_calculation();
		pwm_duty(ATOM1_CH2_P10_5, MID_STEER + cardegree);	//waiting for the ball
		//�ı���ռ�ձ�
		pwm_duty(ATOM0_CH4_P02_4, 0);	//��ǰ
		pwm_duty(ATOM0_CH5_P02_5, 0);	//��ǰ
	    pwm_duty(ATOM0_CH6_P02_6, 0);		//�Һ�
	    pwm_duty(ATOM0_CH7_P02_7, 0);		//���

		if(speed>100)
	    {
	      	startpre_count++;
	    }
		else startpre_count=0;
		if(startpre_count>5)
			start_prepare_flag=1;
	}
	while(start_flag==0&&start_prepare_flag==1)								//preparing to start
	{
		cardegree=degree_calculation();
		pwm_duty(ATOM1_CH2_P10_5, MID_STEER+cardegree);
		//�ı���ռ�ձ�
		pwm_duty(ATOM0_CH4_P02_4, 200);	//��ǰ
	    pwm_duty(ATOM0_CH7_P02_7, 200);	//��ǰ
	    pwm_duty(ATOM0_CH6_P02_6, 0);		//�Һ�
	    pwm_duty(ATOM0_CH5_P02_5, 0);		//���
		if(speed>300)
		{
		   	start_count++;
		}
		else start_count=0;
		if(start_count>5)
		{
			start_flag=1;
		}
	}
	while (start_flag==1)
	{
		//�û��ڴ˴���д�������
		cardegree=degree_calculation();
		rotate(cardegree);
	}
}
