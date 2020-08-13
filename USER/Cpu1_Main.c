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
 * @date       		2020-3-23
 ********************************************************************************************************************/

#include "headfile.h"
#include "control.h"


void core1_main(void)
{
	uint8 start_count=0;
	uint8 startpre_count=0;
	uint8 start_prepare_flag=0;
	uint8 start_flag=0;
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());


    //�û��ڴ˴����ø��ֳ�ʼ��������
    //�����ռ�ձȡ�Ƶ����ɳ���ͬ
	gtm_pwm_init(ATOM2_CH0_P33_4, 50, MID_STEER);
    //��������ʼ��
	gpt12_init(GPT12_T5, GPT12_T5INB_P10_3, GPT12_T5EUDB_P10_1);

	uart_init(UART_0, 9600, UART0_TX_P14_0, UART0_RX_P14_1);
	//PWM��ʼ��
	//�ܽž��Ѿ��복�Ͻӿڶ�Ӧ
	//�����ռ�ձȡ�Ƶ����ɳ���ͬ
	gtm_pwm_init(ATOM0_CH4_P02_4, 10000, 0);//��ǰ//ATOM 0ģ���ͨ��4 ʹ��P02_4�������PWM  PWMƵ��10000HZ  ռ�ձȰٷ�֮0/GTM_ATOM0_PWM_DUTY_MAX*100  GTM_ATOM0_PWM_DUTY_MAX�궨����zf_gtm_pwm.h
	gtm_pwm_init(ATOM0_CH5_P02_5, 10000, 0);//��ǰ
	gtm_pwm_init(ATOM0_CH6_P02_6, 10000, 0);//�Һ�
	gtm_pwm_init(ATOM0_CH7_P02_7, 10000, 0);//���

	pwm_duty(ATOM2_CH0_P33_4, MID_STEER);	//waiting for the ball
			//�ı���ռ�ձ�
	pwm_duty(ATOM0_CH4_P02_4, 0);	//��ǰ
	pwm_duty(ATOM0_CH5_P02_5, 0);	//��ǰ
	pwm_duty(ATOM0_CH6_P02_6, 0);		//�Һ�
	pwm_duty(ATOM0_CH7_P02_7, 0);		//���

	int speed;
	int cardegree=0;
	uint8 temp[9]={0};
	int data;
	while(start_prepare_flag == 0)
	{
		cardegree = degree_calculation();
		pwm_duty(ATOM2_CH0_P33_4, MID_STEER + cardegree);	//waiting for the ball
		//�ı���ռ�ձ�
		pwm_duty(ATOM0_CH4_P02_4, 0);	//��ǰ
		pwm_duty(ATOM0_CH5_P02_5, 0);	//��ǰ
	    pwm_duty(ATOM0_CH6_P02_6, 0);		//�Һ�
	    pwm_duty(ATOM0_CH7_P02_7, 0);		//���

		speed = gpt12_get(GPT12_T5);					//�Һ����ٶ�(������)
    	gpt12_clear(GPT12_T5);
    	speed = -speed;
    	data=speed;
    	if(data<0)
    	{
    		data=1234;
    	}

    	printf("speed: %d\n", speed);
    	if(speed>6)
        {
          	startpre_count++;
        }
		else startpre_count=0;
    	data=startpre_count;
    	int i;
    	for (i=0;i<7;i++)
    	{
    		temp[i]=data%10+'0';
    		data=(data-temp[i])/10;
    	}
    	temp[i]=0;
    	uart_putstr(UART_0 ,"\n startpre : ");
    	uart_putstr(UART_0 ,temp);//data �Ǵ���������
        printf("start: %d\n", startpre_count);
		if(startpre_count>5)
			start_prepare_flag=1;
	}
	while(start_flag==0&&start_prepare_flag==1)								//preparing to start
	{
		cardegree=degree_calculation();
		pwm_duty(ATOM2_CH0_P33_4, MID_STEER+cardegree);
		//�ı���ռ�ձ�
		pwm_duty(ATOM0_CH4_P02_4, 200);	//��ǰ
		pwm_duty(ATOM0_CH5_P02_5, 200);	//��ǰ
		pwm_duty(ATOM0_CH6_P02_6, 0);		//�Һ�
		pwm_duty(ATOM0_CH7_P02_7, 0);		//���
		
		speed = gpt12_get(GPT12_T5);					//�Һ����ٶ�(������)
    	gpt12_clear(GPT12_T5);
    	speed = -speed;

    	printf("start: %d\n", startpre_count);
		if(speed>12)
    	{
    	   	start_count++;
    	}
    	else start_count=0;
		int i;
		data=start_count;
		for (i=0;i<7;i++)
		{
			temp[i]=data%10+'0';
			data=(data-temp[i])/10;
		}
		temp[i]=0;
		uart_putstr(UART_0 ,"\n startpre : ");
		uart_putstr(UART_0 ,temp);//data �Ǵ���������
		if(start_count>5)
		{
			start_flag=1;
		}
		//printf("start: %d\n", start_count);
	}
    while (start_flag==1)
    {
		//�û��ڴ˴���д�������
    	
    	//else startpre_count=0;
    	cardegree=degree_calculation();
    	rotate(cardegree);

    }
}
