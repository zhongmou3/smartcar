/*
 * 	control.c
 *	solve the direction and the speed of the car
 *  Created on: 2020��8��1��
 *      Author: chen
 */

#include "headfile.h"
#include "control.h"


int16 speed=0;
uint8 Lx[MT9V03X_H];                    //�����������ĵ��к�
uint8 Rx[MT9V03X_H];                    //�����������ĵ��к�
uint8 Midx[MT9V03X_H];
uint8 huandao=0;
uint8 zebra_flag=0;
uint8 zebra_end_flag=0;
uint8 stop_flag=0;
uint8 stop_end_flag=0;
uint8 out_huandao=0;
uint8 in_huandao=0;
uint8 in_huandao_end=0;
uint8 huandao_count=0;
uint8 out_begin=0;         //�ж�Ҫ��Ҫ�жϳ�������ֻ�����뻷��֮��ų�����
uint8 time_count_flag=0;
uint32 timecounter=0;
uint8 speed_limit_flag=0;
int16 speed_limit_value=0;
double kp,ki,kd;     //����ʽPID����
int16 ek=0,ek1=0,ek2=0;   //ǰ���������
double out_increment=0;//����ʽPID�������
double out=1200;          //�����
int cardegree=0;
//uint8 start_prepare_flag=0;
//uint8 start_flag=0;
//int start_count=0;
//int startpre_count=0;


int degree_calculation(void)
{
	uint16 i = 0;
  	uint16 j = 0;
  	int cardegree=0;
	//��������㷨
	int InclineValue=0;                       	//��б��
    float ExcursionValue=0;						//ƫ����
    //������б�ȼ���
    for ( i = 1; i <= 15; i++ )
    {
      	if((Midx[50-i] - Midx[50-i-1] < 6) && (Midx[50-i] - Midx[50-i-1] > -6))
        	InclineValue = InclineValue + (Midx[50-i] - Midx[50-i-2]);	//�ò�ַ������������б��
    }
	//ƫ��������
    for ( i = 1; i <= 8; i++ )
    {
        ExcursionValue = ExcursionValue + (Midx[50-i] - 63.5);	//�ò�ַ������ƫ��ֵ�� 69Ϊϣ����������������λ��
    }

    //InclineValue = InclineValue*2.5;	//������б�ȵ�Ӱ��
    float AbsExcursionValue;			//ƫ��������ֵ
	int	AbsInclineValue = InclineValue;	//��б�Ⱦ���ֵ
	double	ExcursionValueCoeff;		//ƫ����ϵ��

    if(ExcursionValue > 0) AbsExcursionValue = ExcursionValue;
    else AbsExcursionValue = -ExcursionValue;

    if(InclineValue > 0) AbsInclineValue = InclineValue;
    else AbsInclineValue = -InclineValue;

	//��������б�ȱȽ�Сʱ����ʾ��ֱ���ϣ����ҳ���Ƚ�������˳����е������߼��ƫ�ƶԷ���Ӱ�����
	//����������б�ȱȽϴ�ʱ����ʾ��������߳�����б����������ʱ����б�ȶԳ��������Ӱ�����
	//��ƫ�����Գ���ǰ�������Ӱ���С���������Է��㳵����ǰ���䣬������ʻ����
    /*if(AbsInclineValue < 20) ExcursionValueCoeff = 0.5;
    else ExcursionValueCoeff = (double)1/(AbsInclineValue*AbsInclineValue*0.015);
    //OLED_Refresh_Gram();*/
    if(AbsExcursionValue > 60) ExcursionValueCoeff = 0.2;
    else if(AbsExcursionValue > 30)
    	ExcursionValueCoeff = (double)(0.004*AbsExcursionValue-0.04);
    else ExcursionValueCoeff = 0.08;
    //ExcursionValueCoeff=2;
    cardegree = (int)(InclineValue*1.945 - 0.14*ExcursionValue);
    if (cardegree>88) cardegree=88;
    if (cardegree<-88) cardegree=-88;
    return cardegree;
}

int speedctrl_calculation(int degree)
{

	//int16 speed;        //��ʵ�ٶ�
	int16 set_speed;    //�����ٶ�
	int speedctrl;

	if(degree<0)
	{
		degree = -degree;
	}
	if(speed_limit_flag==1)
	{
		set_speed = speed_limit_value;
	}
	else if(degree<=20)
	set_speed = 3600;
	else if(degree<=70)
		set_speed=3600-10*(degree-20);
	else
		set_speed=3100;

	ek2 = ek1;//�������ϴ����
	ek1 = ek; //�����ϴ����
	ek = set_speed - speed;//���㵱ǰ���
	//oled_int16(40, 3, ek);
	uint16 absek;
	if(ek>=0) absek=ek;
	else absek=-ek;
	if(absek>2000)
	{
		//����PIDϵ��
		kp = 0.45;
		ki = 0.0035;
		kd = 0;
	}
	else if(absek>1000&&absek<=2000)
	{
		kp = (double)(0.0002*absek+0.05);
		ki = (double)(0.000002*absek-0.0005);
		kd = 0;
	}
	else
	{
		kp = 0.25;
		ki = 0.0015;
		kd = 0;
	}

	//��������ʽPID����
	out_increment = (int16)(kp*(ek-ek1) + ki*ek + kd*(ek-2*ek2+ek2));  //��������
	out = out + out_increment;       //�������

	if(out>4000) out = 4000;     //����޷� ���ܳ���ռ�ձ����ֵ
	if(out<-4000) out = -4000;
	speedctrl = (int)out;    //ǿ��ת��Ϊ������ֵ�����ռ�ձȱ���
	return speedctrl;
}

double rear_diff(int degree)
{
	double xishu;
	if(degree < 0)
	{
		//���ֲ��ٲ���
		int d;
		d=-degree;
		xishu=-B*(-0.000147*d*d+0.016*d-0.00493)/(2*H);
	}
	//��ת��
	else
	{
	    xishu=B*(-0.000147*degree*degree+0.016*degree-0.00493)/(2*H);
	}
}

void rotate(int degree)                         //���ֲ����Լ��ٶȷּ�+������PWM
{
	int speedctrl;      //���ֵ��PWMռ�ձ�(����)
	int speedctrl2;
	int speedctrl2L;		//��ת
	int speedctrl2R;		//��ת
	double xishu=0.1;                             	//���ֲ���ϵ��ֵ


	if(zebra_end_flag==1&&stop_flag==0)			//δ�����ߣ����ٵ�����̬
	{
		pwm_duty(ATOM2_CH0_P33_4, MID_STEER+degree);
		//�ı���ռ�ձ�
		pwm_duty(ATOM0_CH4_P02_4, 0);	//��ǰ
		pwm_duty(ATOM0_CH7_P02_7, 0);	//��ǰ
        pwm_duty(ATOM0_CH6_P02_6, 500);		//�Һ�
        pwm_duty(ATOM0_CH5_P02_5, 500);		//���
	}
	else if(stop_flag == 1&&stop_end_flag==0)	//�ҵ����ߣ�ɲ��
	{
		pwm_duty(ATOM2_CH0_P33_4, MID_STEER);
        //�ı���ռ�ձ�
        pwm_duty(ATOM0_CH4_P02_4, 0);	//��ǰ
        pwm_duty(ATOM0_CH7_P02_7, 0);	//��ǰ
        pwm_duty(ATOM0_CH6_P02_6, 1200);		//�Һ�
        pwm_duty(ATOM0_CH5_P02_5, 1200);		//���
		systick_delay_ms(STM1, 200);	//��ʱ100MS  ʹ��STM0��ʱ��
		mt9v03x_init();	//��ʼ������ͷ
		stop_end_flag=1;
	}
	else if(stop_end_flag==1)					//ͣ����ɣ�Ϩ��
	{
		pwm_duty(ATOM2_CH0_P33_4, MID_STEER);
		//�ı���ռ�ձ�
        pwm_duty(ATOM0_CH4_P02_4, 0);	//��ǰ
        pwm_duty(ATOM0_CH7_P02_7, 0);	//��ǰ
        pwm_duty(ATOM0_CH6_P02_6, 0);		//�Һ�
        pwm_duty(ATOM0_CH5_P02_5, 0);		//���
	}
	else if(out_huandao==1)
	{
		if(time_count_flag==0)
		{
			time_count_flag=1;
			timecounter=0;
		}

		xishu = rear_diff(90);
		pwm_duty(ATOM2_CH0_P33_4, MID_STEER+90);
		speed_limit_flag=1;
		speed_limit_value=2400;
		int ideal_speedctrl = speedctrl_calculation(90);
		if(ideal_speedctrl<0){speedctrl=0; speedctrl2=-ideal_speedctrl;}
		else {speedctrl=ideal_speedctrl; speedctrl2=0;}
		//�ı���ռ�ձ�
		pwm_duty(ATOM0_CH4_P02_4, speedctrl*(1-0.91*xishu));	//��ǰ
		pwm_duty(ATOM0_CH7_P02_7, speedctrl*(1+0.91*xishu));	//��ǰ
		pwm_duty(ATOM0_CH6_P02_6, speedctrl2*(1-0.91*xishu));		//�Һ�
		pwm_duty(ATOM0_CH5_P02_5, speedctrl2*(1+0.91*xishu));		//���
		//systick_delay_ms(STM1, 200);
		//mt9v03x_init();	//��ʼ������ͷ
		if(timecounter==25)
		{
			time_count_flag=0;
			speed_limit_flag=0;
			out_huandao=0;
		}

	}
	else if(in_huandao==1&&in_huandao_end==0)
	{
		//�ı���ռ�ձ�
		degree = 80;
		int ideal_speedctrl = speedctrl_calculation(80);
		if(ideal_speedctrl<0){speedctrl=0; speedctrl2=-ideal_speedctrl;}
		else {speedctrl=ideal_speedctrl; speedctrl2=0;}
		xishu = rear_diff(80);
		pwm_duty(ATOM0_CH4_P02_4, speedctrl*(1-0.91*xishu));	//��ǰ
		pwm_duty(ATOM0_CH7_P02_7, speedctrl*(1+0.91*xishu));	//��ǰ
		pwm_duty(ATOM0_CH6_P02_6, speedctrl2*(1-0.91*xishu));		//�Һ�
		pwm_duty(ATOM0_CH5_P02_5, speedctrl2*(1+0.91*xishu));		//���
		pwm_duty(ATOM2_CH0_P33_4, MID_STEER+80);
		systick_delay_ms(STM1, 200);
		pwm_duty(ATOM0_CH4_P02_4, 0);	//��ǰ
		pwm_duty(ATOM0_CH7_P02_7, 0);	//��ǰ
		pwm_duty(ATOM0_CH6_P02_6, 0);		//�Һ�
		pwm_duty(ATOM0_CH5_P02_5, 0);		//���
		mt9v03x_init();	//��ʼ������ͷ
		in_huandao_end=1;
	}
	else										//������ʻ
	{
		//��ת��
		int ideal_speedctrl = speedctrl_calculation(degree);
		if(ideal_speedctrl < 0){speedctrl = 0; speedctrl2 = -ideal_speedctrl;}
		else {speedctrl=ideal_speedctrl; speedctrl2=0;}
    	if(zebra_flag==0)
    	{
    		pwm_duty(ATOM2_CH0_P33_4, MID_STEER+degree);
    		//�ı���ռ�ձ�
			xishu = rear_diff(degree);
    		pwm_duty(ATOM0_CH4_P02_4, speedctrl*(1-0.91*xishu));	//��ǰ
    		pwm_duty(ATOM0_CH7_P02_7, speedctrl*(1+0.91*xishu));	//��ǰ
    		pwm_duty(ATOM0_CH6_P02_6, speedctrl2*(1-0.91*xishu));		//�Һ�
    		pwm_duty(ATOM0_CH5_P02_5, speedctrl2*(1+0.91*xishu));		//���
    	}
		//��������ת�� 
    	else									
    	{
    		//�ı���ռ�ձ�
    		if(speed>30&&speed<=40)
    		{
    	    	speedctrl = 800 - (speed - 30)*80;
    	    	speedctrl2L = 0;
    	    	speedctrl2R = 0;
    		}
    		else if(speed>40)
    		{
    			speedctrl = 0;
    			speedctrl2L = (speed-40) * 3000;
    			speedctrl2R = (speed-40) * 3000;
    			if(speedctrl2L>5000)speedctrl2L=8000;
    			if(speedctrl2R>5000)speedctrl2R=8000;
    		}
    		else if(speed>20)
    		{
    			speedctrl = 800;
    			speedctrl2L = 0;
    			speedctrl2R = 0;
    	  	}
    	  	else
    	  	{
    	  		speedctrl = 800+(20-speed)*50;
    	  		speedctrl2L = 0;
    	  		speedctrl2R = 0;
    	  	}
    		pwm_duty(ATOM0_CH4_P02_4, 0);	//��ǰ
    		pwm_duty(ATOM0_CH7_P02_7, 2500);	//��ǰ
    		pwm_duty(ATOM0_CH6_P02_6, 3000);		//�Һ�
    		pwm_duty(ATOM0_CH5_P02_5, 0);		//���
			pwm_duty(ATOM2_CH0_P33_4, MID_STEER - 100);
			systick_delay_ms(STM1, 300);
			pwm_duty(ATOM2_CH0_P33_4, MID_STEER - 30);
			pwm_duty(ATOM0_CH4_P02_4, 200);	//��ǰ
			pwm_duty(ATOM0_CH7_P02_7, 200);	//��ǰ
			pwm_duty(ATOM0_CH6_P02_6, 0);		//�Һ�
			pwm_duty(ATOM0_CH5_P02_5, 0);		//���
			systick_delay_ms(STM1, 200);
			pwm_duty(ATOM0_CH4_P02_4, 0);	//��ǰ
			pwm_duty(ATOM0_CH7_P02_7, 0);	//��ǰ
			pwm_duty(ATOM0_CH6_P02_6, 200);		//�Һ�
			pwm_duty(ATOM0_CH5_P02_5, 200);		//���
			systick_delay_ms(STM1, 100);
			mt9v03x_init();	//��ʼ������ͷ
			zebra_end_flag=1;		//������ת������ѽ������������ͣ���׶�
    	}
    }
    	//���ڷ���

    	//for(int i=5;i>=0;i--){ UART_WriteByte(HW_UART3,temp[i]+48);}//data �Ǵ���������
    	//UART_printf(HW_UART3,"\n");
}

