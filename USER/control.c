/*
 * 	control.c
 *	solve the direction and the speed of the car
 *  Created on: 2020年8月1日
 *      Author: chen
 */

#include "headfile.h"
#include "control.h"


int16 speed=0;
uint8 Lx[MT9V03X_H];                    //左引导线中心点列号
uint8 Rx[MT9V03X_H];                    //右引导线中心点列号
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
uint8 out_begin=0;         //判断要不要判断出环岛，只有了入环岛之后才出环岛
uint8 time_count_flag=0;
uint32 timecounter=0;
uint8 speed_limit_flag=0;
int16 speed_limit_value=0;
uint8 left_out_flag=0; 	//左侧将冲出赛道
uint8 right_out_flag=0;	//右侧将冲出赛道
double kp,ki,kd;     	//增量式PID参数
int16 ek=0,ek1=0,ek2=0; //前后三次误差
double out_increment=0;	//增量式PID输出增量
double out=1200;  		//输出量
int cardegree=0;
uint8 speed_gear=2;		//速度挡位
uint8 garage_flag=1;	//为1右转进车库；为2左转进车库



int degree_calculation(void)
{
	uint16 i = 0;
  	uint16 j = 0;
	double speed_fact = 1;
  	int cardegree=0;
	//方向控制算法
  	uint32 sum_out=0;
  	uint16 out_mid=64;
  	//uint32 sum_out_right=0;
	int InclineValue=0;                       	//倾斜度
    float ExcursionValue=0;						//偏移量
    //中线倾斜度计算
    for ( i = 1; i <= 25; i++ )
    {
      	if((Midx[60-i] - Midx[60-i-1] < 6) && (Midx[60-i] - Midx[60-i-1] > -6))
        	InclineValue = InclineValue + (Midx[60-i] - Midx[60-i-2]);	//用差分方法求解中线倾斜度
    }

	//偏移量计算
    for ( i = 1; i <= 20; i++ )
    {
        ExcursionValue = ExcursionValue + (Midx[60-i] - 63.5);	//用差分法求解总偏移值， 63.5为希望车在赛道所处的位置
    }
    for (i=1;i<6;i++)	//检测是否冲出赛道
    {
    	sum_out += Midx[MT9V03X_H-i];
    }
    out_mid = sum_out/5;
    if(out_mid < 30)
    {
    	left_out_flag = 1; right_out_flag = 0;
    }
    else if(out_mid > MT9V03X_W - 30)
    {
    	left_out_flag = 0; right_out_flag = 1;
    }

    else {left_out_flag = 0; right_out_flag = 0;}
    //InclineValue = InclineValue*2.5;	//增大倾斜度的影响
    float AbsExcursionValue;			//偏移量绝对值
	int	AbsInclineValue = InclineValue;	//倾斜度绝对值
	double	ExcursionValueCoeff;		//偏移量系数
	double	InclineValueCoeff;		//偏移量系数

    if(ExcursionValue > 0) AbsExcursionValue = ExcursionValue;
    else AbsExcursionValue = -ExcursionValue;

    if(InclineValue > 0) AbsInclineValue = InclineValue;
    else AbsInclineValue = -InclineValue;

	//当中线倾斜度比较小时，表示在直道上，并且车身比较正。因此车身中点与中线间的偏移对方向影响更大
	//而若中线倾斜度比较大时，表示在弯道或者车身倾斜很厉害。此时，倾斜度对车辆方向的影响更大
	//而偏移量对车辆前进方向的影响更小，这样可以方便车辆提前过弯，减少行驶距离
    /*if(AbsInclineValue < 20) ExcursionValueCoeff = 0.5;
    else ExcursionValueCoeff = (double)1/(AbsInclineValue*AbsInclineValue*0.015);
    //OLED_Refresh_Gram();*/
    /*if(AbsExcursionValue > 60) ExcursionValueCoeff = 0.2;
    else if(AbsExcursionValue > 30)
    	ExcursionValueCoeff = (double)(0.004*AbsExcursionValue-0.04);
    else ExcursionValueCoeff = 0.08;*/
    //ExcursionValueCoeff=2;
	if (speed>1000&&AbsInclineValue>20) speed_fact=(double)(speed*0.00008+1);
	else speed_fact=1;
	if(AbsInclineValue>17) InclineValueCoeff=0.85;
	else InclineValueCoeff=0.1;
    cardegree = (int)(InclineValue*speed_fact*InclineValueCoeff - 0.1*ExcursionValue);
    if (cardegree>88) cardegree=88;
    if (cardegree<-88) cardegree=-88; 
    if (left_out_flag == 1)
    {
    	cardegree = cardegree + 90;
    	if(cardegree>95) cardegree = 95;
    }
    if (right_out_flag == 1)
    {
       	cardegree = cardegree - 90;
       	if(cardegree>95) cardegree = -95;
    }
    return cardegree;
}

int speedctrl_calculation(int degree)
{

	//int16 speed;        //真实速度
	int16 set_speed;    //期望速度
	int speedctrl;
	if(degree<0)
	{
		degree = -degree;
	}
	if(speed_gear==2)
	{
		
		if(speed_limit_flag==1)
		{
			set_speed = speed_limit_value;
		}
		else if(degree<=20)
		set_speed = 3500;
		else if(degree<=70)
			set_speed=3500-8*(degree-20);
		else
			set_speed=3100;
	}
	
	if(speed_gear==1)
	{
		if(speed_limit_flag==1)
		{
			set_speed = speed_limit_value;
		}
		else if(degree<=20)
		set_speed = 3000;
		else if(degree<=70)
			set_speed=3000-8*(degree-20);
		else
			set_speed=2600;
	}

	if(speed_gear==3)
	{
		if(speed_limit_flag==1)
		{
			set_speed = speed_limit_value;
		}
		else if(degree<=20)
		set_speed = 4000;
		else if(degree<=70)
			set_speed=4000-8*(degree-20);
		else
			set_speed=3600;
	}
	ek2 = ek1;//保存上上次误差
	ek1 = ek; //保存上次误差
	ek = set_speed - speed;//计算当前误差
	//oled_int16(40, 3, ek);
	uint16 absek;
	if(ek>=0) absek=ek;
	else absek=-ek;
	kp = 1;
	if(absek>2500)
	{
		kp = 0.8;
		ki = 0.0045;
		kd = 0;
	}
	if(absek>2000)
	{
		//设置PID系数
		ki = 0.0035;
		kd = 0;
	}
	else if(absek>1000&&absek<=2000)
	{
		//kp = (double)(0.0002*absek+0.05);
		ki = (double)(0.000002*absek-0.0005);
		kd = 0;
	}
	else
	{
		//kp = 0.25;
		ki = 0.0005;
		kd = 0;
	}

	//进行增量式PID运算
	out_increment = (int16)(kp*(ek-ek1) + ki*ek + kd*(ek-2*ek1+ek2));  //计算增量
	out = out + out_increment;       //输出增量

	if(out>4000) out = 4000;     //输出限幅 不能超过占空比最大值
	if(out<-4000) out = -4000;
	speedctrl = (int)out;    //强制转换为整数后赋值给电机占空比变量
	return speedctrl;
}

double rear_diff(int degree)
{
	double xishu;
	if(degree < 0)
	{
		//后轮差速部分
		int d;
		d=-degree;
		xishu=-B*(-0.000147*d*d+0.016*d-0.00493)/(2*H);
	}
	//右转弯
	else
	{
	    xishu=B*(-0.000147*degree*degree+0.016*degree-0.00493)/(2*H);
	}
}

void rotate(int degree)                         //后轮差速以及速度分级+舵机电机PWM
{
	int speedctrl;      //后轮电机PWM占空比(正向)
	int speedctrl2;
	int speedctrl2L;		//倒转
	int speedctrl2R;		//倒转
	double xishu=0.1;                             	//后轮差速系数值

	if(zebra_end_flag==1&&stop_flag==0)			//未到底线，慢速调整姿态
	{
		pwm_duty(ATOM1_CH2_P10_5, MID_STEER+degree);
		//改变电机占空比
		pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
		pwm_duty(ATOM0_CH7_P02_7, 0);	//左前
        pwm_duty(ATOM0_CH6_P02_6, 0);		//右后
        pwm_duty(ATOM0_CH5_P02_5, 0);		//左后
	}
	else if(stop_flag == 1&&stop_end_flag==0)	//找到底线，刹车
	{
		pwm_duty(ATOM1_CH2_P10_5, MID_STEER);
        //改变电机占空比
        pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
        pwm_duty(ATOM0_CH7_P02_7, 0);	//左前
        pwm_duty(ATOM0_CH6_P02_6, 700);		//右后
        pwm_duty(ATOM0_CH5_P02_5, 700);		//左后
		systick_delay_ms(STM1, 200);	//延时100MS  使用STM0定时器
		mt9v03x_init();	//初始化摄像头
		stop_end_flag=1;
	}
	else if(stop_end_flag==1)					//停车完成，熄火
	{
		pwm_duty(ATOM1_CH2_P10_5, MID_STEER);
		//改变电机占空比
        pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
        pwm_duty(ATOM0_CH7_P02_7, 0);	//左前
        pwm_duty(ATOM0_CH6_P02_6, 0);		//右后
        pwm_duty(ATOM0_CH5_P02_5, 0);		//左后
	}
	else if(out_huandao==1)
	{
		if(time_count_flag==0)
		{
			time_count_flag=1;
			timecounter=0;	
		}
		if(timecounter<20)
		{
			xishu = rear_diff(90);
			pwm_duty(ATOM1_CH2_P10_5, MID_STEER+90);
			speed_limit_flag=1;
			speed_limit_value=2400;
			int ideal_speedctrl = speedctrl_calculation(90);
			if(ideal_speedctrl<0){speedctrl=0; speedctrl2=-ideal_speedctrl;}
			else {speedctrl=ideal_speedctrl; speedctrl2=0;}
			//改变电机占空比
			pwm_duty(ATOM0_CH4_P02_4, speedctrl*(1-0.91*xishu));	//右前
			pwm_duty(ATOM0_CH7_P02_7, speedctrl*(1+0.91*xishu));	//左前
			pwm_duty(ATOM0_CH6_P02_6, speedctrl2*(1-0.91*xishu));		//右后
			pwm_duty(ATOM0_CH5_P02_5, speedctrl2*(1+0.91*xishu));		//左后
		}
		else 
		{
			speed_limit_flag=0;
			int ideal_speedctrl = speedctrl_calculation(degree);
			if(ideal_speedctrl < 0){speedctrl = 0; speedctrl2 = -ideal_speedctrl;}
			else {speedctrl=ideal_speedctrl; speedctrl2=0;}
    		if(zebra_flag==0)
    		{
    			pwm_duty(ATOM1_CH2_P10_5, MID_STEER+degree);
    			//改变电机占空比
				xishu = rear_diff(degree);
    			pwm_duty(ATOM0_CH4_P02_4, speedctrl*(1-0.91*xishu));	//右前
    			pwm_duty(ATOM0_CH7_P02_7, speedctrl*(1+0.91*xishu));	//左前
    			pwm_duty(ATOM0_CH6_P02_6, speedctrl2*(1-0.91*xishu));		//右后
    			pwm_duty(ATOM0_CH5_P02_5, speedctrl2*(1+0.91*xishu));		//左后
			}
			//systick_delay_ms(STM1, 200);
			//mt9v03x_init();	//初始化摄像头
			if(timecounter>100)
			{
				time_count_flag=0;
				out_huandao=0;
				in_huandao=0;
				in_huandao_end=0;
			}
		}
	}
	else if(in_huandao==1&&in_huandao_end==0)
	{
		//改变电机占空比
		degree = 80;
		speed_limit_flag=1;
		speed_limit_value=1500;
		int ideal_speedctrl = speedctrl_calculation(80);
		if(ideal_speedctrl<0){speedctrl = 0; speedctrl2 = -ideal_speedctrl;}
		else {speedctrl = ideal_speedctrl; speedctrl2 = 0;}
		if(time_count_flag==0)
		{
			//control_in_huandao_flag=1;
			time_count_flag=1;
			timecounter=0;
		}
		xishu = rear_diff(80);
		pwm_duty(ATOM0_CH4_P02_4, speedctrl*(1-0.91*xishu));	//右前
		pwm_duty(ATOM0_CH7_P02_7, speedctrl*(1+0.91*xishu));	//左前
		pwm_duty(ATOM0_CH6_P02_6, speedctrl2*(1-0.91*xishu));		//右后
		pwm_duty(ATOM0_CH5_P02_5, speedctrl2*(1+0.91*xishu));		//左后
		pwm_duty(ATOM1_CH2_P10_5, MID_STEER+80);
		//systick_delay_ms(STM1, 200);
		/*pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
		pwm_duty(ATOM0_CH7_P02_7, 0);	//左前
		pwm_duty(ATOM0_CH6_P02_6, 0);		//右后
		pwm_duty(ATOM0_CH5_P02_5, 0);		//左后*/
		//mt9v03x_init();	//初始化摄像头
		if(timecounter==30)
		{
			//time_count_flag=0;
			speed_limit_flag=0;
			in_huandao_end=1;
			//control_in_huandao_flag=0;
		}
		
	}
	else										//正常行驶
	{
		//转弯
		int ideal_speedctrl = speedctrl_calculation(degree);
		if(ideal_speedctrl < 0){speedctrl = 0; speedctrl2 = -ideal_speedctrl;}
		else {speedctrl=ideal_speedctrl; speedctrl2=0;}
    	if(zebra_flag==0)
    	{
    		pwm_duty(ATOM1_CH2_P10_5, MID_STEER+degree);
    		//改变电机占空比
			xishu = rear_diff(degree);
    		pwm_duty(ATOM0_CH4_P02_4, speedctrl*(1-0.91*xishu));	//右前
    		pwm_duty(ATOM0_CH7_P02_7, speedctrl*(1+0.91*xishu));	//左前
    		pwm_duty(ATOM0_CH6_P02_6, speedctrl2*(1-0.91*xishu));		//右后
    		pwm_duty(ATOM0_CH5_P02_5, speedctrl2*(1+0.91*xishu));		//左后
    	}
		//遇斑马线转弯 
    	else									
    	{
			if(garage_flag==1)
			{
				if(speed_gear==1)
				{
					pwm_duty(ATOM0_CH4_P02_4, 800);	//右前
    				pwm_duty(ATOM0_CH7_P02_7, 800);	//左前
    				pwm_duty(ATOM0_CH6_P02_6, 0);		//右后
    				pwm_duty(ATOM0_CH5_P02_5, 0);		//左后
					pwm_duty(ATOM1_CH2_P10_5, MID_STEER - 100);
					systick_delay_ms(STM1, 300);
					pwm_duty(ATOM1_CH2_P10_5, MID_STEER - 30);
					pwm_duty(ATOM0_CH4_P02_4, 400);	//右前
					pwm_duty(ATOM0_CH7_P02_7, 400);	//左前
					pwm_duty(ATOM0_CH6_P02_6, 0);		//右后
					pwm_duty(ATOM0_CH5_P02_5, 0);		//左后
					systick_delay_ms(STM1, 100);
					pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
					pwm_duty(ATOM0_CH7_P02_7, 0);	//左前
					pwm_duty(ATOM0_CH6_P02_6, 400);		//右后
					pwm_duty(ATOM0_CH5_P02_5, 400);		//左后
					systick_delay_ms(STM1, 100);
					mt9v03x_init();	//初始化摄像头
					zebra_end_flag=1;		//斑马线转弯程序已结束，进入最后停车阶段
				}
				if(speed_gear==2)
				{
					pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
    				pwm_duty(ATOM0_CH7_P02_7, 3000);	//左前
    				pwm_duty(ATOM0_CH6_P02_6, 3500);		//右后
    				pwm_duty(ATOM0_CH5_P02_5, 0);		//左后
					pwm_duty(ATOM1_CH2_P10_5, MID_STEER - 100);
					systick_delay_ms(STM1, 200);
					pwm_duty(ATOM1_CH2_P10_5, MID_STEER - 30);
					pwm_duty(ATOM0_CH4_P02_4, 1000);	//右前
					pwm_duty(ATOM0_CH7_P02_7, 1000);	//左前
					pwm_duty(ATOM0_CH6_P02_6, 0);		//右后
					pwm_duty(ATOM0_CH5_P02_5, 0);		//左后
					systick_delay_ms(STM1, 100);
					pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
					pwm_duty(ATOM0_CH7_P02_7, 0);	//左前
					pwm_duty(ATOM0_CH6_P02_6, 700);		//右后
					pwm_duty(ATOM0_CH5_P02_5, 700);		//左后
					systick_delay_ms(STM1, 100);
					mt9v03x_init();	//初始化摄像头
					zebra_end_flag=1;		//斑马线转弯程序已结束，进入最后停车阶段
				}
				if(speed_gear==3)
				{
					pwm_duty(ATOM0_CH4_P02_4, 800);	//右前
    				pwm_duty(ATOM0_CH7_P02_7, 800);	//左前
    				pwm_duty(ATOM0_CH6_P02_6, 0);		//右后
    				pwm_duty(ATOM0_CH5_P02_5, 0);		//左后
					pwm_duty(ATOM1_CH2_P10_5, MID_STEER - 100);
					systick_delay_ms(STM1, 300);
					pwm_duty(ATOM1_CH2_P10_5, MID_STEER - 30);
					pwm_duty(ATOM0_CH4_P02_4, 400);	//右前
					pwm_duty(ATOM0_CH7_P02_7, 400);	//左前
					pwm_duty(ATOM0_CH6_P02_6, 0);		//右后
					pwm_duty(ATOM0_CH5_P02_5, 0);		//左后
					systick_delay_ms(STM1, 100);
					pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
					pwm_duty(ATOM0_CH7_P02_7, 0);	//左前
					pwm_duty(ATOM0_CH6_P02_6, 400);		//右后
					pwm_duty(ATOM0_CH5_P02_5, 400);		//左后
					systick_delay_ms(STM1, 100);
					mt9v03x_init();	//初始化摄像头
					zebra_end_flag=1;		//斑马线转弯程序已结束，进入最后停车阶段
				}
			}
    	}
    }
    	//串口发送

    	//for(int i=5;i>=0;i--){ UART_WriteByte(HW_UART3,temp[i]+48);}//data 是传感器数据
    	//UART_printf(HW_UART3,"\n");
}

