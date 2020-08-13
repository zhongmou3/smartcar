/*
 * 	control.c
 *	solve the direction and the speed of the car
 *  Created on: 2020年8月1日
 *      Author: chen
 */

#include "headfile.h"
#include "control.h"


int16 speed;
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
float kp,ki,kd;     //增量式PID参数
int16 ek=0,ek1=0,ek2=0;   //前后三次误差
float out_increment=0;//增量式PID输出增量
float out=1200;          //输出量
//uint8 start_prepare_flag=0;
//uint8 start_flag=0;
//int start_count=0;
//int startpre_count=0;


int degree_calculation(void)
{
	uint16 i = 0;
  	uint16 j = 0;
  	int cardegree=0;
	//方向控制算法
	int InclineValue=0;                       	//倾斜度
    int ExcursionValue=0;						//偏移量
    //中线倾斜度计算
    for ( i = 1; i <= 15; i++ )
    {
      	if((Midx[50-i] - Midx[50-i-1] < 6) && (Midx[50-i] - Midx[50-i-1] > -6))
        	InclineValue = InclineValue + (Midx[50-i] - Midx[50-i-2]);	//用差分方法求解中线倾斜度
    }
	//偏移量计算
    for ( i = 1; i <= 8; i++ )
    {
        ExcursionValue = ExcursionValue + (Midx[50-i] - 69);	//用差分法求解总偏移值， 69为希望车在赛道所处的位置
    }

    InclineValue = InclineValue*2.5;	//增大倾斜度的影响
    int AbsExcursionValue;			//偏移量绝对值
	int	AbsInclineValue = InclineValue;	//倾斜度绝对值
	int	ExcursionValueCoeff;		//偏移量系数

    if(ExcursionValue > 0) AbsExcursionValue = ExcursionValue;
    else AbsExcursionValue = -ExcursionValue;

    if(InclineValue > 0) AbsInclineValue = InclineValue;
    else AbsInclineValue = -InclineValue;

	//当中线倾斜度比较小时，表示在直道上，并且车身比较正。因此车身中点与中线间的偏移对方向影响更大
	//而若中线倾斜度比较大时，表示在弯道或者车身倾斜很厉害。此时，倾斜度对车辆方向的影响更大
	//而偏移量对车辆前进方向的影响更小，这样可以方便车辆提前过弯，减少行驶距离
    if(AbsInclineValue < 10) ExcursionValueCoeff = 0.4;
    else ExcursionValueCoeff = 1/(AbsInclineValue*AbsInclineValue*0.018);
    //OLED_Refresh_Gram();
    if(AbsExcursionValue > 100) ExcursionValueCoeff = 2;
    else ExcursionValueCoeff = 0;

    cardegree = InclineValue*0.77 - ExcursionValueCoeff*ExcursionValue*0.105;
    if (cardegree>88) cardegree=88;
    if (cardegree<-88) cardegree=-88;
    return cardegree;
}

int speedctrl_calculation(int degree)
{

	//int16 speed;        //真实速度
	int16 set_speed;    //期望速度
	int speedctrl;
	int data;
	if(degree<0)
	{
		degree = -degree;
	}
	if(degree<=25)
		set_speed=180;
	/*else if(degree<=65)
		set_speed=250-2*degree;*/
	else
		set_speed=180;

	ek2 = ek1;//保存上上次误差
	ek1 = ek; //保存上次误差
	ek = set_speed - speed;//计算当前误差

	//设置PID系数
	kp = 8.5;
	ki = 0.15;
	kd = 0.45;

	//进行增量式PID运算
	out_increment = (int16)(kp*(ek-ek1) + ki*ek + kd*(ek-2*ek2+ek2));  //计算增量
	out += out_increment;       //输出增量
	data=out;
	uart_putchar(UART_0 ,'o');
	uart_putchar(UART_0 ,':');
	int i;
	uint8 temp[8];
	for (i=0;i<7;i++)
	{
		temp[i]=data%10+'0';
		data=(data-temp[i])/10;
	}
	temp[i]=0;
	for (i=0;i<7;i++)
	{
		uart_putchar(UART_0 ,temp[i]+48);
	}

	uart_putchar(UART_0 ,'\n');
	if(out>3300)out = 3300;     //输出限幅 不能超过占空比最大值
	if(out<-3300)out = -3300;
	//out=1000.123;
	speedctrl = (int)out;    //强制转换为整数后赋值给电机占空比变量
	return speedctrl;
}

void rotate(int degree)                         //后轮差速以及速度分级+舵机电机PWM
{
	int data1;
	int speedctrl;      //后轮电机PWM占空比(正向)
	int speedctrl2;
	int speedctrl2L;		//倒转
	int speedctrl2R;		//倒转
	int i=0;
	int temp[6];                                  	//串口发送数据
	double xishu=0.1;                             	//后轮差速系数值
	int16 temp_speed;

	temp_speed = gpt12_get(GPT12_T5);					//右后轮速度(编码器)
	gpt12_clear(GPT12_T5);
	printf("temp_speed: %d\n", temp_speed);
	temp_speed = -temp_speed;

	if(zebra_end_flag==1&&stop_flag==0)			//未到底线，慢速调整姿态
	{
		pwm_duty(ATOM2_CH0_P33_4, MID_STEER+degree);
		//改变电机占空比
		pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
		pwm_duty(ATOM0_CH7_P02_7, 0);	//左前
        pwm_duty(ATOM0_CH6_P02_6, 500);		//右后
        pwm_duty(ATOM0_CH5_P02_5, 500);		//左后
	}
	else if(stop_flag == 1&&stop_end_flag==0)	//找到底线，刹车
	{
		pwm_duty(ATOM2_CH0_P33_4, MID_STEER);
        //改变电机占空比
        pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
        pwm_duty(ATOM0_CH7_P02_7, 0);	//左前
        pwm_duty(ATOM0_CH6_P02_6, 1200);		//右后
        pwm_duty(ATOM0_CH5_P02_5, 1200);		//左后
		systick_delay_ms(STM1, 200);	//延时100MS  使用STM0定时器
		mt9v03x_init();	//初始化摄像头
		stop_end_flag=1;
	}
	else if(stop_end_flag==1)					//停车完成，熄火
	{
		pwm_duty(ATOM2_CH0_P33_4, MID_STEER);
		//改变电机占空比
        pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
        pwm_duty(ATOM0_CH7_P02_7, 0);	//左前
        pwm_duty(ATOM0_CH6_P02_6, 0);		//右后
        pwm_duty(ATOM0_CH5_P02_5, 0);		//左后
	}
	else if(out_huandao==1)
	{
		int ideal_speedctrl = speedctrl_calculation(60);
		if(ideal_speedctrl<0){speedctrl=0; speedctrl2=-ideal_speedctrl;}
		else {speedctrl=ideal_speedctrl; speedctrl2=0;}
		//speedctrl = speedctrl+500;
		pwm_duty(ATOM2_CH0_P33_4, MID_STEER+60);
		//改变电机占空比
		pwm_duty(ATOM0_CH4_P02_4, speedctrl*(1-0.91*xishu));	//右前
		pwm_duty(ATOM0_CH7_P02_7, speedctrl*(1+0.91*xishu));	//左前
		pwm_duty(ATOM0_CH6_P02_6, speedctrl2*(1-0.91*xishu));		//右后
		pwm_duty(ATOM0_CH5_P02_5, speedctrl2*(1+0.91*xishu));		//左后
		systick_delay_ms(STM1, 80);
		mt9v03x_init();	//初始化摄像头
		out_huandao=0;
	}
	else if(in_huandao==1&&in_huandao_end==0)
	{
		//speedctrl = speedctrl_calculation(20);
		//speedctrl = speedctrl+200;
		//pwm_duty(ATOM2_CH0_P33_4, MID_STEER-1);
		//改变电机占空比
		int ideal_speedctrl = speedctrl_calculation(80);
		if(ideal_speedctrl<0){speedctrl=0; speedctrl2=-ideal_speedctrl;}
		else {speedctrl=ideal_speedctrl; speedctrl2=0;}
		pwm_duty(ATOM0_CH4_P02_4, speedctrl*(1-0.91*xishu));	//右前
		pwm_duty(ATOM0_CH7_P02_7, speedctrl*(1+0.91*xishu));	//左前
		pwm_duty(ATOM0_CH6_P02_6, speedctrl2*(1-0.91*xishu));		//右后
		pwm_duty(ATOM0_CH5_P02_5, speedctrl2*(1+0.91*xishu));		//左后
		//systick_delay_ms(STM1, 500);
		pwm_duty(ATOM2_CH0_P33_4, MID_STEER+80);
		systick_delay_ms(STM1, 200);
		pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
		pwm_duty(ATOM0_CH7_P02_7, 0);	//左前
		pwm_duty(ATOM0_CH6_P02_6, 0);		//右后
		pwm_duty(ATOM0_CH5_P02_5, 0);		//左后
		out=0;
		mt9v03x_init();	//初始化摄像头
		//out=100;
		//in_huandao=0;
		in_huandao_end=1;
	}
	else										//正常行驶
	{
		//左转弯
		if(degree<0)
		{
    		//后轮差速部分
			int d;
			//degree=degree;
			d=-degree;
			data1=d;
			for (i=0;i<6;i++)
			{
				temp[i]=data1%10;
				data1=(data1-temp[i])/10;
			}
			xishu=B*(-0.000147*d*d+0.016*d-0.00493)/(2*H);
			speed=temp_speed/(1+0.9*xishu);
		}
		//右转弯
		else
    	{
    	  //r_speed=car_speed+car_speed*B*tan(degree-MID_STEER)/(2*H);
    	  //l_speed=car_speed-car_speed*B*tan(degree-MID_STEER)/(2*H);
    	    //degree=degree*1.2;
    	    xishu=B*(-0.000147*degree*degree+0.016*degree-0.00493)/(2*H);
    	    speed=temp_speed/(1-0.9*xishu);
    	    data1=degree;
    	    for (i=0;i<6;i++)
    	    {
    	        temp[i]=data1%10;
    	        data1=(data1-temp[i])/10;
    	    }
		}
		int ideal_speedctrl = speedctrl_calculation(degree);
		if(ideal_speedctrl<0){speedctrl=0; speedctrl2=-ideal_speedctrl;}
		else {speedctrl=ideal_speedctrl; speedctrl2=0;}
		//speedctrl = speedctrl+600;
    	if(zebra_flag==0)
    	{
    		pwm_duty(ATOM2_CH0_P33_4, MID_STEER+degree);
    		//改变电机占空比
    		pwm_duty(ATOM0_CH4_P02_4, speedctrl*(1-0.95*xishu));	//右前
    		pwm_duty(ATOM0_CH7_P02_7, speedctrl*(1+0.95*xishu));	//左前
    		pwm_duty(ATOM0_CH6_P02_6, speedctrl2*(1-0.95*xishu));		//右后
    		pwm_duty(ATOM0_CH5_P02_5, speedctrl2*(1+0.95*xishu));		//左后
    	}
		//遇斑马线转弯 
    	else									
    	{
    		//pwm_duty(ATOM2_CH0_P33_4, MID_STEER - 20);	//欲左先右
    		//改变电机占空比
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
    		/*
    		pwm_duty(ATOM0_CH4_P02_4, speedctrl*(1-0.9*xishu));	//右前
    		pwm_duty(ATOM0_CH5_P02_5, speedctrl*(1+0.9*xishu));	//左前
    		pwm_duty(ATOM0_CH6_P02_6, speedctrl2R);		//右后
    		pwm_duty(ATOM0_CH7_P02_7, speedctrl2L);		//左后
			*/
    		pwm_duty(ATOM0_CH4_P02_4, 0);	//右前
    		pwm_duty(ATOM0_CH7_P02_7, 2000);	//左前
    		pwm_duty(ATOM0_CH6_P02_6, 2500);		//右后
    		pwm_duty(ATOM0_CH5_P02_5, 0);		//左后
    		//systick_delay_ms(STM1, 35);					//延时35MS  使用STM0定时器
			pwm_duty(ATOM2_CH0_P33_4, MID_STEER - 100);
			systick_delay_ms(STM1, 200);
			pwm_duty(ATOM2_CH0_P33_4, MID_STEER - 30);
			pwm_duty(ATOM0_CH4_P02_4, 200);	//右前
			pwm_duty(ATOM0_CH7_P02_7, 200);	//左前
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
    }
    	//串口发送

    	//for(int i=5;i>=0;i--){ UART_WriteByte(HW_UART3,temp[i]+48);}//data 是传感器数据
    	//UART_printf(HW_UART3,"\n");
}

