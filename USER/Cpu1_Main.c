/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		tasking v6.3r1
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/

#include "headfile.h"
#include "control.h"



void searchline_image(uint8 *);
void change_image(uint8*);
int GetMeanThreshold(uint8 *);
int whiteRoad=105;

int core1_main(void)
{
	disableInterrupts();
	get_clk();//获取时钟频率  务必保留


	//目前的库采集总钻风 图像最后一列为固定的黑色
	//这是由于单片机造成的，不是摄像头的问题
    oled_init();
    oled_fill(0);
	//ips114_init();	//初始化IPS屏幕
	//ips114_showstr(0, 0, "SEEKFREE MT9V03x");
	//ips114_showstr(0, 1, "Initializing...");
	//如果屏幕没有任何显示，请检查屏幕接线


	mt9v03x_init();	//初始化摄像头
	//如果屏幕一直显示初始化信息，请检查摄像头接线
    //如果使用主板，一直卡在while(!uart_receive_flag)，请检查是否电池连接OK?或者摄像头的配置串口与单片机连接是否正确
    //如果图像只采集一次，请检查场信号(VSY)是否连接OK?
	enableInterrupts();
    while (TRUE)
    {

    	if(mt9v03x_finish_flag)
    	{
    		//ips114_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
    		//摄像头通过中断读取到mt9v03x_image,128*64,首地址为mt9v03x_image[0]
    		//后期会提升像素。
    		//disableInterrupts();
			whiteRoad = GetMeanThreshold(mt9v03x_image[0]);
    		//printf("white: %dms\n", whiteRoad);
			searchline_image(mt9v03x_image[0]);
    		change_image(mt9v03x_image[0]);
    		oled_dis_bmp(MT9V03X_H,MT9V03X_W,mt9v03x_image[0],whiteRoad);
    		oled_int16(40, 2, speed);
    		oled_int16(40, 3, ek);
    		int16 ou16= (int16)out;
    		oled_int16(40, 4, ou16);
			mt9v03x_finish_flag = 0;//在图像使用完毕后  务必清除标志位，否则不会开始采集下一幅图像
			//注意：一定要在图像使用完毕后在清除此标志位
    	}
    }
}

void change_image(uint8 *p) //把左线，中线和右线放到数组之中
{
	int CurL=0,Start=0;
	for ( CurL = MT9V03X_H-1; CurL >=Start; --CurL )
	{
		*(p + CurL * MT9V03X_W +Rx[CurL] )=80;
		*(p + CurL * MT9V03X_W +Lx[CurL] )=80;
		*(p + CurL * MT9V03X_W +Midx[CurL])=80;
	}
}

/// <summary>
/// 基于灰度平均值的阈值
/// </summary>
/// <param name="p">摄像头的灰度矩阵</param>
/// <returns></returns> 灰度阈值
int GetMeanThreshold(uint8 * p)	//计算灰度阈值
{
	int HistGram[256];//灰度图像的直方图
	uint8 i=0;
	for (int Y = 0; Y < 256; Y++)
    {
		HistGram[Y]=0;
	}
    for(int CurL=0;CurL<MT9V03X_H;CurL++)
	{
        for(int CurPoint=0;CurPoint<MT9V03X_W;CurPoint++)
		{
			i=*(p + CurL * MT9V03X_W + CurPoint);
			HistGram[i]++;
		}
	}
    int Sum = 0, Amount = 0;
    for (int Y = 0; Y < 256; Y++)
    {
        Amount +=HistGram[Y];
        Sum += Y * HistGram[Y];
    }
    return Sum / Amount;
}
void searchline_image(uint8 *p)	//图像处理
{
	uint16 i;
	uint16 j;
	int	CurL= 0, Start =0;             // CurL  ?前行   Start ?始??的行  第一行?0?始
	uint8   Cur_Offset	= 64;     	// 初始中心64，为摄像头所有像素点中心中心
  	uint8   CurPoint = Cur_Offset;          // CurPoint为之前正在描的中心点
	uint16 ahead_count = 0;
	j = MT9V03X_H-1;
	//若车头前为黑色，重新调整扫描开始点
	for (i=Cur_Offset-3; i<=Cur_Offset+3; i++)	
	{
		if(*(p + j * MT9V03X_W + i) < whiteRoad)
			ahead_count++;
	}
	if(ahead_count>=3)
	{
		ahead_count=0;
		for (i=0; i<=20; i++)	
		{
			if(*(p + j * MT9V03X_W + i) > whiteRoad)
				ahead_count++;
		}
		if(ahead_count>15)
			Cur_Offset = 10;
		else
		{
			ahead_count=0;
			for (i =  MT9V03X_W-20; i <  MT9V03X_W; i++)	
			{
				if(*(p + j * MT9V03X_W + i) > whiteRoad)
					ahead_count++;
			}
			if(ahead_count>14)
				Cur_Offset = MT9V03X_W-10;
		}
	}

  	//最小二乘法各变量(拟合)
  	int FitCur_A=0, FitCur_B=0, FitCur_C=0, FitCur_D=0,FitCur_E=0;
  	double FitCur_k,FitCur_b;
  	int FitCur_A1=0, FitCur_B1=0, FitCur_C1=0, FitCur_D1=0,FitCur_E1=0;
  	double FitCur_k1,FitCur_b1;
  	float SST;
  	float SSR;
  	float x_aver;
  	float r;                          //（废弃变量）最小二乘法优度

  	//int i=0;
//	int UartData[6];                	//串口发送数据
  	uint8 Lx1[MT9V03X_H];                 //扫描原点数组
  	uint8 Rx1[MT9V03X_H];
	uint8 Lx2[MT9V03X_H];                 //扫描原点数组
  	uint8 Rx2[MT9V03X_H];
    uint8 L_M_R[MT9V03X_H];
	/*
	 * 左上角（0,0）
	 * ====================按行扫描左右边界===============================
	 */
	//务必注意！！！
	//务必注意！！！
	//务必注意！！！
	//所有左右都是反的
	uint16 counter_1 = 0;                         //
	uint8 fitting_flag1 = 0;                     //判断需不需要拟合
	uint8 fitting_flag2 = 0;                     //判断需不需要拟合
	uint8 left_flag_count = 0;                   //判断第一次flag为0
	uint8 right_flag_count = 0;                  //计算第一个flag为0
    uint8 left_non[MT9V03X_H];               //左边是否未检测到边线,默认有未检测到边线则为0，反之为1
	uint8 right_non[MT9V03X_H];              //右边是否未检测到边线,默认有未检测到边线则为0，反之为1
	uint8 count_left_non=0;               //左边有几次未检测到边线
	uint8 count_right_non=0;              //右边有几次未检测到边线
	uint8 count_righthuandao_left_non=0;               //左边有几次未检测到边线
	uint8 count_righthuandao_right_non=0;              //右边有几次未检测到边线
	uint8 count_right_exist=0;              //右边有几次检测到边线
	uint8 count_left_non1=0;               //左边有几次未检测到边线
	uint8 count_right_non1=0;              //右边有几次未检测到边线
	uint8 count_left_non2=0;               //左边有几次未检测到边线
	uint8 count_right_non2=0;              //右边有几次未检测到边线
	uint8 turning_point;                   //由减到增的转折点
	uint8 turning_point1;
	uint8 count_dec=0;                       //本应增加却减少的次数
	uint8 count_inc=0;                       //本应减少却增加的次数
	uint8 black_count=0;
	uint8 huandao_after=0;
	uint8 huandao_before=0;
	uint8 count_lr_non=0;

	uint16 in_huandao1;
	if (zebra_end_flag==1)		//斑马线处理完毕，找底线
	{
		uint8 stop_count=0;
		//uint8 i;
		//uint8 j;
		for(i=40; i<90; i++)
		{
			for(j=10;j<62;j++)
			{
				if(*(p + j * MT9V03X_W + i) > whiteRoad && *(p + (j+1) * MT9V03X_W + i) > whiteRoad && *(p + (j-1) * MT9V03X_W + i) < whiteRoad)
				{
					stop_count++;
					break;
				}
			}
		}
		//printf("stop_count: %d\n", stop_count);
		if (stop_count > 40)
			stop_flag = 1;
		else
		{
			stop_count = 0;
		}
	}
	//找中线
	for( CurL = MT9V03X_H; CurL >= Start; --CurL )
	{
		CurPoint = Cur_Offset;          //CurPoint在每一行开始时为上一行中线 */
		uint8 right_flag = 0;               //边界是否扫描到
		uint8 left_flag = 0;

		if(CurL<MT9V03X_H-5 && CurL>28)
		{
		    CurPoint=Cur_Offset;
	        int crossing_num=0;//斑马线的数量
	        uint8 stop_count=0;
	        for(i=40;i<90;i++)
	        {
	            for(j=62;j>=5;j--)
	        	{
	        	    if(*(p + j * MT9V03X_W + i) > whiteRoad && *(p + (j+1) * MT9V03X_W + i) > whiteRoad && *(p + (j-1) * MT9V03X_W + i) < whiteRoad)
	        		{
	        		  	stop_count++;
	        		  	break;
	        		}
	        	}
	        }
	        while ( CurPoint > 0 )         //向右扫描寻找斑马线
	        {
		        if ( *(p + CurL * MT9V03X_W + CurPoint + 1) > whiteRoad&&*(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 2) < whiteRoad )
		        {
			        crossing_num=crossing_num+1;
					 -- CurPoint;
		        }  //由白变为黑
		        else if ( *(p + CurL * MT9V03X_W + CurPoint + 1) < whiteRoad&&*(p + CurL * MT9V03X_W + CurPoint) > whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 1) > whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 2) > whiteRoad )
		        {
		            crossing_num=crossing_num+1;
					 -- CurPoint;
		        }  //由黑变为白
            	else
      	        {//没找到
			        -- CurPoint;
		        }
	        }
            CurPoint = Cur_Offset;
            while ( CurPoint < MT9V03X_W )         //向左扫描寻找斑马线
	        {
		        if ( *(p + CurL * MT9V03X_W + CurPoint - 1) > whiteRoad&&*(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 2) < whiteRoad )
		        {
			        crossing_num=crossing_num+1;
					++ CurPoint;
		        }  //由白变为黑
		        else if ( *(p + CurL * MT9V03X_W + CurPoint - 1) < whiteRoad&&*(p + CurL * MT9V03X_W + CurPoint) > whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 1) > whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 2) > whiteRoad )
		        {
			        crossing_num=crossing_num+1;
					++ CurPoint;
		        }  //由黑变为白
      	        else
      	        {//没找到
			        ++ CurPoint;
		        }
	        }
	        CurPoint = Cur_Offset;
	        if(crossing_num >= 11&&stop_count<=35)
			{
	        	zebra_flag = 1;
	        }
            else
	        {
		        crossing_num=0;
	        }
			//printf("zebra_flag: %d\n", zebra_flag);
		}

        while ( CurPoint > 0 )         //扫描原始右边界
    	{
    		if ( *(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 2) < whiteRoad )      // 找到左边界  并且进行去噪
    		{
    			Rx[CurL] = CurPoint;
    			Rx1[CurL] = Rx[CurL];
    	        right_flag = 1;
    			right_non[CurL]=1;
    			break;
    		}
    		else
    		{//没找到
    			right_non[CurL]=0;
    			-- CurPoint;
    			right_flag = 0;
    			right_flag_count = right_flag_count + 1;
    		}
    	}

    	CurPoint = Cur_Offset;
    	while ( CurPoint < MT9V03X_W )     //扫描原始左边界
    	{
    		if ( *(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 2) < whiteRoad )
    		{
    			Lx[CurL] = CurPoint;
    			//left_old=Lx[CurL];
    			Lx1[CurL]=Lx[CurL];
    			left_flag=1;
    		    left_non[CurL]=1;
    			break;
    		}
    		else
    		{//没找到
    			left_non[CurL]=0;
    			++CurPoint;
    			left_flag=0;
    			left_flag_count=left_flag_count+1;
    	  	}
    	}
			if(right_flag == 0)
			{
				Rx[CurL] = 0;
			}
			if(left_flag==0)
			{
				Lx[CurL]=MT9V03X_W-1;
			}

    		//先找左右边线，找到则直接求中线
    		//在任何情况下，考虑到车头中心总应该在赛道内，因此，
    		//扫描从最靠近车头的那一行开始，并且起始点为该行中心
    		//注意：由于考虑到中线应该是连续的，扫描边线时起点应该从上一次的中线开始
    		//这样可以避免因为弯道较大的因素，每次从中点开始找找不到中线。

   
    	Midx[CurL]	= (Lx[CurL] + Rx[CurL]) >> 1;
    	Cur_Offset= Midx[CurL];
    }//	endfor

	if(in_right_huandao==1 && timecounter>50 && out_right_begin==1)  //右出环岛
	{
		gpio_init(P20_8, GPO, 1, PUSHPULL);//设置P20_8为输出 默认输出低电平  PUSHPULL：推挽输出
		gpio_init(P20_9, GPO, 0, PUSHPULL);
		gpio_init(P21_4, GPO, 0, PUSHPULL);
		gpio_init(P21_5, GPO, 0, PUSHPULL);
		count_righthuandao_right_non=0;
		count_righthuandao_left_non=0;
		for(int t=10; t<50; t++)	//出环岛边线条件
		{
			if(right_non[t]==0)
			    count_righthuandao_right_non++;
			if(left_non[t]==0)
			    count_righthuandao_left_non++;
			//if(right_non[t]==1 && t>MT9V03X_H-32)
			//  count_right_exist++;
		}
    	if(count_righthuandao_left_non>=27 && count_righthuandao_right_non>=27&&(count_righthuandao_left_non-count_righthuandao_right_non<10||count_righthuandao_left_non-count_righthuandao_right_non>-10))//检测是否出环岛，首先满足左右没有边线的条件
		{
			black_count=0;
			for(i=20;i<100;i++)     //找底线
			{
				for(j=0;j<30;j++)
				{
					if(*(p + j * MT9V03X_W + i) > whiteRoad && *(p + (j+1) * MT9V03X_W + i) > whiteRoad && *(p + (j-1) * MT9V03X_W + i) < whiteRoad)
					{
						black_count++;
						break;
					}
				}
			}
			if(black_count>74&&timecounter>10)	//还要满足找到底线的条件
			{
				time_count_flag=0;
				out_right_huandao=1;
				out_right_begin=0;//出了环岛之后就不要再判断是否出环岛
				gpio_init(P20_8, GPO, 1, PUSHPULL);//设置P20_8为输出 默认输出低电平  PUSHPULL：推挽输出
				gpio_init(P20_9, GPO, 1, PUSHPULL);
				gpio_init(P21_4, GPO, 0, PUSHPULL);
				gpio_init(P21_5, GPO, 0, PUSHPULL);
			}
		}
	}
	if(in_huandao==1 && timecounter>50 && out_begin==1)  //左出环岛
	{
		count_right_non=0;
		count_left_non=0;
		for(int t=10; t<50; t++)	//出环岛边线条件
		{
			if(right_non[t]==0)
			    count_right_non++;
			if(left_non[t]==0)
			    count_left_non++;
		}
		black_count=0;
    	if(count_left_non>=27 && count_right_non>=27&&(count_left_non-count_right_non<5||count_left_non-count_right_non>-5))//检测是否出环岛，首先满足左右没有边线的条件
		{
			for(i=20;i<100;i++)     //找底线
			{
				for(j=2;j<30;j++)
				{
					if(*(p + j * MT9V03X_W + i) > whiteRoad && *(p + (j+1) * MT9V03X_W + i) > whiteRoad && *(p + (j-1) * MT9V03X_W + i) < whiteRoad)
					{
						black_count++;
						break;
					}
				}
			}
			if(black_count>74&&timecounter>10)	//还要满足找到底线的条件
			{
				time_count_flag=0;
				out_huandao=1;
				out_begin=0;//出了环岛之后就不要再判断是否出环岛
			}
		}
	}
	//进环岛判定
    if(in_huandao==0)
    {   
		for(CurL = MT9V03X_H-1; CurL >= Start; --CurL)
	   {
		   Rx2[CurL]=Rx[CurL];
		   Lx2[CurL]=Lx[CurL];
	   }
	   //前面有路
		uint8 noroad_ahead_count=0;
		for(i=20; i<100; i++)
	   {
		  	for(j=30; j<62; j++)
		  	{
		  		if(*(p + j * MT9V03X_W + i) > whiteRoad && *(p + (j + 1) * MT9V03X_W + i) > whiteRoad && *(p + (j-1) * MT9V03X_W + i) < whiteRoad)
		  	    {
		  			noroad_ahead_count++;
		  			break;
		  		}
			}
		}

		for( CurL = Start; CurL <MT9V03X_H; ++CurL)//从上往下左边第一个没有黑线
        {
			if(Rx[CurL]==0)
			{
				turning_point1=CurL;
				break;
			}
		}
		count_right_non=0;
		//从左边没有黑线往下扫都是没有黑线个数
		for(CurL = turning_point1; CurL <=MT9V03X_H-1; ++CurL)//都是0
        {
			if(Rx[CurL]==0)
				count_right_non++;
		}
		//从上往下到开始没有黑线的点的个数
		count_right_non2=0;
        for( CurL = Start; CurL <turning_point1 ; ++CurL)//大于10
        {
			if(Rx[CurL]>=20)
				count_right_non2++;  
		}
        count_left_non=0;
		//右边有线
        for( CurL = 0; CurL <= 35; ++CurL)
        {
        	if(Lx[CurL]<MT9V03X_W-2)
        		count_left_non++;
        }
		if(count_right_non >= MT9V03X_H-turning_point1-3 && turning_point1<=20&&count_right_non2 >= turning_point1-1 && count_right_non2>=7 && count_right_non>=22&&noroad_ahead_count<37&&count_left_non>=28)
        {
			in_huandao=1;
			out_begin=1;
			time_count_flag=1;
			timecounter=0;
        }
    }

	//进右环岛判定
    if(in_right_huandao==0)
    {  
		for(CurL = MT9V03X_H-1; CurL >= Start; --CurL)
	    {
		   Rx2[CurL]=Rx[CurL];
		   Lx2[CurL]=Lx[CurL];
	    }
	   //前面有路
		uint8 noroad_ahead_count=0;
		for(i=20; i<100; i++)
	    {
		  	for(j=30; j<62; j++)
		  	{
		  		if(*(p + j * MT9V03X_W + i) > whiteRoad && *(p + (j + 1) * MT9V03X_W + i) > whiteRoad && *(p + (j-1) * MT9V03X_W + i) < whiteRoad)
		  	    {
		  			noroad_ahead_count++;
		  			break;
		  		}
			}
		}
		turning_point1=0;
		for( CurL = Start; CurL <MT9V03X_H; ++CurL)//从上往下右边第一个没有黑线
        {
			if(Lx[CurL]==MT9V03X_W-1)
			{
				turning_point1=CurL;
				break;
			}
		}
		count_right_non=0;
		//从右边没有黑线往下扫都是没有黑线个数
		for(CurL = turning_point1; CurL <=MT9V03X_H-1; ++CurL)//都是0
        {
			if(Lx[CurL]==MT9V03X_W-1)
				count_right_non++;
		}
		//右边从上往下到开始没有黑线的点的个数
		count_right_non2=0;
        for( CurL = Start; CurL < turning_point1; ++CurL)//大于10
        {
			if(Lx[CurL]<=MT9V03X_W-21)
				count_right_non2++;
		}
        count_left_non=0;
		//左边有线
        for( CurL = 0; CurL <= 35; ++CurL)
        {
        	if(Rx[CurL]>0)
        		count_left_non++;
        }
		if(count_right_non >= MT9V03X_H-turning_point1-3 && turning_point1<=20 && count_right_non2 >= turning_point1-1 && count_right_non2>=7 && count_right_non>=22&&noroad_ahead_count<37&&count_left_non>=28)
        {
			
			in_right_huandao=1;
			out_right_begin=1;
			time_count_flag=1;
			timecounter=0;
        }
    }
	//十字路口
	count_lr_non=0;
    for(CurL = 20; CurL <= 45; ++CurL )
	{
		int right_flag;
		int left_flag;
		CurPoint = Cur_Offset;
        while ( CurPoint > 0 )         //扫描原始右边界
    	{
    		if ( *(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 2) < whiteRoad )      // 找到左边界  并且进行去噪
    		{
    	        right_flag = 1;
    			break;
    		}
    		else
    		{//没找到
    			-- CurPoint;
    			right_flag = 0;
    		}
    	}
    	CurPoint = Cur_Offset;
    	while ( CurPoint < MT9V03X_W )     //扫描原始左边界
    	{
    		if ( *(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 2) < whiteRoad )
    		{
    			left_flag=1;
    			break;
    		}
    		else
    		{//没找到
    			++CurPoint;
    			left_flag=0;
    	  	}
    	}
		if(right_flag == 0&&left_flag==0)
		{
			count_lr_non++;
		}
	}
	int count_mid_flag=0;
	if(count_lr_non>=20)
	{
		int right_flag;
		int left_flag;
		uint16 road_count=0;
		CurPoint = Cur_Offset;
		for(i=10; i<120; i++)
		{
			for(j=0;j<22;j++)
			{
				if(*(p + j * MT9V03X_W + i) < whiteRoad)
					break;
			}
			if(j==22) road_count++;
		}
		if(road_count>=35)
		{
		    for( CurL = Start; CurL <=  MT9V03X_H; ++CurL )
	    	{
			    while ( CurPoint > 0 )         //扫描原始右边界
    			{
    				if ( *(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 2) < whiteRoad )      // 找到左边界  并且进行去噪
    				{
    					Rx[CurL] = CurPoint;
    			        right_flag = 1;
    					break;
    				}
    		  		else
    		  		{//没找到
    		  			-- CurPoint;
    		  			right_flag = 0;
    				}
    			}
    			CurPoint = Cur_Offset;

    			while ( CurPoint < MT9V03X_W )     //扫描原始左边界
    			{
    				if ( *(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 2) < whiteRoad )
    				{
    					Lx[CurL] = CurPoint;
    					left_flag=1;
    					break;
    				}
    		   		else
    		  		{//没找到
    		   			++CurPoint;
    		   			left_flag=0;
    			  	}
    			}
				Midx[CurL]	= (Lx[CurL] + Rx[CurL]) >> 1;
				if((right_flag == 0||left_flag==0)&&count_mid_flag==0)
				{
					if(Lx[CurL] - Lx[CurL-1]>9||Lx[CurL] - Lx[CurL-1]<-9||Rx[CurL] - Rx[CurL-1]>9||Rx[CurL] - Rx[CurL-1]<-9)
					{
						Midx[CurL]=Midx[CurL-3];
						count_mid_flag=1;					
					}

				}
				if((right_flag == 0||left_flag==0)&&count_mid_flag==1)
				{
					Midx[CurL]=Midx[CurL-1];
				}
    		    Cur_Offset= Midx[CurL];
	    	}
		}

	}
}


