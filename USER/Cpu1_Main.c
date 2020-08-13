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
int GetMeanThreshold(uint8 * p)
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
void searchline_image(uint8 *p)
{
	int	CurL= 0, Start =0;             // CurL  ?前行   Start ?始??的行  第一行?0?始
	uint8   Cur_Offset	= 64;     	// 初始中心64，为摄像头所有像素点中心中心
  	uint8   CurPoint = Cur_Offset;          // CurPoint为之前正在描的中心点

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
	 * 右上角（0,0）
	 * ====================按行扫描左右边界===============================
	 */
	uint16 counter_1 = 0;                         //
	uint8 fitting_flag1 = 0;                     //判断需不需要拟合
	uint8 fitting_flag2 = 0;                     //判断需不需要拟合
	uint8 left_flag_count = 0;                   //判断第一次flag为0
	uint8 right_flag_count = 0;                  //计算第一个flag为0
	//int zebra_flag=0;                           //遇到斑马线的停止flag,为1停止
    uint8 left_non[MT9V03X_H];               //左边是否未检测到边线,默认有未检测到边线则为0，反之为1
	uint8 right_non[MT9V03X_H];              //右边是否未检测到边线,默认有未检测到边线则为0，反之为1
	uint8 count_left_non=0;               //左边有几次未检测到边线
	uint8 count_right_non=0;              //右边有几次未检测到边线
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
	uint16 i;
	uint16 j;
	uint16 in_huandao1;
	if (zebra_end_flag==1)		//斑马线处理完毕，找底线
	  	{
			uint8 stop_count=0;
	  		//uint8 i;
	  		//uint8 j;
	  		for(i=40;i<90;i++)
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
	  		if (stop_count>40)
	  			stop_flag = 1;
			else
	  		{
	  			stop_count = 0;
	  		}
	  	}
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

   /* 	while ( CurPoint > 0 )         //扫描原始右边界
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

    	if(CurL < MT9V03X_H - 2)                //舍弃斜率过于平坦的点
    	{
    	  	if(left_flag==1 && (2*Lx[CurL]-Lx1[CurL+1]-Lx1[CurL+2]>50 || 2*Lx[CurL]-Lx1[CurL+1]-Lx1[CurL+2]<-50)) //如果边界找到，但与上一个点连线几乎是平的
    	  	  	Lx[CurL]=Lx[CurL+1];
    	  	if(right_flag==1 && (2*Rx[CurL]-Rx1[CurL+1]-Rx1[CurL+2]>50 || 2*Rx[CurL]-Rx1[CurL+1]-Rx1[CurL+2]<-50))
    	  	  	Rx[CurL]=Rx[CurL+1];                             //放弃找到的边线的点，取上一个点
    	}
    	if(left_flag==0 && CurL==MT9V03X_H-1)
    	{
    	  	Lx[CurL]=MT9V03X_W;
    	  	Lx1[CurL]=Lx[CurL];
    	  	fitting_flag1=1;
    	}
    	if(left_flag==0&&CurL>=MT9V03X_H-8&&CurL<MT9V03X_H-1)
    	{
    	  	Lx[CurL]=Lx[CurL+1];                                  //
    	  	Lx1[CurL]=Lx[CurL];
    	  	fitting_flag1=1;
    	}
    	if(left_flag==0&&CurL<MT9V03X_H-8&&fitting_flag1==0)          //符合拟合条件
    	{
    	    FitCur_A=0;
    	    FitCur_B=0;
    	    FitCur_C=0;
    	    FitCur_D=0;
    	    for(i=MT9V03X_H-1;i>=MT9V03X_H-8;i--)
    	    {
    	      FitCur_A=FitCur_A+i;                        //sum yi
    	      FitCur_B=FitCur_B+Lx[i];                    //sum xi
    	      FitCur_C=FitCur_C+i*Lx[i];                  //sum xiyi
    	      FitCur_D=FitCur_D+Lx[i]*Lx[i];              //sum xi^2
    	    }
    	    FitCur_k=(double)(8*FitCur_C-FitCur_A*FitCur_B)/(8*FitCur_D-FitCur_B*FitCur_B);
    	    FitCur_b=(double)(FitCur_A-FitCur_k*FitCur_B)/8;
    	    Lx[CurL]=(CurL-FitCur_b)/FitCur_k;
    	    Lx1[CurL]=Lx[CurL];
    	}                                      //根据第一次leftflag为0时的前八个点拟合
    	if(left_flag==0&&CurL<MT9V03X_H-8&&fitting_flag1==1)
    	{
    	  	Lx[CurL]=Lx[CurL+1];
    	  	Lx1[CurL]=Lx[CurL];
    	}                                             //
    	if(right_flag==0&&CurL>=MT9V03X_H-1)
    	{
    	  	Rx[CurL]=0;
    	  	Rx1[CurL]=Rx[CurL];
    	  	fitting_flag2=1;
    	}
    	if(right_flag==0&&CurL>=MT9V03X_H-8&&CurL<MT9V03X_H-1)
    	{
    	  	Rx[CurL]=Rx[CurL+1];
    	  	Rx1[CurL]=Rx[CurL];
    	  	fitting_flag2=1;
    	}
    	if(right_flag==0&&CurL<MT9V03X_H-8&&fitting_flag2==0)          //符合拟合条件
    	{
    	    FitCur_A1=0;
    	    FitCur_B1=0;
    	    FitCur_C1=0;
    	    FitCur_D1=0;
    	    for(i=MT9V03X_H-1;i>=MT9V03X_H-8;i--)
    	    {
    	      	FitCur_A1=FitCur_A1+i;                        //sum yi
    	      	FitCur_B1=FitCur_B1+Rx[i];                    //sum xi
    	      	FitCur_C1=FitCur_C1+i*Rx[i];                  //sum xiyi
    	      	FitCur_D1=FitCur_D1+Rx[i]*Rx[i];              //sum xi^2
    	    }
    	    FitCur_k1=(double)(8*FitCur_C1-FitCur_A1*FitCur_B1)/(8*FitCur_D1-FitCur_B1*FitCur_B1);
    	    FitCur_b1=(double)(FitCur_A1-FitCur_k1*FitCur_B1)/8;
    	    Rx[CurL]=(CurL-FitCur_b1)/FitCur_k1;
    	    Rx1[CurL]=Rx[CurL];
    	}                                      //根据第一次leftflag为0时的前八个点拟合
    	if(right_flag==0 && CurL<MT9V03X_H-8 && fitting_flag2==1)
    	{
    	  	Rx[CurL]=Rx[CurL+1];
    	  	Rx1[CurL]=Rx[CurL];
    	}
	    */
    	Midx[CurL]	= (Lx[CurL] + Rx[CurL]) >> 1;
    	Cur_Offset= Midx[CurL];
    }//	endfor
	for(int t=10; t<40; t++)	//出环岛边线条件
	{
		if(right_non[t]==0)
		    count_right_non++;
		if(left_non[t]==0)
		    count_left_non++;
		//if(right_non[t]==1 && t>MT9V03X_H-32)
		//  count_right_exist++;
	}
    if(count_left_non>=25 && count_right_non>=25&&out_begin==1&&(count_left_non-count_right_non<2||count_left_non-count_right_non>-2))//检测是否出环岛，首先满足左右没有边线的条件
	{
		for(i=40;i<90;i++)     //找底线
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
		if(black_count>40&&timecounter>30)	//还要满足找到底线的条件
		{
			out_huandao=1;
			out_begin=0;//出了环岛之后就不要再判断是否出环岛
		}
	}
    if(in_huandao==0)
    {    FitCur_A=0;
    	FitCur_B=0;
    	FitCur_C=0;
    	FitCur_D=0;
		FitCur_E=0;
		int m=MT9V03X_H-20;
    	for(i=MT9V03X_H-30;i>=13;i--)
    	{
    		FitCur_A=FitCur_A+i;                        //sum yi
    		FitCur_B=FitCur_B+Lx[i];                    //sum xi
    		FitCur_C=FitCur_C+i*Lx[i];                  //sum xiyi
    		FitCur_D=FitCur_D+Lx[i]*Lx[i];              //sum xi^2
    		FitCur_E=FitCur_E+i*i;                      //sum yi^2
    	}
		r=(double)(FitCur_C-FitCur_A*FitCur_B/m)/sqrt((FitCur_D-FitCur_B*FitCur_B/m)*(FitCur_E-FitCur_A*FitCur_A/m));

		for( CurL = MT9V03X_H-1; CurL >= Start; --CurL )
	   {
		   Rx2[CurL]=Rx[CurL];
		   Lx2[CurL]=Lx[CurL];
	   }
		uint8 stop_count=0;
		for(i=40;i<90;i++)
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

		for( CurL = Start; CurL <MT9V03X_H ; ++CurL)//转折点
        {
			if(Rx[CurL]==0)
			{
				turning_point1=CurL;
				break;
			}
		}
		count_right_non=0;
		for( CurL = turning_point1; CurL <=MT9V03X_H-1; ++CurL)//都是0
        {
			if(Rx[CurL]==0)
				count_right_non++;
		}
        for( CurL = Start; CurL <turning_point1 ; ++CurL)//大于10
        {
			if(Rx[CurL]>=15)
				count_right_non2++;
		}
        count_left_non=0;
        for( CurL = 10; CurL <=35 ; ++CurL)
        {
        	if(Lx[CurL]<MT9V03X_W-2)
        		count_left_non++;
        }
		if(count_right_non>=MT9V03X_H-turning_point1-1&&count_right_non2>=turning_point1-1&&count_right_non2>=8&&count_right_non>=8&&stop_count<40&&count_left_non>=22)
        {
			in_huandao=1;
			out_begin=1;
			timecounter=0;
        }
	/*	for( CurL =Start ; CurL<MT9V03X_H; ++CurL )
		{
			        CurPoint = Cur_Offset;          //CurPoint在每一行开始时为上一行中线
					uint8 right_flag = 0;               //边界是否扫描到
					uint8 left_flag = 0;
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
						Rx[CurL] = Rx[CurL-1];
					}
					if(left_flag==0)
					{
						Lx[CurL]=Lx[CurL-1];
					}
					Midx[CurL]	= (Lx[CurL] + Rx[CurL]) >> 1;
				    Cur_Offset= Midx[CurL];
		}*/

			/*			huandao_after=1;
		else
			huandao_after=0;
		if(huandao_before==0&&huandao_after==1)
			huandao_count++;
		//printf("huandao_count: %d\n", huandao_count);
		int huandao_fucker=huandao_count;
		huandao_before=huandao_after;
		if(huandao_count==2)
			in_huandao=1;
			*/
    }
	//	in_huandao1= in_huandao;
  /*    for( CurL = MT9V03X_H; CurL >= Start; --CurL )
		{
			L_M_R[CurL]=Lx[CurL]-Rx[CurL];
		}
		for( CurL = 40; CurL >=10 ; --CurL)//转折点
        {
			if(CurL<MT9V03X_H&&L_M_R[CurL]-L_M_R[CurL-1]<0&&L_M_R[CurL+1]-L_M_R[CurL]>0)
				turning_point=CurL;
		}
		for( CurL = MT9V03X_H; CurL > Start; --CurL) //判断在转折点的前后变化趋势是否符合进入环岛的变化趋势，即右边线减去左边线先减后增加
        {
			if(L_M_R[CurL]-L_M_R[CurL-1]<0&&CurL>turning_point)
				count_dec++;
			if(L_M_R[CurL]-L_M_R[CurL-1]>0&&CurL<=turning_point)
			    count_inc++;
		}
		count_right_non=0;
		count_left_non=0;
	    for(int t=0; t<=20; t++)	//入环岛边线条件
	    {
			if(right_non[t]==0)
			    count_right_non1++;
			if(left_non[t]==0)
			    count_left_non1++;
	    }
	    for(int t=20; t<=40; t++)	//入环岛边线条件
	    {
	    	if(right_non[t]==0)
	    		count_right_non2++;
	    	if(left_non[t]==0)
	    		count_left_non2++;
	    }
		if(count_dec<=3&&count_inc<=3&&r>0.9&&count_right_non1>=7&&count_left_non1>=7&&count_right_non2>=7&&count_left_non2>=7)
			in_huandao=1;
   */
			//串口检查拟合误差
    /*	  	int data1=r*100;
    	  	for (i=0;i<6;i++)
    	  	{
    	  	  	UartData[i]=data1%10;
    	  	  	data1=(data1-UartData[i])/10;
    	  	}
    	  	for(int i=5;i>=0;i--){ UART_WriteByte(HW_UART3,UartData[i]+48);}    //串口数据发送
    	  	UART_printf(HW_UART3,"\n");
 */
}


