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



void searchline_image(uint8 *);
void change_image(uint8*);
int GetMeanThreshold(uint8 *);
int whiteRoad=105;

int core1_main(void)
{
	disableInterrupts();
	get_clk();//��ȡʱ��Ƶ��  ��ر���


	//Ŀǰ�Ŀ�ɼ������ ͼ�����һ��Ϊ�̶��ĺ�ɫ
	//�������ڵ�Ƭ����ɵģ���������ͷ������
    oled_init();
    oled_fill(0);
	//ips114_init();	//��ʼ��IPS��Ļ
	//ips114_showstr(0, 0, "SEEKFREE MT9V03x");
	//ips114_showstr(0, 1, "Initializing...");
	//�����Ļû���κ���ʾ��������Ļ����


	mt9v03x_init();	//��ʼ������ͷ
	//�����Ļһֱ��ʾ��ʼ����Ϣ����������ͷ����
    //���ʹ�����壬һֱ����while(!uart_receive_flag)�������Ƿ�������OK?��������ͷ�����ô����뵥Ƭ�������Ƿ���ȷ
    //���ͼ��ֻ�ɼ�һ�Σ����鳡�ź�(VSY)�Ƿ�����OK?
	enableInterrupts();
    while (TRUE)
    {

    	if(mt9v03x_finish_flag)
    	{
    		//ips114_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
    		//����ͷͨ���ж϶�ȡ��mt9v03x_image,128*64,�׵�ַΪmt9v03x_image[0]
    		//���ڻ��������ء�
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
			mt9v03x_finish_flag = 0;//��ͼ��ʹ����Ϻ�  ��������־λ�����򲻻Ὺʼ�ɼ���һ��ͼ��
			//ע�⣺һ��Ҫ��ͼ��ʹ����Ϻ�������˱�־λ
    	}
    }
}

void change_image(uint8 *p) //�����ߣ����ߺ����߷ŵ�����֮��
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
/// ���ڻҶ�ƽ��ֵ����ֵ
/// </summary>
/// <param name="p">����ͷ�ĻҶȾ���</param>
/// <returns></returns> �Ҷ���ֵ
int GetMeanThreshold(uint8 * p)	//����Ҷ���ֵ
{
	int HistGram[256];//�Ҷ�ͼ���ֱ��ͼ
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
void searchline_image(uint8 *p)	//ͼ����
{
	uint16 i;
	uint16 j;
	int	CurL= 0, Start =0;             // CurL  ?ǰ��   Start ?ʼ??����  ��һ��?0?ʼ
	uint8   Cur_Offset	= 64;     	// ��ʼ����64��Ϊ����ͷ�������ص���������
  	uint8   CurPoint = Cur_Offset;          // CurPointΪ֮ǰ����������ĵ�
	uint16 ahead_count = 0;
	j = MT9V03X_H-1;
	//����ͷǰΪ��ɫ�����µ���ɨ�迪ʼ��
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

  	//��С���˷�������(���)
  	int FitCur_A=0, FitCur_B=0, FitCur_C=0, FitCur_D=0,FitCur_E=0;
  	double FitCur_k,FitCur_b;
  	int FitCur_A1=0, FitCur_B1=0, FitCur_C1=0, FitCur_D1=0,FitCur_E1=0;
  	double FitCur_k1,FitCur_b1;
  	float SST;
  	float SSR;
  	float x_aver;
  	float r;                          //��������������С���˷��Ŷ�

  	//int i=0;
//	int UartData[6];                	//���ڷ�������
  	uint8 Lx1[MT9V03X_H];                 //ɨ��ԭ������
  	uint8 Rx1[MT9V03X_H];
	uint8 Lx2[MT9V03X_H];                 //ɨ��ԭ������
  	uint8 Rx2[MT9V03X_H];
    uint8 L_M_R[MT9V03X_H];
	/*
	 * ���Ͻǣ�0,0��
	 * ====================����ɨ�����ұ߽�===============================
	 */
	//���ע�⣡����
	//���ע�⣡����
	//���ע�⣡����
	//�������Ҷ��Ƿ���
	uint16 counter_1 = 0;                         //
	uint8 fitting_flag1 = 0;                     //�ж��費��Ҫ���
	uint8 fitting_flag2 = 0;                     //�ж��費��Ҫ���
	uint8 left_flag_count = 0;                   //�жϵ�һ��flagΪ0
	uint8 right_flag_count = 0;                  //�����һ��flagΪ0
    uint8 left_non[MT9V03X_H];               //����Ƿ�δ��⵽����,Ĭ����δ��⵽������Ϊ0����֮Ϊ1
	uint8 right_non[MT9V03X_H];              //�ұ��Ƿ�δ��⵽����,Ĭ����δ��⵽������Ϊ0����֮Ϊ1
	uint8 count_left_non=0;               //����м���δ��⵽����
	uint8 count_right_non=0;              //�ұ��м���δ��⵽����
	uint8 count_righthuandao_left_non=0;               //����м���δ��⵽����
	uint8 count_righthuandao_right_non=0;              //�ұ��м���δ��⵽����
	uint8 count_right_exist=0;              //�ұ��м��μ�⵽����
	uint8 count_left_non1=0;               //����м���δ��⵽����
	uint8 count_right_non1=0;              //�ұ��м���δ��⵽����
	uint8 count_left_non2=0;               //����м���δ��⵽����
	uint8 count_right_non2=0;              //�ұ��м���δ��⵽����
	uint8 turning_point;                   //�ɼ�������ת�۵�
	uint8 turning_point1;
	uint8 count_dec=0;                       //��Ӧ����ȴ���ٵĴ���
	uint8 count_inc=0;                       //��Ӧ����ȴ���ӵĴ���
	uint8 black_count=0;
	uint8 huandao_after=0;
	uint8 huandao_before=0;
	uint8 count_lr_non=0;

	uint16 in_huandao1;
	if (zebra_end_flag==1)		//�����ߴ�����ϣ��ҵ���
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
	//������
	for( CurL = MT9V03X_H; CurL >= Start; --CurL )
	{
		CurPoint = Cur_Offset;          //CurPoint��ÿһ�п�ʼʱΪ��һ������ */
		uint8 right_flag = 0;               //�߽��Ƿ�ɨ�赽
		uint8 left_flag = 0;

		if(CurL<MT9V03X_H-5 && CurL>28)
		{
		    CurPoint=Cur_Offset;
	        int crossing_num=0;//�����ߵ�����
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
	        while ( CurPoint > 0 )         //����ɨ��Ѱ�Ұ�����
	        {
		        if ( *(p + CurL * MT9V03X_W + CurPoint + 1) > whiteRoad&&*(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 2) < whiteRoad )
		        {
			        crossing_num=crossing_num+1;
					 -- CurPoint;
		        }  //�ɰױ�Ϊ��
		        else if ( *(p + CurL * MT9V03X_W + CurPoint + 1) < whiteRoad&&*(p + CurL * MT9V03X_W + CurPoint) > whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 1) > whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 2) > whiteRoad )
		        {
		            crossing_num=crossing_num+1;
					 -- CurPoint;
		        }  //�ɺڱ�Ϊ��
            	else
      	        {//û�ҵ�
			        -- CurPoint;
		        }
	        }
            CurPoint = Cur_Offset;
            while ( CurPoint < MT9V03X_W )         //����ɨ��Ѱ�Ұ�����
	        {
		        if ( *(p + CurL * MT9V03X_W + CurPoint - 1) > whiteRoad&&*(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 2) < whiteRoad )
		        {
			        crossing_num=crossing_num+1;
					++ CurPoint;
		        }  //�ɰױ�Ϊ��
		        else if ( *(p + CurL * MT9V03X_W + CurPoint - 1) < whiteRoad&&*(p + CurL * MT9V03X_W + CurPoint) > whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 1) > whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 2) > whiteRoad )
		        {
			        crossing_num=crossing_num+1;
					++ CurPoint;
		        }  //�ɺڱ�Ϊ��
      	        else
      	        {//û�ҵ�
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

        while ( CurPoint > 0 )         //ɨ��ԭʼ�ұ߽�
    	{
    		if ( *(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 2) < whiteRoad )      // �ҵ���߽�  ���ҽ���ȥ��
    		{
    			Rx[CurL] = CurPoint;
    			Rx1[CurL] = Rx[CurL];
    	        right_flag = 1;
    			right_non[CurL]=1;
    			break;
    		}
    		else
    		{//û�ҵ�
    			right_non[CurL]=0;
    			-- CurPoint;
    			right_flag = 0;
    			right_flag_count = right_flag_count + 1;
    		}
    	}

    	CurPoint = Cur_Offset;
    	while ( CurPoint < MT9V03X_W )     //ɨ��ԭʼ��߽�
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
    		{//û�ҵ�
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

    		//�������ұ��ߣ��ҵ���ֱ��������
    		//���κ�����£����ǵ���ͷ������Ӧ���������ڣ���ˣ�
    		//ɨ��������ͷ����һ�п�ʼ��������ʼ��Ϊ��������
    		//ע�⣺���ڿ��ǵ�����Ӧ���������ģ�ɨ�����ʱ���Ӧ�ô���һ�ε����߿�ʼ
    		//�������Ա�����Ϊ����ϴ�����أ�ÿ�δ��е㿪ʼ���Ҳ������ߡ�

   
    	Midx[CurL]	= (Lx[CurL] + Rx[CurL]) >> 1;
    	Cur_Offset= Midx[CurL];
    }//	endfor

	if(in_right_huandao==1 && timecounter>50 && out_right_begin==1)  //�ҳ�����
	{
		gpio_init(P20_8, GPO, 1, PUSHPULL);//����P20_8Ϊ��� Ĭ������͵�ƽ  PUSHPULL���������
		gpio_init(P20_9, GPO, 0, PUSHPULL);
		gpio_init(P21_4, GPO, 0, PUSHPULL);
		gpio_init(P21_5, GPO, 0, PUSHPULL);
		count_righthuandao_right_non=0;
		count_righthuandao_left_non=0;
		for(int t=10; t<50; t++)	//��������������
		{
			if(right_non[t]==0)
			    count_righthuandao_right_non++;
			if(left_non[t]==0)
			    count_righthuandao_left_non++;
			//if(right_non[t]==1 && t>MT9V03X_H-32)
			//  count_right_exist++;
		}
    	if(count_righthuandao_left_non>=27 && count_righthuandao_right_non>=27&&(count_righthuandao_left_non-count_righthuandao_right_non<10||count_righthuandao_left_non-count_righthuandao_right_non>-10))//����Ƿ��������������������û�б��ߵ�����
		{
			black_count=0;
			for(i=20;i<100;i++)     //�ҵ���
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
			if(black_count>74&&timecounter>10)	//��Ҫ�����ҵ����ߵ�����
			{
				time_count_flag=0;
				out_right_huandao=1;
				out_right_begin=0;//���˻���֮��Ͳ�Ҫ���ж��Ƿ������
				gpio_init(P20_8, GPO, 1, PUSHPULL);//����P20_8Ϊ��� Ĭ������͵�ƽ  PUSHPULL���������
				gpio_init(P20_9, GPO, 1, PUSHPULL);
				gpio_init(P21_4, GPO, 0, PUSHPULL);
				gpio_init(P21_5, GPO, 0, PUSHPULL);
			}
		}
	}
	if(in_huandao==1 && timecounter>50 && out_begin==1)  //�������
	{
		count_right_non=0;
		count_left_non=0;
		for(int t=10; t<50; t++)	//��������������
		{
			if(right_non[t]==0)
			    count_right_non++;
			if(left_non[t]==0)
			    count_left_non++;
		}
		black_count=0;
    	if(count_left_non>=27 && count_right_non>=27&&(count_left_non-count_right_non<5||count_left_non-count_right_non>-5))//����Ƿ��������������������û�б��ߵ�����
		{
			for(i=20;i<100;i++)     //�ҵ���
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
			if(black_count>74&&timecounter>10)	//��Ҫ�����ҵ����ߵ�����
			{
				time_count_flag=0;
				out_huandao=1;
				out_begin=0;//���˻���֮��Ͳ�Ҫ���ж��Ƿ������
			}
		}
	}
	//�������ж�
    if(in_huandao==0)
    {   
		for(CurL = MT9V03X_H-1; CurL >= Start; --CurL)
	   {
		   Rx2[CurL]=Rx[CurL];
		   Lx2[CurL]=Lx[CurL];
	   }
	   //ǰ����·
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

		for( CurL = Start; CurL <MT9V03X_H; ++CurL)//����������ߵ�һ��û�к���
        {
			if(Rx[CurL]==0)
			{
				turning_point1=CurL;
				break;
			}
		}
		count_right_non=0;
		//�����û�к�������ɨ����û�к��߸���
		for(CurL = turning_point1; CurL <=MT9V03X_H-1; ++CurL)//����0
        {
			if(Rx[CurL]==0)
				count_right_non++;
		}
		//�������µ���ʼû�к��ߵĵ�ĸ���
		count_right_non2=0;
        for( CurL = Start; CurL <turning_point1 ; ++CurL)//����10
        {
			if(Rx[CurL]>=20)
				count_right_non2++;  
		}
        count_left_non=0;
		//�ұ�����
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

	//���һ����ж�
    if(in_right_huandao==0)
    {  
		for(CurL = MT9V03X_H-1; CurL >= Start; --CurL)
	    {
		   Rx2[CurL]=Rx[CurL];
		   Lx2[CurL]=Lx[CurL];
	    }
	   //ǰ����·
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
		for( CurL = Start; CurL <MT9V03X_H; ++CurL)//���������ұߵ�һ��û�к���
        {
			if(Lx[CurL]==MT9V03X_W-1)
			{
				turning_point1=CurL;
				break;
			}
		}
		count_right_non=0;
		//���ұ�û�к�������ɨ����û�к��߸���
		for(CurL = turning_point1; CurL <=MT9V03X_H-1; ++CurL)//����0
        {
			if(Lx[CurL]==MT9V03X_W-1)
				count_right_non++;
		}
		//�ұߴ������µ���ʼû�к��ߵĵ�ĸ���
		count_right_non2=0;
        for( CurL = Start; CurL < turning_point1; ++CurL)//����10
        {
			if(Lx[CurL]<=MT9V03X_W-21)
				count_right_non2++;
		}
        count_left_non=0;
		//�������
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
	//ʮ��·��
	count_lr_non=0;
    for(CurL = 20; CurL <= 45; ++CurL )
	{
		int right_flag;
		int left_flag;
		CurPoint = Cur_Offset;
        while ( CurPoint > 0 )         //ɨ��ԭʼ�ұ߽�
    	{
    		if ( *(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 2) < whiteRoad )      // �ҵ���߽�  ���ҽ���ȥ��
    		{
    	        right_flag = 1;
    			break;
    		}
    		else
    		{//û�ҵ�
    			-- CurPoint;
    			right_flag = 0;
    		}
    	}
    	CurPoint = Cur_Offset;
    	while ( CurPoint < MT9V03X_W )     //ɨ��ԭʼ��߽�
    	{
    		if ( *(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 2) < whiteRoad )
    		{
    			left_flag=1;
    			break;
    		}
    		else
    		{//û�ҵ�
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
			    while ( CurPoint > 0 )         //ɨ��ԭʼ�ұ߽�
    			{
    				if ( *(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint - 2) < whiteRoad )      // �ҵ���߽�  ���ҽ���ȥ��
    				{
    					Rx[CurL] = CurPoint;
    			        right_flag = 1;
    					break;
    				}
    		  		else
    		  		{//û�ҵ�
    		  			-- CurPoint;
    		  			right_flag = 0;
    				}
    			}
    			CurPoint = Cur_Offset;

    			while ( CurPoint < MT9V03X_W )     //ɨ��ԭʼ��߽�
    			{
    				if ( *(p + CurL * MT9V03X_W + CurPoint) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 1) < whiteRoad && *(p + CurL * MT9V03X_W + CurPoint + 2) < whiteRoad )
    				{
    					Lx[CurL] = CurPoint;
    					left_flag=1;
    					break;
    				}
    		   		else
    		  		{//û�ҵ�
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


