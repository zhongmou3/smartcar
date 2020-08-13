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
int GetMeanThreshold(uint8 * p)
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
void searchline_image(uint8 *p)
{
	int	CurL= 0, Start =0;             // CurL  ?ǰ��   Start ?ʼ??����  ��һ��?0?ʼ
	uint8   Cur_Offset	= 64;     	// ��ʼ����64��Ϊ����ͷ�������ص���������
  	uint8   CurPoint = Cur_Offset;          // CurPointΪ֮ǰ����������ĵ�

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
	uint16 counter_1 = 0;                         //
	uint8 fitting_flag1 = 0;                     //�ж��費��Ҫ���
	uint8 fitting_flag2 = 0;                     //�ж��費��Ҫ���
	uint8 left_flag_count = 0;                   //�жϵ�һ��flagΪ0
	uint8 right_flag_count = 0;                  //�����һ��flagΪ0
	//int zebra_flag=0;                           //���������ߵ�ֹͣflag,Ϊ1ֹͣ
    uint8 left_non[MT9V03X_H];               //����Ƿ�δ��⵽����,Ĭ����δ��⵽������Ϊ0����֮Ϊ1
	uint8 right_non[MT9V03X_H];              //�ұ��Ƿ�δ��⵽����,Ĭ����δ��⵽������Ϊ0����֮Ϊ1
	uint8 count_left_non=0;               //����м���δ��⵽����
	uint8 count_right_non=0;              //�ұ��м���δ��⵽����
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
	uint16 i;
	uint16 j;
	uint16 in_huandao1;
	if (zebra_end_flag==1)		//�����ߴ�����ϣ��ҵ���
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

   /* 	while ( CurPoint > 0 )         //ɨ��ԭʼ�ұ߽�
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

    	if(CurL < MT9V03X_H - 2)                //����б�ʹ���ƽ̹�ĵ�
    	{
    	  	if(left_flag==1 && (2*Lx[CurL]-Lx1[CurL+1]-Lx1[CurL+2]>50 || 2*Lx[CurL]-Lx1[CurL+1]-Lx1[CurL+2]<-50)) //����߽��ҵ���������һ�������߼�����ƽ��
    	  	  	Lx[CurL]=Lx[CurL+1];
    	  	if(right_flag==1 && (2*Rx[CurL]-Rx1[CurL+1]-Rx1[CurL+2]>50 || 2*Rx[CurL]-Rx1[CurL+1]-Rx1[CurL+2]<-50))
    	  	  	Rx[CurL]=Rx[CurL+1];                             //�����ҵ��ı��ߵĵ㣬ȡ��һ����
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
    	if(left_flag==0&&CurL<MT9V03X_H-8&&fitting_flag1==0)          //�����������
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
    	}                                      //���ݵ�һ��leftflagΪ0ʱ��ǰ�˸������
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
    	if(right_flag==0&&CurL<MT9V03X_H-8&&fitting_flag2==0)          //�����������
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
    	}                                      //���ݵ�һ��leftflagΪ0ʱ��ǰ�˸������
    	if(right_flag==0 && CurL<MT9V03X_H-8 && fitting_flag2==1)
    	{
    	  	Rx[CurL]=Rx[CurL+1];
    	  	Rx1[CurL]=Rx[CurL];
    	}
	    */
    	Midx[CurL]	= (Lx[CurL] + Rx[CurL]) >> 1;
    	Cur_Offset= Midx[CurL];
    }//	endfor
	for(int t=10; t<40; t++)	//��������������
	{
		if(right_non[t]==0)
		    count_right_non++;
		if(left_non[t]==0)
		    count_left_non++;
		//if(right_non[t]==1 && t>MT9V03X_H-32)
		//  count_right_exist++;
	}
    if(count_left_non>=25 && count_right_non>=25&&out_begin==1&&(count_left_non-count_right_non<2||count_left_non-count_right_non>-2))//����Ƿ��������������������û�б��ߵ�����
	{
		for(i=40;i<90;i++)     //�ҵ���
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
		if(black_count>40&&timecounter>30)	//��Ҫ�����ҵ����ߵ�����
		{
			out_huandao=1;
			out_begin=0;//���˻���֮��Ͳ�Ҫ���ж��Ƿ������
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

		for( CurL = Start; CurL <MT9V03X_H ; ++CurL)//ת�۵�
        {
			if(Rx[CurL]==0)
			{
				turning_point1=CurL;
				break;
			}
		}
		count_right_non=0;
		for( CurL = turning_point1; CurL <=MT9V03X_H-1; ++CurL)//����0
        {
			if(Rx[CurL]==0)
				count_right_non++;
		}
        for( CurL = Start; CurL <turning_point1 ; ++CurL)//����10
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
			        CurPoint = Cur_Offset;          //CurPoint��ÿһ�п�ʼʱΪ��һ������
					uint8 right_flag = 0;               //�߽��Ƿ�ɨ�赽
					uint8 left_flag = 0;
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
		for( CurL = 40; CurL >=10 ; --CurL)//ת�۵�
        {
			if(CurL<MT9V03X_H&&L_M_R[CurL]-L_M_R[CurL-1]<0&&L_M_R[CurL+1]-L_M_R[CurL]>0)
				turning_point=CurL;
		}
		for( CurL = MT9V03X_H; CurL > Start; --CurL) //�ж���ת�۵��ǰ��仯�����Ƿ���Ͻ��뻷���ı仯���ƣ����ұ��߼�ȥ������ȼ�������
        {
			if(L_M_R[CurL]-L_M_R[CurL-1]<0&&CurL>turning_point)
				count_dec++;
			if(L_M_R[CurL]-L_M_R[CurL-1]>0&&CurL<=turning_point)
			    count_inc++;
		}
		count_right_non=0;
		count_left_non=0;
	    for(int t=0; t<=20; t++)	//�뻷����������
	    {
			if(right_non[t]==0)
			    count_right_non1++;
			if(left_non[t]==0)
			    count_left_non1++;
	    }
	    for(int t=20; t<=40; t++)	//�뻷����������
	    {
	    	if(right_non[t]==0)
	    		count_right_non2++;
	    	if(left_non[t]==0)
	    		count_left_non2++;
	    }
		if(count_dec<=3&&count_inc<=3&&r>0.9&&count_right_non1>=7&&count_left_non1>=7&&count_right_non2>=7&&count_left_non2>=7)
			in_huandao=1;
   */
			//���ڼ��������
    /*	  	int data1=r*100;
    	  	for (i=0;i<6;i++)
    	  	{
    	  	  	UartData[i]=data1%10;
    	  	  	data1=(data1-UartData[i])/10;
    	  	}
    	  	for(int i=5;i>=0;i--){ UART_WriteByte(HW_UART3,UartData[i]+48);}    //�������ݷ���
    	  	UART_printf(HW_UART3,"\n");
 */
}


