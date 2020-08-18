
#ifndef CONTROL_H_
#define CONTROL_H_

extern uint8 Lx[MT9V03X_H];                    //�����������ĵ��к�
extern uint8 Rx[MT9V03X_H];                    //�����������ĵ��к�
extern uint8 Midx[MT9V03X_H];
extern int16 speed;
extern uint8 time_count_flag;
extern uint32 timecounter;
extern uint8 huandao;
extern uint8 zebra_flag;
extern uint8 zebra_end_flag;
extern uint8 stop_flag;
extern uint8 out_huandao;
extern uint8 in_huandao;
extern uint8 in_huandao_end;
extern uint8 huandao_count;
extern uint8 out_begin;    //�Ƿ������
extern uint8 speed_limit_flag;
extern int16 speed_limit_value;
extern double kp,ki,kd;     //����ʽPID����
extern int16 ek,ek1,ek2;   //ǰ���������
extern double out_increment;//����ʽPID�������
extern double out;          //�����
extern int cardegree;
extern uint8 left_out_flag; //��ཫ�������
extern uint8 right_out_flag;//�Ҳཫ�������
//extern uint8 start_prepare_flag;
//extern uint8 start_flag;
//extern uint8 start_count;
//extern uint8 startpre_count;


#define MID_STEER 735   //�������ʱ��Ӧ��PWMֵ
//#define MAX_STEER 770
//#define MIN_STEER 650
#define H 0.2			//���ֲ���ϵ��ֵ
#define B 0.155			//���ֲ���ϵ��ֵ
#define car_speed 1000  //����ռ�ձ�

int degree_calculation(void);   //�Ƕȼ���
int speedctrl_calculation(int); //�ٶȷּ�
void rotate(int);               //�������
double rear_diff(int);
#endif
