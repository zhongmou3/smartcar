
#ifndef CONTROL_H_
#define CONTROL_H_

extern uint8 Lx[MT9V03X_H];                    //�����������ĵ��к�
extern uint8 Rx[MT9V03X_H];                    //�����������ĵ��к�
extern uint8 Midx[MT9V03X_H];
extern int16 speed;
extern uint8 huandao;
extern uint8 zebra_flag;
extern uint8 zebra_end_flag;
extern uint8 stop_flag;
extern uint8 out_huandao;
extern uint8 in_huandao;
extern uint8 in_huandao_end;
extern uint8 huandao_count;
extern uint8 out_begin;    //�Ƿ������
extern float kp,ki,kd;     //����ʽPID����
extern float out_increment;//����ʽPID�������
extern float out;          //�����
extern int cardegree;
//extern uint8 start_prepare_flag;
//extern uint8 start_flag;
//extern uint8 start_count;
//extern uint8 startpre_count;


#define MID_STEER 705   //�������ʱ��Ӧ��PWMֵ
//#define MAX_STEER 770
//#define MIN_STEER 650
#define H 0.2			//���ֲ���ϵ��ֵ
#define B 0.155			//���ֲ���ϵ��ֵ
#define car_speed 1000  //����ռ�ձ�

int degree_calculation(void);   //�Ƕȼ���
int speedctrl_calculation(int); //�ٶȷּ�
void rotate(int);               //�������
#endif
