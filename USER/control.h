
#ifndef CONTROL_H_
#define CONTROL_H_

extern uint8 Lx[MT9V03X_H];                    //左引导线中心点列号
extern uint8 Rx[MT9V03X_H];                    //右引导线中心点列号
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
extern uint8 out_begin;    //是否出环岛
extern float kp,ki,kd;     //增量式PID参数
extern float out_increment;//增量式PID输出增量
extern float out;          //输出量
extern int cardegree;
//extern uint8 start_prepare_flag;
//extern uint8 start_flag;
//extern uint8 start_count;
//extern uint8 startpre_count;


#define MID_STEER 705   //舵机居中时对应的PWM值
//#define MAX_STEER 770
//#define MIN_STEER 650
#define H 0.2			//后轮差速系数值
#define B 0.155			//后轮差速系数值
#define car_speed 1000  //启动占空比

int degree_calculation(void);   //角度计算
int speedctrl_calculation(int); //速度分级
void rotate(int);               //整体控制
#endif
