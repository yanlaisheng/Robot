/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_VARIAL_H
#define __GLOBAL_VARIAL_H

/*以下包含必要的头文件，包括数据格式文件*/
/****************Start*******************/
//#include "STM32F10x_System.h"
//#include "stm32f10x_type.h"
//#include "stm32f10x.h"
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "Macro.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>
//
//
//#include  "stm32f10x_conf.h"
//#include  <stm32f10x.h>

//#include  <stm32f10x_lib.h>
//

#define MIN(a, b) (a < b) ? (a) : (b)
#define MAX(a, b) (a > b) ? (a) : (b)
#define rt_int8_t int8_t
#define rt_int16_t int16_t
#define rt_int32_t int32_t

#define rt_uint8_t uint8_t
#define rt_uint16_t uint16_t
#define rt_uint32_t uint32_t

#define IDLE 0
#define ACCELERATING 1
#define AT_MAX 2
#define DECELERATING 3

typedef __packed struct
{
	unsigned char en;		   //使能
	unsigned char dir;		   //方向
	unsigned char running;	   //转动完成标志
	unsigned char rstflg;	   //复位标志
	unsigned char divnum;	   //分频数
	unsigned char speedenbale; //是否使能速度控制
	unsigned char clockwise;   //顺时针方向对应的值
	unsigned char id;		   //电机id

	uint32_t pulsecount;
	uint16_t *Counter_Table;		  //指向启动时，时间基数计数表
	uint16_t *Step_Table;			  //指向启动时，每个频率脉冲个数表
	uint16_t CurrentIndex;			  //当前表的位置
	uint16_t TargetIndex;			  //目标速度在表中位置
	uint16_t StartTableLength;		  //启动数据表
	uint16_t StopTableLength;		  //启动数据表
	uint32_t StartSteps;			  //电机启动步数
	uint32_t StopSteps;				  //电机停止步数
	uint32_t RevetDot;				  //电机运动的减速点
	uint32_t PulsesGiven;			  //电机运动的总步数
	uint32_t PulsesHaven;			  //电机已经运行的步数
	uint32_t CurrentPosition;		  //当前位置
	uint32_t MaxPosition;			  //最大位置，超过该位置置0
	uint32_t CurrentPosition_Pulse;	  //当前位置
	uint32_t MaxPosition_Pulse;		  //当前位置
	unsigned long long Time_Cost_Act; //实际运转花费的时间
	unsigned long long Time_Cost_Cal; //计算预估运转花费的时间
	TIM_TypeDef *TIMx;
} MOTOR_CONTROL_S;

typedef __packed struct
{
	unsigned char en;		   //使能
	unsigned char dir;		   //方向
	unsigned char running;	   //转动完成标志
	unsigned char rstflg;	   //复位标志，为1时，限位开关强停。
	unsigned char divnum;	   //分频数
	unsigned char speedenbale; //是否使能速度控制
	unsigned char clockwise;   //顺时针方向对应的值
	unsigned char id;		   //电机id

	uint32_t PulsesGiven; //电机运动的总步数
	uint32_t PulsesHaven; //电机已经运行的步数
	uint32_t step_move;	  //total move requested
	uint32_t step_spmax;  //maximum speed
	uint32_t step_accel;  //accel/decel rate, 8.8 bit format
	uint32_t step_acced;  //steps in acceled stage

	uint32_t step_middle; //mid-point of move, = (step_move - 1) >> 1
	uint32_t step_count;  //step counter	   当前步数
	uint32_t step_frac;	  //step counter fraction	   当前步数分数
	uint32_t step_speed;  //current speed, 16.8 bit format (HI byte always 0)	 当前速度
	uint32_t speed_frac;  //speed counter fraction	 当前速度分数
	uint8_t step_state;	  //move profile state
	uint8_t step_dur;	  //counter for duration of step pulse HI

	uint32_t CurrentPosition;		//当前位置
	uint32_t MaxPosition;			//最大位置，超过该位置置0
	uint32_t CurrentPosition_Pulse; //当前位置
	uint32_t MaxPosition_Pulse;		//当前位置

	TIM_TypeDef *TIMx;
	GPIO_TypeDef *GPIOBASE;
	int32_t PWMGPIO;
} MOTOR_CONTROL_SPTA;

#define M1_CLOCKWISE 0
#define M1_UNCLOCKWISE 1
#define M2_CLOCKWISE 0
#define M2_UNCLOCKWISE 1
#define M3_CLOCKWISE 0
#define M3_UNCLOCKWISE 1
#define M4_CLOCKWISE 0
#define M4_UNCLOCKWISE 1
#define M5_CLOCKWISE 0
#define M5_UNCLOCKWISE 1
#define M6_CLOCKWISE 0
#define M6_UNCLOCKWISE 1

#define PWM1_PreemptionPriority 1 //阶级
#define PWM1_SubPriority 0		  //阶层
#define PWM2_PreemptionPriority 1 //阶级
#define PWM2_SubPriority 1		  //阶层
#define PWM3_PreemptionPriority 2 //阶级
#define PWM3_SubPriority 0		  //阶层
#define PWM4_PreemptionPriority 2 //阶级
#define PWM4_SubPriority 1		  //阶层
/***********************END********************************************/

//电机
extern MOTOR_CONTROL_S motor1;
extern MOTOR_CONTROL_S motor2;
extern MOTOR_CONTROL_S motor3;
extern MOTOR_CONTROL_S motor4;
extern MOTOR_CONTROL_S motor5;
extern MOTOR_CONTROL_S motor6;
// extern MOTOR_CONTROL_SPTA motor4;

extern volatile uint8_t USART1_TxBuffer[USART1TXSIZE]; //串口1发送缓冲区
extern volatile uint16_t PTxBufferUSART11;			   //串口1发送前向位置
extern volatile uint16_t PTxBufferUSART12;			   //串口1发送后向位置，后向-前向=未发送的数据
extern volatile uint16_t USART1_TxCounter;			   //串口1发送计数
extern volatile uint16_t USART1_RxCounter;			   //串口1接收计数
extern volatile uint16_t USART1_NbrOfDataToTransfer;   //串口1要发送的数据个数
extern volatile uint8_t USART1_RxBuffer[USART1RXSIZE]; //串口1接收缓冲区
extern volatile uint16_t PRxBufferUSART11;
extern volatile uint16_t PRxBufferUSART12;
extern volatile uint8_t USART1_NbrOfDataReceived; //串口1要接收的数据个数
extern volatile uint8_t USART1_Received_Flag;

extern uint16_t Motor1TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor1StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor2TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor2StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor3TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
extern uint16_t Motor3StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1];
//extern struct rt_ringbuffer rb_recv;

void USART1_Initial(void);
void USART1_Printf(unsigned char *q, unsigned char len);
void USART1_Printfstr(unsigned char *p);
void Initial_MotorIO(void);
void Initial_Motor(unsigned char MotorID, unsigned char StepDive, unsigned int maxposition);
void MotorRunParaInitial(void);
void Start_Motor12(unsigned char dir1, unsigned int Degree1, unsigned char dir2, unsigned int Degree2);
void Start_Motor_S(unsigned char MotorID, unsigned char dir, uint32_t Degree, uint32_t MaxSpeed_S, uint32_t AccSpeed_S);
void Start_Motor_SPTA(unsigned char MotorID, unsigned char dir, uint32_t Degree, uint32_t MaxSpeed_SPTA, uint32_t AccSpeed_SPTA);
void SetSpeed(unsigned char MotorID, signed char speedindex);
void Do_Reset(unsigned char MotorID);
void Deal_Cmd(void);
void Initial_PWM_Motor1(void);
void Initial_PWM_Motor2(void);
void Initial_PWM_Motor3(void);
void Initial_PWM_Motor4(void);
void EXTI_Configuration(void);
float GetFaaPara(float fstart, float taa, float tua, float tra, float SetSpeed);
void CalcMotorPeriStep_CPF(float fstart, float faa, float taa, float tua, float tra, uint16_t MotorTimeTable[], uint16_t MotorStepTable[]);
#endif
