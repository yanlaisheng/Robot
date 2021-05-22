#ifndef __MACRO_H
#define __MACRO_H

#define M1DIV 1 //定义电机1的细分数	16	 32
#define M2DIV 1 //定义电机2的细分数		32
#define M3DIV 1 //定义电机3的细分数		32
#define M4DIV 1 //定义电机4的细分数		64
#define M5DIV 1 //定义电机5的细分数		64
#define M6DIV 1 //定义电机6的细分数		64

//如果要求电机起始的转速为60rpm，即1秒转1圈，假设发10000个脉冲转1圈，即起始转速为1秒钟发10000个脉冲
#define M_FRE_START 10000 //电机的启动频率	  8000
#define M_FRE_AA 10000    //电机频率的加加速度		 4500
#define M_T_AA 2          //电机频率的加加速时间2
#define M_T_UA 5          //电机频率的匀加速时间	   6
#define M_T_RA 2          //电机频率的减加速时间 2

#define Motor1_XiShu_1 0.5 //系数1
#define Motor2_XiShu_1 0.5 //系数1
#define Motor3_XiShu_1 0.5 //系数1
#define Motor4_XiShu_1 0.5 //系数1
#define Motor5_XiShu_1 0.5 //系数1
#define Motor6_XiShu_1 0.5 //系数1
#define PULSENUM 10000     //每圈脉冲数

//以下是S型参数
#define F2TIME_PARA 10500000 //将频率值转换为定时器寄存器值得转换参数
#define STEP_PARA 500        //任意时刻转动步数修正因子
#define STEP_AA 31           //加加速阶段，离散化点数
#define STEP_UA 31           //匀加速阶段，离散化点数
#define STEP_RA 31           //减加速阶段，离散化点数

#define MAX_POSITION 8388608 //最大位置
#define TIM_Idle_HIGH 1      //定时器不工作时输出高电平

#endif
