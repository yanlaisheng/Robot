
/************************************************************************
*			CopyRight  2006.3.24   技术中心硬件室   				    *
************************************************************************
* 文件名称:GlobalV.h
* 文件版本：1.00
* 创建人员：周成磊					  							
* 创建日期：2006.3.24											
* 功能描述：全局变量定义。
* 相关硬件：C80F1F020,ISD4004-08M,ISL1208,M25P80			
* 仿真环境：KEILC51 V7.09
* C 编译器：KEIL V6.12 COMPILER						
* 修改记录：2006/04/01 08:19   周成磊
*			
*			 
************************************************************************/
#ifndef __GLOBALV_H
#define __GLOBALV_H

#include "GlobalConst.h"
#include "stm32f4xx.h"
#include "typedef.h"

// (定义变量)
uint8_t VfNo;             // 变频泵序号
uint8_t VfNext;           // 可以启动的下一台变频泵
uint8_t SoftClock[8];     // 辅助时钟
uint16_t w_MinRunFreqHex; // 频率运行最小Hex值
float f_I_Mx;             // 积分项前值
//
uint8_t Vf_No;
uint8_t F_AskStop1;
uint8_t RealClock[9];     // 实时时钟
uint8_t B_PumpBad[6];     // B_PumpBad[0]未使用,泵1-5坏标志
uint8_t B_PumpManu[6];    // 泵手动状态1-5标志(检修用)
uint8_t B_SaveSetP;       // 保存设定压力 标志字节
uint8_t B_SaveVoiceValue; // 保存音量大小 标志字节
uint8_t B_ReadVfPar;      // 读取变频器数据
uint8_t B_ModSelParValue; // 修改选择的参数值
uint8_t F_ComErrorPump1;
uint8_t F_ComErrorPump2;
uint8_t B_ComErrorPump1;
uint8_t B_ComErrorPump2;
uint8_t B_RemoteStop1;
uint8_t B_RemoteStop2;
uint8_t B_MaxSupplyStop1;
uint8_t B_MaxSupplyStop2;
uint8_t F_HaveWater_MaxSupply;
uint8_t F_InPPid;
uint8_t F_HengYa;
uint8_t F_waterYeWeiHigh;
uint8_t F_ZhuoDuHigh;
uint8_t F_YuLvHigh;
uint8_t F_PhHigh;
uint8_t F_PhLow;

uint8_t F_ComErrorPump3;
uint8_t F_ComErrorPump4;
uint8_t B_ComErrorPump3;
uint8_t B_ComErrorPump4;
uint8_t B_RemoteStop3;
uint8_t B_RemoteStop4;
uint8_t B_MaxSupplyStop3;
uint8_t B_MaxSupplyStop4;
uint8_t F_PumpExit;
uint8_t PumpExit[6];
uint8_t F_HighYCSetP;
uint8_t S_HighYCSetP;

uint8_t F_Pump1RunOverTime;
uint8_t F_Pump2RunOverTime;
uint8_t F_Pump3RunOverTime;
uint8_t F_Pump4RunOverTime;
uint8_t F_YeJianSmall;
uint8_t F_HengVfPanDuan_S;
uint8_t F_HengVfPanDuan_L;

//uint32_t  w_ParLst_POS[POS_SIZE*POS_NUM];				//20*15=300个双字 //40*30=1200个双字
s32 w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 6 + FLASH_POS_SIZE + FLASH_POS_CMD_SIZE]; // 伺服电机参数字列表区(300+600*6+300+1200)*4=5400*4=21600个字节

//执行命令的顺序
s32 Pos_CMD_Sort_Queue[POS_CMD_NUM];

uint8_t DInb[18];  // 开关量输入参数列表区			//ZCL 2019.11.23 加大长度，一个SZM220主机，带4个从机，带4个JDQ模块，共9个模块
uint8_t DOutb[18]; // 开关量输出参数列表区		//ZCL 2019.11.23 加大长度，一个SZM220主机，带4个从机，带4个JDQ模块，共9个模块
//uint8_t  NBComBuf[52];		// 宁波通讯缓存 2007.10.30
uint16_t w_AI[36]; // 模拟量输入参数列表区		//ZCL 2019.11.23 加大长度，一个SZM220主机，带4个从机，带4个JDQ模块，共9个模块
//uint16_t  w_AQ[36];			// 模拟量输出参数列表区		//ZCL 2019.11.23 加大长度，一个SZM220主机，带4个从机，带4个JDQ模块，共9个模块

uint16_t w_ParLst[840]; // 参数字列表区

//定义每台电机的查询命令缓冲区
//每条定义：序号+命令长度+命令内容
//uint8_t	Com1_Driver12_Query_BUFF[COM_QUERY_SIZE*COM_QUERY_NUM];	//COM_QUERY_SIZE=20，COM_QUERY_NUM=10.定义10条查询命令，每条20个字节
//uint8_t	Com2_Driver34_Query_BUFF[COM_QUERY_SIZE*COM_QUERY_NUM];
//uint8_t	Com3_Driver5_Query_BUFF[COM_QUERY_SIZE*COM_QUERY_NUM];

//定义每台电机的写命令缓冲区
//每条定义：序号+命令长度+命令内容
uint8_t Com1_Driver1_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM]; //COM_CMD_SIZE=20，COM_CMD_NUM=30.定义30条写命令，每条20个字节
uint8_t Com1_Driver2_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM]; //COM_CMD_SIZE=20，COM_CMD_NUM=30.定义30条写命令，每条20个字节
uint8_t Com2_Driver3_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];
uint8_t Com2_Driver4_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];
uint8_t Com3_Driver5_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];
uint8_t Com3_Driver6_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];
uint8_t Com4_Write_BUFF[COM_CMD_SIZE * COM_CMD_NUM];

uint8_t *Com_Write_p;

//定义每台电机的上一条命令
//定义：序号+命令长度+命令内容
uint8_t Com1_LAST_CMD_BUFF[COM_CMD_SIZE]; //定义1条，30个字节
uint8_t Com2_LAST_CMD_BUFF[COM_CMD_SIZE];
uint8_t Com3_LAST_CMD_BUFF[COM_CMD_SIZE];
uint8_t Com4_LAST_CMD_BUFF[COM_CMD_SIZE];

//uint8_t* pByteParLst=(uint8_t *)(&w_ParLst[0]);
uint8_t B_SelCS;
uint16_t w_FaultNo[4];

uint8_t B_ComErrWithVvvf;    // 标志 与变频器通讯失败 标志字节
uint8_t B_ComErrWithLLJ;     // 标志 与流量计通讯失败　LLJ:流量计
uint8_t B_ComErrWithModule1; // 标志 与子模块1通讯失败
uint8_t B_ComErrWithModule2; // 标志 与子模块2通讯失败

uint8_t F_Com2SendNext;        // 软串口发送下一条标志
uint8_t F_Com2RCVCommandOk;    // 软串口接收命令OK标志
uint8_t F_SlaveNoRcvMasterCMD; // 从机没有接收到主机命令

uint16_t w_PreVoiceVal;   // 上次音量大小
uint16_t w_ModRealTimeNo; // 修改实时时钟的序号
uint16_t w_PreRecNo;      // 上次记录号

uint32_t Reset_time_Max; //复位运行时间
//
// 以下为用外部字节定义的开关量输入，开关量输出部分
// 硬件定义
//1  开关量输入 使用
uint8_t K_ManuAuto;       // 手动.自动
uint8_t K_StopRun;        // 启动停止
uint8_t K_OverP;          // 超压报警
uint8_t K_BCQErr;         // 补偿器预警
uint8_t K_VvvfERR;        // 变频器报警
uint8_t K_WaterLeverHigh; // 水箱水位超高
uint8_t K_NoWater;        // 液位计无水停机
uint8_t K_HaveWater;      // 液位计有水开机
uint8_t K_PumpHot[6];     // 1-5号泵热故障输入
uint8_t K_PowerErr;       // 电源故障
uint8_t K_Fire;           // 消防报警
uint8_t K_ResetVvvf;      // 变频器复位
uint8_t K_FluxPluseOn;
uint8_t K_DDPluseOn;
uint8_t K_DDFOpen;
uint8_t K_DDFClose;

//					//2  开关量输入 暂且不用
uint8_t K_1PumpManu;    // 1号泵手动输入
uint8_t K_2PumpManu;    // 2号泵手动输入
uint8_t K_3PumpManu;    // 3号泵手动输入
uint8_t K_4PumpManu;    // 4号泵手动输入
uint8_t K_5PumpManu;    // 5号泵手动输入
uint8_t K_1PumpGp;      // 1号泵工频
uint8_t K_2PumpGp;      // 2号泵工频
uint8_t K_3PumpGp;      // 3号泵工频
uint8_t K_4PumpGp;      // 4号泵工频
uint8_t K_5PumpGp;      // 5号泵工频
uint8_t K_1PumpVf;      // 1号泵变频
uint8_t K_2PumpVf;      // 2号泵变频
uint8_t K_3PumpVf;      // 3号泵变频
uint8_t K_4PumpVf;      // 4号泵变频
uint8_t K_5PumpVf;      // 5号泵变频
uint8_t K_Term13;       // 端子13 DI
uint8_t K_Term14;       // 端子14 DI
uint8_t K_Term15;       // 端子15 DI
                        //3  开关量输出 使用
uint8_t Q_VfStop;       // 空转
uint8_t Q_RstOn;        // 复位继电器
uint8_t Q_PowerOnModem; // 给MODEM上电
uint8_t Q_BcqErr;       // 补偿器预警
uint8_t Q_VfOn[6];      // 变频1--5
uint8_t Q_GpOn[6];      // 工频1--5
uint8_t Q_AlarmError;   // 报警错误继电器
uint8_t Q_FanMotor;     // 风扇电机继电器
uint8_t Q_RunGPRS;
//4  位标志部分 5个字节
// 停机状态标志
uint8_t F_NoWater;            // 无水停机标志
uint8_t F_ManualRunStop;      // 手动启动停止标志
uint8_t F_ManualAuto;         // 手动，自动 （=1,手动）
uint8_t F_VvvfAlarm;          // 变频器报警标志
uint8_t F_PowerFault;         // 电源故障
uint8_t F_WaterLeverHigh;     // 超高液位停机标志
uint8_t F_HardOverP;          // 硬件超压标志
uint8_t F_AllPumpBad;         // 所有泵坏标志
uint8_t F_SmallStop;          // 小流量停机标志
uint8_t F_OutMeterBad;        // 出水口远传压力表坏停机标志  // 暂没用
uint8_t F_InPBigSetStop;      // 进水压力高停机－进水压>设定压停机
uint8_t F_TimeRunStop;        // 定时启动停止标志
uint8_t F_TimePwdStop;        // 定时密码停机标志
uint8_t F_RemoteStop;         // 远程遥控停机标志
uint8_t F_InPBigOutStop;      // 进水大于出水停机标志－进水压>出水压停机
uint8_t F_MaxSupplyStop;      // 最大供水能力停机标志
uint8_t F_Rtcf;               // 后备电池失效
uint8_t F_ComErrWithVvvf;     // 与变频器通讯失败
uint8_t F_ComErrWithLLJ;      // 与流量计通讯失败　LLJ:流量计
uint8_t F_ComErrWithModule1;  // 与子模块1通讯失败
uint8_t F_ComErrWithModule2;  // 与子模块2通讯失败
uint8_t F_TouchAutoManu;      // 触摸自动手动
uint8_t F_TouchRunStop;       // 触摸启动停止
uint8_t F_PSensorProtectStop; // 压力传感器保护停机

//YLS 2020.06.25
uint8_t F_TouchReSet;      //触摸复位
uint8_t F_AskReset;        //请求复位
uint8_t F_Reseting;        //正在复位标志
uint8_t F_TouchForceReSet; //强制复位
uint8_t F_AskForceReset;   //请求复位
uint8_t F_ForceReseting;   //正在复位标志
uint8_t F_Reseting;        //正在复位标志

uint8_t F_StepMode;   //=1，进入手动模式标志
uint8_t F_PowerOnRun; //是否上电自动运行标志

uint8_t F_Stoping;  //正在停止标志
uint8_t F_Starting; //正在启动标志

uint8_t F_Encoder_Read; //编码器数据已读取标志
uint8_t F_Status_Read;  //VDO状态读取标志

// 继续在上添加停机标志
//---------------------
// 内部状态标志
uint8_t F_CycleCheck; // 周期巡检标志
uint8_t F_DealSmall;  // 正在处理小流量标志
uint8_t F_Fire;       // 消防状态标志
// 继续在上添加状态标志
//---------------------
// 内部特殊故障标志
uint8_t F_VvvfBad;   // 变频器坏标志
uint8_t F_SmallBad;  // 小流量破坏标志
uint8_t F_BcqErr;    // 补偿器预警
uint8_t F_SoftOverP; // 软件超压标志
// 内部转换标志
uint8_t F_AskStop; // 请求停机标志
//uint8_t  F_AskExchange;			// 请求交换标志
uint8_t F_VfToGp;        // 变频转工频标志
uint8_t F_VfToVf;        // 变频交换标志
uint8_t F_TimerExchange; // 定时交换标志
//
uint8_t F_SmallBetween; // 小流量间隔标志
//uint8_t  F_DelayCheckVvvfAlarm;	// 第一次上电延时检测变频器报警5秒种后再检测
uint8_t F_Inital;                  // 初始化标志
uint8_t F_VvvfAlarmREC;            // 变频器报警进行记录
uint8_t F_TimeREC;                 // 定时写记录
uint8_t F_ModRealTime;             // 修改实时时钟
uint8_t F_FlashRecNoNum;           // FLASH记录的序号和数量
uint8_t F_FirstPowerNoPlayNoWater; // 第一次上电不播放无水停机

uint8_t F_AllRdy; //所有电机都Ready标志

uint8_t F_AllRun;  //所有电机处于RUN标志
uint8_t F_Reseted; //复位到原点标志

extern uc16 w_ParBootLst[];
extern uint32_t Driver1_Cmd_Group1[];
extern uint32_t Driver2_Cmd_Group1[];
extern uint32_t Driver3_Cmd_Group1[];
extern uint32_t Driver4_Cmd_Group1[];
extern uint32_t Driver5_Cmd_Group1[];
extern uint32_t Driver6_Cmd_Group1[];

extern sc32 Pos_Init[];
extern sc32 Pos_Cmd_Group1[];
extern sc32 Pos_Cmd_Group2[];
extern sc32 Pos_Cmd_Group3[];
extern sc32 Pos_Cmd_Group4[];
extern sc32 Pos_Cmd_Group5[];

s32 *arr_p1;
s32 *arrp_p1_Last; //保存上一条指令的命令指针

s32 *Driver12_arr_p;
s32 *Driver34_arr_p;
s32 *Driver56_arr_p;

uint16_t T_SavePos_Delay1;
uint16_t C_SavePos_Delay1;
uint16_t T_SavePos_Delay2;
uint16_t C_SavePos_Delay2;
uint16_t T_SavePos_Delay3;
uint16_t C_SavePos_Delay3;
uint16_t T_SavePos_Delay4;
uint16_t C_SavePos_Delay4;
uint16_t T_SavePos_Delay5;
uint16_t C_SavePos_Delay5;
uint16_t T_SavePos_Delay6;
uint16_t C_SavePos_Delay6;

uint8_t Driver1_Save_P;
uint8_t Driver2_Save_P;
uint8_t Driver3_Save_P;
uint8_t Driver4_Save_P;
uint8_t Driver5_Save_P;
uint8_t Driver6_Save_P;

uint8_t F_Driver1_WrPos_ErrStatus;
uint8_t F_Driver2_WrPos_ErrStatus;
uint8_t F_Driver3_WrPos_ErrStatus;
uint8_t F_Driver4_WrPos_ErrStatus;
uint8_t F_Driver5_WrPos_ErrStatus;
uint8_t F_Driver6_WrPos_ErrStatus;

uint8_t Driver1_delay_F; //电机暂停延时标志，=1表示正在暂停；=0,表示暂停结束

uint8_t DO_delay_F; //DO输出延时标志
uint8_t F_Open_Hand;
uint8_t F_Close_Hand;

uint32_t Driver1_delay_ms_Count; //电机暂停延时计数，ms
uint32_t Driver2_delay_ms_Count;
uint32_t Driver3_delay_ms_Count;
uint32_t Driver4_delay_ms_Count;
uint32_t Driver5_delay_ms_Count;
uint32_t Driver6_delay_ms_Count;

uint32_t T_Driver1_delay;      //延时定时器
uint32_t C_Driver1_delayCount; //延时计数
uint32_t T_Driver2_delay;
uint32_t C_Driver2_delayCount;
uint32_t T_Driver3_delay;
uint32_t C_Driver3_delayCount;
uint32_t T_Driver4_delay;
uint32_t C_Driver4_delayCount;
uint32_t T_Driver5_delay;
uint32_t C_Driver5_delayCount;
uint32_t T_Driver6_delay;
uint32_t C_Driver6_delayCount;

uint32_t T_DO_delay;           //DO输出延时定时器
uint32_t C_DO_delayCount;      //DO输出延时计数
uint32_t T_DO_Open_delay;      //DO输出持续定时器
uint32_t C_DO_Open_delayCount; //DO输出持续计数

uint16_t T_Driver1_FillCMD;
uint16_t C_Driver1_FillCMD;
uint16_t T_Driver2_FillCMD;
uint16_t C_Driver2_FillCMD;
uint16_t T_Driver3_FillCMD;
uint16_t C_Driver3_FillCMD;
uint16_t T_Driver4_FillCMD;
uint16_t C_Driver4_FillCMD;
uint16_t T_Driver5_FillCMD;
uint16_t C_Driver5_FillCMD;
uint16_t T_Driver6_FillCMD;
uint16_t C_Driver6_FillCMD;

uint16_t T_StepAutoDelay;
uint16_t C_T_StepAutoDelayDelay;
uint16_t T_StepAutoDelay2;
uint16_t C_T_StepAutoDelayDelay2;

uint16_t T_ResetDelay;
uint16_t C_ResetDelay;
uint16_t T_ResetDelay2;
uint16_t C_ResetDelay2;
uint16_t T_ResetDelay3;
uint16_t C_ResetDelay3;
uint8_t F_SendStopCMD2;

uint16_t T_Reset_Driver;
uint16_t C_Reset_Driver;
uint16_t T_Reset_Driver2;
uint16_t C_Reset_Driver2;
uint8_t F_AutoReset;
uint8_t F_SendStopCMD;

uint8_t Driver1_Write_Sort; //驱动器1写位置段顺序，0写第1段，1写第2段
uint8_t Driver2_Write_Sort;
uint8_t Driver3_Write_Sort;
uint8_t Driver4_Write_Sort;
uint8_t Driver5_Write_Sort;
uint8_t Driver6_Write_Sort;
uint8_t F_Hand_Status; //电磁阀状态

uint8_t F_Driver1_notBrake; //1#伺服刹车信号
uint8_t F_Driver2_notBrake;
uint8_t F_Driver3_notBrake;
uint8_t F_Driver4_notBrake;
uint8_t F_Driver5_notBrake;
uint8_t F_Driver6_notBrake;
uint8_t F_Driver_All_notBrake;

uint16_t C_AllStop;

uint8_t F_Sync_6_axis; //6轴同步输出标志
uint8_t F_Resetting;   //正在复位标志

uint8_t F_Driver1_Send_Cmd; //控制器给1#伺服驱动器发出位置命令标志，=1表示已发送命令
uint8_t F_Driver2_Send_Cmd;
uint8_t F_Driver3_Send_Cmd;
uint8_t F_Driver4_Send_Cmd;
uint8_t F_Driver5_Send_Cmd;
uint8_t F_Driver6_Send_Cmd;

uint8_t F_Driver1_Timeout; //1#伺服驱动器发送命令超时标志，=1表示超时
uint8_t F_Driver2_Timeout;
uint8_t F_Driver3_Timeout;
uint8_t F_Driver4_Timeout;
uint8_t F_Driver5_Timeout;
uint8_t F_Driver6_Timeout;

uint8_t F_Driver1_Cmd_Err; //1#伺服驱动器位置命令错误标志，=1表示错误
uint8_t F_Driver2_Cmd_Err;
uint8_t F_Driver3_Cmd_Err;
uint8_t F_Driver4_Cmd_Err;
uint8_t F_Driver5_Cmd_Err;
uint8_t F_Driver6_Cmd_Err;
uint8_t F_HaveDriver_Cmd_Err; //有伺服写入位置错误标志

uint8_t F_Driver1_Cmd_Con_Err; //1#伺服驱动器控制命令错误标志，=1表示错误
uint8_t F_Driver2_Cmd_Con_Err;
uint8_t F_Driver3_Cmd_Con_Err;
uint8_t F_Driver4_Cmd_Con_Err;
uint8_t F_Driver5_Cmd_Con_Err;
uint8_t F_Driver6_Cmd_Con_Err;
uint8_t F_HaveDriver_Cmd_Con_Err; //有伺服写入位置错误标志

uint8_t Driver1_Cmd_PosNo; //1#伺服驱动器发送的命令位置号，位置0或者位置1
uint8_t Driver2_Cmd_PosNo;
uint8_t Driver3_Cmd_PosNo;
uint8_t Driver4_Cmd_PosNo;
uint8_t Driver5_Cmd_PosNo;
uint8_t Driver6_Cmd_PosNo;

uint8_t Driver1_Cmd_Status; //1#伺服驱动器P8910状态
uint8_t Driver2_Cmd_Status;
uint8_t Driver3_Cmd_Status;
uint8_t Driver4_Cmd_Status;
uint8_t Driver5_Cmd_Status;
uint8_t Driver6_Cmd_Status;

uint16_t T_Driver1_WriteCMD;
uint16_t C_Driver1_WriteCMD;
uint16_t T_Driver2_WriteCMD;
uint16_t C_Driver2_WriteCMD;
uint16_t T_Driver3_WriteCMD;
uint16_t C_Driver3_WriteCMD;
uint16_t T_Driver4_WriteCMD;
uint16_t C_Driver4_WriteCMD;
uint16_t T_Driver5_WriteCMD;
uint16_t C_Driver5_WriteCMD;
uint16_t T_Driver6_WriteCMD;
uint16_t C_Driver6_WriteCMD;

uint8_t Driver1_Cmd_Data[9]; //1#伺服驱动器命令数据,4个字节为脉冲，2个字节为速度，2个字节为加减速时间，最后一个字节为位置号（0/1）
uint8_t Driver2_Cmd_Data[9];
uint8_t Driver3_Cmd_Data[9];
uint8_t Driver4_Cmd_Data[9];
uint8_t Driver5_Cmd_Data[9];
uint8_t Driver6_Cmd_Data[9];

uint8_t Driver1_Pos_Start_Sort; //1#伺服，=0，表示还未写入到命令缓冲区；=1，表示已经写入到命令缓冲区；=2，表示已经发送
uint8_t Driver2_Pos_Start_Sort;
uint8_t Driver3_Pos_Start_Sort;
uint8_t Driver4_Pos_Start_Sort;
uint8_t Driver5_Pos_Start_Sort;
uint8_t Driver6_Pos_Start_Sort;

uint8_t Driver1_Status_Sort; //1#伺服，=2，表示已经发送；=3，表示已经接收
uint8_t Driver2_Status_Sort;
uint8_t Driver3_Status_Sort;
uint8_t Driver4_Status_Sort;
uint8_t Driver5_Status_Sort;
uint8_t Driver6_Status_Sort;

//**********COM1**********
uint8_t Txd1Buffer[TXD1_MAX]; // 发送缓冲区
uint8_t Rcv1Buffer[RCV1_MAX]; // 接收缓冲区
uint8_t Tmp_Txd1Buffer[1], Tmp_Rxd1Buffer;
uint16_t Rcv1Counter;   // 接收计数器//
uint16_t Txd1Counter;   // 发送计数器//
uint16_t BakRcv1Count;  // 接收计数器//
uint16_t Txd1Max;       // 有多少个字符需要发送//
uint16_t w_Txd1ChkSum;  // 发送校验和，lo,hi 两位//
uint16_t w_Com1RegAddr; // 串口1寄存器地址

//
uint8_t B_Com1Cmd03;
uint8_t B_Com1Cmd16;
uint8_t B_Com1Cmd01;
uint8_t B_Com1Cmd06;
uint16_t T_NoRcv1Count; // 没有接收计数器
uint16_t C_NoRcv1Count;

//**********COM2**********
uint8_t Txd2Buffer[TXD2_MAX]; // 发送缓冲区
uint8_t Rcv2Buffer[RCV2_MAX]; // 接收缓冲区
uint8_t Tmp_Txd2Buffer[1], Tmp_Rxd2Buffer;
uint16_t Rcv2Counter;   // 接收计数器//
uint16_t Txd2Counter;   // 发送计数器//
uint16_t BakRcv2Count;  // 接收计数器//
uint16_t Txd2Max;       // 有多少个字符需要发送//
uint16_t w_Txd2ChkSum;  // 发送校验和，lo,hi 两位//
uint16_t w_Com2RegAddr; // 串口2寄存器地址

uint8_t B_Com2Cmd03;
uint8_t B_Com2Cmd16;
uint8_t B_Com2Cmd01;
uint8_t B_Com2Cmd06;
uint16_t T_NoRcv2Count; // 没有接收计数器
uint16_t C_NoRcv2Count;

//**********COM3**********
uint8_t Txd3Buffer[TXD3_MAX]; // 发送缓冲区
uint8_t Rcv3Buffer[RCV3_MAX]; // 接收缓冲区
uint8_t Tmp_Txd3Buffer[1], Tmp_Rxd3Buffer;
uint16_t Rcv3Counter;   // 接收计数器//
uint16_t Txd3Counter;   // 发送计数器//
uint16_t BakRcv3Count;  // 接收计数器//
uint16_t Txd3Max;       // 有多少个字符需要发送//
uint16_t w_Txd3ChkSum;  // 发送校验和，lo,hi 两位//
uint16_t w_Com3RegAddr; // 串口3寄存器地址

uint8_t B_Com3Cmd03;
uint8_t B_Com3Cmd16;
uint8_t B_Com3Cmd01;
uint8_t B_Com3Cmd06;
uint16_t T_NoRcv3Count; // 没有接收计数器
uint16_t C_NoRcv3Count;

//**********COM4**********
uint8_t Txd4Buffer[TXD4_MAX]; // 发送缓冲区
uint8_t Rcv4Buffer[RCV4_MAX]; // 接收缓冲区
uint8_t Tmp_Txd4Buffer[1], Tmp_Rxd4Buffer;
uint16_t Rcv4Counter;   // 接收计数器//
uint16_t Txd4Counter;   // 发送计数器//
uint16_t BakRcv4Count;  // 接收计数器//
uint16_t Txd4Max;       // 有多少个字符需要发送//
uint16_t w_Txd4ChkSum;  // 发送校验和，lo,hi 两位//
uint16_t w_Com4RegAddr; // 串口4寄存器地址

uint8_t B_Com4Cmd03;
uint8_t B_Com4Cmd16;
uint8_t B_Com4Cmd01;
uint8_t B_Com4Cmd06;
uint16_t T_NoRcv4Count; // 没有接收计数器
uint16_t C_NoRcv4Count;

//**********COM5**********
uint8_t Txd5Buffer[TXD5_MAX]; // 发送缓冲区
uint8_t Rcv5Buffer[RCV5_MAX]; // 接收缓冲区
uint8_t Tmp_Txd5Buffer[1], Tmp_Rxd5Buffer;
uint16_t Rcv5Counter;   // 接收计数器//
uint16_t Txd5Counter;   // 发送计数器//
uint16_t BakRcv5Count;  // 接收计数器//
uint16_t Txd5Max;       // 有多少个字符需要发送//
uint16_t w_Txd5ChkSum;  // 发送校验和，lo,hi 两位//
uint16_t w_Com5RegAddr; // 串口5寄存器地址

uint8_t B_Com5Cmd03;
uint8_t B_Com5Cmd16;
uint8_t B_Com5Cmd01;
uint8_t B_Com5Cmd06;
uint16_t T_NoRcv5Count; // 没有接收计数器
uint16_t C_NoRcv5Count;

//**********COM6**********
uint8_t Txd6Buffer[TXD6_MAX]; // 发送缓冲区
uint8_t Rcv6Buffer[RCV6_MAX]; // 接收缓冲区
uint8_t Tmp_Txd6Buffer[1], Tmp_Rxd6Buffer;
uint16_t Rcv6Counter;   // 接收计数器//
uint16_t Txd6Counter;   // 发送计数器//
uint16_t BakRcv6Count;  // 接收计数器//
uint16_t Txd6Max;       // 有多少个字符需要发送//
uint16_t w_Txd6ChkSum;  // 发送校验和，lo,hi 两位//
uint16_t w_Com6RegAddr; // 串口6寄存器地址

uint8_t B_Com6Cmd03;
uint8_t B_Com6Cmd16;
uint8_t B_Com6Cmd01;
uint8_t B_Com6Cmd06;
uint16_t T_NoRcv6Count; // 没有接收计数器
uint16_t C_NoRcv6Count;

uint32_t step_to_run_MOTOR1; //要匀速运行的步数       总共运行步数 = ACCELERATED_SPEED_LENGTH*2 + step_to_run_MOTOR1
// float fre_MOTOR1[ACCELERATED_SPEED_LENGTH];             //数组存储加速过程中每一步的频率
unsigned short period_MOTOR1[ACCELERATED_SPEED_LENGTH]; //数组储存加速过程中每一步定时器的自动装载值

uint32_t step_to_run_MOTOR2; //要匀速运行的步数       总共运行步数 = ACCELERATED_SPEED_LENGTH*2 + step_to_run_MOTOR1
// float fre_MOTOR2[ACCELERATED_SPEED_LENGTH];             //数组存储加速过程中每一步的频率
unsigned short period_MOTOR2[ACCELERATED_SPEED_LENGTH]; //数组储存加速过程中每一步定时器的自动装载值

uint32_t step_to_run_MOTOR3; //要匀速运行的步数       总共运行步数 = ACCELERATED_SPEED_LENGTH*2 + step_to_run_MOTOR3
// float fre_MOTOR3[ACCELERATED_SPEED_LENGTH];             //数组存储加速过程中每一步的频率
unsigned short period_MOTOR3[ACCELERATED_SPEED_LENGTH]; //数组储存加速过程中每一步定时器的自动装载值

uint32_t step_to_run_MOTOR4; //要匀速运行的步数       总共运行步数 = ACCELERATED_SPEED_LENGTH*2 + step_to_run_MOTOR1
// float fre_MOTOR4[ACCELERATED_SPEED_LENGTH];             //数组存储加速过程中每一步的频率
unsigned short period_MOTOR4[ACCELERATED_SPEED_LENGTH]; //数组储存加速过程中每一步定时器的自动装载值

uint32_t step_to_run_MOTOR5; //要匀速运行的步数       总共运行步数 = ACCELERATED_SPEED_LENGTH*2 + step_to_run_MOTOR1
// float fre_MOTOR5[ACCELERATED_SPEED_LENGTH];             //数组存储加速过程中每一步的频率
unsigned short period_MOTOR5[ACCELERATED_SPEED_LENGTH]; //数组储存加速过程中每一步定时器的自动装载值

uint32_t step_to_run_MOTOR6; //要匀速运行的步数       总共运行步数 = ACCELERATED_SPEED_LENGTH*2 + step_to_run_MOTOR1
// float fre_MOTOR6[ACCELERATED_SPEED_LENGTH];             //数组存储加速过程中每一步的频率
unsigned short period_MOTOR6[ACCELERATED_SPEED_LENGTH]; //数组储存加速过程中每一步定时器的自动装载值

#endif /* __GLOBALV_H */
