/**
  ******************************************************************************
  * @file    DoWith.c
  * @author  ChengLei Zhou  - 周成磊
  * @version V1.27
  * @date    2014-01-03
  * @brief   数字量输入检测，数字量输出,模拟量输入检测，模拟量输出,其他本机操作
	******************************************************************************

	******************************************************************************	
	*/

/* Includes ------------------------------------------------------------------*/
#include "GlobalConst.h"
#include "GlobalV_Extern.h" // 全局变量声明
#include "DoWith.h"
#include "spi_flash.h"
//#include "ds1302.h"
#include "com1_232.h"
#include "typedef.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t FilterNo; // 过滤序号
//
uint8_t T_BootParLst; // 初始化参数表列
uint16_t C_BootParLst;
uint8_t T_BootParLst2; // 初始化参数表列
uint16_t C_BootParLst2;

uint8_t S_BootParLst;
uint8_t T_SavePar; // 保存参数
uint16_t C_SavePar;
uint8_t S_SavePar;

uint8_t T_ReadRealTime; // 读实时时钟
uint16_t C_ReadRealTime;
uint8_t T_ModRealTime; // 修改（写）实时时钟
uint16_t C_ModRealTime;
uint8_t S_ModRealTime;
uint8_t T_TimeWriteRec; // 定时写记录
uint16_t C_TimeWriteRec;
//
uint8_t T_ForceSavPar;	// 强制保存参数
uint8_t B_ForceSavPar;	// 标志
uint16_t C_ForceSavPar; // 使用次数较大，使用pdata

uint16_t w_DIStableCounter; // 开关量输入PE稳定记数器

uint8_t T_DoWith; // 处理
uint8_t C_bDoWith;
uint8_t T_VADelay; // 电压电流显示延时,为了稳定输出 2007.11.1
uint16_t C_VADelay;

uint8_t F_ReadMemoryBoardNoNum; // 读外扩展存储板记录序号和数量
uint32_t PmupRunTimeH;

uint8_t S_KglInPrompt;	// 开关量输入状态提示 ZCL 2015.9.23
uint8_t S_KglOutPrompt; // 开关量输出状态提示
uint8_t T_KglInPrompt;
uint16_t C_KglInPrompt;
uint8_t T_KglOutPrompt;
uint16_t C_KglOutPrompt;

uint8_t F_KglInPrompt;
uint8_t F_KglOutPrompt;
uint16_t C_SaveDelay;
uint8_t T_SaveDelay;

uint16_t w_DI1StableCounter; // 开关量输入稳定记数器
uint16_t w_DI2StableCounter; // 开关量输入稳定记数器
uint16_t w_DI3StableCounter; // 开关量输入稳定记数器
uint16_t w_DI4StableCounter; // 开关量输入稳定记数器

uint8_t B_DI1;
uint8_t B_DI2;
uint8_t B_DI3;
uint8_t B_DI4;

uint8_t Old_Run_Mode;		  //保存原来的运行模式
uint8_t Old_Pos_Group_Select; //保存原来的组号

//-----------------------------------------------------------------------------------
//EXTERN Global VARIABLES - Bit VARIABLES   外部变量、位变量申明
//-----------------------------------------------------------------------------------

//
extern uint8_t F_Com2SendNext;		  // 软串口发送下一条标志
extern uint8_t F_SlaveNoRcvMasterCMD; // 从机没有接收到主机命令
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

extern s16 Com1_Driver1_Queue_Rear;	 //命令队列中的命令数量，队尾指针
extern s16 Com1_Driver2_Queue_Rear;	 //命令队列中的命令数量，队尾指针
extern s16 Com1_Driver1_Queue_Front; //命令队列中的当前要出队的，队头指针
extern s16 Com1_Driver2_Queue_Front; //命令队列中的当前要出队的，队头指针

extern s16 Com2_Driver3_Queue_Rear;	 //命令队列中的命令数量，队尾指针
extern s16 Com2_Driver4_Queue_Rear;	 //命令队列中的命令数量，队尾指针
extern s16 Com2_Driver3_Queue_Front; //命令队列中的当前要出队的，队头指针
extern s16 Com2_Driver4_Queue_Front; //命令队列中的当前要出队的，队头指针

extern s16 Com3_Driver5_Queue_Rear;
extern s16 Com3_Driver6_Queue_Rear;
extern s16 Com3_Driver5_Queue_Front;
extern s16 Com3_Driver6_Queue_Front;

extern s32 *arr_p1;
extern s32 *arrp_p1_Last; //保存上一条指令的命令指针

extern uint16_t Rcv1Counter; // 接收计数器//
extern uint16_t Txd1Counter; // 发送计数器//
extern uint16_t Rcv2Counter; // 接收计数器//
extern uint16_t Txd2Counter; // 发送计数器//
extern uint16_t Rcv3Counter; // 接收计数器//
extern uint16_t Txd3Counter; // 发送计数器//
extern uint16_t Rcv4Counter; // 接收计数器//
extern uint16_t Txd4Counter; // 发送计数器//

extern uint8_t F_Starting;
extern uint8_t F_Stoping;
extern uint8_t F_AskReset;
extern uint8_t F_Reseting;
extern uint8_t F_AskForceReset;
extern uint8_t F_ForceReseting;
extern uint8_t F_TouchForceReSet;
extern uint8_t F_PowerOnRun;
extern uint8_t F_StepMode;
extern uint8_t F_SendStopCMD2;

extern uint8_t F_Encoder_Read; //编码器数据已读取标志
extern uint8_t F_AllRdy;	   //所有电机都Ready标志
extern uint8_t F_AllRun;	   //所有电机处于RUN标志

extern uint16_t Driver1_RcvCount; //接收计数
extern uint16_t Driver2_RcvCount; //接收计数
extern uint16_t Driver3_RcvCount; //接收计数
extern uint16_t Driver4_RcvCount; //接收计数
extern uint16_t Driver5_RcvCount; //接收计数
extern uint16_t Driver6_RcvCount; //接收计数
extern uint16_t Com4_RcvCount;

extern uint16_t C_Driver1_Pos1_Delay;
extern uint16_t C_Driver1_Pos2_Delay;
extern uint16_t C_Driver2_Pos1_Delay;
extern uint16_t C_Driver2_Pos2_Delay;

extern uint32_t T_Driver1_delay;	  //延时定时器
extern uint32_t C_Driver1_delayCount; //延时计数
extern uint32_t T_Driver2_delay;
extern uint32_t C_Driver2_delayCount;
extern uint32_t T_Driver3_delay;
extern uint32_t C_Driver3_delayCount;
extern uint32_t T_Driver4_delay;
extern uint32_t C_Driver4_delayCount;
extern uint32_t T_Driver5_delay;
extern uint32_t C_Driver5_delayCount;
extern uint32_t T_Driver6_delay;
extern uint32_t C_Driver6_delayCount;

extern uint8_t Driver1_Write_Sort; //驱动器1写位置段顺序，0写第1段，1写第2段
extern uint8_t Driver2_Write_Sort;
extern uint8_t Driver3_Write_Sort;
extern uint8_t Driver4_Write_Sort;
extern uint8_t Driver5_Write_Sort;
extern uint8_t Driver6_Write_Sort;

extern uint8_t K_StopRun; // 启动停止
uint8_t Old_K_StopRun;

extern uint8_t F_Driver1_notBrake; //1#伺服刹车信号
extern uint8_t F_Driver2_notBrake;
extern uint8_t F_Driver3_notBrake;
extern uint8_t F_Driver4_notBrake;
extern uint8_t F_Driver5_notBrake;
extern uint8_t F_Driver6_notBrake;
extern uint8_t F_Driver_All_notBrake;

/* Private function prototypes -----------------------------------------------*/
void I2C_EE_ByteWrite(uint8_t *pBuffer, uint8_t WriteAddr);
void I2C_EE_BufferRead(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
void ParArrayRead_DWord(uint32_t *p_Top, uc32 *p_Base, uint16_t w_ReadSize);

uint16_t CRC16(uint8_t *pCrcData, uint8_t CrcDataLen);

//uint16_t STMFLASH_ReadHalfWord(uint32_t faddr);		  //读出半字
uint32_t STMFLASH_ReadWord(uint32_t faddr);
void STMFLASH_WriteLenByte(uint32_t WriteAddr, uint32_t DataToWrite, uint16_t Len); //指定地址开始写入指定长度的数据
uint32_t STMFLASH_ReadLenByte(uint32_t ReadAddr, uint16_t Len);						//指定地址开始读取指定长度数据
void STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint16_t NumToWrite);	//从指定地址开始写入指定长度的数据
void STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint16_t NumToRead);		//从指定地址开始读出指定长度的数据

//按组进行参数初始化
void ParLst_Init_Group(void);
//清零位置及位置命令
void ParLst_Init_Group2Zero(void);
//清零位置
void ParLst_Init_Pos2Zero(void);

uc16 w_ParBootLst[FLASH_PAR_SIZE] = {
	1500, 1500, 1500, 100, 50,	 // 0-4		0=伺服电机1初始位置,1=伺服电机2初始位置,2=伺服电机3初始位置
								 //			3=有水开机压力,4=无水停机压力
	10, 1, 60, 5, 0,			 // 5-9		5=有水开机延时,6=参数组号选择
								 //			7=变频转工频时间 8=工频退出时间	9=修改参数
	120, 2, 2, 240, 5,			 // 10-14	10=最小频率 11=设备通讯地址 12=水泵数量
								 //			13=交换分钟 14=变频器复位次数
	1, 0, 0, 0, 0,				 // 15-19	15=触摸 启动/停止 16=触摸 复位 17=强制复位
								 //			18=使能巡检功能 19=允许放音 Pw_EnablePlayV
	6, 0, 0x5A, 0, 1,			 // 20-24	20=Pw_BetweenPlaySec 21=变频器通讯地址 22=初始化标志
								 //			23=参数初始化 24=联动位置控制命令组选择（1-5组）
	1, 1000, 10000, 0, 5000,	 // 25-29	25=参数组号选择 26=位置写入后的延时  27=1#手动运行速度
								 //			28=1#手动运行方向 29=2#手动运行脉冲
	10000, 0, 5000, 10000, 0,	 // 30-34	30-2#手动运行速度	31=2#手动运行方向
								 //			32=3#手动运行脉冲
								 //			33=3#手动运行速度 34=3#手动运行方向
	5000, 10000, 0, 1500, 1,	 // 35-39	35=4#手动运行脉冲 36=4#手动运行速度 37=4#手动运行方向
								 //			38=1#伺服电机位置 39=1#伺服电机速度
	20, 1500, 1, 20, 0,			 // 40-44	40=1#伺服电机延时 41=2#伺服电机位置
								 //			42=2#伺服电机速度
								 //			43=2#伺服电机延时 44=1#手动运行使能
	0, 0, 0, 0, 0,				 // 45-49	45=2#手动运行使能 46=3#手动运行使能 47=4#手动运行使能
								 //			48=1#伺服电机手动运行使能 49= 2#伺服电机手动运行使能
								 //
	1, 2, 800, 1500, 1,			 // 50-54	50=手动模式（1），自动模式（0） 51=星期 52=PID_Kc
								 //			53=3#伺服电机位置 54=3#伺服电机速度
	20, 1, 0, 0, 0,				 // 55-59	55=3#伺服电机延时 56= 3#伺服电机手动运行使能  /57=SM510模块1地址
								 //			58=模拟量频闪灯 /59=SM510模块2地址
	30, 500, 2500, 500, 2500,	 // 60-64	60=自来水压力高延时 61=1#伺服电机最小位置 62=1#伺服电机最大位置
								 //			63=2#伺服电机最小位置 64=2#伺服电机最大位置
	500, 2500, 630, 830, 400,	 // 65-69	65=3#伺服电机最小位置 66=3#伺服电机最大位置  67=定时1启动
								 //			68=定时1停止 69=定时1设定压力  66－84 6组定时时间
	1100, 1300, 400, 1700, 2200, // 70-74	70=定时2启动 71=定时2停止 72=定时2设定压力
								 //			73=定时3启动 74=定时3停止
	400, 0, 0, 0, 0,			 // 75-79	75=定时3设定压力 76=定时4启动 77=定时4停止
								 //			78=定时4设定压力 79=定时5启动
	0, 0, 0, 0, 0,				 // 80-84	80=定时5停止 81=定时5设定压力 82=定时6启动
								 //			83=定时6停止 84=定时6设定压力
	0, 0, 0, 0, 0,				 // 85-89	85=密码停机口令 86=密码停机月日 87=进水口压力传感器初值
								 //			88=出水口压力传感器初值 89=小流量不恒压减1使能
	400, 3, 3, 0, 30,			 // 90-94	90-93消防
								 //			93=消防不退泵使能 94=进水口大于出水口压力延时
	30, 1, 1, 1, 1,				 // 95-99	95=最大供水能力延时 96=进水口大于设定压力使能
								 //			97=进水口大于出水口压力使能
								 //			98=最大供水能力使能 99=定时交换使能
								 //
	0, 0, 0, 0, 0,				 // 100-104	100=     101=1#电机当前脉冲  102=2#电机当前脉冲
								 //			103=3#电机当前脉冲 104=4#电机当前脉冲
	20, 0, 0, 0, 0,				 // 105=开关量稳定数目	106=2#伺服电机当前位置
								 //			107=3#伺服电机当前位置
								 //			108=    109=
	0, 0, 0, 0, 0,				 // 110-114	110=  111=1#电机当前执行的命令序号 112=2#电机当前执行的命令序号
								 //			113=3#电机当前执行的命令序号 114=4#电机当前执行的命令序号
	0, 0, 0, 0, 0,				 // 115-119	115=1#伺服电机当前执行的命令序号 116=1#伺服控制命令OK标志
								 //			117=2#伺服控制命令OK标志
								 //			118=3#伺服控制命令OK标志 119=4#伺服控制命令OK标志
	0, 0, 0, 0, 0,				 // 120-124	120=5#伺服控制命令OK标志 121=6#伺服控制命令OK标志 122=主机从机选择
								 //			123=1#伺服电机手动运行使能 124= 2#伺服电机手动运行使能
	200, 250, 0, 2400, 2400,	 // 125-129	125=下降PID_Ts 126=下降PID_Ti 127=0 0:0定点交换
								 //							//			128=2400(无定点) 129=2400(无定点)
	15, 12, 2000, 300, 5000,	 // 130-134	130=变频转工频偏差 131=工频退出偏差 132=2000 流量基值
								 //			133=最小流量300 134=最大流量 5000

	3, 7, 180, 1, 70,			  // 135-139	135=最小压力300 136=最大压力500 137=变量变压使能0
								  //			138=变频转工频使能 139= Cpu自热效应温度
	5, 57600, 57600, 57600, 1800, // 140-144	140=压力传感器保护延时 秒 141=串口1波特率 142=口2波特率  ZCL 2018.12.8
								  //			143=口3波特率 144= 小流量破坏清除秒
	0, 17, 600, 1, 1,			  // 145-149	145= 146= 147=
								  //			148= 149=
								  //
	50, 20, 57600, 57600, 2,	  // 150-154	150= 151= 152=
								  //			153= 154=
	1, 1, 0, 0, 1,				  // 155-159	155= 156= 157=
								  //			158= 159=1#伺服电机Modbus地址
	2, 3, 4, 5, 6,				  // 160-164	160=2#伺服电机Modbus地址 161=3#伺服电机Modbus地址 162=4#伺服电机Modbus地址
								  //			163=5#伺服电机Modbus地址 164=6#伺服电机Modbus地址
	2, 1, 50, 250, 60,			  // 165-169	165= 166=  166=
								  //			168=  169=
	1, 500, 50, 8760, 5,		  // 170-174	170= 171= 172=
								  //			173= 174=
	400, 5, 1, 120, 0,			  // 175-179	175= 176= 177=
								  //			178= 179=运行指令计数
	0, 0, 0, 1, 1,				  // 180-184	180=运行多少圈设定 181=累计运行圈数 182=累计运行圈数_高字
								  //			183=Com5通讯类型 184=通讯数据写入错误停机功能，=1，停机；=0，不停机
	100, 50, 10, 1, 3,			  // 185-189	185= 186= 187=
								  //			188= 189=
	200, 50, 300, 1800, 0,		  // 190-194	190= 191= 192=
								  //			193= 194=
	2, 10, 10, 0, 0,			  // 195-199	195= 196= 197=
								  //			198= 199=
								  //
	10, 200, 20, 0, 400,		  // 200-204	200= 201= 202=
								  //			203= 204=
	0, 1, 1000, 1, 4,			  // 205-209	205= 206= 207=查询间隔时间，ms
								  //			208= 209=
	1, 1, 1, 1, 1,				  // 210-214	210= 211=COM1主动查询使能 212=COM2主动查询使能
								  //			213=COM3主动查询使能 214=COM4主动查询使能
	10, 10, 0, 0, 0,			  // 215-219	215= 216= 217=
								  //			218= 219=
	339, 0, 1, 0, 1,			  // 220-224	220= 221= 222=通讯故障停机功能，=1，停机；=0，不停机
								  //			223= 224=最大位置限制功能，=1，启用；=0，不启动
	15, 0, 0, 0, 0,				  // 225-229	225= 226=1#通讯故障标志 227=2#通讯故障标志
								  //			228=3#通讯故障标志 229=4#通讯故障标志
	0, 0, 10, 10, 10,			  // 230-234	230=5#通讯故障标志 231=6#通讯故障标志 232=1#电机最小位置
								  //			233=1#电机最大位置 234=2#电机最小位置
	10, 10, 10, 10, 10,			  // 235-239	235=2#电机最大位置 236=3#电机最小位置 237=3#电机最大位置
								  //			238=4#电机最小位置 239=4#电机最大位置
								  //---------------------------
	10, 10, 10, 10, 0,			  // 240-244	240=5#电机最小位置 241=5#电机最大位置 242=6#电机最小位置
								  //			243=6#电机最大位置 244=刹车状态
	1, 1, 0, 0, 0,				  // 245-249	245=刹车控制，=1，刹车；=0，不刹车  246=所有电机都停止标志 247=Com4通讯故障标志
								  //			248=刹车控制模式，=1，手动控制刹车；=0，自动控制刹车 249=
	0, 0, 0, 0, 0,				  // 250-254	250=  251= 252=
								  //			253= 254=

};	//0,   1,    2,    3,    4,	//出厂设置
	//5,   6,    7,    8,    9,	//出厂设置

//地址+5000访问对应
uc32 w_ParBootLst_Drive[START_CMD_ADDR] = {
	10000, 0, 10000, 0, 10000, // 0-4		0=1#手动运行脉冲（低字）,1=1#手动运行脉冲（高字）,2=2#手动运行脉冲（低字）
							   //			3=2#手动运行脉冲（高字）,4=3#手动运行脉冲（低字）
	0, 10000, 0, 10000, 0,	   // 5-9		5=3#手动运行脉冲（高字）,6=4#手动运行脉冲（低字）
							   //			7=4#手动运行脉冲（高字）  8=5#手动运行脉冲（低字） 	9=5#手动运行脉冲（高字）
	10000, 0, 0, 0, 0,		   // 10-14	9=6#手动运行脉冲（低字） 	10=6#手动运行脉冲（高字）  12=
							   //			13=  14=
	0, 0, 0, 0, 0,			   // 15-19	15=  16=  17=6#伺服电机状态字
							   //			18= =1，初始化串口  19=
	0, 0, 0, 0, 0,			   // 20-24	20=  21=1#手动运行使能（反转）  22=1#手动运行使能（反转）
							   //			23=1#手动运行使能（反转）  24=1#手动运行使能（反转）
	0, 0, 200, 1000, 0,		   // 25-29	25=1#手动运行使能（反转）  26=1#手动运行使能（反转）   27=1#手动运行速度
							   //			28=1#手动加减速时间 29=
	200, 1000, 0, 200, 1000,   // 30-34	30-2#手动运行速度	31=2#手动加减速时间
							   //			32=  33=3#手动运行速度 34=3#手动加减速时间
	0, 200, 1000, 0, 200,	   // 35-39	35=  36=4#手动运行速度 37=4#手动加减速时间
							   //			38=  39=5#手动运行速度
	1000, 0, 200, 1000, 0,	   // 40-44	40=5#手动加减速时间 41=
							   //			42=6#手动运行速度 43=6#手动加减速时间 44=1#手动运行使能
	0, 0, 0, 0, 0,			   // 45-49	45=2#手动运行使能 46=3#手动运行使能 47=4#手动运行使能
							   //			48=5#手动运行使能 49= 6#手动运行使能
	1, 0, 0, 0, 0,			   // 50-54	50=手动模式（1），自动模式（0） 51=  52=
							   //			53=  54=
	0, 0, 0, 0, 0,			   // 55-59	55=  56=    /57=
							   //			58=  /59=
	0, 0, 0, 0, 0,			   // 60-64	60=  61=  62=
							   //			63=伺服电机1的初始位置 64=
	0, 0, 0, 0, 0,			   // 65-69	65=伺服电机2的初始位置 66=   67=伺服电机3的初始位置
							   //			68=  69=伺服电机4的初始位置
	0, 0, 0, 0, 0,			   // 70-74	70=  71=伺服电机5的初始位置 72=伺服电机5的初始位置
							   //			73=伺服电机6的初始位置 74=
	1, 0, 0, 0, 0,			   // 75-79	75=发生故障停机功能  76=读当前位置  77=急停命令，=1，进行急停
							   //			78=复位命令，=1，进行复位  79=停机复位功能
	1, 180, 0, 0, 0,		   // 80-84	80=触摸 启动/停止 81=默认延时180s保存修改后的参数到FLASH中 82=1#伺服电机参数初始化标志
							   //			83=2#伺服电机参数初始化标志 84=3#伺服电机参数初始化标志
	0, 0, 0, 0x3880, 0x01,	   // 85-89	85=4#伺服电机参数初始化标志 86=5#伺服电机参数初始化标志 87=6#伺服电机参数初始化标志
							   //			88=复位最大限定值  89=
	0, 0, 0, 0, 0,			   // 90-94	90-1#伺服电机位置指令计数器 91= 92-2#伺服电机位置指令计数器
							   //			93= 94=3#伺服电机位置指令计数器
	0, 0, 0, 0, 0,			   // 95-99	95= 96=4#伺服电机位置指令计数器
							   //			97= 98=5#伺服电机位置指令计数器 99=
	0, 0, 0, 0, 0,			   // 100-104	100=6#伺服电机位置指令计数器     101=   102=
							   //			103=  104=
	0, 0, 0, 100, 0,		   // 105-  	106=
							   //			107= 			108=通讯故障延时判断 10ms   109=
	0, 0, 0, 0, 0,			   // 110-114	110=  111=1#电机当前执行的命令序号 112=2#电机当前执行的命令序号
							   //			113=3#电机当前执行的命令序号 114=4#电机当前执行的命令序号
	0, 0, 0, 0, 0,			   // 115-119	115=5#电机当前执行的命令序号 116=6#电机当前执行的命令序号
							   //			117=1#电机运行标志
							   //			118=2#电机运行标志 119=3#电机运行标志
	0, 0, 0, 0, 0,			   // 120-124	120=4#电机运行标志 121=5#伺服电机运行标志 122=6#伺服电机运行标志
							   //			123=1#伺服电机准备好标志 124= 2#伺服电机准备好标志
	0, 0, 0, 0, 0,			   // 125-129	125=3#伺服电机准备好标志 126=4#伺服电机准备好标志 127=5#伺服电机准备好标志
							   //			128=6#伺服电机准备好标志 129=1#伺服电机通讯计数
	0, 0, 0, 0, 0,			   // 130-134	130=2#伺服电机通讯计数  131=3#伺服电机通讯计数  132=4#伺服电机通讯计数
							   //			133=5#伺服电机通讯计数  134=6#伺服电机通讯计数
	0, 0, 0, 0, 0,			   // 135-139	135=1#伺服电机状态字 136=2#伺服电机状态字 137=3#伺服电机状态字
							   //			138=4#伺服电机状态字 139=5#伺服电机状态字
	0, 0, 0, 0, 0,			   // 140-144	140=6#伺服电机状态字 141=  142=
							   //			143=  144=
	0, 0, 0, 0, 0,			   // 145-149	145=  146=  147=1#伺服电机当前最高级别故障码
							   //			148=2#伺服电机当前最高级别故障码 149=3#伺服电机当前最高级别故障码
	0, 0, 0, 2, 1,			   // 150-154	150=4#伺服电机当前最高级别故障码 151=5#伺服电机当前最高级别故障码 152=6#伺服电机当前最高级别故障码
							   //			153=COM延时1  154=COM延时2
	0, 0, 0, 0, 0,			   // 155-159	155=   156=   157=
							   //			158=   159=
	0, 0, 0, 0, 0,			   // 160-164	160=  161=1#伺服电机编码器单圈数据  162=2#伺服电机编码器单圈数据（高字）
							   //			163=2#伺服电机编码器单圈数据  164=3#伺服电机编码器单圈数据（高字）
	0, 0, 0, 0, 0,			   // 165-169	165=3#伺服电机编码器单圈数据  166=4#伺服电机编码器单圈数据（高字）  167=4#伺服电机编码器单圈数据
							   //			168=1#伺服电机编码器单圈数据（高字）  169=5#伺服电机编码器单圈数据
	0, 0, 0, 0, 0,			   // 170-174	170=5#伺服电机编码器单圈数据（高字）    171=6#伺服电机编码器单圈数据  172=6#伺服电机编码器单圈数据（高字）
							   //			173=1#伺服电机编码器多圈数据  174=2#伺服电机编码器多圈数据
	0, 0, 0, 0, 0,			   // 175-179	175=3#伺服电机编码器多圈数据  176=4#伺服电机编码器多圈数据  177=5#伺服电机编码器多圈数据
							   //			178=6#伺服电机编码器多圈数据  179=1#伺服电机编码器单圈初始设定值

	0, 0, 0, 0, 0, // 180-184	180=1#伺服电机编码器单圈初设定值（高字）    181=2#伺服电机编码器单圈初始设定值  182=2#伺服电机编码器单圈初设定值（高字）
				   //			183=3#伺服电机编码器单圈初始设定值  184=3#伺服电机编码器单圈初设定值（高字）
	0, 0, 0, 0, 0, // 185-189	185=4#伺服电机编码器单圈初始设定值  186=4#伺服电机编码器单圈初设定值（高字）  187=5#伺服电机编码器单圈初始设定值
				   //			188=5#伺服电机编码器单圈初设定值（高字）  189=6#伺服电机编码器单圈初始设定值

	0, 0, 0, 0, 0,				  // 190-194	190=6#伺服电机编码器单圈初设定值（高字）    191=1#伺服电机编码器多圈初始设定值  192=2#伺服电机编码器多圈初始设定值
								  //			193=3#伺服电机编码器多圈初始设定值  194=4#伺服电机编码器多圈初始设定值
	0, 0, 200, 5000, 0,			  // 195-199	195=5#伺服电机编码器多圈初始设定值  196=6#伺服电机编码器多圈初始设定值  197= 复位到初始位置延时时间，10ms
								  //			198=位置偏差设定  199=位置偏差设定（高字）
	0, 0, 0, 0, 0,				  // 200-204	200=   201=1#伺服电机P8910参数  202=2#伺服电机P8910参数
								  //			203=3#伺服电机P8910参数  204=4#伺服电机P8910参数
	0, 0, 0, 0, 0,				  // 205-209	205=5#伺服电机P8910参数  206=6#伺服电机P8910参数  207= 所有伺服电机写位置参数使能
								  //			208=1#伺服电机写位置参数使能  209=2#伺服电机写位置参数使能
	0, 0, 0, 0, 1000,			  // 210-214	210=3#伺服电机写位置参数使能   211=4#伺服电机写位置参数使能  212=5#伺服电机写位置参数使能
								  //			213=6#伺服电机写位置参数使能  214=1#伺服电机设定最大运行速度
	1000, 1000, 1000, 1000, 1000, // 215-219	215=2#伺服电机设定最大运行速度  216=3#伺服电机设定最大运行速度  217=4#伺服电机设定最大运行速度
								  //			218=5#伺服电机设定最大运行速度  219=6#伺服电机设定最大运行速度
	5, 6, 0, 0, 0,				  // 220-224	220=COM1延时3  10ms 221=COM延时，手动 10ms     222=定义并保存位置命令
								  //			223=计算位置及速度命令   224=校验位置命令
	0, 0, 0, 1000, 0,			  // 225-229	225=正在运行的命令号   226=手动运行命令   227=当前位置号[1-15]
								  //			228=基准运行速度   229=1，读原点位置命令
	0, 0, 0, 0, 0,				  // 230-234	230=1#命令队列中的条数   231=2#命令队列中的条数   232=3#命令队列中的条数
								  //			233=4#命令队列中的条数   234=5#命令队列中的条数
	0, 1000, 0, 0, 0,			  // 235-239	235=6#命令队列中的条数   236=停止状态检测延时   237=加减速时间延时比例
								  //			238=当前指令运行时间   239=设备状态
	0, 0, 0, 0, 0,				  // 240-244	240=1#当前运行速度   241=2#当前运行速度   242=3#当前运行速度
								  //			243=4#当前运行速度   244=5#当前运行速度
	0, 0, 0, 0, 3,				  // 245-249	245=6#当前运行速度   246=设备状态，低字    247=设备状态，高字
								  //			248=运行模式，=0，喷漆模式；=1，清洗模式   249=继电器板通讯地址
	0, 0, 0, 300, 0,			  // 250-254	250=继电器板DO1控制字    251=继电器板DO1状态字    252=Com4通讯计数
								  //			253=开刹车延时    254=
	0, 0, 20, 40, 1,			  // 255-259	255=    256=     257=程序循环计算次数
								  //			258=电机的最小运行速度    259= 1#电机从没有运行标志
	1, 1, 1, 1, 1,				  // 260-264	260=2#电机从没有运行标志    261=3#电机从没有运行标志    262=4#电机从没有运行标志
								  //			263=5#电机从没有运行标志    264= 6#电机从没有运行标志
	0, 0, 0, 0, 0,				  // 265-269	265=1#电机编码器位置偏差，多圈    266=2#电机编码器位置偏差，多圈     267=3#电机编码器位置偏差，多圈
								  //			268=4#电机编码器位置偏差，多圈     269=5#电机编码器位置偏差，多圈
	0, 0, 0, 0, 0,				  // 270-274	270=6#电机编码器位置偏差，多圈    271=1#电机编码器位置偏差，单圈，低位    272=1#电机编码器位置偏差，单圈，高位
								  //			273=2#电机编码器位置偏差，单圈，低位     274=2#电机编码器位置偏差，单圈，高位
	0, 0, 0, 0, 0,				  // 275-279	275=3#电机编码器位置偏差，单圈，低位     276=3#电机编码器位置偏差，单圈，高位      277=4#电机编码器位置偏差，单圈，低位
								  //			278=4#电机编码器位置偏差，单圈，高位      279=5#电机编码器位置偏差，单圈，低位
	0, 0, 0, 0, 50,				  // 280-284	280=5#电机编码器位置偏差，单圈，高位    281=6#电机编码器位置偏差，单圈，低位    282=6#电机编码器位置偏差，单圈，高位
								  //			283=电机编码器位置偏差手动调整命令     284=判断所有电机位置到达延时
	2, 0, 0, 0, 0,				  // 285-289	285=写位置指令超时时间设定 s    286=发送数据标志      287=1#伺服发送数据OK标志
								  //			288=2#伺服发送数据OK标志      289=3#伺服发送数据OK标志
	0, 0, 0, 0, 0,				  // 290-294	290=4#伺服发送数据OK标志    291=5#伺服发送数据OK标志   292=6#伺服发送数据OK标志
								  //			293=所有伺服发送数据OK标志     294=有伺服发送数据错误标志
	0, 0, 0, 0, 0,				  // 295-299	295=超出位置范围标志    296=      297=
								  //			298=      299=
};								  //0,   1,    2,    3,    4,	//出厂设置
								  //5,   6,    7,    8,    9,	//出厂设置
/* Private functions ---------------------------------------------------------*/
//
void ReadWriteRealTime(void) // 读写实时时钟 ISL1208
{
	uint8_t i;
	uint16_t k;
	uint8_t RWBuf[7]; //I2C 读写缓存
	// 读时钟
	if (T_ReadRealTime != SClk10Ms && !F_ModRealTime)
	{
		T_ReadRealTime = SClk10Ms; //
		C_ReadRealTime++;
		if (C_ReadRealTime > 20)
		{
			T_ReadRealTime = 100; //
			C_ReadRealTime = 0;
			//Read1302Time(RWBuf);	// Read address 0x00 on ISL1208 会影响串口速度

			//			BurstRead1302Clock(RWBuf);

			//RWBuf[2] &=0x7f;				// 小时
			for (i = 0; i < 7; i++) // 16进制转10进制给显示，修改时10进制转16进制
			{
				RWBuf[i] = RWBuf[i] / 16 * 10 + (RWBuf[i] % 16);
			}
			//
			for (i = 0; i < 5; i++) // 7---19  读时间7个寄存器内容
			{
				w_ParLst[i + 45] = RWBuf[i];
				RealClock[i] = RWBuf[i];
			}
			// Pw_SetWeek = RWBuf[5];		  // 星期
			// Pw_SetYear = RWBuf[6] + 2000; // 年＋2000
			// RealClock[6] = RWBuf[5];	  //星期
			// RealClock[5] = RWBuf[6];	  //年
		}
	}

	// 修改时钟的具体处理操作
	if (F_ModRealTime)
	{
		if (T_ModRealTime != SClk10Ms)
		{
			T_ModRealTime = SClk10Ms; //
			C_ModRealTime++;
			if (C_ModRealTime > 10 && S_ModRealTime == 0)
			{
				RWBuf[0] = 0x90;
				//I2C_EE_ByteWrite(RWBuf, 0x07);	// 允许写时钟
				S_ModRealTime = 1;
			}
			if (C_ModRealTime > 20 && S_ModRealTime == 1)
			{
				k = w_ParLst[45 + w_ModRealTimeNo]; // k 为修改内容
				//
				if (w_ModRealTimeNo == 5) // 年
				{
					k = k - 2000;
					w_ModRealTimeNo++; //周成磊 2015.5.14 用于DS1302
				}
				else if (w_ModRealTimeNo == 6) //周
				{
					w_ModRealTimeNo = w_ModRealTimeNo - 1;
				}
				else if (w_ModRealTimeNo == 2)
				{
					//k |=0x80;  ISL1208 .7=1为24小时制
				}

				k = k / 10 * 16 + (k % 10); // 修改时16进制转10进制

				RWBuf[0] = k;
				//				Write1302ByteTime(w_ModRealTimeNo,RWBuf[0]);		// 修改时钟
				S_ModRealTime = 2;
			}
			if (C_ModRealTime > 30 && S_ModRealTime == 2)
			{
				RWBuf[0] = 0x80;
				//I2C_EE_ByteWrite(RWBuf, 0x07);	// 禁止写
				S_ModRealTime = 0;
				T_ModRealTime = 100; //
				C_ModRealTime = 0;
				F_ModRealTime = 0;
			}
		}
	}
}

void Variable_Init(void) //	变量初始化
{
	int i;
	s32 *arrp;

	w_SoftVer = 100;	// 软件版本号 VERSION
	w_WriteDate = 2021; //程序编写日期
	w_Writetime = 0516; //程序编写时间
						//	w_GongSiSelect=0;               //  公司选择，0=中de美，1=三利集团

	F_ManualRunStop = 1;

	F_Starting = 0;
	F_Stoping = 0;
	F_AskReset = 0;
	F_Reseting = 0;
	F_AskForceReset = 0;
	F_ForceReseting = 0;
	F_TouchForceReSet = 0;

	F_PowerOnRun = 1;
	F_AskStop = 1;
	Pw_StepAutoMode = 0; //上电，=0，自动状态
	F_StepMode = 0;

	Pw_TouchRunStop = 1; //上电，=1，停止
	F_TouchRunStop = 1;

	arr_p1 = &w_ParLst_Pos_CMD; //指向位置命令队列头
	arrp_p1_Last = arr_p1;
	Pr_Driver_Running_No = 0;
	Pr_Driver_Previous_No = 0; //前一个执行指令号

	Pr_F_Drive1_Runing = 0;
	Pr_F_Drive2_Runing = 0;
	Pr_F_Drive3_Runing = 0;
	Pr_F_Drive4_Runing = 0;
	Pr_F_Drive5_Runing = 0;
	Pr_F_Drive6_Runing = 0;

	F_Encoder_Read = 0; //=0，编码器初始数据还未读取
	Pw_ResetCMD = 0;
	Pw_Read_CurrentPos = 0; //=1,读当前位置
	Pw_Read_Init_Pos = 0;	//=1,读原点位置

	Pr_Drive1_Status = 0;
	Pr_Drive2_Status = 0;
	Pr_Drive3_Status = 0;
	Pr_Drive4_Status = 0;
	Pr_Drive5_Status = 0;
	Pr_Drive6_Status = 0;

	Pr_Driver1_ComCount = 0;
	Pr_Driver2_ComCount = 0;
	Pr_Driver3_ComCount = 0;
	Pr_Driver4_ComCount = 0;
	Pr_Driver5_ComCount = 0;
	Pr_Driver6_ComCount = 0;

	Pr_Drive1_FaultNo = 0;
	Pr_Drive2_FaultNo = 0;
	Pr_Drive3_FaultNo = 0;
	Pr_Drive4_FaultNo = 0;
	Pr_Drive5_FaultNo = 0;
	Pr_Drive6_FaultNo = 0;

	F_AllRdy = 0;
	Pr_F_AllStopped = 1; //上电默认所有电机都停止
	Pr_F_HaveFault = 0;
	Pw_Step_Pos_CMD = 0; //单步运行模式=0

	Pr_RUN_Count = 0;
	Pr_BRAKE_Control = 1; //上电默认刹车
						  //	START_BRAKE_SYSTEM;				//刹车

	T_Driver1_delay = 0;
	C_Driver1_delayCount = 0;
	T_Driver2_delay = 0;
	C_Driver2_delayCount = 0;
	T_Driver3_delay = 0;
	C_Driver3_delayCount = 0;
	T_Driver4_delay = 0;
	C_Driver4_delayCount = 0;
	T_Driver5_delay = 0;
	C_Driver5_delayCount = 0;
	T_Driver6_delay = 0;
	C_Driver6_delayCount = 0;

	Pw_EquipStatus = 0; //设备状态

	Driver1_Write_Sort = 0; //驱动器1写位置段顺序，0写第1段，1写第2段
	Driver2_Write_Sort = 0;
	Driver3_Write_Sort = 0;
	Driver4_Write_Sort = 0;
	Driver5_Write_Sort = 0;
	Driver6_Write_Sort = 0;

	Pw_Run_Mode = 0; //=0，默认为喷漆模式
	F_SendStopCMD2 = 0;

	arrp = &w_ParLst_PosPar;
	for (i = 1; i <= 15; i++)
	{
		arrp[POS_SIZE * (i - 1)] = i; //位置序列中第1个字即为位置序号，共15个位置,1-15
	}

	arrp = &w_ParLst_Pos_CMD;
	for (i = 1; i <= POS_CMD_NUM; i++)
	{
		arrp[POS_CMD_SIZE * (i - 1)] = i; //命令序列中第1个字即为序号，共30个命令,1-30
	}

	Pr_Driver1_NeverRun = 1;
	Pr_Driver2_NeverRun = 1;
	Pr_Driver3_NeverRun = 1;
	Pr_Driver4_NeverRun = 1;
	Pr_Driver5_NeverRun = 1;
	Pr_Driver6_NeverRun = 1;

	Pr_Driver1_Control_OK_F = 0;
	Pr_Driver2_Control_OK_F = 0;
	Pr_Driver3_Control_OK_F = 0;
	Pr_Driver4_Control_OK_F = 0;
	Pr_Driver5_Control_OK_F = 0;
	Pr_Driver6_Control_OK_F = 0;

	Pr_AllDriver_Cmd_OK_F = 0;
	Pr_HaveDriver_Cmd_Err_F = 0;
	Pr_OverMaxPos_F = 0;

	C_NoRcv1Count = 0;
	C_NoRcv2Count = 0;
	C_NoRcv3Count = 0;
	C_NoRcv4Count = 0;
	C_NoRcv5Count = 0;
	C_NoRcv6Count = 0;
}

// RAM中参数表列 初始化 (读出)
void ParLst_Init(void)
{
	//	//从铁电存储器FM25L16中读出设定参数、记录号参数、水泵累计运行参数
	//	B_SelCS=CS_FMRAM1;
	//	SPI_FMRAM_BufferRead((uint8_t *)(&w_ParLst[0]), 0, FLASH_PAR_SIZE*2);			// 从FMRAM的地址0开始读出设定参数，255个字
	//	SPI_FMRAM_BufferRead((uint8_t *)(&w_ParLst[512]), 1024, FLASH_ID_SIZE*2);		// 读出电话号码等参数，从地址1024开始，读128个字

	//	//从FLASH ROM中读取保存的参数，写到w_ParLst_Drive中，方向：——>
	//	STMFLASH_Read(FLASH_SAVE_ADDR0,(uint32_t*)&w_ParLst_Drive,START_CMD_ADDR);
	//
	//	//读取位置信息
	//	STMFLASH_Read(FLASH_SAVE_POSTION1,(uint32_t*)&w_ParLst_PosPar,FLASH_POS_SIZE);
	//
	//	//读取位置控制指令
	//	//从FLASH_SAVE_POS_CMD1读出来，写入到w_ParLst_Pos_CMD中		方向：——>
	//	if(Pos_Group_Select>5 || Pos_Group_Select<1)
	//	{
	//		Pos_Group_Select=1;
	//		Old_Pos_Group_Select=1;
	//	}
	//	STMFLASH_Read(FLASH_SAVE_POS_CMD1+0X00001000*(Pos_Group_Select-1),(uint32_t*)&w_ParLst_Pos_CMD,FLASH_POS_CMD_SIZE);
}

////从指定地址开始读出指定长度的数据
////ReadAddr:起始地址
////pBuffer:数据指针
////NumToWrite:半字(16位)数
//void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint16_t NumToRead)
//{
//	uint16_t i;
//	for(i=0;i<NumToRead;i++)
//	{
//		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取2个字节.
//		ReadAddr+=4;//偏移2个字节.
//	}
//}

void ParArrayRead(uint16_t *p_Top, uc16 *p_Base, uint16_t w_ReadSize)
{
	uint16_t i;
	for (i = 0; i < w_ReadSize; i++)
	{
		*p_Top++ = *p_Base++; // zcl 地址自动加2！
	}
}

void ParArrayRead_Word(uint32_t *p_Top, uc32 *p_Base, uint w_ReadSize)
{
	uint16_t i;
	for (i = 0; i < w_ReadSize; i++)
	{
		*p_Top++ = *p_Base++; // zcl 地址自动加4！
	}
}

//void ParArrayRead_DWord(uint32_t *p_Top,uc32 *p_Base, uint16_t w_ReadSize)
//{
//	uint16_t i;
//	for (i=0;i<w_ReadSize ;i++ )
//	{
//		*p_Top++ = *p_Base++;	// zcl 地址自动加4！
//	}
//}

//恢复设定参数为出厂值
//只有STMFLASH_Read函数，才是从左往右写，——>
//其他的，如STMFLASH_Write，ParArrayRead，都是从右往左写，<——
void Boot_ParLst(void) // 恢复设定参数为出厂值
{
	//	uint8_t  i;
	//	uint8_t	tmp_Group_Pos;
	//
	//	i=0;
	//
	//	//如果!=0x5A，则进行初始化
	//	if(Pw_Initial_F!=0x5A)
	//	{
	//		Pw_ModPar=2000;
	//		Pw_ParInitial=4321;
	//		S_BootParLst=0;
	//		Pw_Initial_F=0x5A;
	//	}
	//	// Pw_ModPar=2000,可以修改参数// Pw_ParInitial==4321 初始化参数
	//	if ( Pw_ModPar==2000 && Pw_ParInitial==4321 && S_BootParLst==0 )
	//	{
	//		i=1;
	//	}
	//
	//	if ( i==1 )
	//	{
	//		//初始化系统参数
	//		//RAM<——程序数组
	//		//限制命令组号
	//		if(Pos_Group_Select>5 || Pos_Group_Select<1)
	//			Pos_Group_Select=1;
	//
	//		tmp_Group_Pos=Pos_Group_Select;
	//		ParArrayRead(w_ParLst,w_ParBootLst,FLASH_PAR_SIZE);		// 从程序数组中读出初始化参数,255个字
	//		Pos_Group_Select=tmp_Group_Pos;

	//		//初始化驱动参数，并保存在FLASH中
	//		//FLASH<——程序数组
	//		STMFLASH_Write(FLASH_SAVE_ADDR0,(uint32_t*)w_ParBootLst_Drive,START_CMD_ADDR);				//把程序数组中的值写入到FLASH中
	//		//RAM<——程序数组
	//		ParArrayRead_Word((uint32_t*)&w_ParLst_Drive,(uint32_t*)&w_ParBootLst_Drive,START_CMD_ADDR);				// 读出初始化参数
	//
	//		arr_p1=&w_ParLst_Pos_CMD;
	//		arrp_p1_Last=arr_p1;
	////		arr_p1[37]=0;											//发送标志清0，可以发送位置

	//		// 暂不保存参数，当Pw_ModPar==5000，可以保存参数到FLASH
	//		Pw_ModPar=2000;
	//		Pw_ParInitial=4321;
	//		S_BootParLst=1;
	//	}

	//	//
	//	if( T_BootParLst!=SClk10Ms && S_BootParLst!=0 ) // 用于MD304L提示状态，人性化设计 ZCL
	//	{
	//		T_BootParLst=SClk10Ms;			// 现在Pw_ParInitial=4321
	//		C_BootParLst++;
	//		if ( C_BootParLst>150 && S_BootParLst==1 )
	//		{
	//			S_BootParLst=2;
	//			Pw_ParInitial=6000;
	//		}
	//		else if ( C_BootParLst>300 && S_BootParLst==2 )
	//		{
	//			T_BootParLst=100;
	//			C_BootParLst=0;
	//			S_BootParLst=0;
	//			Pw_ParInitial=0;
	//		}
	//	}
}

//只按组进行初始化位置命令，从保存在FLASH中的某个组中调入到RAM中
//对位置信息不初始化
void ParLst_Init_Group(void)
{
	//	if(Pw_ModPar==2000 && Pw_ParInitial==4000)
	//	{
	//		//限制命令组号
	//		if(Pos_Group_Select>5 || Pos_Group_Select<1)
	//			Pos_Group_Select=1;
	//		//FLASH——>RAM
	//		STMFLASH_Read(FLASH_SAVE_POS_CMD1+0X00001000*(Pos_Group_Select-1),(uint32_t*)&w_ParLst_Pos_CMD,FLASH_POS_CMD_SIZE);
	//
	//		Pw_ModPar=0;
	//		Pw_ParInitial=0;
	//	}
}

//清零位置
void ParLst_Init_Pos2Zero(void)
{
	//	if(Pw_ModPar==2000 && Pw_ParInitial==5432)
	//	{
	//		//清零位置
	//		//FLASH<——程序数组
	//		STMFLASH_Write(FLASH_SAVE_POSTION1,(uint32_t*)Pos_Init,FLASH_POS_SIZE);
	//		//RAM<——程序数组，初始化位置，清零
	//		ParArrayRead_Word((uint32_t*)&w_ParLst_PosPar,(uint32_t*)Pos_Init,FLASH_POS_SIZE);
	//
	//		Pw_ModPar=0;
	//		Pw_ParInitial=0;
	//	}
}

void SavePar_Prompt(void) // 保存参数+状态提示
{
	//	if ( B_ForceSavPar==1 && S_SavePar==0 )		// 强制保存参数
	//	{
	//		Pw_ModPar=0;		// 防止把 Pw_ModPar==5000 保存到FMRAM
	//		B_SelCS=CS_FMRAM1;
	//		SPI_FMRAM_BufferWrite((uint8_t *)(&w_ParLst[0]), 0, FLASH_PAR_SIZE*2);
	//		SPI_FMRAM_BufferWrite((uint8_t *)(&w_ParLst[512]), 1024, FLASH_ID_SIZE*2);		//FLASH_ID_SIZE 128
	//		Pw_ModPar=18;
	//		// 再加修改端子配置 ZCL
	//		S_SavePar=1;
	//		B_ForceSavPar=0;

	//		//写入，方向：<——
	//		STMFLASH_Write(FLASH_SAVE_ADDR0,(uint32_t*)&w_ParLst_Drive,START_CMD_ADDR);		//把数组变量的值保存到FLASH中

	//		//保存位置信息
	//		STMFLASH_Write(FLASH_SAVE_POSTION1,(uint32_t*)&w_ParLst_PosPar,FLASH_POS_SIZE);
	//
	//		//限制命令组号
	//		if(Pos_Group_Select>5 || Pos_Group_Select<1)
	//			Pos_Group_Select=1;
	//		//保存位置指令
	//		//将w_ParLst_Pos_CMD保存到FLASH_SAVE_POS_CMD1中		方向：<——
	//		STMFLASH_Write(FLASH_SAVE_POS_CMD1+0X00001000*(Pos_Group_Select-1),(uint32_t*)&w_ParLst_Pos_CMD,FLASH_POS_CMD_SIZE);
	//	}
	//
	//	if ( Pw_ModPar==5000 && S_SavePar==0 )
	//	{
	//		Pw_ModPar=0;		// 防止把 Pw_ModPar==5000 保存到FMRAM
	//		B_SelCS=CS_FMRAM1;
	//		SPI_FMRAM_BufferWrite((uint8_t *)(&w_ParLst[0]), 0, FLASH_PAR_SIZE*2);			//FLASH_PAR_SIZE=255，255*2个字节，即255个字
	//		SPI_FMRAM_BufferWrite((uint8_t *)(&w_ParLst[512]), 1024, FLASH_ID_SIZE*2);		//FLASH_ID_SIZE 128
	//		Pw_ModPar=5000;
	//		// 再加修改端子配置 ZCL
	//		S_SavePar=1;

	//		//把RAM中的参数设定值保存到FLASH中
	//		STMFLASH_Write(FLASH_SAVE_ADDR0,(uint32_t*)&w_ParLst_Drive,START_CMD_ADDR);

	//		//保存位置信息到FALSH
	//		STMFLASH_Write(FLASH_SAVE_POSTION1,(uint32_t*)&w_ParLst_PosPar,FLASH_POS_SIZE);
	//
	//		//限制命令组号
	//		if(Pos_Group_Select>5 || Pos_Group_Select<1)
	//			Pos_Group_Select=1;
	//		//保存位置指令到FALSH
	//		//将w_ParLst_Pos_CMD保存到FLASH_SAVE_POS_CMD1中		方向：<——
	//		STMFLASH_Write(FLASH_SAVE_POS_CMD1+0X00001000*(Pos_Group_Select-1),(uint32_t*)&w_ParLst_Pos_CMD,FLASH_POS_CMD_SIZE);
	//	}

	//	//
	//	if( T_SavePar!=SClk10Ms && S_SavePar!=0 )		// 用于MD304L提示状态，人性化设计 ZCL
	//	{
	//		T_SavePar=SClk10Ms;			// Pw_ModPar=50
	//		C_SavePar++;
	//		if ( C_SavePar>150 && S_SavePar==1 )
	//		{
	//			S_SavePar=2;
	//			Pw_ModPar=6000;
	//		}
	//		else if ( C_SavePar>300 && S_SavePar==2 )
	//		{
	//			T_SavePar=100;
	//			C_SavePar=0;
	//			S_SavePar=0;
	//			Pw_ModPar=0;
	//		}
	//	}
}

// Pw_ModPar=2000,规定时间内（默认90s）没有修改和保存参数,则保存参数一次
void ForceSavePar(void)
{
	//	//停机状态下，才能自动保存
	//	if(Pw_TouchRunStop==1 && Pr_F_AllStopped!=0)
	//	{
	//		if ( Pw_ModPar==2000 && T_ForceSavPar!=SClkSecond)
	//		{
	//			T_ForceSavPar=SClkSecond;
	//			C_ForceSavPar++;
	//			if ( C_ForceSavPar>90 )		//90s
	//			{
	//				T_ForceSavPar=100;
	//				C_ForceSavPar=0;
	//				B_ForceSavPar=1;
	//			}
	//		}
	//	}
}

uint16_t T_ComErr;

void EquipStatus(void) // 设备状态
{
}

void KglStatus(void) // 开关量状态
{
	if (F_KglOutPrompt && T_KglOutPrompt != SClk10Ms)
	{
		T_KglOutPrompt = SClk10Ms; // 延时清除	F_AlarmStopPrompt标志
		C_KglOutPrompt++;
		if (C_KglOutPrompt > 100)
		{
			T_KglOutPrompt = 100;
			C_KglOutPrompt = 0;
			F_KglOutPrompt = 0;
		}
	}
}

void FilterDI(void) // 过滤开关量输入 2016.4.12
{
	//DI1
	if (!B_DI1)
	{
		if (DI1)
		{
			if (w_DI1StableCounter++ > Pw_DIStableSetNum) // 用循环次数过滤
			{
				w_DI1StableCounter = 0;
				DInb[0] = DInb[0] | 0x01;
				B_DI1 = 1;
			}
		}
		else if (w_DI1StableCounter > 0)
			w_DI1StableCounter--;
	}

	else
	{
		if (!DI1)
		{
			if (w_DI1StableCounter++ > Pw_DIStableSetNum) // 用循环次数过滤
			{
				w_DI1StableCounter = 0;
				DInb[0] = DInb[0] & 0xFE;
				B_DI1 = 0;
			}
		}
		else if (w_DI1StableCounter > 0)
			w_DI1StableCounter--;
	}
	//DI2
	if (!B_DI2)
	{
		if (DI2)
		{
			if (w_DI2StableCounter++ > Pw_DIStableSetNum) // 用循环次数过滤
			{
				w_DI2StableCounter = 0;
				DInb[0] = DInb[0] | 0x02;
				B_DI2 = 1;
			}
		}
		else if (w_DI2StableCounter > 0)
			w_DI2StableCounter--;
	}

	else
	{
		if (!DI2)
		{
			if (w_DI2StableCounter++ > Pw_DIStableSetNum) // 用循环次数过滤
			{
				w_DI2StableCounter = 0;
				DInb[0] = DInb[0] & 0xFD;
				B_DI2 = 0;
			}
		}
		else if (w_DI2StableCounter > 0)
			w_DI2StableCounter--;
	}
	//DI3
	if (!B_DI3)
	{
		if (DI3)
		{
			if (w_DI3StableCounter++ > Pw_DIStableSetNum) // 用循环次数过滤
			{
				w_DI3StableCounter = 0;
				DInb[0] = DInb[0] | 0x04;
				B_DI3 = 1;
			}
		}
		else if (w_DI3StableCounter > 0)
			w_DI3StableCounter--;
	}

	else
	{
		if (!DI3)
		{
			if (w_DI3StableCounter++ > Pw_DIStableSetNum) // 用循环次数过滤
			{
				w_DI3StableCounter = 0;
				DInb[0] = DInb[0] & 0xFB;
				B_DI3 = 0;
			}
		}
		else if (w_DI3StableCounter > 0)
			w_DI3StableCounter--;
	}
	//DI4
	if (!B_DI4)
	{
		if (DI4)
		{
			if (w_DI4StableCounter++ > Pw_DIStableSetNum) // 用循环次数过滤
			{
				w_DI4StableCounter = 0;
				DInb[0] = DInb[0] | 0x08;
				B_DI4 = 1;
			}
		}
		else if (w_DI4StableCounter > 0)
			w_DI4StableCounter--;
	}

	else
	{
		if (!DI4)
		{
			if (w_DI4StableCounter++ > Pw_DIStableSetNum) // 用循环次数过滤
			{
				w_DI4StableCounter = 0;
				DInb[0] = DInb[0] & 0xF7;
				B_DI4 = 0;
			}
		}
		else if (w_DI4StableCounter > 0)
			w_DI4StableCounter--;
	}
}

//开关量输出配置值子函数 ZCL
void DOConfigValue(uint8_t DOValue, uint8_t DO_BitNo) // DO_BitNo:DO 位号
{
	//DOValue: 要输出的继电器值；DO_BitNo:DO 位号，指定哪个继电器输出
	uint32_t BBDIValue;
	uint8_t j;
	BBDIValue = (uint32_t)(DOutb[(DO_BitNo - 1) / 8]);
	j = (DO_BitNo - 1) % 8;
	if (DOValue)
		MEM_ADDR(BITBAND((uint32_t)&BBDIValue, j)) = 1;
	else
		MEM_ADDR(BITBAND((uint32_t)&BBDIValue, j)) = 0;

	DOutb[(DO_BitNo - 1) / 8] = BBDIValue;
}

void ParLimit(void) // 参数限制
{
	if (Pw_ComBufType != 1 && Pw_ComBufType != 2) // 通讯协议类型 2007.11.1
	{
		Pw_ComBufType = 1;
	}

	//波特率限制
	if (Pw_BaudRate1 != 2400 && Pw_BaudRate1 != 4800 && Pw_BaudRate1 != 9600 && Pw_BaudRate1 != 19200 && Pw_BaudRate1 != 38400 && Pw_BaudRate1 != 57600)
	{
		Pw_BaudRate1 = 57600; // 串口1波特率
	}
	if (Pw_BaudRate2 != 2400 && Pw_BaudRate2 != 4800 && Pw_BaudRate2 != 9600 && Pw_BaudRate2 != 19200 && Pw_BaudRate2 != 38400 && Pw_BaudRate2 != 57600)
	{
		Pw_BaudRate2 = 57600; // 串口2波特率
	}
	if (Pw_BaudRate3 != 2400 && Pw_BaudRate3 != 4800 && Pw_BaudRate3 != 9600 && Pw_BaudRate3 != 19200 && Pw_BaudRate3 != 38400 && Pw_BaudRate3 != 57600)
	{
		Pw_BaudRate3 = 57600; // 串口3波特率
	}
	if (Pw_BaudRate4 != 2400 && Pw_BaudRate4 != 4800 && Pw_BaudRate4 != 9600 && Pw_BaudRate4 != 19200 && Pw_BaudRate4 != 38400 && Pw_BaudRate4 != 57600)
	{
		Pw_BaudRate4 = 57600; // 串口4波特率
	}
	if (Pw_BaudRate5 != 2400 && Pw_BaudRate5 != 4800 && Pw_BaudRate5 != 9600 && Pw_BaudRate5 != 19200 && Pw_BaudRate5 != 38400 && Pw_BaudRate5 != 57600)
	{
		Pw_BaudRate5 = 57600; // 串口5波特率
	}

	if (Pr_Driver1_ComCount > 65535)
	{
		Pr_Driver1_ComCount = 0;
	}

	if (Pr_Driver2_ComCount > 65535)
	{
		Pr_Driver2_ComCount = 0;
	}

	if (Pr_Driver3_ComCount > 65535)
	{
		Pr_Driver3_ComCount = 0;
	}

	if (Pr_Driver4_ComCount > 65535)
	{
		Pr_Driver4_ComCount = 0;
	}

	if (Pr_Driver5_ComCount > 65535)
	{
		Pr_Driver5_ComCount = 0;
	}

	if (Pr_Driver6_ComCount > 65535)
	{
		Pr_Driver6_ComCount = 0;
	}

	//限制命令组号
	if (Pos_Group_Select > 5 || Pos_Group_Select < 1)
		Pos_Group_Select = 1;

	//限制当前位置号
	if (Pw_Current_Pos_No < 1)
		Pw_Current_Pos_No = 1;

	if (Pw_Current_Pos_No > 15)
		Pw_Current_Pos_No = 15;

	//限制当前命令号
	if (Pw_Running_Pos_CMD < 1)
		Pw_Running_Pos_CMD = 1;

	if (Pw_Running_Pos_CMD > COM_CMD_NUM)
		Pw_Running_Pos_CMD = COM_CMD_NUM;

	//限制通讯错误超时时间
	if (Pw_Write_Timeout_Set < 1)
		Pw_Write_Timeout_Set = 1;

	if (Pw_Write_Timeout_Set > 500)
		Pw_Write_Timeout_Set = 500;

	if (Pw_EquipmentNo1 == 0)
		Pw_EquipmentNo1 = 1;

	if (Pw_EquipmentNo2 == 0)
		Pw_EquipmentNo2 = 1;

	if (Pw_EquipmentNo3 == 0)
		Pw_EquipmentNo3 = 1;

	if (Pw_EquipmentNo4 == 0)
		Pw_EquipmentNo4 = 4;

	if (Pw_EquipmentNo5 == 0)
		Pw_EquipmentNo5 = 5;

	if (Pw_EquipmentNo6 == 0)
		Pw_EquipmentNo6 = 6;

	//--------1# Start--------------------------------------------------------
	if (Pw_Driver1_Pluse_HW == 0 && Pw_Driver1_Pluse == 0)
		Pw_Driver1_Pluse = 10000; //发送的脉冲数

	if (Pw_Driver1_AccTime == 0)
		Pw_Driver1_AccTime = 100; //加速度

	if (Pw_Driver1_Speed == 0)
		Pw_Driver1_Speed = 1000; //设定速度

	if (Pw_Motor1_PULSENUM == 0)
		Pw_Motor1_PULSENUM = 10000; //每圈脉冲数

	if (Pw_Motor1_FRE_START == 0)
		Pw_Motor1_FRE_START = 10000; //起始频率

	if (Pw_Motor1_FRE_AA == 0)
		Pw_Motor1_FRE_AA = 10000; //加加速度

	if (Pw_Motor1_STEP_PARA == 0)
		Pw_Motor1_STEP_PARA = 500; //步进步数修正因子

	if (Pw_Motor1_maxposition == 0)
		Pw_Motor1_maxposition = 8388608; //最大位置

	//--------2# Start--------------------------------------------------------
	if (Pw_Driver2_Pluse_HW == 0 && Pw_Driver2_Pluse == 0)
		Pw_Driver2_Pluse = 10000; //发送的脉冲数

	if (Pw_Driver2_AccTime == 0)
		Pw_Driver2_AccTime = 100; //加速度

	if (Pw_Driver2_Speed == 0)
		Pw_Driver2_Speed = 1000; //设定速度

	if (Pw_Motor2_PULSENUM == 0)
		Pw_Motor2_PULSENUM = 10000; //每圈脉冲数

	if (Pw_Motor2_FRE_START == 0)
		Pw_Motor2_FRE_START = 10000; //起始频率

	if (Pw_Motor2_FRE_AA == 0)
		Pw_Motor2_FRE_AA = 10000; //加加速度

	if (Pw_Motor2_STEP_PARA == 0)
		Pw_Motor2_STEP_PARA = 500; //步进步数修正因子

	if (Pw_Motor2_maxposition == 0)
		Pw_Motor2_maxposition = 8388608; //最大位置

	//--------3# Start--------------------------------------------------------
	if (Pw_Driver3_Pluse_HW == 0 && Pw_Driver3_Pluse == 0)
		Pw_Driver3_Pluse = 10000; //发送的脉冲数

	if (Pw_Driver3_AccTime == 0)
		Pw_Driver3_AccTime = 100; //加速度

	if (Pw_Driver3_Speed == 0)
		Pw_Driver3_Speed = 1000; //设定速度

	if (Pw_Motor3_PULSENUM == 0)
		Pw_Motor3_PULSENUM = 10000; //每圈脉冲数

	if (Pw_Motor3_FRE_START == 0)
		Pw_Motor3_FRE_START = 10000; //起始频率

	if (Pw_Motor3_FRE_AA == 0)
		Pw_Motor3_FRE_AA = 10000; //加加速度

	if (Pw_Motor3_STEP_PARA == 0)
		Pw_Motor3_STEP_PARA = 500; //步进步数修正因子

	if (Pw_Motor3_maxposition == 0)
		Pw_Motor3_maxposition = 8388608; //最大位置

	//--------4# Start--------------------------------------------------------
	if (Pw_Driver4_Pluse_HW == 0 && Pw_Driver4_Pluse == 0)
		Pw_Driver4_Pluse = 10000; //发送的脉冲数

	if (Pw_Driver4_AccTime == 0)
		Pw_Driver4_AccTime = 100; //加速度

	if (Pw_Driver4_Speed == 0)
		Pw_Driver4_Speed = 1000; //设定速度

	if (Pw_Motor4_PULSENUM == 0)
		Pw_Motor4_PULSENUM = 10000; //每圈脉冲数

	if (Pw_Motor4_FRE_START == 0)
		Pw_Motor4_FRE_START = 10000; //起始频率

	if (Pw_Motor4_FRE_AA == 0)
		Pw_Motor4_FRE_AA = 10000; //加加速度

	if (Pw_Motor4_STEP_PARA == 0)
		Pw_Motor4_STEP_PARA = 500; //步进步数修正因子

	if (Pw_Motor4_maxposition == 0)
		Pw_Motor4_maxposition = 8388608; //最大位置

	//--------5# Start--------------------------------------------------------
	if (Pw_Driver5_Pluse_HW == 0 && Pw_Driver5_Pluse == 0)
		Pw_Driver5_Pluse = 10000; //发送的脉冲数

	if (Pw_Driver5_AccTime == 0)
		Pw_Driver5_AccTime = 100; //加速度

	if (Pw_Driver5_Speed == 0)
		Pw_Driver5_Speed = 1000; //设定速度

	if (Pw_Motor5_PULSENUM == 0)
		Pw_Motor5_PULSENUM = 10000; //每圈脉冲数

	if (Pw_Motor5_FRE_START == 0)
		Pw_Motor5_FRE_START = 10000; //起始频率

	if (Pw_Motor5_FRE_AA == 0)
		Pw_Motor5_FRE_AA = 10000; //加加速度

	if (Pw_Motor5_STEP_PARA == 0)
		Pw_Motor5_STEP_PARA = 500; //步进步数修正因子

	if (Pw_Motor5_maxposition == 0)
		Pw_Motor5_maxposition = 8388608; //最大位置

	//--------6# Start--------------------------------------------------------
	if (Pw_Driver6_Pluse_HW == 0 && Pw_Driver6_Pluse == 0)
		Pw_Driver6_Pluse = 10000; //发送的脉冲数

	if (Pw_Driver6_AccTime == 0)
		Pw_Driver6_AccTime = 100; //加速度

	if (Pw_Driver6_Speed == 0)
		Pw_Driver6_Speed = 1000; //设定速度

	if (Pw_Motor6_PULSENUM == 0)
		Pw_Motor6_PULSENUM = 10000; //每圈脉冲数

	if (Pw_Motor6_FRE_START == 0)
		Pw_Motor6_FRE_START = 10000; //起始频率

	if (Pw_Motor6_FRE_AA == 0)
		Pw_Motor6_FRE_AA = 10000; //加加速度

	if (Pw_Motor6_STEP_PARA == 0)
		Pw_Motor6_STEP_PARA = 500; //步进步数修正因子

	if (Pw_Motor6_maxposition == 0)
		Pw_Motor6_maxposition = 8388608; //最大位置
}

void Time_Output(void) // 软件时钟输出	 2008.10.21
{
	//软件时钟，非真实时钟
	if (SClkSecond >= 60) // 秒
	{
		SClkSecond = 0;
		SClkMinute++;
		//w_SetMinute=SClkMinute;
		if (SClkMinute >= 60) // 分
		{
			SClkMinute = 0;
			SClkHour++;
			//w_SetHour=SClkHour;
			if (SClkHour >= 24) // 时
			{
				SClkHour = 0;
				SClkDay++;
				//w_SetDay=SClkDay;
				if (SClkDay > 30) // 日
				{
					SClkDay = 1;
					SClkMonth++;
					//w_SetMonth=SClkMonth;
					if (SClkMonth > 12) // 月
					{
						SClkMonth = 1;
						SClkYear++;
					}
				}
			}
		}
	}
}

//开关量输入配置值子函数
uint8_t DIConfigValue(uint8_t DI_BitNo) // DI_BitNo:DI 位号,将来用数组值代替
{
	uint32_t BBDIValue;
	uint8_t j;
	BBDIValue = (uint32_t)(DInb[(DI_BitNo - 1) / 8]);
	j = (DI_BitNo - 1) % 8;
	if (MEM_ADDR(BITBAND((uint32_t)&BBDIValue, j)))
		return 1;
	else
		return 0;
}

void DigitalIn(void)
{
	// 开关量输入检测； =1,来信号(短接地，光耦输出1进入MCU单片机)；=0,无信号。
	FilterDI();

	K_StopRun = DIConfigValue(1); // 启动停止					//注意： 闭合时，运行。断开时，停止运行。
}
