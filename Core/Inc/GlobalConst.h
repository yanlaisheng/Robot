/**
  ******************************************************************************
  * @file    GlobalConst.h
  * @author  ChengLei Zhou  - 周成磊
  * @version V1.27
  * @date    2014-01-03
  * @brief   全局变量声明
	******************************************************************************
	*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __GLOBALCONST_H
#define __GLOBALCONST_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f40x.h"

#define SECTORWORDNUM 256          // 扇区字数量
#define FLASH_PAR_SIZE 255         // NV参数数量
#define FLASH_ID_SIZE 128          // ID参数数量
#define PROGRAM_LEN 0xE000         // 0-0xE000 程序空间长度
#define FLASH_PAR_ADDR 0xE000      // NV参数地址
#define FLASH_ID_ADDR 0xE200       // NV 电话等参数地址
#define FLASH_SETP_ADDR 0xE400     // 设定压力区
#define FLASH_VOICEVAL_ADDR 0xE600 // 设定音量区
#define FLASH_FAULT_ADDR 0xE800    // 故障记录区
#define FLASH_REC_ADDR 0xEA00      // FLASH 记录
#define FLASH_RUNTSUM_ADDR 0xF000  // 水泵累计运行时间 记录
#define FLASH_REC_SIZE 64          // 一条FLASH记录的长度(字节)
#define NORCVMAXMS 5               // 20 2007.7.5
#define SECTOR_SIZE 65536          // 扇区长度
#define RDWR_SIZE 64               // 读写FLASH的长度
#define ISL1208 0xDE               // Device address for chip A--ISL1208
#define AT24C256 0xA4              // Device address for chip B
#define FREQMAXDEC 500             // 频率最大DEC值
#define CS_Flash1 1                // 片选FLASH1
#define CS_Flash2 2                // 片选FLASH2
#define CS_FMRAM1 3                // 片选FMRAM1
#define FLASH_REC_MAX 16384        // FLASH记录最大数
#define FAULT_REC_MAX 64           //故障记录最大数
#define FMADD_FLASH_REC_NO 512
#define FMADD_FLASH_REC_NUM 514
#define FMADD_FAULT_REC_NUM 516 //定义在前，为了跟SM510对应起来
#define FMADD_FAULT_REC_NO 518

#define TXD1_MAX 255 // 最大发送数量
#define RCV1_MAX 255 // 接收缓冲区长度 //256*8.5
#define TXD2_MAX 255 // 最大发送数量
#define RCV2_MAX 255 // 接收缓冲区长度 //256*8.5
#define TXD3_MAX 255 // 最大发送数量
#define RCV3_MAX 255 // 接收缓冲区长度 //256*8.5
#define TXD4_MAX 255 // 最大发送数量									//ZCL 2018.12.8
#define RCV4_MAX 255 // 接收缓冲区长度 //256*8.5
#define TXD5_MAX 255 // 最大发送数量
#define RCV5_MAX 255 // 接收缓冲区长度 //256*8.5			//ZCL 2018.12.8
#define TXD6_MAX 255 // 最大发送数量
#define RCV6_MAX 255 // 接收缓冲区长度 //256*8.5			//ZCL 2018.12.8

// 把“位带地址＋位序号”转换别名地址宏
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
//把该地址转换成一个指针
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
//IO口地址映射
#define GPIOA_ODR_Addr (GPIOA_BASE + 12) //0x4001080C
#define GPIOB_ODR_Addr (GPIOB_BASE + 12) //0x40010C0C
#define GPIOC_ODR_Addr (GPIOC_BASE + 12) //0x4001100C
#define GPIOD_ODR_Addr (GPIOD_BASE + 12) //0x4001140C
#define GPIOE_ODR_Addr (GPIOE_BASE + 12) //0x4001180C
#define GPIOF_ODR_Addr (GPIOF_BASE + 12) //0x40011A0C
#define GPIOG_ODR_Addr (GPIOG_BASE + 12) //0x40011E0C

#define GPIOA_IDR_Addr (GPIOA_BASE + 8) //0x40010808
#define GPIOB_IDR_Addr (GPIOB_BASE + 8) //0x40010C08
#define GPIOC_IDR_Addr (GPIOC_BASE + 8) //0x40011008
#define GPIOD_IDR_Addr (GPIOD_BASE + 8) //0x40011408
#define GPIOE_IDR_Addr (GPIOE_BASE + 8) //0x40011808
#define GPIOF_IDR_Addr (GPIOF_BASE + 8) //0x40011A08
#define GPIOG_IDR_Addr (GPIOG_BASE + 8) //0x40011E08

//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) //输出
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  //输入

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) //输出
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  //输入

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) //输出
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  //输入

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) //输出
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  //输入

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) //输出
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  //输入

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) //输出
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  //输入

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) //输出
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  //输入

#define COM1_DATA PDout(8)  //=1,灭；=0,亮	就是：Data 指示灯 ！！！
#define COM0_DATA PDout(15) //=1,灭；=0,亮

//#define		COM0_TX				PDout(9)		//经过测试，COM1_LED输出模式，输出状态不能读取
#define COM0_RX PDin(10) //状态输入脚
//#define		COM1_TX				PDout(10)		//经过测试，COM1_LED输出模式，输出状态不能读取
#define COM1_RX PDin(11) //状态输入脚

/* BSP硬件定义  */
//----------------------------------------------------------------

//开关量输入
//DI1 -- PC8		//DI2 -- PC7		//DI3 -- PA8		DI4 -- PC9
#define DI1 PCin(8) // 2016.2.1
#define DI2 PCin(7) //
#define DI3 PAin(8) //
#define DI4 PCin(9) //

//继电器输出			在DSP上，通过输出DO1，DO2可以间接控制继电器 2015.5.10 必须DSP复位等都正常后
#define DO1 PAout(11) // =1 OPEN; =0 CLOSE
#define DO2 PAout(12) // =1 OPEN; =0 CLOSE

#define Qb_Q1 PAout(11) //=1 打开,=0闭合		//数字量输出1  连接到DSP B4
#define Qb_Q2 PAout(12) //=1 打开,=0闭合		//数字量输出2  连接到DSP B5

#define Qb_Q1_IN PAin(11) //=1 打开,=0闭合		//数字量输出1  连接到DSP B4
#define Qb_Q2_IN PAin(12) //=1 打开,=0闭合		//数字量输出2  连接到DSP B5

#define STOP_BRAKE_SYSTEM GPIO_ResetBits(GPIOA, GPIO_Pin_11) //停止刹车，允许运行
#define START_BRAKE_SYSTEM GPIO_SetBits(GPIOA, GPIO_Pin_11)  //开始刹车，不允许运行

#define Brake_Status_DO1 PAin(11) //刹车状态

#define OPEN_HAND GPIO_ResetBits(GPIOA, GPIO_Pin_12) //打开机械手臂
#define CLOSE_HAND GPIO_SetBits(GPIOA, GPIO_Pin_12)  //闭合机械手臂

//SPI NSS脚
#define SPI2_NSS_L GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define SPI2_NSS_H GPIO_SetBits(GPIOB, GPIO_Pin_12)

#define SPI3_NSS_L GPIO_ResetBits(GPIOC, GPIO_Pin_6)
#define SPI3_NSS_H GPIO_SetBits(GPIOC, GPIO_Pin_6)

//指示灯 周成磊 2016.03.10
#define LED1 PFout(8) //=1,灭; =0亮;
#define LED2 PBout(6)
#define LED3 PBout(5)
#define LED4 PBout(4)
#define LED5 PBout(3)
#define LED6 PAout(15) //ZCL 2018.12.8

#define LED1_IN PFin(8) //读
#define LED2_IN PBin(6)
#define LED3_IN PBin(5)
#define LED4_IN PBin(4)
#define LED5_IN PBin(3)
#define LED6_IN PAin(15) //ZCL 2018.12.8

#define HAND_STATUS PAin(12) //0打开电磁阀；1关闭电磁阀

#define S_ACCEL 1
#define T_ACCEL 0

/* S型加速参数 */
#define ACCELERATED_SPEED_LENGTH 200 //定义加速度的点数（其实也是3000个细分步的意思），调这个参数改变加速点
#define FRE_MIN 500                  //最低的运行频率，调这个参数调节最低运行速度
#define FRE_MAX 35000                //最高的运行频率，调这个参数调节匀速时的最高速度35000

// 定义设定参数
#define Pw_Motor1SendPulse w_ParLst[0]     // 电机1发送脉冲数
#define Pw_Motor1SendPulse_HW w_ParLst[1]  // 电机1发送脉冲数高字
#define Pw_Motor1_ACCSpeed w_ParLst[2]     // 电机1加速度
#define Pw_Motor1_ACCSpeed_HW w_ParLst[3]  // 电机1加速度高字
#define Pw_Motor1_SetSpeed w_ParLst[4]     // 电机1设定速度
#define Pw_Motor1_SetSpeed_HW w_ParLst[5]  // 电机1设定速度高字
#define Pw_Motor1_PULSENUM w_ParLst[6]     // 电机1每圈脉冲数
#define Pw_Motor1_FRE_START w_ParLst[7]    // 电机1启动频率
#define Pw_Motor1_FRE_AA w_ParLst[8]       // 电机1加加速度
#define Pw_ModPar w_ParLst[9]              // 修改参数
#define Pw_Motor1_STEP_PARA w_ParLst[10]   // 电机1步进步数修正因子
#define Pw_Motor1_maxposition w_ParLst[11] // 电机1最大位置

#define Pw_Motor2SendPulse w_ParLst[16]    // 电机2发送脉冲数
#define Pw_Motor2SendPulse_HW w_ParLst[17] // 电机2发送脉冲数高字
#define Pw_Motor2_ACCSpeed w_ParLst[18]    // 电机2加速度
#define Pw_Motor2_ACCSpeed_HW w_ParLst[19] // 电机2加速度高字
#define Pw_Motor2_SetSpeed w_ParLst[20]    // 电机2设定速度
#define Pw_Motor2_SetSpeed_HW w_ParLst[21] // 电机2设定速度高字
#define Pw_Motor2_PULSENUM w_ParLst[22]    // 电机2每圈脉冲数
#define Pw_Motor2_FRE_START w_ParLst[23]   // 电机2启动频率
#define Pw_Motor2_FRE_AA w_ParLst[24]      // 电机2加加速度
#define Pw_Motor2_STEP_PARA w_ParLst[25]   // 电机2步进步数修正因子
#define Pw_Motor2_maxposition w_ParLst[26] // 电机2最大位置

#define Pw_Initial_F w_ParLst[40]      // 初始化标志，=0x5A，表示已经初始化
#define Pw_ParInitial w_ParLst[41]     // 参数初始化
#define Pos_Group_Select w_ParLst[42]  // 联动位置控制命令组选择（1-5组）
#define Group_Select w_ParLst[43]      //参数组号选择
#define Pw_Driver_PosDely w_ParLst[44] // 位置写入后的延时，250ms

#define Pw_Motor3SendPulse w_ParLst[51]    // 电机3发送脉冲数
#define Pw_Motor3SendPulse_HW w_ParLst[52] // 电机3发送脉冲数高字
#define Pw_Motor3_ACCSpeed w_ParLst[53]    // 电机3加速度
#define Pw_Motor3_ACCSpeed_HW w_ParLst[54] // 电机3加速度高字
#define Pw_Motor3_SetSpeed w_ParLst[55]    // 电机3设定速度
#define Pw_Motor3_SetSpeed_HW w_ParLst[56] // 电机3设定速度高字
#define Pw_Motor3_PULSENUM w_ParLst[57]    // 电机3每圈脉冲数
#define Pw_Motor3_FRE_START w_ParLst[58]   // 电机3启动频率
#define Pw_Motor3_FRE_AA w_ParLst[59]      // 电机3加加速度
#define Pw_Motor3_STEP_PARA w_ParLst[60]   // 电机3步进步数修正因子
#define Pw_Motor3_maxposition w_ParLst[61] // 电机3最大位置

#define Pw_Motor4SendPulse w_ParLst[66]    // 电机4发送脉冲数
#define Pw_Motor4SendPulse_HW w_ParLst[67] // 电机4发送脉冲数高字
#define Pw_Motor4_ACCSpeed w_ParLst[68]    // 电机4加速度
#define Pw_Motor4_ACCSpeed_HW w_ParLst[69] // 电机4加速度高字
#define Pw_Motor4_SetSpeed w_ParLst[70]    // 电机4设定速度
#define Pw_Motor4_SetSpeed_HW w_ParLst[71] // 电机4设定速度高字
#define Pw_Motor4_PULSENUM w_ParLst[72]    // 电机4每圈脉冲数
#define Pw_Motor4_FRE_START w_ParLst[73]   // 电机4启动频率
#define Pw_Motor4_FRE_AA w_ParLst[74]      // 电机4加加速度
#define Pw_Motor4_STEP_PARA w_ParLst[75]   // 电机4步进步数修正因子
#define Pw_Motor4_maxposition w_ParLst[76] // 电机4最大位置

#define Pw_Motor5SendPulse w_ParLst[81]    // 电机5发送脉冲数
#define Pw_Motor5SendPulse_HW w_ParLst[82] // 电机5发送脉冲数高字
#define Pw_Motor5_ACCSpeed w_ParLst[83]    // 电机5加速度
#define Pw_Motor5_ACCSpeed_HW w_ParLst[84] // 电机5加速度高字
#define Pw_Motor5_SetSpeed w_ParLst[85]    // 电机5设定速度
#define Pw_Motor5_SetSpeed_HW w_ParLst[86] // 电机5设定速度高字
#define Pw_Motor5_PULSENUM w_ParLst[87]    // 电机5每圈脉冲数
#define Pw_Motor5_FRE_START w_ParLst[88]   // 电机5启动频率
#define Pw_Motor5_FRE_AA w_ParLst[89]      // 电机5加加速度
#define Pw_Motor5_STEP_PARA w_ParLst[90]   // 电机5步进步数修正因子
#define Pw_Motor5_maxposition w_ParLst[91] // 电机5最大位置

#define Pw_Motor6SendPulse w_ParLst[96]     // 电机6发送脉冲数
#define Pw_Motor6SendPulse_HW w_ParLst[97]  // 电机6发送脉冲数高字
#define Pw_Motor6_ACCSpeed w_ParLst[98]     // 电机6加速度
#define Pw_Motor6_ACCSpeed_HW w_ParLst[99]  // 电机6加速度高字
#define Pw_Motor6_SetSpeed w_ParLst[100]    // 电机6设定速度
#define Pw_Motor6_SetSpeed_HW w_ParLst[101] // 电机6设定速度高字
#define Pw_Motor6_PULSENUM w_ParLst[102]    // 电机6每圈脉冲数
#define Pw_Motor6_FRE_START w_ParLst[103]   // 电机6启动频率
#define Pw_Motor6_FRE_AA w_ParLst[104]      // 电机6加加速度
#define Pw_Motor6_STEP_PARA w_ParLst[105]   // 电机6步进步数修正因子
#define Pw_Motor6_maxposition w_ParLst[106] // 电机6最大位置

#define Pw_DIStableSetNum w_ParLst[115]       // 开关量稳定数目
#define Pr_Driver1_Control_OK_F w_ParLst[116] //1#伺服控制命令OK标志
#define Pr_Driver2_Control_OK_F w_ParLst[117] //2#伺服控制命令OK标志
#define Pr_Driver3_Control_OK_F w_ParLst[118] //3#伺服控制命令OK标志
#define Pr_Driver4_Control_OK_F w_ParLst[119] //4#伺服控制命令OK标志
#define Pr_Driver5_Control_OK_F w_ParLst[120] //5#伺服控制命令OK标志
#define Pr_Driver6_Control_OK_F w_ParLst[121] //6#伺服控制命令OK标志

#define Pw_SendPWM w_ParLst[122]       // 发PWM波命令
#define Pw_S_ParaInitial w_ParLst[123] // S曲线初始化命令

#define Pw_BaudRate1 w_ParLst[141] // 波特率1
#define Pw_BaudRate2 w_ParLst[142] // 波特率2
#define Pw_BaudRate3 w_ParLst[143] // 波特率3
//#define	Pw_SmallBadClrSec		w_ParLst[144]	// 小流量破坏清除秒
//#define	Pw_SendToGprsEn			w_ParLst[145]	// 主动发送给GPRS返回数据使能
//#define	Pw_SendToGprsDataLen	w_ParLst[146]	// 主动发送给GPRS返回数据长度 字
//#define	Pw_SendToGprsSec		w_ParLst[147]	// 主动发送给GPRS返回数据间隔时间 秒
#define Pw_ComBufType w_ParLst[148] // 通讯缓存类型(通讯协议类型) 普通=1,宁波=2
//#define	Pw_ASensorType			w_ParLst[149]	// 电流传感器类型
//#define	Pw_ASensorZero			w_ParLst[150]	// 电流传感器初值
//#define	Pw_VASumFilter			w_ParLst[151]	// 和滤波
#define Pw_HighYCSetPEK2 w_ParLst[151]
#define Pw_BaudRate4 w_ParLst[152] // 波特率4
#define Pw_BaudRate5 w_ParLst[153] // 波特率5
#define Pw_Com5Addr w_ParLst[154]  //com5口485通讯地址
//----
#define Pw_LLJCaiJiSelect w_ParLst[155] //流量计采集方式
#define Pw_DDBCaiJiSelect w_ParLst[156]
#define Pw_DDBAddR w_ParLst[157] //电能表地址

#define Pw_EquipmentNo1 w_ParLst[159] //1#伺服电机Modbus地址
#define Pw_EquipmentNo2 w_ParLst[160] //2#伺服电机Modbus地址
#define Pw_EquipmentNo3 w_ParLst[161] //3#伺服电机Modbus地址
#define Pw_EquipmentNo4 w_ParLst[162] //4#伺服电机Modbus地址
#define Pw_EquipmentNo5 w_ParLst[163] //5#伺服电机Modbus地址
#define Pw_EquipmentNo6 w_ParLst[164] //6#伺服电机Modbus地址

#define Pw_Com3Add w_ParLst[165]           //com3-T3通讯地址
#define Pw_InP_CTLVfEn w_ParLst[166]       //进水限制频率使能 2010.8.3 qhd
#define Pw_VfFreqDownSircle w_ParLst[167]  //每秒递减频率值
#define Pw_VfFreqDownMin w_ParLst[168]     //最小运行频率
#define Pw_SetInP w_ParLst[169]            //进水设定压力
#define Pw_PumpSoftStopEn w_ParLst[170]    //设备软停使能
#define Pw_VfOnDelay w_ParLst[171]         //变频转工频时启动变频泵延时  秦汉东2010.8.3
#define Pw_DownFreqHex w_ParLst[172]       //停泵每秒递减频率值
#define Pw_AllPumpStopDelay w_ParLst[174]  //软停泵延时时间
#define Pw_GpExitVfOnFreqHex w_ParLst[175] //工频退出变频泵频率赋值
#define Pw_VfAlarmDelay w_ParLst[176]      //变频器报警停机延时
#define Pw_PumpRunTimEn w_ParLst[177]      //水泵累计运行时间记录使能
#define Pw_GpExitFreq w_ParLst[178]        //工频退出频率

#define Pr_RUN_Count w_ParLst[179]     //运行指令计数
#define Pr_RUN_Count_Set w_ParLst[180] //运行多少圈设定，初始化为0

#define Pw_Total_RUN_Count w_ParLst[181]    //累计运行圈数，出厂值为0
#define Pw_Total_RUN_Count_HW w_ParLst[182] //累计运行圈数_高字
#define Pw_Com5BufType w_ParLst[183]        //Com5通讯类型
#define Pw_ComWriteErr_Stop w_ParLst[184]   //通讯数据写入错误停机功能，=1，停机；=0，不停机
#define Pw_HighYCSetP1 w_ParLst[185]
#define Pw_HighYCSetPEK w_ParLst[186]
#define Pw_HighYCSetPDelay w_ParLst[187]
#define Pw_PIDSlowEN w_ParLst[188]         //PID缓慢调节频率使能
#define Pw_PID_UPSlowCount w_ParLst[189]   //PID上升缓慢调节倍数
#define Pw_PID_PEKUpperlimit w_ParLst[190] //PID调节压差上限
#define Pw_PID_PEKLowerlimit w_ParLst[191] //PID调节压差下限
#define Pw_HengVfDelay w_ParLst[192]       // 小流量恒pin时间
#define Pw_HaveWater_MaxSupplyDelay w_ParLst[193]

#define Pw_Com3BufType w_ParLst[210]
#define Pw_Drive1_MinPos_SINGLE w_ParLst[211]    //1#电机最小位置（多圈+单圈位置）
#define Pw_Drive1_MinPos_SINGLE_HW w_ParLst[212] //1#电机最小位置_高字（多圈+单圈位置）
#define Pw_Drive1_MaxPos_SINGLE w_ParLst[213]    //1#电机最大位置（多圈+单圈位置）
#define Pw_Drive1_MaxPos_SINGLE_HW w_ParLst[214] //1#电机最大位置_高字（多圈+单圈位置）
#define Pw_Drive2_MinPos_SINGLE w_ParLst[215]    //2#电机最小位置（多圈+单圈位置）
#define Pw_Drive2_MinPos_SINGLE_HW w_ParLst[216] //2#电机最小位置_高字（多圈+单圈位置）
#define Pw_Drive2_MaxPos_SINGLE w_ParLst[217]    //2#电机最大位置（多圈+单圈位置）
#define Pw_Drive2_MaxPos_SINGLE_HW w_ParLst[218] //2#电机最大位置_高字（多圈+单圈位置）

#define Pw_Com3SendAdd w_ParLst[220] //T3口主动发送地址 339
#define Pw_Com3Sendnum w_ParLst[221] //T3口主动发送字数32

#define Pw_ComErr_Stop w_ParLst[222]  //通讯故障停机功能，=1，停机；=0，不停机
#define Pr_F_HaveFault w_ParLst[223]  //有电机故障标志
#define Pw_Limit_MaxPos w_ParLst[224] //最大位置限制功能，=1，启用；=0，不启动

#define Pr_F_ComErr w_ParLst[225]       //有通讯故障标志
#define Pr_Driver1_ComErr w_ParLst[226] //Driver1通讯故障标志
#define Pr_Driver2_ComErr w_ParLst[227] //Driver2通讯故障标志
#define Pr_Driver3_ComErr w_ParLst[228] //Driver3通讯故障标志
#define Pr_Driver4_ComErr w_ParLst[229] //Driver4通讯故障标志
#define Pr_Driver5_ComErr w_ParLst[230] //Driver5通讯故障标志
#define Pr_Driver6_ComErr w_ParLst[231] //Driver6通讯故障标志

#define Pw_Drive1_MinPos w_ParLst[232] //1#电机最小位置（多圈位置）
#define Pw_Drive1_MaxPos w_ParLst[233] //1#电机最大位置（多圈位置）
#define Pw_Drive2_MinPos w_ParLst[234] //2#电机最小位置（多圈位置）
#define Pw_Drive2_MaxPos w_ParLst[235] //2#电机最大位置（多圈位置）
#define Pw_Drive3_MinPos w_ParLst[236] //3#电机最小位置（多圈位置）
#define Pw_Drive3_MaxPos w_ParLst[237] //3#电机最大位置（多圈位置）
#define Pw_Drive4_MinPos w_ParLst[238] //4#电机最小位置（多圈位置）
#define Pw_Drive4_MaxPos w_ParLst[239] //4#电机最大位置（多圈位置）
#define Pw_Drive5_MinPos w_ParLst[240] //5#电机最小位置（多圈位置）
#define Pw_Drive5_MaxPos w_ParLst[241] //5#电机最大位置（多圈位置）
#define Pw_Drive6_MinPos w_ParLst[242] //6#电机最小位置（多圈位置）
#define Pw_Drive6_MaxPos w_ParLst[243] //6#电机最大位置（多圈位置）

#define Pr_BRAKE_Status w_ParLst[244]       //刹车状态
#define Pr_BRAKE_Control w_ParLst[245]      //刹车控制，=1，刹车；=0，不刹车
#define Pr_F_AllStopped w_ParLst[246]       //所有电机都停止标志
#define Pr_Com4_ComErr w_ParLst[247]        //Com4通讯故障标志
#define Pr_BRAKE_Control_Mode w_ParLst[248] //刹车控制模式，=1，手动控制刹车；=0，自动控制刹车
#define Pr_AllRun w_ParLst[249]             //所有电机都运行标志

// 定义模拟量端子配置参数：从地址240开始－地址254。共15个

// 定义只读参数：从地址256－511开始：
#define w_Uk1 w_ParLst[256]             // 频率输出Hex值
#define w_InPSensorValue w_ParLst[257]  // 进水口压力传感器读值
#define w_OutPSensorValue w_ParLst[258] // 出水口压力传感器读值
#define w_FaultRecNum w_ParLst[259]     // 故障记录数量 ZCL 定义在前，为了跟SM510对应起来
#define w_FaultRecNo w_ParLst[260]      // 故障记录号  ZCL

#define w_TSensorValue w_ParLst[261]  // 温度传感器检测值  2016.3.22
#define w_SaveBaudRate0 w_ParLst[262] // 波特率保存值0
#define w_SaveBaudRate1 w_ParLst[263] // 波特率保存值1
#define w_SaveBaudRate2 w_ParLst[264] // 波特率保存值2
#define w_SaveBaudRate3 w_ParLst[265] // 波特率保存值3
#define w_SaveBaudRate4 w_ParLst[266] // 波特率保存值4		ZCL 2018.12.8
#define w_SaveBaudRate5 w_ParLst[267] // 波特率保存值5		ZCL 2018.12.8

#define w_GongSiSelect w_ParLst[268] //  公司选择，0=中的美，1=三利集团
//#define	w_FlashWrRdLock			w_ParLst[267]	// Flash写读锁定		ZCL 2018.12.8
//#define	w_FlashRecNoNumPointerNo	w_ParLst[268]	// FLASH记录序号数量的指针序号
#define w_SelFaultNo w_ParLst[269] // 选择故障号
#define w_SelRecNo w_ParLst[270]   // 选择记录号
#define w_PreFaultNo w_ParLst[271] // 上次故障号
//
#define w_TestItemSel w_ParLst[272]        // 测试项选择设定参数 (可以修改，不可以保存)
#define w_VvvfAlmNum w_ParLst[273]         // 变频报警次数
#define w_BetweenSmall w_ParLst[274]       // 小流量间隔状态
#define w_SmallStableRunSec w_ParLst[275]  // 小流量恒压走时
#define w_SoftVer w_ParLst[276]            // 软件版本
#define w_TimePwdStopST w_ParLst[277]      // 定时密码停机状态
#define w_EquipOperateStatus w_ParLst[278] // 设备操作状态
#define w_EquipOperateNum w_ParLst[279]    // 设备操作状态数量
#define w_EquipAlarmStatus w_ParLst[280]   // 设备停机原因（报警）
#define w_EquipStopNum w_ParLst[281]       // 设备停机数量（报警）
//
#define w_EquipAlarmLast6 w_ParLst[282]    // 设备故障停机原因上6次
#define w_EquipAlarm6YM w_ParLst[283]      // 设备故障停机6时间－年月
#define w_EquipAlarm6DH w_ParLst[284]      // 设备故障停机6时间－日时
#define w_EquipAlarm6MS w_ParLst[285]      // 设备故障停机6时间－分秒
#define w_EquipAlarmLast5 w_ParLst[286]    // 设备故障停机原因上5次
#define w_EquipAlarm5YM w_ParLst[287]      // 设备故障停机5时间－年月
#define w_EquipAlarm5DH w_ParLst[288]      // 设备故障停机5时间－日时
#define w_EquipAlarm5MS w_ParLst[289]      // 设备故障停机5时间－分秒
#define w_EquipAlarmLast4 w_ParLst[290]    // 设备故障停机原因上4次
#define w_EquipAlarm4YM w_ParLst[291]      // 设备故障停机4时间－年月
#define w_EquipAlarm4DH w_ParLst[292]      // 设备故障停机4时间－日时
#define w_EquipAlarm4MS w_ParLst[293]      // 设备故障停机4时间－分秒
#define w_EquipAlarmLast3 w_ParLst[294]    // 设备故障停机原因上3次
#define w_EquipAlarm3YM w_ParLst[295]      // 设备故障停机3时间－年月
#define w_EquipAlarm3DH w_ParLst[296]      // 设备故障停机3时间－日时
#define w_EquipAlarm3MS w_ParLst[297]      // 设备故障停机3时间－分秒
#define w_EquipAlarmLast2 w_ParLst[298]    // 设备故障停机原因上2次
#define w_EquipAlarm2YM w_ParLst[299]      // 设备故障停机2时间－年月
#define w_EquipAlarm2DH w_ParLst[300]      // 设备故障停机2时间－日时
#define w_EquipAlarm2MS w_ParLst[301]      // 设备故障停机2时间－分秒
#define w_EquipAlarmLast1 w_ParLst[302]    // 设备故障停机原因上1次
#define w_EquipAlarm1YM w_ParLst[303]      // 设备故障停机1时间－年月
#define w_EquipAlarm1DH w_ParLst[304]      // 设备故障停机1时间－日时
#define w_EquipAlarm1MS w_ParLst[305]      // 设备故障停机1时间－分秒
#define w_SelEquipAlarm w_ParLst[306]      // 选择的设备故障停机原因
#define w_SelEquipAlarm1YM w_ParLst[307]   // 选择的设备故障停机时间－年月
#define w_SelEquipAlarm1DH w_ParLst[308]   // 选择的设备故障停机时间－日时
#define w_SelEquipAlarm1MS w_ParLst[309]   // 选择的设备故障停机时间－分秒
#define w_FlashRecNo w_ParLst[310]         // FLASH记录号
#define w_FlashRecNum w_ParLst[311]        // FLASH记录数量
#define w_TimePwdStopDays w_ParLst[312]    // 定时密码停机天数
#define w_RemoteStop w_ParLst[313]         // 远程遥控变频停止 =1停机
#define w_RemoteGpRun w_ParLst[314]        // 远程遥控工频启停 .0 GP1, .1 GP2, .2 GP3, .3 GP4, .4 GP5
#define w_RemoteVfRstRelayOn w_ParLst[315] // 遥控变频器复位继电器ON
#define w_TouchAutoManu w_ParLst[316]      // 触摸 自动/手动
//#define	Pw_TouchRunStop			w_ParLst[317]	// 触摸 启动/停止
#define w_Touch1Gp w_ParLst[318]        // 触摸1工频
#define w_Touch2Gp w_ParLst[319]        // 触摸2工频
#define w_Touch3Gp w_ParLst[320]        // 触摸3工频
#define w_Touch4Gp w_ParLst[321]        // 触摸4工频
#define w_Touch5Gp w_ParLst[322]        // 触摸5工频 \
                                        //#define	w_RunTimeSumNo			w_ParLst[323]	// 水泵累计运行指针序号
#define w_FlashUPSetParNo w_ParLst[324] // FLASH上传参数 序号
#define w_FlashDWSetParNo w_ParLst[325] // FLASH下载参数 序号
#define w_FlashUPProgNo w_ParLst[326]   // FLASH上传程序
#define w_FlashDWProgNo w_ParLst[327]   // FLASH下载程序
#define w_FlashDWRecordNo w_ParLst[328] // FLASH下载历史记录
//#define	w_DisMemoryBoardRecord	w_ParLst[329]	// 显示FLASH 存储板记录
#define w_SaveRecNo w_ParLst[330]         // 保存Flash记录号
#define w_SaveRecNum w_ParLst[331]        // 保存Flash记录数量
#define w_CpuTemperatureHex w_ParLst[332] // Cpu温度Hex
#define w_CpuWendu w_ParLst[333]          // Cpu温度换算值

#define w_SBaudTimeCount w_ParLst[334]   // 软串口波特率计数器
#define w_SBaudThTimeCount w_ParLst[335] // 软串口波特率计数器（接收起始计数器）
#define w_SelPar w_ParLst[336]           // 选择参数
#define w_SelParValue w_ParLst[337]      // 选择参数的值

// 远程监控集中查询常用参数 32个
#define w_ProcessNo w_ParLst[339] // 过程序号
// 远程监控集中查询常用参数 31个字
#define w_Pump12Status w_ParLst[340] // 泵12状态
#define w_Pump34Status w_ParLst[341] // 泵34状态
#define w_Pump56Status w_ParLst[342] // 泵56状态
#define w_Flag1Unit w_ParLst[343]    // 标志1单元
#define w_Flag2Unit w_ParLst[344]    // 标志2单元
#define w_Flag3Unit w_ParLst[345]    // 标志3单元
//#define	w_ResidualCL			w_ParLst[346]	//
#define w_Flag4Unit w_ParLst[346]

#define w_PIDCalcP w_ParLst[347]     // PID运算压力
#define w_VvvfFreq w_ParLst[348]     // 变频器频率
#define w_InPDec w_ParLst[349]       // 进水口压力
#define w_OutPDec w_ParLst[350]      // 出水口压力
#define w_InstanFlux w_ParLst[351]   // 瞬时流量		//ZCL 2007.6.15
#define w_Pump1Current w_ParLst[352] // 1号泵电流
#define w_Pump2Current w_ParLst[353] // 2号泵电流
#define w_Pump3Current w_ParLst[354] // 3号泵电流
#define w_Pump4Current w_ParLst[355] // 4号泵电流
#define w_Pump5Current w_ParLst[356] // 5号泵电流
#define w_YeWeiDeep w_ParLst[357]    // 液位深度
#define w_SysVoltage w_ParLst[358]   // 系统电压		//ZCL 2007.6.15
#define w_SumFluxL w_ParLst[359]     // 累计流量低字
#define w_SumFluxH w_ParLst[360]     // 累计流量高字
#define w_DDBSumFluxL w_ParLst[361]  // 累计电量低字
#define w_DDBSumFluxH w_ParLst[362]  //
#define w_ZhuoDuValue w_ParLst[363]  // 浊度
#define w_YuLvValue w_ParLst[364]    //
#define w_WenDuValue w_ParLst[365]   //
#define w_PHValue w_ParLst[366]      // PH值

//#define	w_VvvfFreq1			    w_ParLst[363]	//
#define w_YeWeiDeep2 w_ParLst[367] //2#水箱液位
#define w_NowYM w_ParLst[368]      // 年月
#define w_NowDH w_ParLst[369]      // 日时
#define w_NowMS w_ParLst[370]      // 分秒
#define w_ShiDuValue w_ParLst[371] //
#define w_ZaoYinValue w_ParLst[372]
#define w_DDF1OpenValue w_ParLst[373] // 1#电动阀开度
#define w_DDF2OpenValue w_ParLst[374] // 2#电动阀开度
//#define	w_DDF3OpenValue			w_ParLst[375]	// 1#电动阀开度
//#define	w_DDF4OpenValue			w_ParLst[376]	// 2#电动阀开度
#define w_VvvfFreq1 w_ParLst[375]
#define w_VvvfFreq2 w_ParLst[376]
#define w_VvvfFreq3 w_ParLst[377] //
#define w_VvvfFreq4 w_ParLst[378]
//#define	w_VvvfFreq5			    w_ParLst[379]
//

// 640开始存放过程变量参数，也保存到FLASH，但不读出来，没有用。
#define w_WriteDate w_ParLst[642] //程序编写日期
#define w_Writetime w_ParLst[643] //程序编写时间

#define w_VfNum w_ParLst[680]
#define w_GpPumpNum w_ParLst[681]
#define w_StartPump1 w_ParLst[682]
#define w_StartPump2 w_ParLst[683]
#define w_StartPump3 w_ParLst[684]
#define w_StartPump4 w_ParLst[685]
#define w_w_Uk1 w_ParLst[686]
#define w_TimeStopP w_ParLst[687]
#define w_UK_NO1 w_ParLst[688]
#define w_UK_NO2 w_ParLst[689]
#define w_VfNo w_ParLst[690]
#define w_VfNext w_ParLst[691]
#define w_NeedPidCalcP w_ParLst[692]
#define w_Alarm w_ParLst[693] // 报警

//软件时钟
#define SClk1Ms SoftClock[0]    // 软件时钟 1ms
#define SClk10Ms SoftClock[1]   // 软件时钟 10ms
#define SClkSecond SoftClock[2] // 软件时钟  s
#define SClkMinute SoftClock[3] // 软件时钟  m
#define SClkHour SoftClock[4]   // 软件时钟  h
#define SClkDay SoftClock[5]    // 软件时钟  d
#define SClkMonth SoftClock[6]  // 软件时钟  m
#define SClkYear SoftClock[7]   // 软件时钟  y
#define SClk0r5Ms SoftClock[8]  // 软件时钟  0.5MS	2010.8.6 定时中断中采集电流值

#define RealSecond RealClock[0] // 实时时钟
#define RealMinute RealClock[1] // 实时时钟
#define RealHour RealClock[2]   // 实时时钟
#define RealDay RealClock[3]    // 实时时钟
#define RealMonth RealClock[4]  // 实时时钟
#define RealYear RealClock[5]   // 实时时钟

// 定义端子配置参数：从地址240开始：
#define AIsel1 w_ParLst[TERMADDR + 0] / 100 - 1   // 模拟量输入选择
#define AIsel2 w_ParLst[TERMADDR + 0] % 100 - 1   // 模拟量输入选择
#define AIsel3 w_ParLst[TERMADDR + 1] / 100 - 1   // 模拟量输入选择
#define AIsel4 w_ParLst[TERMADDR + 1] % 100 - 1   // 模拟量输入选择
#define AIsel5 w_ParLst[TERMADDR + 2] / 100 - 1   // 模拟量输入选择
#define AIsel6 w_ParLst[TERMADDR + 2] % 100 - 1   // 模拟量输入选择
#define AIsel7 w_ParLst[TERMADDR + 3] / 100 - 1   // 模拟量输入选择
#define AIsel8 w_ParLst[TERMADDR + 3] % 100 - 1   // 模拟量输入选择
#define AIsel9 w_ParLst[TERMADDR + 4] / 100 - 1   // 模拟量输入选择
#define AIsel10 w_ParLst[TERMADDR + 4] % 100 - 1  // 模拟量输入选择
#define AIsel11 w_ParLst[TERMADDR + 5] / 100 - 1  // 模拟量输入选择
#define AIsel12 w_ParLst[TERMADDR + 5] % 100 - 1  // 模拟量输入选择
#define AIsel13 w_ParLst[TERMADDR + 6] / 100 - 1  // 模拟量输入选择
#define AIsel14 w_ParLst[TERMADDR + 6] % 100 - 1  // 模拟量输入选择
#define AIsel15 w_ParLst[TERMADDR + 7] / 100 - 1  // 模拟量输入选择
#define AIsel16 w_ParLst[TERMADDR + 7] % 100 - 1  // 模拟量输入选择
#define AIsel17 w_ParLst[TERMADDR + 8] / 100 - 1  // 模拟量输入选择
#define AIsel18 w_ParLst[TERMADDR + 8] % 100 - 1  // 模拟量输入选择
#define AIsel19 w_ParLst[TERMADDR + 9] / 100 - 1  // 模拟量输入选择
#define AIsel20 w_ParLst[TERMADDR + 9] % 100 - 1  // 模拟量输入选择
#define AIsel21 w_ParLst[TERMADDR + 10] / 100 - 1 // 模拟量输入选择
#define AIsel22 w_ParLst[TERMADDR + 10] % 100 - 1 // 模拟量输入选择
#define AIsel23 w_ParLst[TERMADDR + 11] / 100 - 1 // 模拟量输入选择
#define AIsel24 w_ParLst[TERMADDR + 11] % 100 - 1 // 模拟量输入选择
//
#define AQsel1 w_ParLst[TERMADDR + 12 + 0] / 100 - 1 // 模拟量输出选择
#define AQsel2 w_ParLst[TERMADDR + 12 + 0] % 100 - 1 // 模拟量输出选择
#define AQsel3 w_ParLst[TERMADDR + 12 + 1] / 100 - 1 // 模拟量输出选择
#define AQsel4 w_ParLst[TERMADDR + 12 + 1] % 100 - 1 // 模拟量输出选择
#define AQsel5 w_ParLst[TERMADDR + 12 + 2] / 100 - 1 // 模拟量输出选择
#define AQsel6 w_ParLst[TERMADDR + 12 + 2] % 100 - 1 // 模拟量输出选择

#define FLASH_CMD_SIZE CMD_LIN_NUM *CMD_LIN_SIZE //30条指令，每条指令20个四字节，共600个四字节，即2400个字节
#define CMD_LIN_NUM 30                           //30条指令
#define CMD_LIN_SIZE 20                          //一条命令占用的字数

//M25P32FLASH存储芯片，32Mbit，即8M字节（64个扇区，每个扇区256页，每页256字节，即每个扇区64K字节）
//设置FLASH 保存地址
#define FLASH_SAVE_DRIVER_PAR 0X300000 //64个扇区，每个扇区64K字节

#define FLASH_SAVE_ADDR1_Group1 0X020000 //64个扇区，每个扇区64K字节
#define FLASH_SAVE_ADDR2_Group1 0X030000 //不能跨扇区写，所以一段保存到一个扇区
#define FLASH_SAVE_ADDR3_Group1 0X040000
#define FLASH_SAVE_ADDR4_Group1 0X050000
#define FLASH_SAVE_ADDR5_Group1 0X060000
#define FLASH_SAVE_ADDR6_Group1 0X070000

#define FLASH_SAVE_ADDR1_Group2 0X080000 //64个扇区，每个扇区64K字节
#define FLASH_SAVE_ADDR2_Group2 0X090000 //不能跨扇区写，所以一段保存到一个扇区
#define FLASH_SAVE_ADDR3_Group2 0X0A0000
#define FLASH_SAVE_ADDR4_Group2 0X0B0000
#define FLASH_SAVE_ADDR5_Group2 0X0C0000
#define FLASH_SAVE_ADDR6_Group2 0X0D0000

#define FLASH_SAVE_ADDR1_Group3 0X0E0000 //64个扇区，每个扇区64K字节
#define FLASH_SAVE_ADDR2_Group3 0X0F0000 //不能跨扇区写，所以一段保存到一个扇区
#define FLASH_SAVE_ADDR3_Group3 0X100000
#define FLASH_SAVE_ADDR4_Group3 0X110000
#define FLASH_SAVE_ADDR5_Group3 0X120000
#define FLASH_SAVE_ADDR6_Group3 0X130000

//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
//128K之前为保存code代码，128K之后为保存设定参数
#define FLASH_SAVE_ADDR0 0X08020000 //不能跨扇区写，所以一段保存到一个扇区，每个扇区4096个字节

#define FLASH_SAVE_ADDR1 0X08021000 //不能跨扇区写，所以一段保存到一个扇区
#define FLASH_SAVE_ADDR2 0X08022000
#define FLASH_SAVE_ADDR3 0X08023000
#define FLASH_SAVE_ADDR4 0X08024000
#define FLASH_SAVE_ADDR5 0X08025000
#define FLASH_SAVE_ADDR6 0X08026000

#define FLASH_SAVE_ADDR1_2 0X08027000
#define FLASH_SAVE_ADDR2_2 0X08028000
#define FLASH_SAVE_ADDR3_2 0X08029000
#define FLASH_SAVE_ADDR4_2 0X0802A000
#define FLASH_SAVE_ADDR5_2 0X0802B000
#define FLASH_SAVE_ADDR6_2 0X0802C000

#define FLASH_SAVE_ADDR1_3 0X0802D000
#define FLASH_SAVE_ADDR2_3 0X0802E000
#define FLASH_SAVE_ADDR3_3 0X0802F000
#define FLASH_SAVE_ADDR4_3 0X08030000
#define FLASH_SAVE_ADDR5_3 0X08031000
#define FLASH_SAVE_ADDR6_3 0X08032000

#define FLASH_SAVE_POSTION1 0X08033000 //保存记录的各个位置点，格式：序号+6个关节的多圈数据及单圈数据

#define FLASH_SAVE_POS_CMD1 0X08034000 //同步联动指令，可保持5组指令
#define FLASH_SAVE_POS_CMD2 0X08035000
#define FLASH_SAVE_POS_CMD3 0X08036000
#define FLASH_SAVE_POS_CMD4 0X08037000
#define FLASH_SAVE_POS_CMD5 0X08038000

#define START_CMD_ADDR 300
#define w_ParLst_DrivePar w_ParLst_Drive[0]
#define w_ParLst_Drive1 w_ParLst_Drive[START_CMD_ADDR]
#define w_ParLst_Drive2 w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE]
#define w_ParLst_Drive3 w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 2]
#define w_ParLst_Drive4 w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 3]
#define w_ParLst_Drive5 w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 4]
#define w_ParLst_Drive6 w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 5]

//20*15=300个双字，位置序号+6个多圈+12个单圈编码器值+标志
//位置号(0)
//1#多圈(1)+1#单圈_低(2)+1#单圈_高(3)+2#多圈(4)+2#单圈_低(5)+2#单圈_高(6)+
//3#多圈(7)+3#单圈_低(8)+3#单圈_高(9)+4#多圈(10)+4#单圈_低(11)+4#单圈_高(12)+
//5#多圈(13)+5#单圈_低(14)+5#单圈_高(15)+6#多圈(16)+6#单圈_低(17)+6#单圈_高(18)
#define w_ParLst_PosPar w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 6]

//40*30=1200个双字
//组号(0)+命令号(1)+位置号(2)+运行速度(3)+加减速时间(4)+暂停(5)+停止(6)+条件1(7)+条件2(8)+DO延时(9)+DO输出持续时间(10)+输出(11)
//+1#脉冲_低(12)+1#脉冲_高(13)+1#速度(14)+1#加减速(15)+2#脉冲_低(16)+2#脉冲_高(17)+2#速度(18)+2#加减速(19)
//+3#脉冲_低(20)+3#脉冲_高(21)+3#速度(22)+3#加减速(23)+4#脉冲_低(24)+4#脉冲_高(25)+4#速度(26)+4#加减速(27)
//+5#脉冲_低(28)+5#脉冲_高(29)+5#速度(30)+5#加减速(31)+6#脉冲_低(32)+6#脉冲_高(33)+6#速度(34)+6#加减速(35)
#define w_ParLst_Pos_CMD w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 6 + POS_SIZE * POS_NUM]

//最后一条命令
#define w_ParLst_LastPos_CMD w_ParLst_Drive[START_CMD_ADDR + FLASH_CMD_SIZE * 6 + FLASH_POS_SIZE + FLASH_POS_CMD_SIZE - POS_CMD_SIZE]

//定义通讯命令缓冲区长度
#define COM_QUERY_SIZE 10 //查询指令长度
#define COM_QUERY_NUM 10  //查询指令数量

#define COM_CMD_SIZE 20 //命令指令长度
#define COM_CMD_NUM 30  //命令指令数量

#define PULSE_NUM 5000         //电机每旋转一圈的脉冲数
#define ELEC_GEAR 8388608      //电子齿轮2^23=8388608
#define ELEC_GEAR_US200 131072 //电子齿轮2^17=131072

#define POS_SIZE 20 //每个数据点的长度，序号+6个多圈+12个单圈编码器值+标志
#define POS_NUM 15  //定义数据点的数量，15个
#define FLASH_POS_SIZE POS_SIZE *POS_NUM

#define POS_CMD_SIZE 40 //联动位置指令长度
//40*30=120个双字
//组号(0)+命令号(1)+位置号(2)+运行速度(3)+加减速时间(4)+暂停(5)+停止(6)+条件1(7)+条件2(8)+DO延时(9)+DO输出持续时间(10)+输出(11)
//+1#脉冲_低(12)+1#脉冲_高(13)+1#速度(14)+1#加减速(15)+2#脉冲_低(16)+2#脉冲_高(17)+2#速度(18)+2#加减速(19)
//+3#脉冲_低(20)+3#脉冲_高(21)+3#速度(22)+3#加减速(23)+4#脉冲_低(24)+4#脉冲_高(25)+4#速度(26)+4#加减速(27)
//+5#脉冲_低(28)+5#脉冲_高(29)+5#速度(30)+5#加减速(31)+6#脉冲_低(32)+6#脉冲_高(33)+6#速度(34)+6#加减速(35)
//运行时间(36)+填充数据标志(37)+已发送标志(38)
#define POS_CMD_NUM 30 //联动位置指令数量
#define FLASH_POS_CMD_SIZE POS_CMD_SIZE *POS_CMD_NUM

#define MOTOR_MAX_SPEED12 1500  //1#、2#电机最大转速1500r/min
#define MOTOR_MAX_SPEED3 2000   //3#电机最大转速2000r/min
#define MOTOR_MAX_SPEED456 3000 //3#电机最大转速3000r/min
#define D_RUN_TIME 36           //定义运行时间，在命令指令中是位置36

// 定义设定参数

#define Pw_Driver1_Pluse w_ParLst_Drive[0]     //1#手动运行脉冲（低字）
#define Pw_Driver1_Pluse_HW w_ParLst_Drive[1]  //1#手动运行脉冲（高字）
#define Pw_Driver2_Pluse w_ParLst_Drive[2]     //2#手动运行脉冲（低字）
#define Pw_Driver2_Pluse_HW w_ParLst_Drive[3]  //2#手动运行脉冲（高字）
#define Pw_Driver3_Pluse w_ParLst_Drive[4]     //3#手动运行脉冲（低字）
#define Pw_Driver3_Pluse_HW w_ParLst_Drive[5]  //3#手动运行脉冲（高字）
#define Pw_Driver4_Pluse w_ParLst_Drive[6]     //4#手动运行脉冲（低字）
#define Pw_Driver4_Pluse_HW w_ParLst_Drive[7]  //4#手动运行脉冲（高字）
#define Pw_Driver5_Pluse w_ParLst_Drive[8]     //5#手动运行脉冲（低字）
#define Pw_Driver5_Pluse_HW w_ParLst_Drive[9]  //5#手动运行脉冲（高字）
#define Pw_Driver6_Pluse w_ParLst_Drive[10]    //6#手动运行脉冲（低字）
#define Pw_Driver6_Pluse_HW w_ParLst_Drive[11] //6#手动运行脉冲（高字）

#define Pr_Drive1_Status1 w_ParLst_Drive[12] //1#伺服电机状态字，对应伺服地址P8901
#define Pr_Drive2_Status1 w_ParLst_Drive[13] //2#伺服电机状态字
#define Pr_Drive3_Status1 w_ParLst_Drive[14] //3#伺服电机状态字
#define Pr_Drive4_Status1 w_ParLst_Drive[15] //4#伺服电机状态字
#define Pr_Drive5_Status1 w_ParLst_Drive[16] //5#伺服电机状态字
#define Pr_Drive6_Status1 w_ParLst_Drive[17] //6#伺服电机状态字

#define Pw_Driver1_R_Enable w_ParLst_Drive[21] //1#手动运行使能（反转）
#define Pw_Driver2_R_Enable w_ParLst_Drive[22] //2#手动运行使能（反转）
#define Pw_Driver3_R_Enable w_ParLst_Drive[23] //3#手动运行使能（反转）
#define Pw_Driver4_R_Enable w_ParLst_Drive[24] //4#手动运行使能（反转）
#define Pw_Driver5_R_Enable w_ParLst_Drive[25] //5#手动运行使能（反转）
#define Pw_Driver6_R_Enable w_ParLst_Drive[26] //6#手动运行使能（反转）

#define Pw_Driver1_Speed w_ParLst_Drive[27]   //1#手动运行速度
#define Pw_Driver1_AccTime w_ParLst_Drive[28] //1#手动加减速时间

#define Pw_Driver2_Speed w_ParLst_Drive[30]   //2#手动运行速度
#define Pw_Driver2_AccTime w_ParLst_Drive[31] //2#手动加减速时间

#define Pw_Driver3_Speed w_ParLst_Drive[33]   //3#手动运行速度
#define Pw_Driver3_AccTime w_ParLst_Drive[34] //3#手动加减速时间

#define Pw_Driver4_Speed w_ParLst_Drive[36]   //4#手动运行速度
#define Pw_Driver4_AccTime w_ParLst_Drive[37] //4#手动加减速时间

#define Pw_Driver5_Speed w_ParLst_Drive[39]   //5#手动运行速度
#define Pw_Driver5_AccTime w_ParLst_Drive[40] //5#手动加减速时间

#define Pw_Driver6_Speed w_ParLst_Drive[42]   //6#手动运行速度
#define Pw_Driver6_AccTime w_ParLst_Drive[43] //6#手动加减速时间

#define Pw_Driver1_Enable w_ParLst_Drive[44] //1#手动运行使能（正转）
#define Pw_Driver2_Enable w_ParLst_Drive[45] //2#手动运行使能（正转）
#define Pw_Driver3_Enable w_ParLst_Drive[46] //3#手动运行使能（正转）
#define Pw_Driver4_Enable w_ParLst_Drive[47] //4#手动运行使能（正转）
#define Pw_Driver5_Enable w_ParLst_Drive[48] //5#手动运行使能（正转）
#define Pw_Driver6_Enable w_ParLst_Drive[49] //6#手动运行使能（正转）

#define Pw_StepAutoMode w_ParLst_Drive[50] //=1，手动模式；=0，全自动模式

#define Pw_Driver1_SetValue w_ParLst_Drive[51]    //1#伺服电机设定值
#define Pw_Driver1_SetValue_HW w_ParLst_Drive[52] //1#伺服电机设定值（高字）
#define Pw_Driver2_SetValue w_ParLst_Drive[53]    //1#伺服电机设定值
#define Pw_Driver2_SetValue_HW w_ParLst_Drive[54] //1#伺服电机设定值（高字）
#define Pw_Driver3_SetValue w_ParLst_Drive[55]    //1#伺服电机设定值
#define Pw_Driver3_SetValue_HW w_ParLst_Drive[56] //1#伺服电机设定值（高字）
#define Pw_Driver4_SetValue w_ParLst_Drive[57]    //1#伺服电机设定值
#define Pw_Driver4_SetValue_HW w_ParLst_Drive[58] //1#伺服电机设定值（高字）
#define Pw_Driver5_SetValue w_ParLst_Drive[59]    //1#伺服电机设定值
#define Pw_Driver5_SetValue_HW w_ParLst_Drive[60] //1#伺服电机设定值（高字）
#define Pw_Driver6_SetValue w_ParLst_Drive[61]    //1#伺服电机设定值
#define Pw_Driver6_SetValue_HW w_ParLst_Drive[62] //1#伺服电机设定值（高字）

#define Pw_SF_Driver1_InitPos w_ParLst_Drive[63]    //伺服电机1的初始位置
#define Pw_SF_Driver1_InitPos_HW w_ParLst_Drive[64] //伺服电机1的初始位置
#define Pw_SF_Driver2_InitPos w_ParLst_Drive[65]    //伺服电机2的初始位置
#define Pw_SF_Driver2_InitPos_HW w_ParLst_Drive[66] //伺服电机2的初始位置
#define Pw_SF_Driver3_InitPos w_ParLst_Drive[67]    //伺服电机3的初始位置
#define Pw_SF_Driver3_InitPos_HW w_ParLst_Drive[67] //伺服电机3的初始位置
#define Pw_SF_Driver4_InitPos w_ParLst_Drive[69]    //伺服电机4的初始位置
#define Pw_SF_Driver4_InitPos_HW w_ParLst_Drive[70] //伺服电机4的初始位置
#define Pw_SF_Driver5_InitPos w_ParLst_Drive[71]    //伺服电机5的初始位置
#define Pw_SF_Driver5_InitPos_HW w_ParLst_Drive[72] //伺服电机5的初始位置
#define Pw_SF_Driver6_InitPos w_ParLst_Drive[73]    //伺服电机6的初始位置
#define Pw_SF_Driver6_InitPos_HW w_ParLst_Drive[74] //伺服电机6的初始位置

#define Pw_Fault_Stop w_ParLst_Drive[75]      // 发生伺服故障停机功能，=1，停机；=0，不停机，仍然运行
#define Pw_Read_CurrentPos w_ParLst_Drive[76] // 读初始位置，=1，读位置
#define Pw_EMERGENCY_STOP w_ParLst_Drive[77]  // 急停命令，=1，进行急停
#define Pw_ResetCMD w_ParLst_Drive[78]        // 复位命令，=1，进行复位
#define Pw_Stop_Reset w_ParLst_Drive[79]      // 停机复位功能，=1，停机后进行复位；=0，停机后不复位

#define Pw_TouchRunStop w_ParLst_Drive[80] // 触摸 启动/停止
#define Pw_SaveDelay w_ParLst_Drive[81]    //默认延时180s保存修改后的参数到FLASH中

//#define	Pw_Driver1_Par_Init_F		w_ParLst_Drive[82]		//1#伺服电机参数初始化标志
//#define	Pw_Driver2_Par_Init_F		w_ParLst_Drive[83]		//2#伺服电机参数初始化标志
//#define	Pw_Driver3_Par_Init_F		w_ParLst_Drive[84]		//3#伺服电机参数初始化标志
//#define	Pw_Driver4_Par_Init_F		w_ParLst_Drive[85]		//4#伺服电机参数初始化标志
//#define	Pw_Driver5_Par_Init_F		w_ParLst_Drive[86]		//5#伺服电机参数初始化标志
//#define	Pw_Driver6_Par_Init_F		w_ParLst_Drive[87]		//6#伺服电机参数初始化标志

//#define	Pw_Reset_MaxValue			w_ParLst_Drive[88]		//复位最大限定值
//#define	Pw_Reset_MaxValue_HW		w_ParLst_Drive[89]		//复位最大限定值（高字）

#define Pr_current_pos1 w_ParLst_Drive[90] //1#伺服电机位置指令计数器，对应伺服地址P090A
#define Pr_current_pos1_HW w_ParLst_Drive[91]
#define Pr_current_pos2 w_ParLst_Drive[92]     //2#电机当前脉冲
#define Pr_current_pos2_HW w_ParLst_Drive[93]  //2#电机当前脉冲
#define Pr_current_pos3 w_ParLst_Drive[94]     //3#电机当前脉冲
#define Pr_current_pos3_HW w_ParLst_Drive[95]  //3#电机当前脉冲
#define Pr_current_pos4 w_ParLst_Drive[96]     //4#电机当前脉冲
#define Pr_current_pos4_HW w_ParLst_Drive[97]  //4#电机当前脉冲
#define Pr_current_pos5 w_ParLst_Drive[98]     //5#电机当前脉冲
#define Pr_current_pos5_HW w_ParLst_Drive[99]  //5#电机当前脉冲
#define Pr_current_pos6 w_ParLst_Drive[100]    //6#电机当前脉冲
#define Pr_current_pos6_HW w_ParLst_Drive[101] //6#电机当前脉冲

#define Pr_F_Drive1_Stop w_ParLst_Drive[102] //1#电机停止标志（位置到达）
#define Pr_F_Drive2_Stop w_ParLst_Drive[103] //2#电机停止标志（位置到达）
#define Pr_F_Drive3_Stop w_ParLst_Drive[104] //3#电机停止标志（位置到达）
#define Pr_F_Drive4_Stop w_ParLst_Drive[105] //4#电机停止标志（位置到达）
#define Pr_F_Drive5_Stop w_ParLst_Drive[106] //5#电机停止标志（位置到达）
#define Pr_F_Drive6_Stop w_ParLst_Drive[107] //6#电机停止标志（位置到达）

#define Pw_ComErrCount w_ParLst_Drive[108] //通讯故障延时判断

#define Pr_Driver_Running_No w_ParLst_Drive[111]  //当前执行的命令序号
#define Pr_Driver_Previous_No w_ParLst_Drive[112] //前一个执行的命令序号
//#define	Pr_Drive3_Run_No			w_ParLst_Drive[113]			//3#电机当前执行的命令序号
//#define	Pr_Drive4_Run_No			w_ParLst_Drive[114]			//4#电机当前执行的命令序号
//#define	Pr_Drive5_Run_No			w_ParLst_Drive[115]			//5#电机当前执行的命令序号
//#define	Pr_Drive6_Run_No			w_ParLst_Drive[116]			//6#电机当前执行的命令序号

#define Pr_F_Drive1_Runing w_ParLst_Drive[117] //1#电机运行标志
#define Pr_F_Drive2_Runing w_ParLst_Drive[118] //2#电机运行标志
#define Pr_F_Drive3_Runing w_ParLst_Drive[119] //3#电机运行标志
#define Pr_F_Drive4_Runing w_ParLst_Drive[120] //4#电机运行标志
#define Pr_F_Drive5_Runing w_ParLst_Drive[121] //5#电机运行标志
#define Pr_F_Drive6_Runing w_ParLst_Drive[122] //6#电机运行标志

#define Pr_F_Driver1_Rdy w_ParLst_Drive[123] //1#伺服电机准备好标志
#define Pr_F_Driver2_Rdy w_ParLst_Drive[124] //2#伺服电机准备好标志
#define Pr_F_Driver3_Rdy w_ParLst_Drive[125] //3#伺服电机准备好标志
#define Pr_F_Driver4_Rdy w_ParLst_Drive[126] //4#伺服电机准备好标志
#define Pr_F_Driver5_Rdy w_ParLst_Drive[127] //5#伺服电机准备好标志
#define Pr_F_Driver6_Rdy w_ParLst_Drive[128] //6#伺服电机准备好标志

#define Pr_Driver1_ComCount w_ParLst_Drive[129] //1#伺服电机通讯计数
#define Pr_Driver2_ComCount w_ParLst_Drive[130] //2#伺服电机通讯计数
#define Pr_Driver3_ComCount w_ParLst_Drive[131] //3#伺服电机通讯计数
#define Pr_Driver4_ComCount w_ParLst_Drive[132] //4#伺服电机通讯计数
#define Pr_Driver5_ComCount w_ParLst_Drive[133] //5#伺服电机通讯计数
#define Pr_Driver6_ComCount w_ParLst_Drive[134] //6#伺服电机通讯计数

#define Pr_Drive1_Status w_ParLst_Drive[135] //1#伺服电机状态字，对应伺服地址P8902
#define Pr_Drive2_Status w_ParLst_Drive[136] //2#伺服电机状态字
#define Pr_Drive3_Status w_ParLst_Drive[137] //3#伺服电机状态字
#define Pr_Drive4_Status w_ParLst_Drive[138] //4#伺服电机状态字
#define Pr_Drive5_Status w_ParLst_Drive[139] //5#伺服电机状态字
#define Pr_Drive6_Status w_ParLst_Drive[140] //6#伺服电机状态字

#define Pr_F_Drvier1_Err w_ParLst_Drive[141] //1#伺服电机故障标志
#define Pr_F_Drvier2_Err w_ParLst_Drive[142] //2#伺服电机故障标志
#define Pr_F_Drvier3_Err w_ParLst_Drive[143] //3#伺服电机故障标志
#define Pr_F_Drvier4_Err w_ParLst_Drive[144] //4#伺服电机故障标志
#define Pr_F_Drvier5_Err w_ParLst_Drive[145] //5#伺服电机故障标志
#define Pr_F_Drvier6_Err w_ParLst_Drive[146] //6#伺服电机故障标志

#define Pr_Drive1_FaultNo w_ParLst_Drive[147] //1#伺服电机当前最高级别故障码，对应伺服地址P0931
#define Pr_Drive2_FaultNo w_ParLst_Drive[148] //2#伺服电机当前最高级别故障码
#define Pr_Drive3_FaultNo w_ParLst_Drive[149] //3#伺服电机当前最高级别故障码
#define Pr_Drive4_FaultNo w_ParLst_Drive[150] //4#伺服电机当前最高级别故障码
#define Pr_Drive5_FaultNo w_ParLst_Drive[151] //5#伺服电机当前最高级别故障码
#define Pr_Drive6_FaultNo w_ParLst_Drive[152] //6#伺服电机当前最高级别故障码

#define Pw_Com_Delay1 w_ParLst_Drive[153] //COM延时1
#define Pw_Com_Delay2 w_ParLst_Drive[154] //COM延时2
//#define	Pw_Com2_Delay1						w_ParLst_Drive[155]			//COM2延时1
//#define	Pw_Com2_Delay2						w_ParLst_Drive[156]			//COM2延时2
//#define	Pw_Com3_Delay1						w_ParLst_Drive[157]			//COM3延时1
//#define	Pw_Com3_Delay2						w_ParLst_Drive[158]			//COM3延时2
//#define	Pw_Com3_Delay3						w_ParLst_Drive[159]			//COM3延时3
//#define	Pw_Com4_Delay2						w_ParLst_Drive[160]			//COM4延时2

#define Pr_Drive1_singleData w_ParLst_Drive[161]    //1#伺服电机编码器单圈数据，对应伺服地址P091A
#define Pr_Drive1_singleData_HW w_ParLst_Drive[162] //1#伺服电机编码器单圈数据（高字）
#define Pr_Drive2_singleData w_ParLst_Drive[163]    //2#伺服电机编码器单圈数据，对应伺服地址P091A
#define Pr_Drive2_singleData_HW w_ParLst_Drive[164] //2#伺服电机编码器单圈数据（高字）
#define Pr_Drive3_singleData w_ParLst_Drive[165]    //3#伺服电机编码器单圈数据，对应伺服地址P091A
#define Pr_Drive3_singleData_HW w_ParLst_Drive[166] //3#伺服电机编码器单圈数据（高字）
#define Pr_Drive4_singleData w_ParLst_Drive[167]    //4#伺服电机编码器单圈数据，对应伺服地址P091A
#define Pr_Drive4_singleData_HW w_ParLst_Drive[168] //4#伺服电机编码器单圈数据（高字）
#define Pr_Drive5_singleData w_ParLst_Drive[169]    //5#伺服电机编码器单圈数据，对应伺服地址P091A
#define Pr_Drive5_singleData_HW w_ParLst_Drive[170] //5#伺服电机编码器单圈数据（高字）
#define Pr_Drive6_singleData w_ParLst_Drive[171]    //6#伺服电机编码器单圈数据，对应伺服地址P091A
#define Pr_Drive6_singleData_HW w_ParLst_Drive[172] //6#伺服电机编码器单圈数据（高字）

#define Pr_Drive1_MultiData w_ParLst_Drive[173] //1#伺服电机编码器多圈数据，对应伺服地址P091C
#define Pr_Drive2_MultiData w_ParLst_Drive[174] //2#伺服电机编码器多圈数据，对应伺服地址P091C
#define Pr_Drive3_MultiData w_ParLst_Drive[175] //3#伺服电机编码器多圈数据，对应伺服地址P091C
#define Pr_Drive4_MultiData w_ParLst_Drive[176] //4#伺服电机编码器多圈数据，对应伺服地址P091C
#define Pr_Drive5_MultiData w_ParLst_Drive[177] //5#伺服电机编码器多圈数据，对应伺服地址P091C
#define Pr_Drive6_MultiData w_ParLst_Drive[178] //6#伺服电机编码器多圈数据，对应伺服地址P091C

#define Pr_Drive1_singleData_Init w_ParLst_Drive[179]    //1#伺服电机编码器单圈初始设定值，对应伺服地址P091A
#define Pr_Drive1_singleData_Init_HW w_ParLst_Drive[180] //1#伺服电机编码器单圈初设定值（高字）
#define Pr_Drive2_singleData_Init w_ParLst_Drive[181]    //2#伺服电机编码器单圈初始设定值，对应伺服地址P091A
#define Pr_Drive2_singleData_Init_HW w_ParLst_Drive[182] //2#伺服电机编码器单圈初始设定值（高字）
#define Pr_Drive3_singleData_Init w_ParLst_Drive[183]    //3#伺服电机编码器单圈初始设定值，对应伺服地址P091A
#define Pr_Drive3_singleData_Init_HW w_ParLst_Drive[184] //3#伺服电机编码器单圈初始设定值（高字）
#define Pr_Drive4_singleData_Init w_ParLst_Drive[185]    //4#伺服电机编码器单圈初始设定值，对应伺服地址P091A
#define Pr_Drive4_singleData_Init_HW w_ParLst_Drive[186] //4#伺服电机编码器单圈初始设定值（高字）
#define Pr_Drive5_singleData_Init w_ParLst_Drive[187]    //5#伺服电机编码器单圈初始设定值，对应伺服地址P091A
#define Pr_Drive5_singleData_Init_HW w_ParLst_Drive[188] //5#伺服电机编码器单圈初始设定值（高字）
#define Pr_Drive6_singleData_Init w_ParLst_Drive[189]    //6#伺服电机编码器单圈初始设定值，对应伺服地址P091A
#define Pr_Drive6_singleData_Init_HW w_ParLst_Drive[190] //6#伺服电机编码器单圈初始设定值（高字）

#define Pr_Drive1_MultiData_Init w_ParLst_Drive[191] //1#伺服电机编码器多圈初始设定值，对应伺服地址P091C
#define Pr_Drive2_MultiData_Init w_ParLst_Drive[192] //2#伺服电机编码器多圈初始设定值，对应伺服地址P091C
#define Pr_Drive3_MultiData_Init w_ParLst_Drive[193] //3#伺服电机编码器多圈初始设定值，对应伺服地址P091C
#define Pr_Drive4_MultiData_Init w_ParLst_Drive[194] //4#伺服电机编码器多圈初始设定值，对应伺服地址P091C
#define Pr_Drive5_MultiData_Init w_ParLst_Drive[195] //5#伺服电机编码器多圈初始设定值，对应伺服地址P091C
#define Pr_Drive6_MultiData_Init w_ParLst_Drive[196] //6#伺服电机编码器多圈初始设定值，对应伺服地址P091C

#define Pr_Reset_Delay w_ParLst_Drive[197] //复位到初始位置延时时间，ms

#define Pw_PosError_Set w_ParLst_Drive[198]    //位置偏差设定
#define Pw_PosError_Set_HW w_ParLst_Drive[199] //位置偏差设定（高字）

//#define Pw_Drive1_P8910						w_ParLst_Drive[201]			//1#伺服电机P8910参数
//#define Pw_Drive2_P8910						w_ParLst_Drive[202]			//2#伺服电机P8910参数
//#define Pw_Drive3_P8910						w_ParLst_Drive[203]			//3#伺服电机P8910参数
//#define Pw_Drive4_P8910						w_ParLst_Drive[204]			//4#伺服电机P8910参数
//#define Pw_Drive5_P8910						w_ParLst_Drive[205]			//5#伺服电机P8910参数
//#define Pw_Drive6_P8910						w_ParLst_Drive[206]			//6#伺服电机P8910参数

//#define Pw_Driver_AllSavePos_Enable			w_ParLst_Drive[207]			//所有伺服电机写位置参数使能
//#define Pw_Driver1_SavePos_Enable			w_ParLst_Drive[208]			//1#伺服电机写位置参数使能
//#define Pw_Driver2_SavePos_Enable			w_ParLst_Drive[209]			//2#伺服电机写位置参数使能
//#define Pw_Driver3_SavePos_Enable			w_ParLst_Drive[210]			//3#伺服电机写位置参数使能
//#define Pw_Driver4_SavePos_Enable			w_ParLst_Drive[211]			//4#伺服电机写位置参数使能
//#define Pw_Driver5_SavePos_Enable			w_ParLst_Drive[212]			//5#伺服电机写位置参数使能
//#define Pw_Driver6_SavePos_Enable			w_ParLst_Drive[213]			//6#伺服电机写位置参数使能

#define Pw_Set_Run_Speed1 w_ParLst_Drive[214] //1#伺服电机设定最大运行速度
#define Pw_Set_Run_Speed2 w_ParLst_Drive[215] //2#伺服电机设定最大运行速度
#define Pw_Set_Run_Speed3 w_ParLst_Drive[216] //3#伺服电机设定最大运行速度
#define Pw_Set_Run_Speed4 w_ParLst_Drive[217] //4#伺服电机设定最大运行速度
#define Pw_Set_Run_Speed5 w_ParLst_Drive[218] //5#伺服电机设定最大运行速度
#define Pw_Set_Run_Speed6 w_ParLst_Drive[219] //6#伺服电机设定最大运行速度

#define Pw_Com_Delay3 w_ParLst_Drive[220]       //COM延时3
#define Pw_Com_Delay_Manual w_ParLst_Drive[221] //COM延时，手动

#define Pw_Define_Save_Pos w_ParLst_Drive[222] //定义并记录位置命令
#define Pw_Cal_Pos_CMD w_ParLst_Drive[223]     //计算位置及速度命令
#define Pw_Verify_Pos_CMD w_ParLst_Drive[224]  //校验位置命令
#define Pw_Running_Pos_CMD w_ParLst_Drive[225] //正在运行的命令号
#define Pw_Step_Pos_CMD w_ParLst_Drive[226]    //单步运行命令
#define Pw_Current_Pos_No w_ParLst_Drive[227]  //当前位置号[1-15]
#define Pw_Set_Run_Speed w_ParLst_Drive[228]   //基准运行速度
#define Pw_Read_Init_Pos w_ParLst_Drive[229]   //保存原点位置命令

#define Pw_Com1_Driver1_BufferNum w_ParLst_Drive[230] //1#命令队列中的条数
#define Pw_Com1_Driver2_BufferNum w_ParLst_Drive[231] //2#命令队列中的条数
#define Pw_Com2_Driver3_BufferNum w_ParLst_Drive[232] //3#命令队列中的条数
#define Pw_Com2_Driver4_BufferNum w_ParLst_Drive[233] //4#命令队列中的条数
#define Pw_Com3_Driver5_BufferNum w_ParLst_Drive[234] //5#命令队列中的条数
#define Pw_Com3_Driver6_BufferNum w_ParLst_Drive[235] //6#命令队列中的条数
#define Pw_StopStatus_Delay w_ParLst_Drive[236]       //停止状态检测延时
#define Pw_Acc_Delay_Ratio w_ParLst_Drive[237]        //加减速时间延时比例
#define Pw_Current_Run_Time w_ParLst_Drive[238]       //当前指令运行时间

#define Pw_Driver1_AutoSpeed w_ParLst_Drive[240] //1#当前运行速度
#define Pw_Driver2_AutoSpeed w_ParLst_Drive[241] //2#当前运行速度
#define Pw_Driver3_AutoSpeed w_ParLst_Drive[242] //3#当前运行速度
#define Pw_Driver4_AutoSpeed w_ParLst_Drive[243] //4#当前运行速度
#define Pw_Driver5_AutoSpeed w_ParLst_Drive[244] //5#当前运行速度
#define Pw_Driver6_AutoSpeed w_ParLst_Drive[245] //6#当前运行速度
#define Pw_EquipStatus w_ParLst_Drive[246]       //设备状态，低字
#define Pw_EquipStatus_HW w_ParLst_Drive[247]    //设备状态，高字
#define Pw_Run_Mode w_ParLst_Drive[248]          //运行模式，=0，喷漆模式；=1，清洗模式
#define Pw_JDQ_Addr w_ParLst_Drive[249]          //继电器板通讯地址

#define Pw_JDQ_Control w_ParLst_Drive[250]    //继电器板DO1控制字，=0断开；=1闭合
#define Pr_JDQ_Status w_ParLst_Drive[251]     //继电器板DO1状态字
#define Pr_Com4_ComCount w_ParLst_Drive[252]  //Com4通讯计数
#define Pw_Brake_Delay w_ParLst_Drive[253]    //开刹车延时
#define Pr_runtime_show w_ParLst_Drive[254]   //指令延时时间
#define Pr_pausetime_show w_ParLst_Drive[255] //指令暂停时间

#define Pr_cyclecounter_show w_ParLst_Drive[256] //程序循环计数
#define Pr_cyclecounter_HW w_ParLst_Drive[257]   //程序循环计算次数

#define Pw_Driver_Run_MinSpeed w_ParLst_Drive[258] //电机的最小运行速度，要比伺服设定的电机旋转检出阈值至少大10rpm

#define Pr_Driver1_NeverRun w_ParLst_Drive[259] //1#电机从没有运行标志
#define Pr_Driver2_NeverRun w_ParLst_Drive[260] //2#电机从没有运行标志
#define Pr_Driver3_NeverRun w_ParLst_Drive[261] //3#电机从没有运行标志
#define Pr_Driver4_NeverRun w_ParLst_Drive[262] //4#电机从没有运行标志
#define Pr_Driver5_NeverRun w_ParLst_Drive[263] //5#电机从没有运行标志
#define Pr_Driver6_NeverRun w_ParLst_Drive[264] //6#电机从没有运行标志

#define Pw_Driver1_PosErr_Muti w_ParLst_Drive[265] //1#电机编码器位置偏差，多圈
#define Pw_Driver2_PosErr_Muti w_ParLst_Drive[266] //2#电机编码器位置偏差，多圈
#define Pw_Driver3_PosErr_Muti w_ParLst_Drive[267] //3#电机编码器位置偏差，多圈
#define Pw_Driver4_PosErr_Muti w_ParLst_Drive[268] //4#电机编码器位置偏差，多圈
#define Pw_Driver5_PosErr_Muti w_ParLst_Drive[269] //5#电机编码器位置偏差，多圈
#define Pw_Driver6_PosErr_Muti w_ParLst_Drive[270] //6#电机编码器位置偏差，多圈

#define Pw_Driver1_PosErr_Sing w_ParLst_Drive[271]    //1#电机编码器位置偏差，单圈，低位
#define Pw_Driver1_PosErr_Sing_HW w_ParLst_Drive[272] //1#电机编码器位置偏差，单圈，高位
#define Pw_Driver2_PosErr_Sing w_ParLst_Drive[273]    //2#电机编码器位置偏差，单圈，低位
#define Pw_Driver2_PosErr_Sing_HW w_ParLst_Drive[274] //2#电机编码器位置偏差，单圈，高位
#define Pw_Driver3_PosErr_Sing w_ParLst_Drive[275]    //3#电机编码器位置偏差，单圈，低位
#define Pw_Driver3_PosErr_Sing_HW w_ParLst_Drive[276] //3#电机编码器位置偏差，单圈，高位
#define Pw_Driver4_PosErr_Sing w_ParLst_Drive[277]    //4#电机编码器位置偏差，单圈，低位
#define Pw_Driver4_PosErr_Sing_HW w_ParLst_Drive[278] //4#电机编码器位置偏差，单圈，高位
#define Pw_Driver5_PosErr_Sing w_ParLst_Drive[279]    //5#电机编码器位置偏差，单圈，低位
#define Pw_Driver5_PosErr_Sing_HW w_ParLst_Drive[280] //5#电机编码器位置偏差，单圈，高位
#define Pw_Driver6_PosErr_Sing w_ParLst_Drive[281]    //6#电机编码器位置偏差，单圈，低位
#define Pw_Driver6_PosErr_Sing_HW w_ParLst_Drive[282] //6#电机编码器位置偏差，单圈，高位

#define Pw_Pos_Adj_Cmd w_ParLst_Drive[283]       //电机编码器位置偏差手动调整命令
#define Pw_AllStopped_Delay w_ParLst_Drive[284]  //判断所有电机位置到达延时
#define Pw_Write_Timeout_Set w_ParLst_Drive[285] //写位置指令超时时间设定
#define Pr_Send_Data_F w_ParLst_Drive[286]       //发送数据标志
#define Pr_Driver1_Cmd_OK_F w_ParLst_Drive[287]  //1#伺服发送数据OK标志
#define Pr_Driver2_Cmd_OK_F w_ParLst_Drive[288]  //2#伺服发送数据OK标志
#define Pr_Driver3_Cmd_OK_F w_ParLst_Drive[289]  //3#伺服发送数据OK标志
#define Pr_Driver4_Cmd_OK_F w_ParLst_Drive[290]  //4#伺服发送数据OK标志

#define Pr_Driver5_Cmd_OK_F w_ParLst_Drive[291]     //5#伺服发送数据OK标志
#define Pr_Driver6_Cmd_OK_F w_ParLst_Drive[292]     //6#伺服发送数据OK标志
#define Pr_AllDriver_Cmd_OK_F w_ParLst_Drive[293]   //所有伺服发送数据OK标志
#define Pr_HaveDriver_Cmd_Err_F w_ParLst_Drive[294] //有伺服发送数据错误标志
#define Pr_OverMaxPos_F w_ParLst_Drive[295]         //超出位置范围标志

#endif
