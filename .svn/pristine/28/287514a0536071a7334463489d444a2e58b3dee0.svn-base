/**
  ******************************************************************************
  * @file    DoWith.h
  * @author  ChengLei Zhou  - 周成磊
  * @version V1.27
  * @date    2014-01-03
  * @brief   数字量输入检测，数字量输出,模拟量输入检测，模拟量输出,其他本机操作
	******************************************************************************
	*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __DOWITH_H
#define __DOWITH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "typedef.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ReadWriteRealTime(void); // 读写实时时钟 ISL1208
void delay(void);             // 延时
void Variable_Init(void);     // 变量初始化
void ParLst_Init(void);       // RAM中参数表列 初始化 (读出)
void ParArrayRead_Word(uint32_t *p_Top, uc32 *p_Base, uint w_ReadSize);
void FilterDI(void); // 过滤开关量输入
void DigitalIn(void);
//void DOConfigValue(uchar DOValue,uchar DO_BitNo);			// DO_BitNo:DO 位号
void ParLimit(void); // 参数限制
//void DoWith(void);					// 一些数据,记录的处理
void Manual_Control(void); //手动控制启停

void ParArrayWrite(uint16_t *p_Top, uint16_t *p_Base, uint16_t w_WriteSize);
void ParArrayRead(uint16_t *p_Top, uc16 *p_Base, uint16_t w_ReadSize);
void Boot_ParLst(void);    // 初始化设定参数
void SavePar_Prompt(void); // 保存参数+状态提示
void ForceSavePar(void);   // 强制保存参数

void Time_Output(void);       // 软件时钟输出	 2008.10.21
void EquipStatus(void);       // 设备状态
void ReadWriteRealTime(void); // 读写实时时钟 ISL12087
void KglStatus(void);         // 开关量状态

#endif /* __DOWITH_H */

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
