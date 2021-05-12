/**
  ******************************************************************************

	******************************************************************************
	*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __WK_PORT1_H
#define __WK_PORT1_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "GlobalConst.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//void GPIO_Com1_Configuration(void);							//GPIO配置
//void Com1_config(void);
//void Com1_Init(void);
void Wk_Port1_RcvProcess(void);
void Wk_Port1_SlaveSend(void);	// 串口0从机发送
void Wk_Port1_MasterSend(void); // 串口0主发送程序　

#endif /* __WK_PORT1_H */

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
