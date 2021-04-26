/** 
  ******************************************************************************

	******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "wk_port1.h"
#include "GlobalV_Extern.h" // 全局变量声明
//#include "GlobalConst.h"
#include <stdio.h> //加上此句可以用printf
//#include "CRCdata.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
uint8_t WkPort1_TxdBuffer[TXD1_MAX]; // 发送缓冲区
uint8_t WkPort1_RcvBuffer[RCV1_MAX]; // 接收缓冲区

uint8_t WkPort2_RcvBuffer[RCV1_MAX]; // 接收缓冲区
uint8_t WkPort3_RcvBuffer[RCV1_MAX]; // 接收缓冲区
uint8_t WkPort4_RcvBuffer[RCV1_MAX]; // 接收缓冲区

//uint8_t		Tmp_Txd1Buffer[1],Tmp_Rxd1Buffer[1];
uint16_t WkPort1_RcvCounter;  // 接收计数器//
uint16_t WkPort1_TxdCounter;  // 发送计数器//
uint16_t WkPort1_bakRcvCount; // 接收计数器//
uint16_t WkPort1_TxdMax;      // 有多少个字符需要发送//
uint16_t WkPort1_TxdChkSum;   // 发送校验和，lo,hi 两位//
uint16_t WkPort1_RegAddr;     // 串口1寄存器地址
//
//uint8_t		B_Com1Send;

//uint8_t		B_Com1Cmd03;
//uint8_t		B_Com1Cmd16;
//uint8_t		B_Com1Cmd01;
//uint8_t		B_Com1Cmd06;
//uint8_t		B_Com1Cmd73;
//uint16_t		T_NoRcv1Count;						// 没有接收计数器
//uint16_t		C_NoRcv1Count;
//uint16_t     C_Com1_MasterSendDelay;
//uint16_t	   T_Com1_MasterSendDelay;
//uint16_t     C_Com1_MasterSendDelay2;
//uint16_t	   T_Com1_MasterSendDelay2;
//uint16_t     C_Com1_MasterSendDelay5;
//uint16_t	   T_Com1_MasterSendDelay5;

//uint8_t	Com1_Query_PC=0;		//COM1查询缓冲区指针
//uint8_t	Com1_CMD_PC=0;			//COM1命令缓冲区指针

//s16	Com1_Driver1_Queue_Rear;			//命令队列中的命令数量，队尾指针
//s16	Com1_Driver2_Queue_Rear;			//命令队列中的命令数量，队尾指针

//s16	Com1_Driver1_Queue_Front;			//命令队列中的当前要出队的，队头指针
//s16	Com1_Driver2_Queue_Front;			//命令队列中的当前要出队的，队头指针

//uint8_t	Com1_QReceiveDataTargetLength;	//接收的数据应该的长度

//extern	UART_HandleTypeDef huart1;
////判断COM1命令队列是否为空
//extern	uint8_t Com1_Driver1_Queue_isEmpty(void);
//extern	uint8_t Com1_Driver2_Queue_isEmpty(void);
//extern	uint8_t Com2_Driver3_Queue_isEmpty(void);
//extern	uint8_t Com2_Driver4_Queue_isEmpty(void);
//extern	uint8_t Com3_Driver5_Queue_isEmpty(void);
//extern	uint8_t Com3_Driver6_Queue_isEmpty(void);

////判断COM1命令队列是否为满
//extern	uint8_t Com1_Driver1_Queue_isFull(void);
//extern	uint8_t Com1_Driver2_Queue_isFull(void);
//extern	uint8_t Com2_Driver3_Queue_isFull(void);
//extern	uint8_t Com2_Driver4_Queue_isFull(void);
//extern	uint8_t Com3_Driver5_Queue_isFull(void);
//extern	uint8_t Com3_Driver6_Queue_isFull(void);

////定义每台电机的写命令缓冲区
////每条定义：序号+命令长度+命令内容
//extern	uint8_t	Com1_Driver1_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];	//COM_CMD_SIZE=30，COM_CMD_NUM=6.定义6条写命令，每条30个字节
//extern	uint8_t	Com1_Driver2_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];
//extern	uint8_t	Com2_Driver3_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];
//extern	uint8_t	Com2_Driver4_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];
//extern	uint8_t	Com3_Driver5_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];
//extern	uint8_t	Com3_Driver6_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];

////定义每台电机的上一条命令
////定义：序号+命令长度+命令内容
//extern	uint8_t	Com1_LAST_CMD_BUFF[COM_CMD_SIZE];	//定义1条，30个字节
//extern	uint8_t	Com2_LAST_CMD_BUFF[COM_CMD_SIZE];
//extern	uint8_t	Com3_LAST_CMD_BUFF[COM_CMD_SIZE];
//extern	uint8_t	Com4_LAST_CMD_BUFF[COM_CMD_SIZE];

////查询指令表
//extern	uint8_t	Com1_Query_Data[];
//extern	uint8_t	Com2_Query_Data[];
//extern	uint8_t	Com3_Query_Data[];

//extern	uint16_t	Driver1_RcvCount;		//接收计数
//extern	uint16_t	Driver2_RcvCount;		//接收计数

//uint16_t	Com1_Send_Sort=0;				//Com1发送命令顺序，是发1#命令队列，还是发2#命令队列
//uint16_t	Com1_Send_CMD_Type_Sort=0;	//Com1是发送查询还是发送控制命令，=0，查询；=1，控制命令

///* Private function prototypes -----------------------------------------------*/
//void GPIO_Com1_Configuration(void);							//GPIO配置
//void Com1_config(void);
//void Com1_Driver1_Send_CMD(void);					//发送命令
//void Com1_Driver2_Send_CMD(void);					//发送命令
//uint16_t CRC16(uint8_t * pCrcData,uint8_t CrcDataLen);
//void Delay_MS(vu16 nCount);
//uint16_t Address(uint16_t *p, uint16_t Area); 					//绝对地址
//extern	void PowerDelay(uint16_t nCount);

//extern	uint8_t	F_Sync_6_axis;						//6轴同步输出标志

//extern	uint8_t	F_Driver1_Send_Cmd;					//控制器给1#伺服驱动器发出位置命令标志，=1表示已发送命令
//extern	uint8_t	F_Driver2_Send_Cmd;
//extern	uint8_t	F_Driver3_Send_Cmd;
//extern	uint8_t	F_Driver4_Send_Cmd;
//extern	uint8_t	F_Driver5_Send_Cmd;
//extern	uint8_t	F_Driver6_Send_Cmd;

//extern	uint8_t	F_Driver1_Timeout;					//1#伺服驱动器发送命令超时标志，=1表示超时
//extern	uint8_t	F_Driver2_Timeout;
//extern	uint8_t	F_Driver3_Timeout;
//extern	uint8_t	F_Driver4_Timeout;
//extern	uint8_t	F_Driver5_Timeout;
//extern	uint8_t	F_Driver6_Timeout;

//extern	uint8_t	F_Driver1_Cmd_Err;					//1#伺服驱动器位置命令错误标志，=1表示错误
//extern	uint8_t	F_Driver2_Cmd_Err;
//extern	uint8_t	F_Driver3_Cmd_Err;
//extern	uint8_t	F_Driver4_Cmd_Err;
//extern	uint8_t	F_Driver5_Cmd_Err;
//extern	uint8_t	F_Driver6_Cmd_Err;

//extern	uint8_t	F_Driver1_Cmd_Con_Err;					//1#伺服驱动器控制命令错误标志，=1表示错误
//extern	uint8_t	F_Driver2_Cmd_Con_Err;
//extern	uint8_t	F_Driver3_Cmd_Con_Err;
//extern	uint8_t	F_Driver4_Cmd_Con_Err;
//extern	uint8_t	F_Driver5_Cmd_Con_Err;
//extern	uint8_t	F_Driver6_Cmd_Con_Err;

//extern	uint8_t	Driver1_Cmd_PosNo;					//1#伺服驱动器发送的命令位置号，位置0或者位置1
//extern	uint8_t	Driver2_Cmd_PosNo;
//extern	uint8_t	Driver3_Cmd_PosNo;
//extern	uint8_t	Driver4_Cmd_PosNo;
//extern	uint8_t	Driver5_Cmd_PosNo;
//extern	uint8_t	Driver6_Cmd_PosNo;

//extern	uint8_t	Driver1_Cmd_Status;					//1#伺服驱动器P8910状态
//extern	uint8_t	Driver2_Cmd_Status;
//extern	uint8_t	Driver3_Cmd_Status;
//extern	uint8_t	Driver4_Cmd_Status;
//extern	uint8_t	Driver5_Cmd_Status;
//extern	uint8_t	Driver6_Cmd_Status;

//extern	uint16_t	T_Driver1_WriteCMD;
//extern	uint16_t	C_Driver1_WriteCMD;
//extern	uint16_t	T_Driver2_WriteCMD;
//extern	uint16_t	C_Driver2_WriteCMD;
//extern	uint16_t	T_Driver3_WriteCMD;
//extern	uint16_t	C_Driver3_WriteCMD;
//extern	uint16_t	T_Driver4_WriteCMD;
//extern	uint16_t	C_Driver4_WriteCMD;
//extern	uint16_t	T_Driver5_WriteCMD;
//extern	uint16_t	C_Driver5_WriteCMD;
//extern	uint16_t	T_Driver6_WriteCMD;
//extern	uint16_t	C_Driver6_WriteCMD;

//extern	uint8_t Driver1_Cmd_Data[9];							//1#伺服驱动器命令数据,4个字节为脉冲，2个字节为速度，2个字节为加减速时间，最后一个字节为位置号（0/1）
//extern	uint8_t Driver2_Cmd_Data[9];
//extern	uint8_t Driver3_Cmd_Data[9];
//extern	uint8_t Driver4_Cmd_Data[9];
//extern	uint8_t Driver5_Cmd_Data[9];
//extern	uint8_t Driver6_Cmd_Data[9];

//extern	s32 *arr_p1;

//extern	uint8_t Driver1_Pos_Start_Sort;						//1#伺服控制是发位置指令，还是发启动命令。=0，发位置；=1，发启动命令
//extern	uint8_t Driver2_Pos_Start_Sort;
//extern	uint8_t Driver3_Pos_Start_Sort;
//extern	uint8_t Driver4_Pos_Start_Sort;
//extern	uint8_t Driver5_Pos_Start_Sort;
//extern	uint8_t Driver6_Pos_Start_Sort;

//extern	uint8_t Driver1_Status_Sort;							//1#伺服，=2，表示已经发送；=3，表示已经接收
//extern	uint8_t Driver2_Status_Sort;
//extern	uint8_t Driver3_Status_Sort;
//extern	uint8_t Driver4_Status_Sort;
//extern	uint8_t Driver5_Status_Sort;
//extern	uint8_t Driver6_Status_Sort;

///* Private functions ---------------------------------------------------------*/

////接收处理程序 校验程序
//void Com1_RcvProcess(void)
//{
//	uint8_t k,s,i=0;       						// 临时变量
//    uint16_t j;
//	//作为主机,指定接收时间到了,就可以处理接收到的字符串了
//	// 在没收到串行字符的时间超过设定时，可以对接收缓存进行处理了
//	// **********************************rcv_counter<>0,收到字符才能处理
//	if ( Rcv1Counter>0 &&  T_NoRcv1Count!=SClk1Ms )
//	{								// 接收处理过程
//		T_NoRcv1Count=SClk1Ms;				//
//		C_NoRcv1Count++;
//		if ( C_NoRcv1Count>NORCVMAXMS )				//
//		{
//			/* Disable the UART Parity Error Interrupt and RXNE interrupt*/
////			CLEAR_BIT(huart1->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
//			BakRcv1Count=Rcv1Counter;		// 把 Rcv1Counter 保存
//			C_NoRcv1Count=0;				// 清没有接收计数器
//			//
//			if(BakRcv1Count<=RCV1_MAX)		// 接收长度正确,继续处理.
//			{
//				// 从地址检测－接收到的上位机查询指令  与文本通讯
//				if( Rcv1Buffer[0]==Pw_EquipmentNo1 )
//				{
//					j=CRC16( Rcv1Buffer, BakRcv1Count-2);		// CRC 校验
//					k=j>>8;
//					s=j;
//					if ( k==Rcv1Buffer[BakRcv1Count-2]
//						&& s==Rcv1Buffer[BakRcv1Count-1] )
//					{											// CRC校验正确
//						if ( Rcv1Buffer[1]==3 )		// 03读取保持寄存器
//						{
//							B_Com1Cmd03=1;
//							j=Rcv1Buffer[2];
//							w_Com1RegAddr=(j<<8)+Rcv1Buffer[3];
//						}
//						else if ( Rcv1Buffer[1]==16 )	// 16预置多寄存器
//						{
////							C_ForceSavPar=0;		// 强制保存参数计数器=0
//							B_Com1Cmd16=1;
//							j=Rcv1Buffer[2];
//							w_Com1RegAddr=(j<<8)+Rcv1Buffer[3];
//						}
//						else if ( Rcv1Buffer[1]==1 )		// 01读取线圈状态
//						{
//							B_Com1Cmd01=1;
//						}
//						else if ( Rcv1Buffer[1]==6 )		// 06预置单寄存器
//						{
////							C_ForceSavPar=0;		// 强制保存参数计数器=0
//							B_Com1Cmd06=1;
//							j=Rcv1Buffer[2];
//							w_Com1RegAddr=(j<<8)+Rcv1Buffer[3];
//						}
//
//						else
//							i=1;
//					}
//					else
//						i=2;
//				}
//			}
//			else
//				i=4;
////			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//			HAL_UART_Receive_IT(&huart1, (uint8_t *)&Tmp_Rxd1Buffer, 1);
//			Rcv1Counter=0;					// 准备下次接收到缓存开始
//		}
//	}
//	if(i>0)
//	{
//		for(j=0;j<20;j++)
//		{
//			Rcv1Buffer[j]=0;
//		}
////		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//		HAL_UART_Receive_IT(&huart1, (uint8_t *)&Tmp_Rxd1Buffer, 1);
//	}
//}

//void Com1_SlaveSend(void)			// 串口1从机发送
//{
//	uint16_t	m,n;
//	uint8_t	j=0,k;
//	uint16_t	 *p_wRead;
//	s32	 *p_wRead2;
//	uint8_t	 *p_bMove;
//	uint8_t	 *p_bGen;
//	uint16_t	 *p_wTarget;			// 指向目标字符串　xdata zcl
//	s32	 *p_wTarget2;

//	//
//	if ( B_Com1Cmd03 )		// 读取保持寄存器
//	{
//		Txd1Buffer[0]=Rcv1Buffer[0];	// 设备从地址Pw_EquipmentNo
//		Txd1Buffer[1]=Rcv1Buffer[1];	// 功能码
//		Txd1Buffer[2]=Rcv1Buffer[5]*2;	// Rcv1Buffer[5]=字数 　
//		//
//		if ( w_Com1RegAddr<0x800 && Pw_ComBufType==1 )	// 常规查询 Pw_ComBufType==1 2016.4.21
//		{
//			p_wRead=w_ParLst;			// PAR区
//			p_bMove=Txd1Buffer;
//			//
//			for ( k=0;k<Rcv1Buffer[5] ;k++ )	// 填充查询内容
//			{
//				m=*(p_wRead+w_Com1RegAddr+k);
//				*(p_bMove+3+k*2)=m>>8;
//				*(p_bMove+3+k*2+1)=m;
//			}
//		}
//		else if ( w_Com1RegAddr>=5000 && Pw_ComBufType==1 )		// 读取电机设定参数，参数地址减5000
//		{
//			p_wRead2=w_ParLst_Drive;			// PAR区
//			p_bMove=Txd1Buffer;
//			//
//			for ( k=0;k<Rcv1Buffer[5] ;k++ )	// 填充查询内容
//			{
//				m=*(p_wRead2+w_Com1RegAddr-5000+k);		//参数地址减5000
//				*(p_bMove+3+k*2)=m>>8;
//				*(p_bMove+3+k*2+1)=m;
//			}
//		}
//		//
//		w_Txd1ChkSum=CRC16( Txd1Buffer, Txd1Buffer[2]+3 );
//		Txd1Buffer[Txd1Buffer[2]+3]=w_Txd1ChkSum>>8;			// /256
//		Txd1Buffer[Txd1Buffer[2]+4]=w_Txd1ChkSum;				// 低位字节
//		Txd1Max=Txd1Buffer[2]+5;
//		//
//		B_Com1Cmd03=0;
//		Txd1Counter=0;
//		// 2010.7.4 周成磊 改成	USART_IT_TC
////		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);				// 开始发送.
////		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
//
////		USART_SendData(USART1,Txd1Buffer[Txd1Counter++]);
////		USART_ITConfig(USART1, USART_IT_TC, ENABLE);				// 开始发送.
//		while(huart1.gState != HAL_UART_STATE_READY);
//		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&Txd1Buffer[Txd1Counter++], 1);
//	}
//	//
//	else if ( B_Com1Cmd16 || B_Com1Cmd06 )					// 16预置多寄存器
//	{
//		// 填充
//		// 可以直接修改的参数
//		if ( w_Com1RegAddr==9 || w_Com1RegAddr==56 || 	// 修改参数	// 播放公司简介
//			w_Com1RegAddr==269 || w_Com1RegAddr==270 ||	// 选择故障号-选择记录号
//			w_Com1RegAddr==336 || w_Com1RegAddr==313 )  // 336:选择参数   313远程启停  秦汉东2017.3.27新加，监控软件可直接远程启停
//		{
//			j=1;
//		}
//		else if ( w_Com1RegAddr<=6000 ) //YLS 2020.06.23，不用密码就可以修改参数
//		{
//			j=1;
//		}
//		//ZCL 2016.8.29
//		else if ( w_Com1RegAddr>=352 && w_Com1RegAddr<=358 ) // 1-5号泵电流，系统电压
//		{
//			j=1;
//		}
//		else if ( w_Com1RegAddr>=671 && w_Com1RegAddr<=675 ) // dsp板变频故障
//		{
//			j=1;
//		}
//		//
//		else if ( w_Com1RegAddr==2 )			// 设定压力
//		{
//			j=1;
//			B_SaveSetP=1;
//		}
//		//
//		else if ( w_Com1RegAddr==470 )			// 读取变频器数据
//		{
//			j=1;
//			B_ReadVfPar=1;
//		}
//		else if ( w_Com1RegAddr==337 && Pw_ModPar==3000 )		// 修改选择参数的值
//		{
//			j=1;
//			B_ModSelParValue=1;
//		}
//		//
//		// 修改时间		45.46.47.48.49.50.51 时间单元
//		// 不允许用户修改时间　20060918
//		//
//		// FLASH记录号－FLASH记录数量　用特殊密码可以修改，特殊实现M25P80删除操作。
//		else if (  w_Com1RegAddr==310 ||  w_Com1RegAddr==311  || w_Com1RegAddr==326	// 310:FLASH记录号  311:FLASH记录数量 326:FLASH上传程序
//			|| w_Com1RegAddr==313 ||  w_Com1RegAddr==314  || w_Com1RegAddr==315		// 313-315 远程遥控参数 （变频停止 =1停机）
//			|| w_Com1RegAddr==143 	)	// 143:口1波特率
//		{
//			if ( Pw_ModPar==3000 )		// 用3000修改一些特殊的参数，只有Port1口有
//			{
//				j=1;
//			}
//		}
//		//
//		// 需要口令才可以修改的参数
//		else if ( Pw_ModPar==2000 )	// 需要先把Pw_ModPar 修改成规定值，才能修改的参数
//		{
//			j=1;
//		}

//		// -------------------------
//		// 修改参数单元
//		if ( j )
//		{
//			if ( w_Com1RegAddr>=45 && w_Com1RegAddr<=51 )			// 修改时间
//			{
//				w_ModRealTimeNo=w_Com1RegAddr-45;
//				F_ModRealTime=1;
//			}
//			//
//			if ( B_Com1Cmd06 )		// 预置单个
//			{
//				if ( w_Com1RegAddr<0x800 )
//				{
//					m=Rcv1Buffer[4];
//					w_ParLst[w_Com1RegAddr]=(m<<8)+Rcv1Buffer[5];
//				}
//				//-------------------------
//				else if ( w_Com1RegAddr>=5000)			// 修改伺服电机参数
//				{
//					m=Rcv1Buffer[4];
//					w_ParLst_Drive[w_Com1RegAddr-5000]=(m<<8)+Rcv1Buffer[5];	//地址-5000
//				}
//				//-------------------------
//			}
//			else if ( B_Com1Cmd16 )	// 预置多个
//			{
//				if ( Rcv1Buffer[5]<100 )
//				{
//					if ( w_Com1RegAddr<0x800 )
//					{
//						p_bGen=Rcv1Buffer;
//						p_wTarget=w_ParLst;
//						for ( k=0;k<Rcv1Buffer[5] ;k++ )		// Rcv1Buffer[5]=字数
//						{
//							m = *(p_bGen+7+k*2);
//							n = *(p_bGen+7+k*2+1);
//							*(p_wTarget+w_Com1RegAddr+k)= (m<<8) + n;
//						}
//					}
//					else if ( w_Com1RegAddr>=5000)			// 修改伺服电机参数
//					{
//						p_bGen=Rcv1Buffer;
//						p_wTarget2=w_ParLst_Drive;
//						for ( k=0;k<Rcv1Buffer[5] ;k++ )		// Rcv1Buffer[5]=字数
//						{
//							m = *(p_bGen+7+k*2);
//							n = *(p_bGen+7+k*2+1);
//							*(p_wTarget2+w_Com1RegAddr-5000+k)= (m<<8) + n;
//						}
//					}
//				}
//			}
//		}

//		// -------------------------
//		// 返回数据
//		Txd1Buffer[0]=2;				// 设备从地址
//		Txd1Buffer[1]=Rcv1Buffer[1];	// 功能码
//		Txd1Buffer[2]=Rcv1Buffer[2];	// 开始地址高位字节
//		Txd1Buffer[3]=Rcv1Buffer[3];	// 开始地址低位字节
//		Txd1Buffer[4]=Rcv1Buffer[4];	// 寄存器数量高位
//		Txd1Buffer[5]=Rcv1Buffer[5];	// 寄存器数量低位
//		if ( j==0 )							// 如果不能被正常预置，返回FFFF zcl
//		{
//			Txd1Buffer[4]=0xff;				// 寄存器数量高位、预置数据
//			Txd1Buffer[5]=0xff;				// 寄存器数量低位、预置数据
//		}
//		w_Txd1ChkSum=CRC16( Txd1Buffer, 6);
//		Txd1Buffer[6]=w_Txd1ChkSum>>8;		// /256
//		Txd1Buffer[7]=w_Txd1ChkSum;				// 低位字节
//		Txd1Max=8;
//
//		B_Com1Cmd16=0;
//		B_Com1Cmd06=0;
//		Txd1Counter=0;
//		// 2010.7.4 周成磊 改成	USART_IT_TC
//		//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);				// 开始发送.
////		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
////		USART_SendData(USART1,Txd1Buffer[Txd1Counter++]);
////		USART_ITConfig(USART1, USART_IT_TC, ENABLE);				// 开始发送.
//		while(huart1.gState != HAL_UART_STATE_READY);
//		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&Txd1Buffer[Txd1Counter++], 1);

//	}// 06、16预置寄存器 结束
//}

//uint16_t CRC16(uint8_t * pCrcData,uint8_t CrcDataLen)
//{
//	uint8_t CRC16Hi=0xff;                   /* 高CRC字节初始化 */
//  uint8_t CRC16Lo=0xff;                   /* 低CRC字节初始化*/
//	uint8_t Index=0;
//	uint16_t  w_CRC16=0;
//	while(CrcDataLen--)
//	{
//		Index = CRC16Hi ^* pCrcData++;
//		CRC16Hi = CRC16Lo ^ CRC_H[Index];
//		CRC16Lo = CRC_L[Index];
//	}
//	w_CRC16 = (CRC16Hi << 8) | CRC16Lo;
//	return (w_CRC16);
//}

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
