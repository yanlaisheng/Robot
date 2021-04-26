/** 
  ******************************************************************************

	******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "wk_port1.h"
#include "GlobalV_Extern.h" // ȫ�ֱ�������
//#include "GlobalConst.h"
#include <stdio.h> //���ϴ˾������printf
//#include "CRCdata.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
uint8_t WkPort1_TxdBuffer[TXD1_MAX]; // ���ͻ�����
uint8_t WkPort1_RcvBuffer[RCV1_MAX]; // ���ջ�����

uint8_t WkPort2_RcvBuffer[RCV1_MAX]; // ���ջ�����
uint8_t WkPort3_RcvBuffer[RCV1_MAX]; // ���ջ�����
uint8_t WkPort4_RcvBuffer[RCV1_MAX]; // ���ջ�����

//uint8_t		Tmp_Txd1Buffer[1],Tmp_Rxd1Buffer[1];
uint16_t WkPort1_RcvCounter;  // ���ռ�����//
uint16_t WkPort1_TxdCounter;  // ���ͼ�����//
uint16_t WkPort1_bakRcvCount; // ���ռ�����//
uint16_t WkPort1_TxdMax;      // �ж��ٸ��ַ���Ҫ����//
uint16_t WkPort1_TxdChkSum;   // ����У��ͣ�lo,hi ��λ//
uint16_t WkPort1_RegAddr;     // ����1�Ĵ�����ַ
//
//uint8_t		B_Com1Send;

//uint8_t		B_Com1Cmd03;
//uint8_t		B_Com1Cmd16;
//uint8_t		B_Com1Cmd01;
//uint8_t		B_Com1Cmd06;
//uint8_t		B_Com1Cmd73;
//uint16_t		T_NoRcv1Count;						// û�н��ռ�����
//uint16_t		C_NoRcv1Count;
//uint16_t     C_Com1_MasterSendDelay;
//uint16_t	   T_Com1_MasterSendDelay;
//uint16_t     C_Com1_MasterSendDelay2;
//uint16_t	   T_Com1_MasterSendDelay2;
//uint16_t     C_Com1_MasterSendDelay5;
//uint16_t	   T_Com1_MasterSendDelay5;

//uint8_t	Com1_Query_PC=0;		//COM1��ѯ������ָ��
//uint8_t	Com1_CMD_PC=0;			//COM1�������ָ��

//s16	Com1_Driver1_Queue_Rear;			//��������е�������������βָ��
//s16	Com1_Driver2_Queue_Rear;			//��������е�������������βָ��

//s16	Com1_Driver1_Queue_Front;			//��������еĵ�ǰҪ���ӵģ���ͷָ��
//s16	Com1_Driver2_Queue_Front;			//��������еĵ�ǰҪ���ӵģ���ͷָ��

//uint8_t	Com1_QReceiveDataTargetLength;	//���յ�����Ӧ�õĳ���

//extern	UART_HandleTypeDef huart1;
////�ж�COM1��������Ƿ�Ϊ��
//extern	uint8_t Com1_Driver1_Queue_isEmpty(void);
//extern	uint8_t Com1_Driver2_Queue_isEmpty(void);
//extern	uint8_t Com2_Driver3_Queue_isEmpty(void);
//extern	uint8_t Com2_Driver4_Queue_isEmpty(void);
//extern	uint8_t Com3_Driver5_Queue_isEmpty(void);
//extern	uint8_t Com3_Driver6_Queue_isEmpty(void);

////�ж�COM1��������Ƿ�Ϊ��
//extern	uint8_t Com1_Driver1_Queue_isFull(void);
//extern	uint8_t Com1_Driver2_Queue_isFull(void);
//extern	uint8_t Com2_Driver3_Queue_isFull(void);
//extern	uint8_t Com2_Driver4_Queue_isFull(void);
//extern	uint8_t Com3_Driver5_Queue_isFull(void);
//extern	uint8_t Com3_Driver6_Queue_isFull(void);

////����ÿ̨�����д�������
////ÿ�����壺���+�����+��������
//extern	uint8_t	Com1_Driver1_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];	//COM_CMD_SIZE=30��COM_CMD_NUM=6.����6��д���ÿ��30���ֽ�
//extern	uint8_t	Com1_Driver2_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];
//extern	uint8_t	Com2_Driver3_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];
//extern	uint8_t	Com2_Driver4_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];
//extern	uint8_t	Com3_Driver5_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];
//extern	uint8_t	Com3_Driver6_Write_BUFF[COM_CMD_SIZE*COM_CMD_NUM];

////����ÿ̨�������һ������
////���壺���+�����+��������
//extern	uint8_t	Com1_LAST_CMD_BUFF[COM_CMD_SIZE];	//����1����30���ֽ�
//extern	uint8_t	Com2_LAST_CMD_BUFF[COM_CMD_SIZE];
//extern	uint8_t	Com3_LAST_CMD_BUFF[COM_CMD_SIZE];
//extern	uint8_t	Com4_LAST_CMD_BUFF[COM_CMD_SIZE];

////��ѯָ���
//extern	uint8_t	Com1_Query_Data[];
//extern	uint8_t	Com2_Query_Data[];
//extern	uint8_t	Com3_Query_Data[];

//extern	uint16_t	Driver1_RcvCount;		//���ռ���
//extern	uint16_t	Driver2_RcvCount;		//���ռ���

//uint16_t	Com1_Send_Sort=0;				//Com1��������˳���Ƿ�1#������У����Ƿ�2#�������
//uint16_t	Com1_Send_CMD_Type_Sort=0;	//Com1�Ƿ��Ͳ�ѯ���Ƿ��Ϳ������=0����ѯ��=1����������

///* Private function prototypes -----------------------------------------------*/
//void GPIO_Com1_Configuration(void);							//GPIO����
//void Com1_config(void);
//void Com1_Driver1_Send_CMD(void);					//��������
//void Com1_Driver2_Send_CMD(void);					//��������
//uint16_t CRC16(uint8_t * pCrcData,uint8_t CrcDataLen);
//void Delay_MS(vu16 nCount);
//uint16_t Address(uint16_t *p, uint16_t Area); 					//���Ե�ַ
//extern	void PowerDelay(uint16_t nCount);

//extern	uint8_t	F_Sync_6_axis;						//6��ͬ�������־

//extern	uint8_t	F_Driver1_Send_Cmd;					//��������1#�ŷ�����������λ�������־��=1��ʾ�ѷ�������
//extern	uint8_t	F_Driver2_Send_Cmd;
//extern	uint8_t	F_Driver3_Send_Cmd;
//extern	uint8_t	F_Driver4_Send_Cmd;
//extern	uint8_t	F_Driver5_Send_Cmd;
//extern	uint8_t	F_Driver6_Send_Cmd;

//extern	uint8_t	F_Driver1_Timeout;					//1#�ŷ��������������ʱ��־��=1��ʾ��ʱ
//extern	uint8_t	F_Driver2_Timeout;
//extern	uint8_t	F_Driver3_Timeout;
//extern	uint8_t	F_Driver4_Timeout;
//extern	uint8_t	F_Driver5_Timeout;
//extern	uint8_t	F_Driver6_Timeout;

//extern	uint8_t	F_Driver1_Cmd_Err;					//1#�ŷ�������λ����������־��=1��ʾ����
//extern	uint8_t	F_Driver2_Cmd_Err;
//extern	uint8_t	F_Driver3_Cmd_Err;
//extern	uint8_t	F_Driver4_Cmd_Err;
//extern	uint8_t	F_Driver5_Cmd_Err;
//extern	uint8_t	F_Driver6_Cmd_Err;

//extern	uint8_t	F_Driver1_Cmd_Con_Err;					//1#�ŷ�������������������־��=1��ʾ����
//extern	uint8_t	F_Driver2_Cmd_Con_Err;
//extern	uint8_t	F_Driver3_Cmd_Con_Err;
//extern	uint8_t	F_Driver4_Cmd_Con_Err;
//extern	uint8_t	F_Driver5_Cmd_Con_Err;
//extern	uint8_t	F_Driver6_Cmd_Con_Err;

//extern	uint8_t	Driver1_Cmd_PosNo;					//1#�ŷ����������͵�����λ�úţ�λ��0����λ��1
//extern	uint8_t	Driver2_Cmd_PosNo;
//extern	uint8_t	Driver3_Cmd_PosNo;
//extern	uint8_t	Driver4_Cmd_PosNo;
//extern	uint8_t	Driver5_Cmd_PosNo;
//extern	uint8_t	Driver6_Cmd_PosNo;

//extern	uint8_t	Driver1_Cmd_Status;					//1#�ŷ�������P8910״̬
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

//extern	uint8_t Driver1_Cmd_Data[9];							//1#�ŷ���������������,4���ֽ�Ϊ���壬2���ֽ�Ϊ�ٶȣ�2���ֽ�Ϊ�Ӽ���ʱ�䣬���һ���ֽ�Ϊλ�úţ�0/1��
//extern	uint8_t Driver2_Cmd_Data[9];
//extern	uint8_t Driver3_Cmd_Data[9];
//extern	uint8_t Driver4_Cmd_Data[9];
//extern	uint8_t Driver5_Cmd_Data[9];
//extern	uint8_t Driver6_Cmd_Data[9];

//extern	s32 *arr_p1;

//extern	uint8_t Driver1_Pos_Start_Sort;						//1#�ŷ������Ƿ�λ��ָ����Ƿ��������=0����λ�ã�=1������������
//extern	uint8_t Driver2_Pos_Start_Sort;
//extern	uint8_t Driver3_Pos_Start_Sort;
//extern	uint8_t Driver4_Pos_Start_Sort;
//extern	uint8_t Driver5_Pos_Start_Sort;
//extern	uint8_t Driver6_Pos_Start_Sort;

//extern	uint8_t Driver1_Status_Sort;							//1#�ŷ���=2����ʾ�Ѿ����ͣ�=3����ʾ�Ѿ�����
//extern	uint8_t Driver2_Status_Sort;
//extern	uint8_t Driver3_Status_Sort;
//extern	uint8_t Driver4_Status_Sort;
//extern	uint8_t Driver5_Status_Sort;
//extern	uint8_t Driver6_Status_Sort;

///* Private functions ---------------------------------------------------------*/

////���մ������� У�����
//void Com1_RcvProcess(void)
//{
//	uint8_t k,s,i=0;       						// ��ʱ����
//    uint16_t j;
//	//��Ϊ����,ָ������ʱ�䵽��,�Ϳ��Դ������յ����ַ�����
//	// ��û�յ������ַ���ʱ�䳬���趨ʱ�����ԶԽ��ջ�����д�����
//	// **********************************rcv_counter<>0,�յ��ַ����ܴ���
//	if ( Rcv1Counter>0 &&  T_NoRcv1Count!=SClk1Ms )
//	{								// ���մ�������
//		T_NoRcv1Count=SClk1Ms;				//
//		C_NoRcv1Count++;
//		if ( C_NoRcv1Count>NORCVMAXMS )				//
//		{
//			/* Disable the UART Parity Error Interrupt and RXNE interrupt*/
////			CLEAR_BIT(huart1->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
//			BakRcv1Count=Rcv1Counter;		// �� Rcv1Counter ����
//			C_NoRcv1Count=0;				// ��û�н��ռ�����
//			//
//			if(BakRcv1Count<=RCV1_MAX)		// ���ճ�����ȷ,��������.
//			{
//				// �ӵ�ַ��⣭���յ�����λ����ѯָ��  ���ı�ͨѶ
//				if( Rcv1Buffer[0]==Pw_EquipmentNo1 )
//				{
//					j=CRC16( Rcv1Buffer, BakRcv1Count-2);		// CRC У��
//					k=j>>8;
//					s=j;
//					if ( k==Rcv1Buffer[BakRcv1Count-2]
//						&& s==Rcv1Buffer[BakRcv1Count-1] )
//					{											// CRCУ����ȷ
//						if ( Rcv1Buffer[1]==3 )		// 03��ȡ���ּĴ���
//						{
//							B_Com1Cmd03=1;
//							j=Rcv1Buffer[2];
//							w_Com1RegAddr=(j<<8)+Rcv1Buffer[3];
//						}
//						else if ( Rcv1Buffer[1]==16 )	// 16Ԥ�ö�Ĵ���
//						{
////							C_ForceSavPar=0;		// ǿ�Ʊ������������=0
//							B_Com1Cmd16=1;
//							j=Rcv1Buffer[2];
//							w_Com1RegAddr=(j<<8)+Rcv1Buffer[3];
//						}
//						else if ( Rcv1Buffer[1]==1 )		// 01��ȡ��Ȧ״̬
//						{
//							B_Com1Cmd01=1;
//						}
//						else if ( Rcv1Buffer[1]==6 )		// 06Ԥ�õ��Ĵ���
//						{
////							C_ForceSavPar=0;		// ǿ�Ʊ������������=0
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
//			Rcv1Counter=0;					// ׼���´ν��յ����濪ʼ
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

//void Com1_SlaveSend(void)			// ����1�ӻ�����
//{
//	uint16_t	m,n;
//	uint8_t	j=0,k;
//	uint16_t	 *p_wRead;
//	s32	 *p_wRead2;
//	uint8_t	 *p_bMove;
//	uint8_t	 *p_bGen;
//	uint16_t	 *p_wTarget;			// ָ��Ŀ���ַ�����xdata zcl
//	s32	 *p_wTarget2;

//	//
//	if ( B_Com1Cmd03 )		// ��ȡ���ּĴ���
//	{
//		Txd1Buffer[0]=Rcv1Buffer[0];	// �豸�ӵ�ַPw_EquipmentNo
//		Txd1Buffer[1]=Rcv1Buffer[1];	// ������
//		Txd1Buffer[2]=Rcv1Buffer[5]*2;	// Rcv1Buffer[5]=���� ��
//		//
//		if ( w_Com1RegAddr<0x800 && Pw_ComBufType==1 )	// �����ѯ Pw_ComBufType==1 2016.4.21
//		{
//			p_wRead=w_ParLst;			// PAR��
//			p_bMove=Txd1Buffer;
//			//
//			for ( k=0;k<Rcv1Buffer[5] ;k++ )	// ����ѯ����
//			{
//				m=*(p_wRead+w_Com1RegAddr+k);
//				*(p_bMove+3+k*2)=m>>8;
//				*(p_bMove+3+k*2+1)=m;
//			}
//		}
//		else if ( w_Com1RegAddr>=5000 && Pw_ComBufType==1 )		// ��ȡ����趨������������ַ��5000
//		{
//			p_wRead2=w_ParLst_Drive;			// PAR��
//			p_bMove=Txd1Buffer;
//			//
//			for ( k=0;k<Rcv1Buffer[5] ;k++ )	// ����ѯ����
//			{
//				m=*(p_wRead2+w_Com1RegAddr-5000+k);		//������ַ��5000
//				*(p_bMove+3+k*2)=m>>8;
//				*(p_bMove+3+k*2+1)=m;
//			}
//		}
//		//
//		w_Txd1ChkSum=CRC16( Txd1Buffer, Txd1Buffer[2]+3 );
//		Txd1Buffer[Txd1Buffer[2]+3]=w_Txd1ChkSum>>8;			// /256
//		Txd1Buffer[Txd1Buffer[2]+4]=w_Txd1ChkSum;				// ��λ�ֽ�
//		Txd1Max=Txd1Buffer[2]+5;
//		//
//		B_Com1Cmd03=0;
//		Txd1Counter=0;
//		// 2010.7.4 �ܳ��� �ĳ�	USART_IT_TC
////		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);				// ��ʼ����.
////		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
//
////		USART_SendData(USART1,Txd1Buffer[Txd1Counter++]);
////		USART_ITConfig(USART1, USART_IT_TC, ENABLE);				// ��ʼ����.
//		while(huart1.gState != HAL_UART_STATE_READY);
//		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&Txd1Buffer[Txd1Counter++], 1);
//	}
//	//
//	else if ( B_Com1Cmd16 || B_Com1Cmd06 )					// 16Ԥ�ö�Ĵ���
//	{
//		// ���
//		// ����ֱ���޸ĵĲ���
//		if ( w_Com1RegAddr==9 || w_Com1RegAddr==56 || 	// �޸Ĳ���	// ���Ź�˾���
//			w_Com1RegAddr==269 || w_Com1RegAddr==270 ||	// ѡ����Ϻ�-ѡ���¼��
//			w_Com1RegAddr==336 || w_Com1RegAddr==313 )  // 336:ѡ�����   313Զ����ͣ  �غ���2017.3.27�¼ӣ����������ֱ��Զ����ͣ
//		{
//			j=1;
//		}
//		else if ( w_Com1RegAddr<=6000 ) //YLS 2020.06.23����������Ϳ����޸Ĳ���
//		{
//			j=1;
//		}
//		//ZCL 2016.8.29
//		else if ( w_Com1RegAddr>=352 && w_Com1RegAddr<=358 ) // 1-5�űõ�����ϵͳ��ѹ
//		{
//			j=1;
//		}
//		else if ( w_Com1RegAddr>=671 && w_Com1RegAddr<=675 ) // dsp���Ƶ����
//		{
//			j=1;
//		}
//		//
//		else if ( w_Com1RegAddr==2 )			// �趨ѹ��
//		{
//			j=1;
//			B_SaveSetP=1;
//		}
//		//
//		else if ( w_Com1RegAddr==470 )			// ��ȡ��Ƶ������
//		{
//			j=1;
//			B_ReadVfPar=1;
//		}
//		else if ( w_Com1RegAddr==337 && Pw_ModPar==3000 )		// �޸�ѡ�������ֵ
//		{
//			j=1;
//			B_ModSelParValue=1;
//		}
//		//
//		// �޸�ʱ��		45.46.47.48.49.50.51 ʱ�䵥Ԫ
//		// �������û��޸�ʱ�䡡20060918
//		//
//		// FLASH��¼�ţ�FLASH��¼��������������������޸ģ�����ʵ��M25P80ɾ��������
//		else if (  w_Com1RegAddr==310 ||  w_Com1RegAddr==311  || w_Com1RegAddr==326	// 310:FLASH��¼��  311:FLASH��¼���� 326:FLASH�ϴ�����
//			|| w_Com1RegAddr==313 ||  w_Com1RegAddr==314  || w_Com1RegAddr==315		// 313-315 Զ��ң�ز��� ����Ƶֹͣ =1ͣ����
//			|| w_Com1RegAddr==143 	)	// 143:��1������
//		{
//			if ( Pw_ModPar==3000 )		// ��3000�޸�һЩ����Ĳ�����ֻ��Port1����
//			{
//				j=1;
//			}
//		}
//		//
//		// ��Ҫ����ſ����޸ĵĲ���
//		else if ( Pw_ModPar==2000 )	// ��Ҫ�Ȱ�Pw_ModPar �޸ĳɹ涨ֵ�������޸ĵĲ���
//		{
//			j=1;
//		}

//		// -------------------------
//		// �޸Ĳ�����Ԫ
//		if ( j )
//		{
//			if ( w_Com1RegAddr>=45 && w_Com1RegAddr<=51 )			// �޸�ʱ��
//			{
//				w_ModRealTimeNo=w_Com1RegAddr-45;
//				F_ModRealTime=1;
//			}
//			//
//			if ( B_Com1Cmd06 )		// Ԥ�õ���
//			{
//				if ( w_Com1RegAddr<0x800 )
//				{
//					m=Rcv1Buffer[4];
//					w_ParLst[w_Com1RegAddr]=(m<<8)+Rcv1Buffer[5];
//				}
//				//-------------------------
//				else if ( w_Com1RegAddr>=5000)			// �޸��ŷ��������
//				{
//					m=Rcv1Buffer[4];
//					w_ParLst_Drive[w_Com1RegAddr-5000]=(m<<8)+Rcv1Buffer[5];	//��ַ-5000
//				}
//				//-------------------------
//			}
//			else if ( B_Com1Cmd16 )	// Ԥ�ö��
//			{
//				if ( Rcv1Buffer[5]<100 )
//				{
//					if ( w_Com1RegAddr<0x800 )
//					{
//						p_bGen=Rcv1Buffer;
//						p_wTarget=w_ParLst;
//						for ( k=0;k<Rcv1Buffer[5] ;k++ )		// Rcv1Buffer[5]=����
//						{
//							m = *(p_bGen+7+k*2);
//							n = *(p_bGen+7+k*2+1);
//							*(p_wTarget+w_Com1RegAddr+k)= (m<<8) + n;
//						}
//					}
//					else if ( w_Com1RegAddr>=5000)			// �޸��ŷ��������
//					{
//						p_bGen=Rcv1Buffer;
//						p_wTarget2=w_ParLst_Drive;
//						for ( k=0;k<Rcv1Buffer[5] ;k++ )		// Rcv1Buffer[5]=����
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
//		// ��������
//		Txd1Buffer[0]=2;				// �豸�ӵ�ַ
//		Txd1Buffer[1]=Rcv1Buffer[1];	// ������
//		Txd1Buffer[2]=Rcv1Buffer[2];	// ��ʼ��ַ��λ�ֽ�
//		Txd1Buffer[3]=Rcv1Buffer[3];	// ��ʼ��ַ��λ�ֽ�
//		Txd1Buffer[4]=Rcv1Buffer[4];	// �Ĵ���������λ
//		Txd1Buffer[5]=Rcv1Buffer[5];	// �Ĵ���������λ
//		if ( j==0 )							// ������ܱ�����Ԥ�ã�����FFFF zcl
//		{
//			Txd1Buffer[4]=0xff;				// �Ĵ���������λ��Ԥ������
//			Txd1Buffer[5]=0xff;				// �Ĵ���������λ��Ԥ������
//		}
//		w_Txd1ChkSum=CRC16( Txd1Buffer, 6);
//		Txd1Buffer[6]=w_Txd1ChkSum>>8;		// /256
//		Txd1Buffer[7]=w_Txd1ChkSum;				// ��λ�ֽ�
//		Txd1Max=8;
//
//		B_Com1Cmd16=0;
//		B_Com1Cmd06=0;
//		Txd1Counter=0;
//		// 2010.7.4 �ܳ��� �ĳ�	USART_IT_TC
//		//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);				// ��ʼ����.
////		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
////		USART_SendData(USART1,Txd1Buffer[Txd1Counter++]);
////		USART_ITConfig(USART1, USART_IT_TC, ENABLE);				// ��ʼ����.
//		while(huart1.gState != HAL_UART_STATE_READY);
//		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&Txd1Buffer[Txd1Counter++], 1);

//	}// 06��16Ԥ�üĴ��� ����
//}

//uint16_t CRC16(uint8_t * pCrcData,uint8_t CrcDataLen)
//{
//	uint8_t CRC16Hi=0xff;                   /* ��CRC�ֽڳ�ʼ�� */
//  uint8_t CRC16Lo=0xff;                   /* ��CRC�ֽڳ�ʼ��*/
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