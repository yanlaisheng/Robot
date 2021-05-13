//头文件
#include "include.h"
//#include "ringbuffer.h"
#include "bsp_MOTOR1.h"
#include "bsp_MOTOR2.h"
#include "bsp_MOTOR3.h"
#include "bsp_MOTOR4.h"
#include "bsp_MOTOR5.h"
#include "bsp_MOTOR6.h"
#include "GlobalConst.h"
// #include "GlobalV.h"
#include "Macro.h"

volatile uint8_t USART1_TxBuffer[USART1TXSIZE];	  //串口1发送缓冲区
volatile uint16_t PTxBufferUSART11 = 0;			  //串口1发送前向位置
volatile uint16_t PTxBufferUSART12 = 0;			  //串口1发送后向位置，后向-前向=未发送的数据
volatile uint16_t USART1_TxCounter = 0;			  //串口1发送计数
volatile uint16_t USART1_RxCounter = 0;			  //串口1接收计数
volatile uint16_t USART1_NbrOfDataToTransfer = 0; //串口1要发送的数据个数
volatile uint8_t USART1_RxBuffer[USART1RXSIZE];	  //串口1接收缓冲区
volatile uint16_t PRxBufferUSART11 = 0;
volatile uint16_t PRxBufferUSART12 = 0;
volatile uint8_t USART1_NbrOfDataReceived = 0; //串口1要接收的数据个数
volatile uint8_t USART1_Received_Flag = 0;	   //串口1收到数据标志，1：接收到数据，0：没有接收到数据
//struct rt_ringbuffer rb_recv;

//以下是SPTA参数
#define STEP_SPTA 100		//SPTA最大速度等级	20
#define MAXSPEED_SPTA 20000 //SPTA最大速度	80000
#define ACCSPEED_SPTA 3000	//SPTA加速度	  150000
//static unsigned char MY_Cmd_Buff[16];

uint16_t Motor1TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor1StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor2TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor2StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor3TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor3StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor4TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor4StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor5TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor5StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor6TimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
uint16_t Motor6StepTable[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};

int i_M1;
float fi_M1[2 * (STEP_AA + STEP_UA + STEP_RA) + 1] = {0};
float tua_temp1;

unsigned long long Get_Time_Cost(unsigned char MotorID);
extern void MOTOR1_GPIO_Init(void);
extern void MOTOR2_GPIO_Init(void);
extern void MOTOR3_GPIO_Init(void);
extern void MOTOR4_GPIO_Init(void);
extern void MOTOR5_GPIO_Init(void);
extern void MOTOR6_GPIO_Init(void);

extern uint16_t w_ParLst[840]; // 参数字列表区
////USART1时钟初始化
///*入口参数：无******************************/
///*函数功能：USART1时钟初始化*/
///*返回参数：无******************************/
///**********************Start****************/
//void USART1_RCC_Configuration(void)
//{
//  /*******************串口1的时钟初始化********************/
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
//}
///**********************End****************/
//
////USART1引脚初始化
///*入口参数：无******************************/
///*函数功能：USART1引脚初始化*/
///*返回参数：无******************************/
///**********************Start****************/
//void USART1_GPIO_Configuration(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//
//  /* Enable the USART2 Pins Software Remapping */
//  //GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
//
//  /* Configure USART1 Rx as input floating */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//  /* Configure USART1 Tx as alternate function push-pull */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//}
///**********************End****************/
//
//
////USART1中断函数初始化
///*入口参数：无******************************/
///*函数功能：USART1中断函数初始化*/
///*返回参数：无******************************/
///**********************Start****************/
//void USART1_NVIC_Configuration(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;
//
//  /* Configure the NVIC Preemption Priority Bits */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
//
//  /* Enable the USART1 Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =4;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//
//}
///**********************End****************/
//
////USART1寄存器初始化
///*入口参数：无******************************/
///*函数功能：USART1寄存器初始化*/
///*返回参数：无******************************/
///**********************Start****************/
//void USART1_Initial(void)
//{
//  USART_InitTypeDef USART_InitStructure;
//  USART1_RCC_Configuration();
//  USART1_GPIO_Configuration();
//  USART1_NVIC_Configuration();
//  USART_InitStructure.USART_BaudRate =115200;
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;
//  USART_InitStructure.USART_Parity = USART_Parity_No;
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//  USART_Init(USART1, &USART_InitStructure);
//
//  //USART1_NbrOfDataToTransfer=0;
//
//  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
//  USART_Cmd(USART1, ENABLE);
//}
////USART1发送数据
///*入口参数：要发送的buffer指针，要发送的数据长度****/
///*函数功能：USART1发送数据**************************/
///*返回参数：无**************************************/
///*对于不定长的发送，可以采用先判断要发送的数据长度和
//  剩余缓冲区大小，再决定如何拷贝的方法，以减少
//  PTxBufferUSART12是否越界的判断，但是之前处理的方式
//  是默认不会溢出，后期可能会将调试信息删除，所以就不
//  改了，下同
//*********************Start************************/
//void USART1_Printf(unsigned char *q,unsigned char len)
//{
//  u8 i=0;
//
//  for(i=0;i<len;i++)
//  {
//    USART1_TxBuffer[PTxBufferUSART12++]=q[i];
//		if(PTxBufferUSART12>=USART1TXSIZE)
//		{
//			PTxBufferUSART12=0;
//		}
//  }
//  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
//}
//
////USART1发送数据
///*入口参数：要发送的buffer指针，一般是字符数组***/
///*函数功能：USART1发送数据***********************/
///*返回参数：无***********************************/
///**********************Start*********************/
//void USART1_Printfstr(unsigned char *p)
//{
//
//  while((*p)!=0)
//  {
//		USART1_TxBuffer[PTxBufferUSART12++]=*p;
//		p++;
//		if(PTxBufferUSART12>=USART1TXSIZE)
//		{
//			PTxBufferUSART12=0;
//		}
//  }
//  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
//}
///**********************End****************/
//
////USART2时钟初始化
///*入口参数：无******************************/
///*函数功能：USART1时钟初始化*/
///*返回参数：无******************************/
///**********************Start****************/
//void USART2_RCC_Configuration(void)
//{
//  /*******************串口1的时钟初始化********************/
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
//}
///**********************End****************/
//
////USART2引脚初始化
///*入口参数：无******************************/
///*函数功能：USART1引脚初始化*/
///*返回参数：无******************************/
///**********************Start****************/
//void USART2_GPIO_Configuration(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//
//  /* Enable the USART2 Pins Software Remapping */
//  //GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
//
//  /* Configure USART2 Rx as input floating */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//  /* Configure USART2 Tx as alternate function push-pull */
//  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  //GPIO_Init(GPIOA, &GPIO_InitStructure);
//}
///**********************End****************/
//
//
////USART2中断函数初始化
///*入口参数：无******************************/
///*函数功能：USART2中断函数初始化*/
///*返回参数：无******************************/
///**********************Start****************/
//void USART2_NVIC_Configuration(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;
//
//  /* Configure the NVIC Preemption Priority Bits */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
//
//  /* Enable the USART1 Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =4;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//
//}
///**********************End****************/
//
////USART2寄存器初始化
///*入口参数：无******************************/
///*函数功能：USART1寄存器初始化*/
///*返回参数：无******************************/
///**********************Start****************/
//void USART2_Initial(void)
//{
//  USART_InitTypeDef USART_InitStructure;
//  USART2_RCC_Configuration();
//  USART2_GPIO_Configuration();
//  USART2_NVIC_Configuration();
//  USART_InitStructure.USART_BaudRate =9600;
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;
//  USART_InitStructure.USART_Parity = USART_Parity_No;
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//  USART_Init(USART2, &USART_InitStructure);
//
//  //USART1_NbrOfDataToTransfer=0;
//
//  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//  //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
//  USART_Cmd(USART2, ENABLE);
//}

/*
电机1：PA8(pwm),PE9(CW),PE8(ENABLE),MXX:PA11,PA12,PE7
电机2：PA0(pwm),PA1(CW),PC3(ENABLE),MXX:PC0,PC1,PC2
电机3：PA6(pwm),PA7(CW),PC4(ENABLE),MXX:PA3,PA4,PA5
电机4：PB6(pwm,gpio),PB9(CW),PB8(ENABLE),MXX:PD7,PB5,PB7
*/
//void Initial_MotorIO(void)
//{
//  GPIO_InitTypeDef  GPIO_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);
//
//  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_11|GPIO_Pin_12;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//  GPIO_ResetBits(GPIOA,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_11|GPIO_Pin_12);
//
//  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  GPIO_ResetBits(GPIOB,GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
//
//  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//  GPIO_ResetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
//
//	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_7;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);
//  GPIO_ResetBits(GPIOD,GPIO_Pin_7);
//
//  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(GPIOE, &GPIO_InitStructure);
//  GPIO_ResetBits(GPIOE,GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
//
//
//  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_8|GPIO_Pin_6;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//  GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_8|GPIO_Pin_6);
//
//	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  GPIO_ResetBits(GPIOB,GPIO_Pin_6);
//}

/**************************************************************************************
 初始化电机的参数，主要是细分选择，使用的定时器，顺时针方向值，电机ID等
 **************************************************************************************/
void Initial_Motor(unsigned char MotorID, unsigned char StepDive, unsigned int maxposition)
{
	unsigned int i = 0;
	MOTOR_CONTROL_S *pmotor = NULL;
	MOTOR_CONTROL_SPTA *pmotor_spta = NULL;
	uint16_t *MotorTimeTable;
	uint16_t *MotorStepTable;

	switch (MotorID)
	{
	case 1:
		pmotor = &motor1;
		motor1.id = 1;
		motor1.clockwise = M1_CLOCKWISE;
		motor1.TIMx = TIM2;
		MotorTimeTable = Motor1TimeTable;
		MotorStepTable = Motor1StepTable;
		break;
	case 2:
		pmotor = &motor2;
		motor2.id = 2;
		motor2.clockwise = M2_CLOCKWISE;
		motor2.TIMx = TIM4;
		MotorTimeTable = Motor2TimeTable;
		MotorStepTable = Motor2StepTable;
		break;
	case 3:
		pmotor = &motor3;
		motor3.id = 3;
		motor3.clockwise = M3_CLOCKWISE;
		motor3.TIMx = TIM8;
		MotorTimeTable = Motor3TimeTable;
		MotorStepTable = Motor3StepTable;
		break;
	case 4:
		pmotor = &motor4;
		motor4.id = 4;
		motor4.clockwise = M4_CLOCKWISE;
		motor4.TIMx = TIM3;
		MotorTimeTable = Motor4TimeTable;
		MotorStepTable = Motor4StepTable;
		break;
	case 5:
		pmotor = &motor5;
		motor5.id = 5;
		motor5.clockwise = M5_CLOCKWISE;
		motor5.TIMx = TIM3;
		MotorTimeTable = Motor5TimeTable;
		MotorStepTable = Motor5StepTable;
		break;
	case 6:
		pmotor = &motor6;
		motor6.id = 6;
		motor6.clockwise = M6_CLOCKWISE;
		motor6.TIMx = TIM3;
		MotorTimeTable = Motor6TimeTable;
		MotorStepTable = Motor6StepTable;
		break;
	default:
		break;
	}

	if (MotorID <= 6 && MotorID >= 1)
	{
		pmotor->divnum = StepDive;
		pmotor->MaxPosition = maxposition;
		pmotor->MaxPosition_Pulse = maxposition * StepDive;

		pmotor->CurrentPosition = 0;
		pmotor->CurrentPosition_Pulse = 0;
		pmotor->StartTableLength = STEP_AA + STEP_UA + STEP_RA + 1;
		pmotor->StopTableLength = STEP_AA + STEP_UA + STEP_RA;
		pmotor->Counter_Table = MotorTimeTable;
		pmotor->Step_Table = MotorStepTable;

		pmotor->CurrentIndex = 0;
		pmotor->speedenbale = 0;
		pmotor->StartSteps = 0; //必须清零，后面是累加，否则会把前一次的加上
		pmotor->StopSteps = 0;	//同上
		for (i = 0; i < pmotor->StartTableLength; i++)
			pmotor->StartSteps += pmotor->Step_Table[i];
		for (i = 0; i < pmotor->StopTableLength; i++)
			pmotor->StopSteps += pmotor->Step_Table[i + pmotor->StartTableLength];

		pmotor->TIMx->ARR = pmotor->Counter_Table[0]; //设置周期
		// pmotor->TIMx->CCR1 = pmotor->Counter_Table[0] >> 1; //设置占空比
		if (pmotor->TIMx == TIM2)
		{
			pmotor->TIMx->CCR2 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
			pmotor->TIMx->CNT = 0;
		}
		else if (pmotor->TIMx == TIM4)
		{
			pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
			pmotor->TIMx->CNT = 0;
		}
		else if (pmotor->TIMx == TIM8)
		{
			pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
			pmotor->TIMx->CNT = 0;
		}
		else if (pmotor->TIMx == TIM3)
		{
			pmotor->TIMx->CCR4 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
			pmotor->TIMx->CNT = 0;
		}
		else if (pmotor->TIMx == TIM1)
		{
			pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
			pmotor->TIMx->CNT = 0;
		}
		else if (pmotor->TIMx == TIM5)
		{
			pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
			pmotor->TIMx->CNT = 0;
		}
	}
}

/*多轴协同使用了算法原理进行时间预估，所以修改该算法时记得
 这两处保持同步*/
/*计算S型曲线反转点，S型曲线在运行时，加减速过程是完全对称的*/
unsigned long long Get_TimeCost_ReverDot_S(unsigned char MotorID)
{
	unsigned long long time_cost = 0;
	unsigned long long time_cost2 = 0;
	unsigned int pulsecnt = 0;
	int i = 0, j;
	MOTOR_CONTROL_S *pmotor = NULL;
	switch (MotorID)
	{
	case 1:
		pmotor = &motor1;
		break;
	case 2:
		pmotor = &motor2;
		break;
	case 3:
		pmotor = &motor3;
		break;
	case 4:
		pmotor = &motor4;
		break;
	case 5:
		pmotor = &motor5;
		break;
	case 6:
		pmotor = &motor6;
		break;
	default:
		return 0;
	}

	if (pmotor->PulsesGiven >= pmotor->StartSteps + pmotor->StopSteps)
	{
		for (i = 0; i < pmotor->StartTableLength; i++)
			time_cost += (pmotor->Step_Table[i] * pmotor->Counter_Table[i]);
		for (i = 0; i < pmotor->StopTableLength; i++)
			time_cost += (pmotor->Step_Table[i + pmotor->StartTableLength] * pmotor->Counter_Table[i + pmotor->StartTableLength]);
		time_cost += (pmotor->PulsesGiven - pmotor->StartSteps - pmotor->StopSteps) * pmotor->Counter_Table[pmotor->StartTableLength - 1];

		pmotor->RevetDot = pmotor->PulsesGiven - pmotor->StopSteps;
	}
	else
	{
		//考虑这种情况，第一频率142 步，第二频率148步，要是运动200步该怎么运行
		//所以这里要改变第二频率的步数
		while ((pulsecnt + pmotor->Step_Table[i]) <= (pmotor->PulsesGiven >> 1))
		{
			time_cost += (pmotor->Step_Table[i] * pmotor->Counter_Table[i]);
			time_cost2 += (pmotor->Step_Table[i] * pmotor->Counter_Table[i]);
			pulsecnt += pmotor->Step_Table[i];
			i++;
		}
		time_cost += time_cost2;
		if (pmotor->Step_Table[i] < pmotor->PulsesGiven - 2 * pulsecnt)
		{
			pmotor->Step_Table[i] = pmotor->PulsesGiven - 2 * pulsecnt;
			pmotor->StartSteps = 0; //必须清零，后面是累加，否则会把前一次的加上
			pmotor->StopSteps = 0;	//同上
			for (j = 0; j < pmotor->StartTableLength; j++)
				pmotor->StartSteps += pmotor->Step_Table[j];
			for (j = 0; j < pmotor->StopTableLength; j++)
				pmotor->StopSteps += pmotor->Step_Table[j + pmotor->StartTableLength];
		}
		time_cost += (pmotor->Counter_Table[i] * (pmotor->PulsesGiven - 2 * pulsecnt));
		pmotor->RevetDot = pmotor->PulsesGiven - pulsecnt;
	}
	pmotor->Time_Cost_Cal = time_cost;
	return time_cost;
}
// unsigned long long Get_Time_Cost2(unsigned char MotorID)
// {
// 	extern void TIM1_UP_IRQHandler(void);
// 	extern void TIM2_IRQHandler(void);
// 	extern void TIM3_IRQHandler(void);
// 	switch (MotorID)
// 	{
// 	case 1:
// 		while (motor1.running == 1)
// 		{
// 			TIM1_CC_IRQHandler();
// 		}
// 		return motor1.Time_Cost_Act;
// 	case 2:
// 		while (motor2.running == 1)
// 		{
// 			TIM2_IRQHandler();
// 		}
// 		return motor2.Time_Cost_Act;
// 	case 3:
// 		while (motor3.running == 1)
// 		{
// 			TIM3_IRQHandler();
// 		}
// 		return motor3.Time_Cost_Act;
// 	}
// 	return 0;
// }

/*重新初始化电机运行时相关参数*/
void Motor_Reinitial(unsigned char MotorID)
{
	int i = 0;
	MOTOR_CONTROL_S *pmotor = NULL;
	switch (MotorID)
	{
	case 1:
		pmotor = &motor1;
		break;
	case 2:
		pmotor = &motor2;
		break;
	case 3:
		pmotor = &motor3;
		break;
	case 4:
		pmotor = &motor4;
		break;
	case 5:
		pmotor = &motor5;
		break;
	case 6:
		pmotor = &motor6;
		break;
	default:
		return;
	}
	pmotor->pulsecount = 0;
	pmotor->CurrentIndex = 0;
	pmotor->speedenbale = 0;
	pmotor->StartSteps = 0; //必须清零，后面是累加，否则会把前一次的加上
	pmotor->StopSteps = 0;	//同上
	for (i = 0; i < pmotor->StartTableLength; i++)
		pmotor->StartSteps += pmotor->Step_Table[i];
	for (i = 0; i < pmotor->StopTableLength; i++)
		pmotor->StopSteps += pmotor->Step_Table[i + pmotor->StartTableLength];
	pmotor->TIMx->ARR = pmotor->Counter_Table[0]; //设置周期
	// pmotor->TIMx->CCR1 = pmotor->Counter_Table[0] >> 1; //设置占空比
	if (pmotor->TIMx == TIM2)
	{
		pmotor->TIMx->CCR2 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
		pmotor->TIMx->CNT = 0;
	}
	else if (pmotor->TIMx == TIM4)
	{
		pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
		pmotor->TIMx->CNT = 0;
	}
	else if (pmotor->TIMx == TIM8)
	{
		pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
		pmotor->TIMx->CNT = 0;
	}
	else if (pmotor->TIMx == TIM3)
	{
		pmotor->TIMx->CCR4 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
		pmotor->TIMx->CNT = 0;
	}
	else if (pmotor->TIMx == TIM1)
	{
		pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
		pmotor->TIMx->CNT = 0;
	}
	else if (pmotor->TIMx == TIM5)
	{
		pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
		pmotor->TIMx->CNT = 0;
	}
	pmotor->Time_Cost_Act = pmotor->TIMx->ARR;
	Get_TimeCost_ReverDot_S(MotorID);
}

/*根据S型曲线参数获取某个时刻的频率*/ //t就是当前运行的时间
float GetFreAtTime(float fstart, float faa, float taa, float tua, float tra, float t)
{
	//根据公式计算从开始到最高速过冲中，t时刻的转动频率
	if (t >= 0 && t <= taa)
	{
		//加加速阶段
		return fstart + M_XiShu_1 * faa * t * t; //抛物线方
	}
	else if (taa < t && t <= (taa + tua))
	{
		//匀加速阶段
		return fstart + M_XiShu_1 * faa * taa * taa + (t - taa) * faa * taa;
	}
	else if ((taa + tua) < t && t <= (taa + tua + tra))
	{
		//减加速阶段
		return fstart + M_XiShu_1 * faa * taa * taa + (tua)*faa * taa + M_XiShu_1 * faa * taa * tra - M_XiShu_1 * faa * taa * (taa + tua + tra - t) * (taa + tua + tra - t) / (tra);
	}
	return 0;
}

//根据设定速度反推加速度
float GetFaaPara(float fstart, float taa, float tua, float tra, float SetSpeed)
{
	float temp1;
	temp1 = (SetSpeed * PULSENUM / 60 - fstart) / (M_XiShu_1 * taa * taa + taa * tua + M_XiShu_1 * taa * tra);
	return temp1;
}

/*计算S型曲线算法的每一步定时器周期及步进数*/
void CalcMotorPeriStep_CPF(float fstart, float faa, float taa, float tua, float tra, uint16_t MotorTimeTable[], uint16_t MotorStepTable[])
{
	// int i;
	// float fi;

	for (i_M1 = 0; i_M1 < STEP_AA; i_M1++)
	{
		fi_M1[i_M1] = GetFreAtTime(fstart, faa, taa, tua, tra, taa / STEP_AA * i_M1);
		MotorTimeTable[i_M1] = F2TIME_PARA / fi_M1[i_M1];							//每步的时间周期，这个地方要注意，容易溢出！——YLS 2021.05.07
		MotorStepTable[i_M1] = fi_M1[i_M1] * (taa / STEP_AA) / Pw_Motor1_STEP_PARA; //每个时钟周期运行的步数
	}
	for (i_M1 = STEP_AA; i_M1 < STEP_AA + STEP_UA; i_M1++)
	{
		fi_M1[i_M1] = GetFreAtTime(fstart, faa, taa, tua, tra, taa + (tua / STEP_UA) * (i_M1 - STEP_AA));
		MotorTimeTable[i_M1] = F2TIME_PARA / fi_M1[i_M1];
		MotorStepTable[i_M1] = fi_M1[i_M1] * (tua / STEP_UA) / Pw_Motor1_STEP_PARA;
	}
	for (i_M1 = STEP_AA + STEP_UA; i_M1 < STEP_AA + STEP_UA + STEP_RA; i_M1++)
	{
		fi_M1[i_M1] = GetFreAtTime(fstart, faa, taa, tua, tra, taa + tua + tra / STEP_RA * (i_M1 - STEP_AA - STEP_UA));
		MotorTimeTable[i_M1] = F2TIME_PARA / fi_M1[i_M1];
		MotorStepTable[i_M1] = fi_M1[i_M1] * (tra / STEP_RA) / Pw_Motor1_STEP_PARA;
	}
	fi_M1[i_M1] = GetFreAtTime(fstart, faa, taa, tua, tra, taa + tua + tra);
	MotorTimeTable[STEP_AA + STEP_UA + STEP_RA] = F2TIME_PARA / fi_M1[i_M1];
	MotorStepTable[STEP_AA + STEP_UA + STEP_RA] = fi_M1[i_M1] * (tra / STEP_RA) / Pw_Motor1_STEP_PARA;

	for (i_M1 = STEP_AA + STEP_UA + STEP_RA + 1; i_M1 < 2 * (STEP_AA + STEP_UA + STEP_RA) + 1; i_M1++)
	{
		MotorTimeTable[i_M1] = MotorTimeTable[2 * (STEP_AA + STEP_UA + STEP_RA) - i_M1];
		MotorStepTable[i_M1] = MotorStepTable[2 * (STEP_AA + STEP_UA + STEP_RA) - i_M1];
	}
}

/**************************************************************************************
电机运行参数初始化*/
void MotorRunParaInitial(void)
{
	/*FIXME:用户可以改变该参数实现S型曲线的升降特性*/
	// CalcMotorPeriStep_CPF(Pw_Motor1_FRE_START, Pw_Motor1_FRE_AA, M_T_AA, M_T_UA, M_T_RA, Motor1TimeTable, Motor1StepTable);
	CalcMotorPeriStep_CPF(M_FRE_START, M_FRE_AA, M_T_AA, M_T_UA, M_T_RA, Motor1TimeTable, Motor1StepTable);
	CalcMotorPeriStep_CPF(M_FRE_START, M_FRE_AA, M_T_AA, M_T_UA, M_T_RA, Motor2TimeTable, Motor2StepTable);
	CalcMotorPeriStep_CPF(M_FRE_START, M_FRE_AA, M_T_AA, M_T_UA, M_T_RA, Motor3TimeTable, Motor3StepTable);
	CalcMotorPeriStep_CPF(M_FRE_START, M_FRE_AA, M_T_AA, M_T_UA, M_T_RA, Motor4TimeTable, Motor4StepTable);
	CalcMotorPeriStep_CPF(M_FRE_START, M_FRE_AA, M_T_AA, M_T_UA, M_T_RA, Motor5TimeTable, Motor5StepTable);
	CalcMotorPeriStep_CPF(M_FRE_START, M_FRE_AA, M_T_AA, M_T_UA, M_T_RA, Motor6TimeTable, Motor6StepTable);
}

/**************************************************************************************
两个电机同时运行时，花费时间较少的电机要根据时间长的电机调整运行参数
*/
void Find_BestTimeCost(unsigned char ID, unsigned long long time_cost, unsigned char dir, unsigned int Degree)
{
	int id = ID;
	float i = 0, j = 0;
	float fi = M_FRE_START, fj = M_FRE_START;
	int cal_ij = 1;
	unsigned int PulsesGiven = 0;
	uint16_t *MotorTimeTable;
	uint16_t *MotorStepTable;
	MOTOR_CONTROL_S *pmotor = NULL;
	unsigned long long time_cost_tmp = 0;
	unsigned long long time_cost_min = 0xffffffffff;
	float i_o = 0, j_o = 0;
	float fi_o = 0, fj_o = 0;
	switch (ID)
	{
	case 1:
		pmotor = &motor1;
		MotorTimeTable = Motor1TimeTable;
		MotorStepTable = Motor1StepTable;
		break;
	case 2:
		pmotor = &motor2;
		MotorTimeTable = Motor2TimeTable;
		MotorStepTable = Motor2StepTable;
		break;
	}
	if (pmotor == NULL)
	{
		return;
	}
	j = M_FRE_AA;
	i = 0;
	while (1)
	{
		if (cal_ij)
		{
			CalcMotorPeriStep_CPF(M_FRE_START, (i + j) / 2.0, M_T_AA, M_T_UA, M_T_RA, MotorTimeTable, MotorStepTable);
		}
		else
		{
			CalcMotorPeriStep_CPF((fi + fj) / 2, 0, M_T_AA, M_T_UA, M_T_RA, MotorTimeTable, MotorStepTable);
		}
		pmotor->en = 1;
		pmotor->dir = dir;
		pmotor->running = 1;
		pmotor->PulsesHaven = 0;
		PulsesGiven = Degree;
		pmotor->Time_Cost_Act = 0;
		pmotor->PulsesGiven = PulsesGiven * pmotor->divnum;
		//pmotor->PulsesGiven+=300;
		Motor_Reinitial(id);
		//time_cost_tmp=Get_Time_Cost2(id);
		time_cost_tmp = pmotor->Time_Cost_Cal;
		if (time_cost_tmp < time_cost)
		{
			if (time_cost - time_cost_tmp < time_cost_min)
			{
				time_cost_min = time_cost - time_cost_tmp;
				i_o = i;
				j_o = j;
				fi_o = fi;
				fj_o = fj;
			}
		}
		else
		{
			if (time_cost_tmp - time_cost < time_cost_min)
			{
				time_cost_min = time_cost_tmp - time_cost;
				i_o = i;
				j_o = j;
				fi_o = fi;
				fj_o = fj;
			}
		}
		if (time_cost_tmp >= time_cost - 32 * 4 && time_cost_tmp <= time_cost + 32 * 4)
		{
			break;
		}
		if (cal_ij)
		{
			if (j < 0.1)
			{
				//说明即使是使用最基本的启动速度都无法同时停止，则修改为降低速度的梯形
				i_o = 0;
				j_o = 0;
				j = 0;
				i = 0;
				cal_ij = 0;
				fi = 0;
			}
		}
		if (cal_ij)
		{
			if ((i > j && i - j < 0.02 || (i < j) && j - i < 0.02))
			{
				break;
			}
			if (time_cost_tmp > time_cost)
			{
				i = (i + j) / 2.0;
			}
			else
			{
				j = (i + j) / 2.0;
			}
		}
		else
		{
			if ((fi > fj && fi - fj < 0.02 || (fi < fj) && fj - fi < 0.02))
			{
				break;
			}
			if (time_cost_tmp > time_cost)
			{
				fi = (fi + fj) / 2.0;
			}
			else
			{
				fj = (fi + fj) / 2.0;
			}
		}
	}
	CalcMotorPeriStep_CPF((fi_o + fj_o) / 2, (i_o + j_o) / 2.0, M_T_AA, M_T_UA, M_T_RA, MotorTimeTable, MotorStepTable);
}

/**************************************************************************************
启动电机按照S型曲线参数运行*/
void Start_Motor_S(unsigned char MotorID, unsigned char dir, uint32_t Degree, uint32_t MaxSpeed_S, uint32_t AccSpeed_S)
{
	unsigned int PulsesGiven = 0;
	MOTOR_CONTROL_S *pmotor = NULL;
	if (Degree == 0)
	{
		return;
	}
	switch (MotorID)
	{
	case 1:
		pmotor = &motor1;
		if (0 == dir)
			MOTOR1_DIR_FORWARD();
		else
			MOTOR1_DIR_REVERSAL();
		break;
	case 2:
		pmotor = &motor2;
		if (1 == dir)
			MOTOR2_DIR_FORWARD();
		else
			MOTOR2_DIR_REVERSAL();
		break;
	case 3:
		pmotor = &motor3;
		if (0 == dir)
			MOTOR3_DIR_FORWARD();
		else
			MOTOR3_DIR_REVERSAL();
		break;
	case 4:
		pmotor = &motor4;
		if (0 == dir)
			MOTOR4_DIR_FORWARD();
		else
			MOTOR4_DIR_REVERSAL();
		break;
	case 5:
		pmotor = &motor5;
		if (0 == dir)
			MOTOR5_DIR_FORWARD();
		else
			MOTOR5_DIR_REVERSAL();
		break;
	case 6:
		pmotor = &motor6;
		if (0 == dir)
			MOTOR6_DIR_FORWARD();
		else
			MOTOR6_DIR_REVERSAL();
		break;
	default:
		return;
	}
	pmotor->en = 1;
	pmotor->dir = dir;
	pmotor->running = 1;
	pmotor->PulsesHaven = 0;
	PulsesGiven = Degree;
	pmotor->Time_Cost_Act = 0;
	pmotor->PulsesGiven = PulsesGiven * pmotor->divnum;
	Motor_Reinitial(MotorID);
	pmotor->CurrentIndex = 0;
	pmotor->speedenbale = 0;

	pmotor->TIMx->ARR = Motor1TimeTable[0]; //设置周期
	// pmotor->TIMx->CCR1 = Motor1TimeTable[0] >> 1; //设置占空比
	if (pmotor->TIMx == TIM2)
	{
		pmotor->TIMx->CCR2 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
		pmotor->TIMx->CNT = 0;
	}
	else if (pmotor->TIMx == TIM4)
	{
		pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
		pmotor->TIMx->CNT = 0;
	}
	else if (pmotor->TIMx == TIM8)
	{
		pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
		pmotor->TIMx->CNT = 0;
	}
	else if (pmotor->TIMx == TIM3)
	{
		pmotor->TIMx->CCR4 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
		pmotor->TIMx->CNT = 0;
	}
	else if (pmotor->TIMx == TIM1)
	{
		pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
		pmotor->TIMx->CNT = 0;
	}
	else if (pmotor->TIMx == TIM5)
	{
		pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
		pmotor->TIMx->CNT = 0;
	}
	// TIM_Cmd(pmotor->TIMx, ENABLE);				  //DISABLE
	switch (MotorID)
	{
	case 1:
		pmotor = &motor1;
		htim2_MOTOR1.Instance = pmotor->TIMx;
		HAL_TIM_Base_Start(&htim2_MOTOR1);
		break;
	case 2:
		pmotor = &motor2;
		htim4_MOTOR2.Instance = pmotor->TIMx;
		HAL_TIM_Base_Start(&htim4_MOTOR2);
		break;
	case 3:
		pmotor = &motor3;
		htim8_MOTOR3.Instance = pmotor->TIMx;
		HAL_TIM_Base_Start(&htim8_MOTOR3);
		break;
	case 4:
		pmotor = &motor4;
		htim3_MOTOR4.Instance = pmotor->TIMx;
		HAL_TIM_Base_Start(&htim3_MOTOR4);
		break;
	case 5:
		pmotor = &motor5;
		htim1_MOTOR5.Instance = pmotor->TIMx;
		HAL_TIM_Base_Start(&htim1_MOTOR5);
		break;
	case 6:
		pmotor = &motor6;
		htim5_MOTOR6.Instance = pmotor->TIMx;
		HAL_TIM_Base_Start(&htim5_MOTOR6);
		break;
	default:
		return;
	}; // 禁止使能定时器     YLS 04.22
}

// /*启动电机按照SPTA方式运行*/
// void Start_Motor_SPTA(unsigned char MotorID, unsigned char dir, uint32_t Degree, uint32_t MaxSpeed_SPTA, uint32_t AccSpeed_SPTA)
// {
// 	unsigned int PulsesGiven = 0;
// 	MOTOR_CONTROL_SPTA *pmotor = NULL;
// 	if (Degree == 0)
// 	{
// 		return;
// 	}
// 	switch (MotorID)
// 	{
// 	case 4:
// 		pmotor = &motor4;
// 		if (0 == dir)
// 		{
// 			//		    GPIO_SetBits(GPIOB,GPIO_Pin_9);
// 			MOTOR4_DIR_FORWARD();
// 		}
// 		else
// 		{
// 			//		    GPIO_ResetBits(GPIOB,GPIO_Pin_9);
// 			MOTOR4_DIR_REVERSAL();
// 		}
// 		break;
// 	default:
// 		return;
// 	}

// 	pmotor->en = 1;
// 	pmotor->dir = dir;
// 	pmotor->running = 1;
// 	pmotor->speedenbale = 0;
// 	PulsesGiven = Degree;
// 	pmotor->step_move = PulsesGiven * pmotor->divnum;
// 	pmotor->step_middle = pmotor->step_move >> 1;
// 	/*FIXME:这两个参数可以由用户自行改变测试*/
// 	pmotor->step_spmax = MaxSpeed_SPTA; //SPTA最大速度 MAXSPEED_SPTA
// 	pmotor->step_accel = AccSpeed_SPTA; //SPTA加速度 ACCSPEED_SPTA
// 	pmotor->step_state = ACCELERATING;
// 	pmotor->step_frac = 0;
// 	pmotor->speed_frac = 0;
// 	pmotor->step_acced = 0;
// 	pmotor->step_speed = 0;
// 	pmotor->step_count = 0;

// 	// TIM_Cmd(pmotor->TIMx, ENABLE);
// 	htim3_MOTOR4.Instance = pmotor->TIMx;
// 	HAL_TIM_Base_Start(&htim3_MOTOR4); // 禁止使能定时器     YLS 04.22
// }

/*启动电机，根据电机号决定调用哪个*/
void Start_Motor(unsigned char MotorID, unsigned char dir, unsigned int Degree, uint32_t MaxSpeed_SPTA, uint32_t AccSpeed_SPTA)
{
	if (MotorID > 3)
	{
		// Start_Motor_S(MotorID, dir, Degree);
	}
	else
	{
		Start_Motor_SPTA(MotorID, dir, Degree, MaxSpeed_SPTA, AccSpeed_SPTA);
	}
}

/*重新定位，让正在运行的电机运行到指定位置*/
void Reposition_Motor(unsigned char MotorID, unsigned int NewPos, uint32_t MaxSpeed_SPTA, uint32_t AccSpeed_SPTA)
{
	MOTOR_CONTROL_S *pmotor_s = NULL;
	MOTOR_CONTROL_SPTA *pmotor_spta = NULL;
	switch (MotorID)
	{
	case 1:
		pmotor_s = &motor1;
		break;
	case 2:
		pmotor_s = &motor2;
		break;
	case 3:
		pmotor_s = &motor3;
		break;
	case 4:
		pmotor_s = &motor4;
		break;
	case 5:
		pmotor_s = &motor5;
		break;
	case 6:
		pmotor_s = &motor6;
		break;
	default:
		return;
	}
	if (pmotor_s != NULL)
	{
		if (NewPos <= pmotor_s->MaxPosition && NewPos != pmotor_s->CurrentPosition)
		{
			if (NewPos > pmotor_s->CurrentPosition)
			{
				// Start_Motor_S(MotorID, pmotor_s->clockwise, NewPos - pmotor_s->CurrentPosition);
			}
			else
			{
				// Start_Motor_S(MotorID, !pmotor_s->clockwise, pmotor_s->CurrentPosition - NewPos);
			}
			while (pmotor_s->running == 1)
				;
		}
	}
}
/*同时启动电机12*/
void Start_Motor12(unsigned char dir1, unsigned int Degree1, unsigned char dir2, unsigned int Degree2)
{

	unsigned int PulsesGiven = 0;
	unsigned long long time_cost = 0;
	if (Degree1 == 0 || Degree2 == 0)
	{
		return;
	}
	MotorRunParaInitial();
	Motor_Reinitial(1);
	Motor_Reinitial(2);
	if (Degree1 <= Degree2)
	{
		motor2.en = 1;
		motor2.dir = dir1;
		motor2.running = 1;
		motor2.PulsesHaven = 0;
		PulsesGiven = Degree2;
		motor2.Time_Cost_Act = 0;
		motor2.PulsesGiven = PulsesGiven * motor2.divnum;
		time_cost = Get_TimeCost_ReverDot_S(2);
		Find_BestTimeCost(1, time_cost, dir1, Degree1);
	}
	else
	{
		motor1.en = 1;
		motor1.dir = dir2;
		motor1.running = 1;
		motor1.PulsesHaven = 0;
		PulsesGiven = Degree1;
		motor1.Time_Cost_Act = 0;
		motor1.PulsesGiven = PulsesGiven * motor1.divnum;
		time_cost = Get_TimeCost_ReverDot_S(1);
		Find_BestTimeCost(2, time_cost, dir2, Degree2);
	}

	// Start_Motor_S(1, dir1, Degree1);
	// Start_Motor_S(2, dir2, Degree2);
}

/*设置电机运行速度，入口参数是速度等级*/
void SetSpeed(unsigned char MotorID, signed char speedindex)
{
	int currentindex, i;
	unsigned int destspeed;
	unsigned int stepstostop = 0;
	MOTOR_CONTROL_S *pmotor_s = NULL;
	MOTOR_CONTROL_SPTA *pmotor_spta = NULL;
	switch (MotorID)
	{
	case 1:
		pmotor_s = &motor1;
		htim2_MOTOR1.Instance = pmotor_s->TIMx;
		HAL_TIM_Base_Stop(&htim2_MOTOR1); // 禁止使能定时器     YLS 04.29
		break;
	case 2:
		pmotor_s = &motor2;
		htim4_MOTOR2.Instance = pmotor_s->TIMx;
		HAL_TIM_Base_Stop(&htim4_MOTOR2); // 禁止使能定时器     YLS 04.29
		break;
	case 3:
		pmotor_s = &motor3;
		htim8_MOTOR3.Instance = pmotor_s->TIMx;
		HAL_TIM_Base_Stop(&htim8_MOTOR3); // 禁止使能定时器     YLS 04.29
		break;
	case 4:
		pmotor_s = &motor4;
		htim3_MOTOR4.Instance = pmotor_s->TIMx;
		HAL_TIM_Base_Stop(&htim3_MOTOR4); // 禁止使能定时器     YLS 04.29
		break;
	case 5:
		pmotor_s = &motor5;
		htim1_MOTOR5.Instance = pmotor_s->TIMx;
		HAL_TIM_Base_Stop(&htim1_MOTOR5); // 禁止使能定时器     YLS 04.29
		break;
	case 6:
		pmotor_s = &motor6;
		htim5_MOTOR6.Instance = pmotor_s->TIMx;
		HAL_TIM_Base_Stop(&htim5_MOTOR6); // 禁止使能定时器     YLS 04.29
		break;
	default:
		return;
	}
	if (pmotor_s != NULL)
	{
		// TIM_Cmd(pmotor_s->TIMx, DISABLE);
		// htim_MOTOR_G.Instance = pmotor_s->TIMx;
		// HAL_TIM_Base_Stop(&htim_MOTOR_G); // 禁止使能定时器     YLS 04.22
		if (speedindex >= 0 && speedindex <= STEP_AA + STEP_UA + STEP_RA)
		{
			//直接向下一速度
			currentindex = pmotor_s->CurrentIndex;
			pmotor_s->PulsesHaven = 0;
			if (pmotor_s->CurrentIndex >= pmotor_s->StartTableLength)
			{
				currentindex = pmotor_s->StartTableLength + pmotor_s->StopTableLength - pmotor_s->CurrentIndex - 1;
			}
			if (currentindex > speedindex)
			{
				//需要减速
				pmotor_s->PulsesGiven = pmotor_s->PulsesHaven + pmotor_s->StopSteps - 2;
			}
			else
			{
				//需要加速
				pmotor_s->PulsesGiven = 0xffffffff;
			}
			pmotor_s->CurrentIndex = currentindex;
			pmotor_s->pulsecount = pmotor_s->Step_Table[pmotor_s->CurrentIndex];
			pmotor_s->TargetIndex = speedindex;
			pmotor_s->speedenbale = 1;
			pmotor_s->running = 1;
			pmotor_s->en = 1;
		}
		else
		{
			//停止电机,pmotor->CurrentIndex=currentindex-1;直接向下一减速
			pmotor_s->speedenbale = 0;
			currentindex = pmotor_s->CurrentIndex;
			if (pmotor_s->CurrentIndex >= pmotor_s->StartTableLength)
			{
				currentindex = pmotor_s->StartTableLength + pmotor_s->StopTableLength - pmotor_s->CurrentIndex - 1;
			}
			if (1) //currentindex>=1)
			{
				for (i = 0; i < currentindex; i++)
				{
					stepstostop += pmotor_s->Step_Table[i];
				}
				/*进入减速index*/
				if (pmotor_s->CurrentIndex < pmotor_s->StartTableLength)
				{
					currentindex = pmotor_s->StartTableLength + pmotor_s->StopTableLength - pmotor_s->CurrentIndex - 1;
				}
				pmotor_s->CurrentIndex = currentindex;
				pmotor_s->pulsecount = pmotor_s->Step_Table[pmotor_s->CurrentIndex];
				pmotor_s->PulsesHaven = 0;
				pmotor_s->PulsesGiven = pmotor_s->PulsesHaven + stepstostop;
			}
			else
			{
				pmotor_s->PulsesGiven = pmotor_s->PulsesHaven;
			}
			if (stepstostop == 0)
			{
				//已经停止
				return;
			}
		}
		// TIM_Cmd(pmotor_s->TIMx, ENABLE);
		switch (MotorID)
		{
		case 1:
			pmotor_s = &motor1;
			htim2_MOTOR1.Instance = pmotor_s->TIMx;
			HAL_TIM_Base_Start(&htim2_MOTOR1); // 禁止使能定时器     YLS 04.29
			break;
		case 2:
			pmotor_s = &motor2;
			htim4_MOTOR2.Instance = pmotor_s->TIMx;
			HAL_TIM_Base_Start(&htim4_MOTOR2); // 禁止使能定时器     YLS 04.29
			break;
		case 3:
			pmotor_s = &motor3;
			htim8_MOTOR3.Instance = pmotor_s->TIMx;
			HAL_TIM_Base_Start(&htim8_MOTOR3); // 禁止使能定时器     YLS 04.29
			break;
		case 4:
			pmotor_s = &motor4;
			htim3_MOTOR4.Instance = pmotor_s->TIMx;
			HAL_TIM_Base_Start(&htim3_MOTOR4); // 禁止使能定时器     YLS 04.29
			break;
		case 5:
			pmotor_s = &motor5;
			htim1_MOTOR5.Instance = pmotor_s->TIMx;
			HAL_TIM_Base_Start(&htim1_MOTOR5); // 禁止使能定时器     YLS 04.29
			break;
		case 6:
			pmotor_s = &motor6;
			htim5_MOTOR6.Instance = pmotor_s->TIMx;
			HAL_TIM_Base_Start(&htim5_MOTOR6); // 禁止使能定时器     YLS 04.29
			break;
		default:
			return;
		}
	}
}

/*设置电机的位置，电机运行到指定的位置*/
void SetPosition(unsigned char MotorID, unsigned int dest, uint32_t MaxSpeed_SPTA, uint32_t AccSpeed_SPTA)
{
	MOTOR_CONTROL_S *pmotor_s = NULL;
	MOTOR_CONTROL_SPTA *pmotor_spta = NULL;
	switch (MotorID)
	{
	case 1:
		pmotor_s = &motor1;
		break;
	case 2:
		pmotor_s = &motor2;
		break;
	case 3:
		pmotor_s = &motor3;
		break;
	case 4:
		pmotor_s = &motor4;
		break;
	case 5:
		pmotor_s = &motor5;
		break;
	case 6:
		pmotor_s = &motor6;
		break;
	default:
		return;
	}
	if (pmotor_s != NULL)
	{
		if (dest <= pmotor_s->MaxPosition && dest != pmotor_s->CurrentPosition)
		{
			if (dest > pmotor_s->CurrentPosition)
			{
				// Start_Motor_S(MotorID, pmotor_s->clockwise, dest - pmotor_s->CurrentPosition);
			}
			else
			{
				// Start_Motor_S(MotorID, !pmotor_s->clockwise, pmotor_s->CurrentPosition - dest);
			}
			while (pmotor_s->running == 1)
				;
		}
	}
}

/*复位电机*/
void Do_Reset(unsigned char MotorID)
{
	MOTOR_CONTROL_S *pmotor_s = NULL;
	MOTOR_CONTROL_SPTA *pmotor_spta = NULL;
	switch (MotorID)
	{
	case 1:
		pmotor_s = &motor1;
		break;
	case 2:
		pmotor_s = &motor2;
		break;
	case 3:
		pmotor_s = &motor3;
		break;
	case 4:
		pmotor_s = &motor4;
		break;
	case 5:
		pmotor_s = &motor5;
		break;
	case 6:
		pmotor_s = &motor6;
		break;
	default:
		return;
	}
	/*do reset*/
	if (pmotor_s != NULL)
	{
		pmotor_s->rstflg = 1;
		pmotor_s->running = 1;
		SetSpeed(MotorID, 16);
		while (pmotor_s->running == 1)
			;
	}
}

// void Deal_Serail_Cmd(char *buf, int len)
// {
// 	unsigned int dest = 0;
// 	/*
// 			0xAA 0x55(起始标志）
// 			Data0(电机号，1,2,3,4)
// 			Data1(0,复位，1，设置速度，2，停止电机，3 ，让电机运行到指定位置?
// 			Data2（在data1=1时候，该字节表示速度等级，在data1=3时，该字节表示位置的高8位
// 			Data3（在data1=3时，该字节表示位置的中8位
// 			Data4（在data1=3时，该字节表示位置的低8位
// 			*/B
// 	if (buf[0] == 0xAA)
// 	{
// 		if (buf[3] == 0)
// 		{
// 			Do_Reset(buf[2]);
// 		}
// 		else if (buf[3] == 1)
// 		{
// 			SetSpeed(buf[2], buf[4]);
// 		}
// 		else if (buf[3] == 2)
// 		{
// 			SetSpeed(buf[2], -1);
// 		}
// 		else if (buf[3] == 3)
// 		{
// 			dest = (buf[4] << 16) + (buf[5] << 8) + (buf[6]);
// 			SetPosition(buf[2], dest);
// 		}
// 	}
// }

/*
void Deal_Cmd(void)
{  
	unsigned char sync[2]; 
	while(rt_ringbuffer_data_len(&rb_recv)>=7)
	{ 
		if(rt_ringbuffer_peek(&rb_recv,sync,2)==2)
		{
			if(sync[0]==0xaa&&sync[1]==0x55)
			{
				if(rt_ringbuffer_get(&rb_recv,MY_Cmd_Buff,7)==7)
				{
				//	Deal_Serail_Cmd(MY_Cmd_Buff,7); 
				}	
			}
			else if(sync[1]==0xaa)
			{
				rt_ringbuffer_getchar(&rb_recv,sync);
			}
			else
			{
				rt_ringbuffer_getchar(&rb_recv,sync);
				rt_ringbuffer_getchar(&rb_recv,sync);
			}
		}
	} 
}
  */
/*电机1的PWM输出初始化，使用的是定时器4*/
void Initial_PWM_Motor1(void)
{
	TIM_OC_InitTypeDef sConfigOC; // 定时器通道比较输出

	MOTOR1_TIM2_RCC_CLK_ENABLE();

	/* STEPMOTOR相关GPIO初始化配置 */
	MOTOR1_GPIO_Init();

	/* 定时器基本环境配置 */
	htim2_MOTOR1.Instance = MOTOR1_TIM2;					  // 定时器编号
	htim2_MOTOR1.Init.Prescaler = MOTOR1_TIM_PRESCALER;		  // 定时器预分频器
	htim2_MOTOR1.Init.CounterMode = TIM_COUNTERMODE_UP;		  // 计数方向：向上计数
	htim2_MOTOR1.Init.Period = 1000;						  // 定时器周期MOTOR1_TIM_PERIOD
	htim2_MOTOR1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频
	HAL_TIM_Base_Init(&htim2_MOTOR1);

	/* 定时器比较输出配置 */
	sConfigOC.OCMode = TIM_OCMODE_PWM2;				 // 比较输出模式：PWM2输出
	sConfigOC.Pulse = 500;							 // 脉冲数
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;		 // 输出极性
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;	 // 互补通道输出极性
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;		 // 快速模式
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;	 // 空闲电平
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // 互补通道空闲电平
	HAL_TIM_PWM_ConfigChannel(&htim2_MOTOR1, &sConfigOC, MOTOR1_TIM2_CHANNEL_x);
	/* 使能比较输出通道 */
	TIM_CCxChannelCmd(MOTOR1_TIM2, MOTOR1_TIM2_CHANNEL_x, TIM_CCx_ENABLE);

	/* 配置定时器中断优先级并使能 */
	HAL_NVIC_SetPriority(MOTOR1_TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR1_TIM2_IRQn);

	//清中断，以免一启用中断后立即产生中断
	// TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	__HAL_TIM_CLEAR_FLAG(&htim2_MOTOR1, MOTOR1_TIM2_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htim2_MOTOR1, TIM_IT_UPDATE);

	//使能TIM1中断源
	// TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	// __HAL_TIM_ENABLE_IT(&htim2_MOTOR1, MOTOR1_TIM2_IT_CCx);
	__HAL_TIM_ENABLE_IT(&htim2_MOTOR1, TIM_IT_UPDATE);

	/* Enable the main output */
	// TIM_Cmd(TIM1, DISABLE);
	// TIM_CtrlPWMOutputs(TIM1, ENABLE); //使能PWM输出
	HAL_TIM_Base_Stop(&htim2_MOTOR1); // 使能定时器
	__HAL_TIM_MOE_ENABLE(&htim2_MOTOR1);
}

void Initial_PWM_Motor2(void)
{
	TIM_OC_InitTypeDef sConfigOC; // 定时器通道比较输出

	MOTOR2_TIM4_RCC_CLK_ENABLE();

	/* STEPMOTOR相关GPIO初始化配置 */
	MOTOR2_GPIO_Init();

	/* 定时器基本环境配置 */
	htim4_MOTOR2.Instance = MOTOR2_TIM4;					  // 定时器编号
	htim4_MOTOR2.Init.Prescaler = MOTOR2_TIM_PRESCALER;		  // 定时器预分频器
	htim4_MOTOR2.Init.CounterMode = TIM_COUNTERMODE_UP;		  // 计数方向：向上计数
	htim4_MOTOR2.Init.Period = 1000;						  // 定时器周期MOTOR2_TIM_PERIOD
	htim4_MOTOR2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频
	HAL_TIM_Base_Init(&htim4_MOTOR2);

	/* 定时器比较输出配置 */
	sConfigOC.OCMode = TIM_OCMODE_PWM2;				 // 比较输出模式：PWM2输出
	sConfigOC.Pulse = 500;							 // 脉冲数
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;		 // 输出极性
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;	 // 互补通道输出极性
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;		 // 快速模式
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;	 // 空闲电平
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // 互补通道空闲电平
	HAL_TIM_PWM_ConfigChannel(&htim4_MOTOR2, &sConfigOC, MOTOR2_TIM4_CHANNEL_x);
	/* 使能比较输出通道 */
	TIM_CCxChannelCmd(MOTOR2_TIM4, MOTOR2_TIM4_CHANNEL_x, TIM_CCx_ENABLE);

	/* 配置定时器中断优先级并使能 */
	HAL_NVIC_SetPriority(MOTOR2_TIM4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR2_TIM4_IRQn);

	//清中断，以免一启用中断后立即产生中断
	__HAL_TIM_CLEAR_FLAG(&htim4_MOTOR2, MOTOR2_TIM4_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htim4_MOTOR2, TIM_IT_UPDATE);

	//使能TIM1中断源
	// TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	// __HAL_TIM_ENABLE_IT(&htim4_MOTOR2, MOTOR2_TIM4_IT_CCx);
	__HAL_TIM_ENABLE_IT(&htim4_MOTOR2, TIM_IT_UPDATE);

	/* Enable the main output */
	// TIM_Cmd(TIM1, DISABLE);
	// TIM_CtrlPWMOutputs(TIM1, ENABLE); //使能PWM输出
	HAL_TIM_Base_Stop(&htim4_MOTOR2); // 使能定时器
	__HAL_TIM_MOE_ENABLE(&htim4_MOTOR2);
}
//
void Initial_PWM_Motor3(void)
{
	TIM_OC_InitTypeDef sConfigOC; // 定时器通道比较输出

	MOTOR3_TIM8_RCC_CLK_ENABLE();

	/* STEPMOTOR相关GPIO初始化配置 */
	MOTOR3_GPIO_Init();

	/* 定时器基本环境配置 */
	htim8_MOTOR3.Instance = MOTOR3_TIM8;					  // 定时器编号
	htim8_MOTOR3.Init.Prescaler = MOTOR3_TIM_PRESCALER;		  // 定时器预分频器
	htim8_MOTOR3.Init.CounterMode = TIM_COUNTERMODE_UP;		  // 计数方向：向上计数
	htim8_MOTOR3.Init.Period = 1000;						  // 定时器周期MOTOR3_TIM_PERIOD
	htim8_MOTOR3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频
	HAL_TIM_Base_Init(&htim8_MOTOR3);

	/* 定时器比较输出配置 */
	sConfigOC.OCMode = TIM_OCMODE_PWM2;				 // 比较输出模式：PWM2输出
	sConfigOC.Pulse = 500;							 // 脉冲数
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;		 // 输出极性
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;	 // 互补通道输出极性
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;		 // 快速模式
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;	 // 空闲电平
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // 互补通道空闲电平
	HAL_TIM_PWM_ConfigChannel(&htim8_MOTOR3, &sConfigOC, MOTOR3_TIM8_CHANNEL_x);
	/* 使能比较输出通道 */
	TIM_CCxChannelCmd(MOTOR3_TIM8, MOTOR3_TIM8_CHANNEL_x, TIM_CCx_ENABLE);

	/* 配置定时器中断优先级并使能 */
	HAL_NVIC_SetPriority(MOTOR3_TIM8_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR3_TIM8_IRQn);

	//清中断，以免一启用中断后立即产生中断
	// TIM_ClearFlag(TIM8, TIM_FLAG_Update);
	__HAL_TIM_CLEAR_FLAG(&htim8_MOTOR3, MOTOR3_TIM8_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htim8_MOTOR3, TIM_IT_UPDATE);

	//使能TIM8中断源
	// TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	// __HAL_TIM_ENABLE_IT(&htim8_MOTOR3, MOTOR3_TIM8_IT_CCx);
	__HAL_TIM_ENABLE_IT(&htim8_MOTOR3, TIM_IT_UPDATE);

	/* Enable the main output */
	// TIM_Cmd(TIM8, DISABLE);
	// TIM_CtrlPWMOutputs(TIM8, ENABLE); //使能PWM输出
	HAL_TIM_Base_Stop(&htim8_MOTOR3); // 使能定时器
	__HAL_TIM_MOE_ENABLE(&htim8_MOTOR3);
}
//
void Initial_PWM_Motor4(void)
{
	TIM_OC_InitTypeDef sConfigOC; // 定时器通道比较输出

	MOTOR4_TIM3_RCC_CLK_ENABLE();

	/* STEPMOTOR相关GPIO初始化配置 */
	MOTOR4_GPIO_Init();

	/* 定时器基本环境配置 */
	htim3_MOTOR4.Instance = MOTOR4_TIM3;					  // 定时器编号
	htim3_MOTOR4.Init.Prescaler = MOTOR4_TIM_PRESCALER;		  // 定时器预分频器
	htim3_MOTOR4.Init.CounterMode = TIM_COUNTERMODE_UP;		  // 计数方向：向上计数
	htim3_MOTOR4.Init.Period = 1000;						  // 定时器周期MOTOR4_TIM_PERIOD
	htim3_MOTOR4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频
	HAL_TIM_Base_Init(&htim3_MOTOR4);

	/* 定时器比较输出配置 */
	sConfigOC.OCMode = TIM_OCMODE_PWM2;				 // 比较输出模式：PWM2输出
	sConfigOC.Pulse = 500;							 // 脉冲数
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;		 // 输出极性
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;	 // 互补通道输出极性
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;		 // 快速模式
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;	 // 空闲电平
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // 互补通道空闲电平
	HAL_TIM_PWM_ConfigChannel(&htim3_MOTOR4, &sConfigOC, MOTOR4_TIM3_CHANNEL_x);
	/* 使能比较输出通道 */
	TIM_CCxChannelCmd(MOTOR4_TIM3, MOTOR4_TIM3_CHANNEL_x, TIM_CCx_ENABLE);

	/* 配置定时器中断优先级并使能 */
	HAL_NVIC_SetPriority(MOTOR4_TIM3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR4_TIM3_IRQn);

	//清中断，以免一启用中断后立即产生中断
	// TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	__HAL_TIM_CLEAR_FLAG(&htim3_MOTOR4, MOTOR4_TIM3_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htim3_MOTOR4, TIM_IT_UPDATE);

	//使能TIM3中断源
	// TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	// __HAL_TIM_ENABLE_IT(&htim3_MOTOR4, MOTOR4_TIM3_IT_CCx);
	__HAL_TIM_ENABLE_IT(&htim3_MOTOR4, TIM_IT_UPDATE);

	/* Enable the main output */
	// TIM_Cmd(TIM3, DISABLE);
	// TIM_CtrlPWMOutputs(TIM3, ENABLE); //使能PWM输出
	HAL_TIM_Base_Stop(&htim3_MOTOR4); // 使能定时器
	__HAL_TIM_MOE_ENABLE(&htim3_MOTOR4);
}

void Initial_PWM_Motor5(void)
{
	TIM_OC_InitTypeDef sConfigOC; // 定时器通道比较输出

	MOTOR5_TIM1_RCC_CLK_ENABLE();

	/* STEPMOTOR相关GPIO初始化配置 */
	MOTOR5_GPIO_Init();

	/* 定时器基本环境配置 */
	htim1_MOTOR5.Instance = MOTOR5_TIM1;					  // 定时器编号
	htim1_MOTOR5.Init.Prescaler = MOTOR5_TIM_PRESCALER;		  // 定时器预分频器
	htim1_MOTOR5.Init.CounterMode = TIM_COUNTERMODE_UP;		  // 计数方向：向上计数
	htim1_MOTOR5.Init.Period = 1000;						  // 定时器周期MOTOR5_TIM_PERIOD
	htim1_MOTOR5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频
	HAL_TIM_Base_Init(&htim1_MOTOR5);

	/* 定时器比较输出配置 */
	sConfigOC.OCMode = TIM_OCMODE_PWM2;				 // 比较输出模式：PWM2输出
	sConfigOC.Pulse = 500;							 // 脉冲数
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;		 // 输出极性
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;	 // 互补通道输出极性
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;		 // 快速模式
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;	 // 空闲电平
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // 互补通道空闲电平
	HAL_TIM_PWM_ConfigChannel(&htim1_MOTOR5, &sConfigOC, MOTOR5_TIM1_CHANNEL_x);
	/* 使能比较输出通道 */
	TIM_CCxChannelCmd(MOTOR5_TIM1, MOTOR5_TIM1_CHANNEL_x, TIM_CCx_ENABLE);

	/* 配置定时器中断优先级并使能 */
	HAL_NVIC_SetPriority(MOTOR5_TIM1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR5_TIM1_IRQn);

	//清中断，以免一启用中断后立即产生中断
	// TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	__HAL_TIM_CLEAR_FLAG(&htim1_MOTOR5, MOTOR5_TIM1_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htim1_MOTOR5, TIM_IT_UPDATE);

	//使能TIM1中断源
	// TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	// __HAL_TIM_ENABLE_IT(&htim1_MOTOR5, MOTOR5_TIM1_IT_CCx);
	__HAL_TIM_ENABLE_IT(&htim1_MOTOR5, TIM_IT_UPDATE);

	/* Enable the main output */
	// TIM_Cmd(TIM1, DISABLE);
	// TIM_CtrlPWMOutputs(TIM1, ENABLE); //使能PWM输出
	HAL_TIM_Base_Stop(&htim1_MOTOR5); // 使能定时器
	__HAL_TIM_MOE_ENABLE(&htim1_MOTOR5);
}

void Initial_PWM_Motor6(void)
{
	TIM_OC_InitTypeDef sConfigOC; // 定时器通道比较输出

	MOTOR6_TIM5_RCC_CLK_ENABLE();

	/* STEPMOTOR相关GPIO初始化配置 */
	MOTOR6_GPIO_Init();

	/* 定时器基本环境配置 */
	htim5_MOTOR6.Instance = MOTOR6_TIM5;					  // 定时器编号
	htim5_MOTOR6.Init.Prescaler = MOTOR6_TIM_PRESCALER;		  // 定时器预分频器
	htim5_MOTOR6.Init.CounterMode = TIM_COUNTERMODE_UP;		  // 计数方向：向上计数
	htim5_MOTOR6.Init.Period = 1000;						  // 定时器周期MOTOR6_TIM_PERIOD
	htim5_MOTOR6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频
	HAL_TIM_Base_Init(&htim5_MOTOR6);

	/* 定时器比较输出配置 */
	sConfigOC.OCMode = TIM_OCMODE_PWM2;				 // 比较输出模式：PWM2输出
	sConfigOC.Pulse = 500;							 // 脉冲数
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;		 // 输出极性
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;	 // 互补通道输出极性
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;		 // 快速模式
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;	 // 空闲电平
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // 互补通道空闲电平
	HAL_TIM_PWM_ConfigChannel(&htim5_MOTOR6, &sConfigOC, MOTOR6_TIM5_CHANNEL_x);
	/* 使能比较输出通道 */
	TIM_CCxChannelCmd(MOTOR6_TIM5, MOTOR6_TIM5_CHANNEL_x, TIM_CCx_ENABLE);

	/* 配置定时器中断优先级并使能 */
	HAL_NVIC_SetPriority(MOTOR6_TIM5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR6_TIM5_IRQn);

	//清中断，以免一启用中断后立即产生中断
	// TIM_ClearFlag(TIM5, TIM_FLAG_Update);
	__HAL_TIM_CLEAR_FLAG(&htim5_MOTOR6, MOTOR6_TIM5_FLAG_CCx);
	__HAL_TIM_CLEAR_FLAG(&htim5_MOTOR6, TIM_IT_UPDATE);

	//使能TIM5中断源
	// TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	// __HAL_TIM_ENABLE_IT(&htim5_MOTOR6, MOTOR6_TIM5_IT_CCx);
	__HAL_TIM_ENABLE_IT(&htim5_MOTOR6, TIM_IT_UPDATE);

	/* Enable the main output */
	// TIM_Cmd(TIM5, DISABLE);
	// TIM_CtrlPWMOutputs(TIM5, ENABLE); //使能PWM输出
	HAL_TIM_Base_Stop(&htim5_MOTOR6); // 使能定时器
	__HAL_TIM_MOE_ENABLE(&htim5_MOTOR6);
}
//void EXTI_Configuration(void)
//{
//  EXTI_InitTypeDef EXTI_InitStructure;        //EXTI初始化结构定义
//   GPIO_InitTypeDef  GPIO_InitStructure;
//   NVIC_InitTypeDef NVIC_InitStructure;
//
//   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);	 //E0-E5  时针使能
//
//   GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4;//选择引脚
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//   GPIO_Init(GPIOE, &GPIO_InitStructure);
//
//
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//
//   EXTI_ClearITPendingBit(EXTI_Line4);//清除中断标志
//   GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource4);
//
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//事件选择
//   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//触发模式
//   EXTI_InitStructure.EXTI_Line = EXTI_Line4; //线路选择
//   EXTI_InitStructure.EXTI_LineCmd = ENABLE;//启动中断
//   EXTI_Init(&EXTI_InitStructure);//初始化
//
//}
