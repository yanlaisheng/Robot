/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdlib.h>
#include <math.h>
#include "ecm_define.h"
#include "GlobalConst.h"
#include "GlobalV.h"
#include "GlobalV_Extern.h" // ȫ�ֱ�������
#include "typedef.h"
#include "wk2xxx.h"
#include "led.h"
#include "w25qxx.h"
#include "bsp_MOTOR3.h"
#include "bsp_MOTOR1.h"
#include "bsp_MOTOR2.h"
#include "bsp_MOTOR4.h"
#include "bsp_MOTOR5.h"
#include "bsp_MOTOR6.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef GNUC
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* GNUC */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint8_t			bNetworkMap[42] = {DRIVE,DRIVE,DRIVE,DRIVE,DRIVE,DRIVE,DRIVE,DRIVE};
uint8_t bNetworkMap[42] = {IO, None, None, None, None, None, None, None};

#define NUM 1000
uint32_t send_Buf[NUM] = {0};

// �ٶ����ֵ���������͵����������Щ�����1800����Щ���Դﵽ4000
__IO uint32_t set_speed = 180; // �ٶ� ��λΪ0.1rad/sec
// ���ٶȺͼ��ٶ�ѡȡһ�����ʵ����Ҫ��ֵԽ���ٶȱ仯Խ�죬�Ӽ��ٽ׶αȽ϶���
// ���Լ��ٶȺͼ��ٶ�ֵһ������ʵ��Ӧ���жೢ�Գ����Ľ��
__IO uint32_t step_accel = 10; // ���ٶ� ��λΪ0.1rad/sec^2
__IO uint32_t step_decel = 10; // ���ٶ� ��λΪ0.1rad/sec^2

#if S_ACCEL
void CalculateSModelLine(float fre[], unsigned short period[], float len, float fre_max, float fre_min, float flexible)
{
	int i = 0;
	float deno;
	float melo;
	float delt = fre_max - fre_min;
	for (; i < len; i++)
	{
		melo = flexible * (i - len / 2) / (len / 2);
		deno = 1.0f / (1 + expf(-melo)); //expf is a library function of exponential(e)?
		fre[i] = delt * deno + fre_min;
		period[i] = (unsigned short)(10000000.0f / fre[i]); // 10000000 is the timer driver frequency
	}
	return;
}
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int startflag = 0;
int32_t targetp;

int32_t current_Pos;
int32_t current_Pos2;
int32_t temp1;
int32_t temp2, temp3, temp4;
uint16_t Master_CMD;
uint16_t Master_state;
uint16_t current_state;
int32_t distance; //���о���
uint8_t dir = 0;  //����
uint16_t asc;	  //���ټ���

struct rbuff_st
{
	uint16_t CMD;
	uint16_t Parm;
	int32_t Data1;
	int32_t Data2;
}; /* Receive Buffer */
extern struct rbuff_st rbuff[];

//Ҫд�뵽W25Q64���ַ�������
const u8 TEXT_Buffer[] = {"Explorer STM32F4 SPI TEST"};
#define SIZE sizeof(TEXT_Buffer)

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//�ض���printf��scanf���޸�fgetc��fputc
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

GETCHAR_PROTOTYPE
{
	int ch;
	HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
int ECMSPIReadWrite(uint8_t *rdata, uint8_t *wdata, int rwlength)
{
	// uint8_t *pTxBuffPtr; /*!< Pointer to SPI Tx transfer Buffer        */
	// uint8_t *pRxBuffPtr; /*!< Pointer to SPI Rx transfer Buffer        */
	// pRxBuffPtr = (uint8_t *)rdata;
	// pTxBuffPtr = (uint8_t *)wdata;
	while (HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin))
		;
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
#if 1
	HAL_SPI_TransmitReceive(&hspi3, wdata, rdata, rwlength, 10);
#else
	for (int i = 0; i < rwlength; i++)
	{
		*((uint8_t *)pRxBuffPtr) = spi3_SendByte(*((uint8_t *)pTxBuffPtr));
		pTxBuffPtr += sizeof(uint8_t);
		pRxBuffPtr += sizeof(uint8_t);
	}
#endif
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
	return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	//	uint32_t temp1, temp2, temp3;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	HAL_Delay(2000); //��ʱ
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM8_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART6_UART_Init();
	MX_TIM7_Init();
	/* USER CODE BEGIN 2 */

	/* ���ö�ʱ��������� */
	MOTOR1_TIMx_Init();
	MOTOR2_TIMx_Init();
	MOTOR3_TIMx_Init();
	MOTOR4_TIMx_Init();
	MOTOR5_TIMx_Init();
	MOTOR6_TIMx_Init();

	ParLst_Init();	 // RAM���趨�������г�ʼ����FLASH����>RAM
	Boot_ParLst();	 // ZCL 2015.7.11 ��ʼ��STM32�趨����
	Variable_Init(); // Initialize VARIABLE
	ParLimit();		 // ��������

	//����EEPROM
	// Test_EEPROM();

	//EtherCAT��ʼ��
//	Init_EtherCAT();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		ParLimit();
		Boot_ParLst(); // ��ʼ���趨����

		Uart_RT(); //�����շ�
		Test_Send_Pulse();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* USER CODE END 3 */
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
  */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
  */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
//��ʱ1us
void PowerDelay(uint16_t nCount) //2015.9.12
{
	//ZCL 2015.9.12 ���� unsigned int = uint32_t ���Դ���65535������ARM STM32��˵
	unsigned int i = 36;
	while (nCount--)
	{
		while (i > 0)
			i--;
		i = 36;
	}
}

void HAL_SYSTICK_Callback(void)
{
	static uint8_t time = 0;
	static uint32_t kk = 0;
	//	int32_t tmp1;
	if (startflag)
	{
		//		targetp+=1000;
		time++;
		if (time > 0)
		{
			if ((rbuff[0].CMD & 0x00FF) == 0x00BF && (rbuff[0].Parm & 0x00FF) == 0x0008) //&&(rbuff[1].Parm&0xFF00)==0x0600
			{
				for (kk = 0; kk < 100; kk++)
				{
					//					tmp1=rbuff[1].Data1+80;
					if (dir == 1) //�����������λ�ü�
					{
						if (distance < 10) //���мӼ�������
							asc += 2;
						else if (distance > 40)
							asc -= 2;
						if (asc > 2000)
							asc = 2000;
						else if (asc <= 200)
							asc = 200;
						targetp += asc;
					}
					else //����淽����λ�ü�
					{
						if (distance > 40)
							asc += 2;
						else if (distance < 10)
							asc -= 2;
						if (asc > 2000)
							asc = 2000;
						else if (asc <= 200)
							asc = 200;
						targetp -= asc;
					}

					if (abs(targetp) > 10)
					{
						CMD15_CSP(1, targetp);
						SPI_exchange_polling();
						//						printf("current_Pos1= =%x\n\r", tmp1);
					}
					else
					{
						CMD00_ALL_GET_STATUS();
						SPI_exchange_polling();
						//						printf("current_Pos0= =%x\n\r", rbuff[1].Data1);
					}

					if ((rbuff[0].Data2 & 0xFF) < 20)
					{
						PowerDelay(20000);
					}
				}

				if (dir == 1)
					distance++;
				else
					distance--;
				if (distance >= 50 && dir == 1)
				{
					dir = 0;
					asc = 200;
					printf("current_Pos1(Real)= =%x\n\r", rbuff[1].Data1);
					printf("current_Pos1(targetp)= =%x\n\r", targetp);
				}
				else if (distance <= 0 && dir == 0)
				{
					dir = 1;
					asc = 200;
					printf("current_Pos2(Real)= =%x\n\r", rbuff[1].Data1);
					printf("current_Pos2(targetp)= =%x\n\r", targetp);
				}
			}
			else
			{
				CMD00_ALL_GET_STATUS();
				SPI_exchange_polling();
				//				printf("current_Pos0= =%x\n\r", rbuff[1].Data1);
			}
			time = 0;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//	 u8 gifr,sier,sifr;
	//	 int txlen,rxlen,len,num,i;
	u8 gifr;
	// int rxlen;
	//  printf("IN EXTI1_IRQ");
	//	printf("\n\r\n\r");
	switch (GPIO_Pin)
	{
	case GPIO_PIN_0:
		gifr = Wk2xxxReadReg(WK2XXX_PORT1, WK2XXX_GIFR); /**/
		if (gifr & WK2XXX_UT1INT)						 //�ж��Ӵ���1�Ƿ����ж�
		{
			/*���ݽ���*/
			// rxlen = wk_RxChars(WK2XXX_PORT1, WkPort1_RcvBuffer); //һ�ν��յ����ݲ��ᳬ��256Byte
			wk_RxChars(WK2XXX_PORT1, WkPort1_RcvBuffer); //һ�ν��յ����ݲ��ᳬ��256Byte
		}

		gifr = Wk2xxxReadReg(WK2XXX_PORT2, WK2XXX_GIFR); /**/
		if (gifr & WK2XXX_UT2INT)						 //�ж��Ӵ���1�Ƿ����ж�
		{
			/*���ݽ���*/
			wk_RxChars(WK2XXX_PORT2, WkPort2_RcvBuffer); //һ�ν��յ����ݲ��ᳬ��256Byte
		}

		gifr = Wk2xxxReadReg(WK2XXX_PORT3, WK2XXX_GIFR); /**/
		if (gifr & WK2XXX_UT3INT)						 //�ж��Ӵ���1�Ƿ����ж�
		{
			/*���ݽ���*/
			wk_RxChars(WK2XXX_PORT3, WkPort3_RcvBuffer); //һ�ν��յ����ݲ��ᳬ��256Byte
		}

		gifr = Wk2xxxReadReg(WK2XXX_PORT4, WK2XXX_GIFR); /**/
		if (gifr & WK2XXX_UT4INT)						 //�ж��Ӵ���1�Ƿ����ж�
		{
			/*���ݽ���*/
			wk_RxChars(WK2XXX_PORT4, WkPort4_RcvBuffer); //һ�ν��յ����ݲ��ᳬ��256Byte
		}

		break;
	case GPIO_PIN_1:
		break;
	default:
		break;

		//		gifr=WkReadGReg(WK2XXX_GIFR);
		//			delay_ms(1000);
		//		printf("over\n");
		//		printf("\n\r\n\r");
	}
}

// PWM DMA ��ɻص�����
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{
		HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	}
	else if (htim == &htim2)
	{
		HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_2);
	}
	//����
}

//����GPIO
void test_GPIO(void)
{
	if (HAL_GPIO_ReadPin(DI1_GPIO_Port, DI1_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI2_GPIO_Port, DI2_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI3_GPIO_Port, DI3_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI4_GPIO_Port, DI4_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO4_GPIO_Port, DO4_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO4_GPIO_Port, DO4_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI5_GPIO_Port, DI5_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO5_GPIO_Port, DO5_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO5_GPIO_Port, DO5_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI6_GPIO_Port, DI6_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI7_GPIO_Port, DI7_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO7_GPIO_Port, DO7_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO7_GPIO_Port, DO7_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI8_GPIO_Port, DI8_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO8_GPIO_Port, DO8_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO8_GPIO_Port, DO8_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI9_GPIO_Port, DI9_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO9_GPIO_Port, DO9_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO9_GPIO_Port, DO9_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI10_GPIO_Port, DI10_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO10_GPIO_Port, DO10_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO10_GPIO_Port, DO10_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI11_GPIO_Port, DI11_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO11_GPIO_Port, DO11_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO11_GPIO_Port, DO11_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI12_GPIO_Port, DI12_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO12_GPIO_Port, DO12_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO12_GPIO_Port, DO12_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI13_GPIO_Port, DI13_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO13_GPIO_Port, DO13_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO13_GPIO_Port, DO13_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI14_GPIO_Port, DI14_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO14_GPIO_Port, DO14_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO14_GPIO_Port, DO14_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI15_GPIO_Port, DI15_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO15_GPIO_Port, DO15_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO15_GPIO_Port, DO15_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI16_GPIO_Port, DI16_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO16_GPIO_Port, DO16_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO16_GPIO_Port, DO16_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI17_GPIO_Port, DI17_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO17_GPIO_Port, DO17_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO17_GPIO_Port, DO17_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI18_GPIO_Port, DI18_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO18_GPIO_Port, DO18_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO18_GPIO_Port, DO18_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI19_GPIO_Port, DI19_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO19_GPIO_Port, DO19_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO19_GPIO_Port, DO19_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI20_GPIO_Port, DI20_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO20_GPIO_Port, DO20_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO20_GPIO_Port, DO20_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI21_GPIO_Port, DI21_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO21_GPIO_Port, DO21_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO21_GPIO_Port, DO21_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI22_GPIO_Port, DI22_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(DO22_GPIO_Port, DO22_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(DO22_GPIO_Port, DO22_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI23_GPIO_Port, DI23_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(LED_H63_GPIO_Port, LED_H63_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_H63_GPIO_Port, LED_H63_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI24_GPIO_Port, DI24_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(LED_H64_GPIO_Port, LED_H64_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_H64_GPIO_Port, LED_H64_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI25_GPIO_Port, DI25_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(LED_H65_GPIO_Port, LED_H65_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_H65_GPIO_Port, LED_H65_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI26_GPIO_Port, DI26_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(LED_H66_GPIO_Port, LED_H66_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_H66_GPIO_Port, LED_H66_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI27_GPIO_Port, DI27_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(LED_H67_GPIO_Port, LED_H67_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_H67_GPIO_Port, LED_H67_Pin, GPIO_PIN_SET);

	if (HAL_GPIO_ReadPin(DI28_GPIO_Port, DI28_Pin) == GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(LED_H68_GPIO_Port, LED_H68_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_H69_GPIO_Port, LED_H69_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_H70_GPIO_Port, LED_H70_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_H68_GPIO_Port, LED_H68_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_H69_GPIO_Port, LED_H69_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_H70_GPIO_Port, LED_H70_Pin, GPIO_PIN_SET);
	}
}

//�����жϻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		if (Rcv1Counter < RCV1_MAX - 2)
		{
			Rcv1Buffer[Rcv1Counter++] = Tmp_Rxd1Buffer;
		}
		//�ٴο�������
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&Tmp_Rxd1Buffer, 1);
	}
	else if (huart->Instance == USART2)
	{
		if (Rcv2Counter < RCV2_MAX - 2)
		{
			Rcv2Buffer[Rcv2Counter++] = Tmp_Rxd2Buffer;
		}
		//�ٴο�������
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&Tmp_Rxd2Buffer, 1);
	}
	else if (huart->Instance == USART3)
	{
		if (Rcv3Counter < RCV3_MAX - 2)
		{
			Rcv3Buffer[Rcv3Counter++] = Tmp_Rxd3Buffer;
		}
		//�ٴο�������
		HAL_UART_Receive_IT(&huart3, (uint8_t *)&Tmp_Rxd3Buffer, 1);
	}
	else if (huart->Instance == UART4)
	{
		if (Rcv4Counter < RCV4_MAX - 2)
		{
			Rcv4Buffer[Rcv4Counter++] = Tmp_Rxd4Buffer;
		}
		//�ٴο�������
		HAL_UART_Receive_IT(&huart4, (uint8_t *)&Tmp_Rxd4Buffer, 1);
	}
	else if (huart->Instance == UART5)
	{
		if (Rcv5Counter < RCV5_MAX - 2)
		{
			Rcv5Buffer[Rcv5Counter++] = Tmp_Rxd5Buffer;
		}
		//�ٴο�������
		HAL_UART_Receive_IT(&huart5, (uint8_t *)&Tmp_Rxd5Buffer, 1);
	}
	else if (huart->Instance == USART6)
	{
		if (Rcv6Counter < RCV6_MAX - 2)
		{
			Rcv6Buffer[Rcv6Counter++] = Tmp_Rxd6Buffer;
		}
		//�ٴο�������
		HAL_UART_Receive_IT(&huart6, (uint8_t *)&Tmp_Rxd6Buffer, 1);
	}
}

//�����жϻص�����
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//	if(Txd1Counter >= Txd1Max)
	//    {
	//		Txd1Counter=0;
	//		Txd1Max=0;
	//    }
	//	else
	//	{
	//		/* Write one byte to the transmit data register */
	//		while(huart1.gState != HAL_UART_STATE_READY);
	//		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&Txd1Buffer[Txd1Counter++], 1);
	//	}
}

//����EEPROM-W25Q16
void Test_EEPROM(void)
{
	//wk2xxx��ض���
	uint8_t gena;
	u8 datatemp[SIZE];
	u32 FLASH_SIZE;
	u8 i;
	u16 j;
	uint8_t read_buf[10] = {0};
	uint8_t write_buf[10] = {0};
	uint32_t device_id;

	W25qxx_Init();

	while ((W25qxx_ReadID() & 0x0000FFFF) != _W25Q64) //��ⲻ��W25Q16
	{
		printf("W25Q16 Check Failed!");
		delay_ms(500);
		printf("Please Check!      ");
		delay_ms(500);
		HAL_GPIO_TogglePin(LED_H63_GPIO_Port, LED_H63_Pin); //DS0��˸
	}

	printf("W25Q16 Ready!");
	FLASH_SIZE = 64 * 1024 * 1024;
	printf("Start Write W25Q16....");
	SPI_FLASH_BufferWrite((u8 *)TEXT_Buffer, FLASH_SIZE - 100, SIZE); //�ӵ�����100����ַ����ʼ,д��SIZE���ȵ�����
	printf("W25Q16 Write Finished!");								  //��ʾ�������

	printf("Start Read W25Q16.... ");
	W25qxx_ReadBytes(datatemp, FLASH_SIZE - 100, SIZE); //�ӵ�����100����ַ����ʼ,����SIZE���ֽ�
	printf("The Data Readed Is:   ");					//��ʾ�������
	for (i = 0; i < sizeof(datatemp); i++)
		printf(" %d ", datatemp[i]); //��ʾ�������ַ���

	//����һ�β��Զ�дW25Q26�ĳ���
	printf("W25Q64 SPI Flash Test By Mculover666\r\n");
	device_id = W25qxx_ReadID();
	printf("W25Q64 Device ID is 0x%04x\r\n", device_id);

	/* Ϊ����֤�����ȶ�ȡҪд���ַ�������� */
	printf("-------- read data before write -----------\r\n");
	W25qxx_ReadBytes(read_buf, 0, 10);

	for (i = 0; i < 10; i++)
	{
		printf("[0x%08x]:0x%02x\r\n", i, *(read_buf + i));
	}

	/* ���������� */
	printf("-------- erase sector 0 -----------\r\n");
	W25qxx_EraseSector(0);

	/* �ٴζ����� */
	printf("-------- read data after erase -----------\r\n");
	W25qxx_ReadBytes(read_buf, 0, 10);
	for (i = 0; i < 10; i++)
	{
		printf("[0x%08x]:0x%02x\r\n", i, *(read_buf + i));
	}

	/* д���� */
	printf("-------- write data -----------\r\n");
	for (i = 0; i < 10; i++)
	{
		write_buf[i] = i;
	}
	W25qxx_WriteBytes_Page(write_buf, 0, 10);

	/* �ٴζ����� */
	printf("-------- read data after write -----------\r\n");
	W25qxx_ReadBytes(read_buf, 0, 10);
	for (i = 0; i < 10; i++)
	{
		printf("[0x%08x]:0x%02x\r\n", i, *(read_buf + i));
	}

	//    WK2XXX_RST_Init();
	WK2XXX_SPI_Init();
	//	WK2XXX_Reset_Init();
	Wk2xxxInit(WK2XXX_PORT1);
	Wk2xxxInit(WK2XXX_PORT2);
	Wk2xxxInit(WK2XXX_PORT3);
	Wk2xxxInit(WK2XXX_PORT4);
	Wk2xxxSetBaud(WK2XXX_PORT1, B57600);
	Wk2xxxSetBaud(WK2XXX_PORT2, B57600);
	Wk2xxxSetBaud(WK2XXX_PORT3, B57600);
	Wk2xxxSetBaud(WK2XXX_PORT4, B57600);
	////	���Գ���ʱ��ͨ����GENA�Ĵ�����ֵ���ж����ӿ�ͨ���Ƿ�������δ��ʼ����Ĭ��ֵ������Ϊ0x30������0x3F
	gena = Wk2xxxReadReg(WK2XXX_GPORT, WK2XXX_GENA);
	printf("gena=%x\n", gena);

	hspi3.Instance->SR = (uint16_t)(~SPI_FLAG_RXNE);
	SPI_CS_OFF;

	//��䷢�����壬ռ�ձ�=send_Buf[j]/ARR
	for (j = 0; j < NUM; j++)
	{
		send_Buf[j] = 20 * (j + 1);
	}
}

void Init_EtherCAT(void)
{
	/*ģʽ����*/
	CMD01_SET_STATE(STATE_PRE_OP); //	PRE_OP״̬
	SPI_exchange_polling();
	HAL_Delay(300);
	CMD02_SET_AXIS(0, (uint8_t *)bNetworkMap); //�趨��վ���ͣ����õ�һ����վΪDRIVE��վ
	SPI_exchange_polling();
	HAL_Delay(1000);
	CMD06_DRIVE_MODE(1, CSP_MODE, 1);
	SPI_exchange_polling();
	HAL_Delay(1000);			//yan chi da yi xie
	CMD03_SET_DC(5000, 0xffff); //�趨��������250us ,������250us������
	SPI_exchange_polling();
	HAL_Delay(100);

	CMD01_SET_STATE(STATE_OPERATIONAL); //OP״̬
	SPI_exchange_polling();
	HAL_Delay(1000);
	CMD11_SVON(1);
	SPI_exchange_polling();
	HAL_Delay(1000);
	//	while(abs(rbuff[1].Data1)<=1000)	//(rbuff[0].CMD&0x00FF)!=0x00BF &&(rbuff[0].Parm&0x00FF)!=0x0008 &&
	//	{
	//		CMD00_ALL_GET_STATUS();
	//		SPI_exchange_polling();
	////		HAL_Delay(1000);
	//	}
	targetp = rbuff[1].Data1;
	startflag = 1;
	dir = 1;
	distance = 0;
}

//������չ����
void Test_Ext_Serial(void)
{
	static unsigned char dat1, dat2, dat3, dat4;

	LED_Blue_Toggle;
	//HAL_GPIO_TogglePin(LedBlue_GPIO_Port, LedBlue_Pin);

	HAL_Delay(500);
	//			delay_ms(500);

	if ((Wk2xxxReadReg(WK2XXX_PORT1, WK2XXX_FSR) & WK2XXX_RDAT)) //��ȡFIFO״̬���ж��Ƿ�Ϊ��
	{
		dat1 = Wk2xxxReadReg(WK2XXX_PORT1, WK2XXX_FDAT); //��FIFO����
		Wk2xxxWriteReg(WK2XXX_PORT1, WK2XXX_FDAT, dat1); //����ȡFIFO����ͨ��TX���ͳ�ȥ
	}
	else if ((Wk2xxxReadReg(WK2XXX_PORT2, WK2XXX_FSR) & WK2XXX_RDAT)) //��ȡFIFO״̬���ж��Ƿ�Ϊ��
	{
		dat2 = Wk2xxxReadReg(WK2XXX_PORT2, WK2XXX_FDAT); //��FIFO����
		Wk2xxxWriteReg(WK2XXX_PORT2, WK2XXX_FDAT, dat2); //����ȡFIFO����ͨ��TX���ͳ�ȥ
	}
	else if ((Wk2xxxReadReg(WK2XXX_PORT3, WK2XXX_FSR) & WK2XXX_RDAT)) //��ȡFIFO״̬���ж��Ƿ�Ϊ��
	{
		dat3 = Wk2xxxReadReg(WK2XXX_PORT3, WK2XXX_FDAT); //��FIFO����
		Wk2xxxWriteReg(WK2XXX_PORT3, WK2XXX_FDAT, dat3); //����ȡFIFO����ͨ��TX���ͳ�ȥ
	}
	else if ((Wk2xxxReadReg(WK2XXX_PORT4, WK2XXX_FSR) & WK2XXX_RDAT)) //��ȡFIFO״̬���ж��Ƿ�Ϊ��
	{
		dat4 = Wk2xxxReadReg(WK2XXX_PORT4, WK2XXX_FDAT); //��FIFO����
		Wk2xxxWriteReg(WK2XXX_PORT4, WK2XXX_FDAT, dat4); //����ȡFIFO����ͨ��TX���ͳ�ȥ
	}

	//    printf("12345\n");
	delay_ms(500);
	//      PBout(12)=1;
	//      HAL_Delay(500);
	//      PBout(12)=0;
	//      HAL_Delay(500);

	//������ʱ�Ƿ�׼ȷ
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2); //ÿ1us*1000=1ms��ת1�Σ���ʾ�������Ƿ�׼ȷ
										   //	delay_us(1000);
	HAL_Delay(1000);

	//����TIM DMA����PWM����
	HAL_GPIO_WritePin(LED_H64_GPIO_Port, LED_H64_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED_H64_GPIO_Port, LED_H64_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)send_Buf, NUM);
}

//�����շ�
void Uart_RT(void)
{
	Com1_RcvProcess(); // ����1���մ���
	Com1_SlaveSend();  // ����1�ӻ�����

	Com2_RcvProcess(); // ����2���մ���
	Com2_SlaveSend();  // ����2�ӻ�����

	Com3_RcvProcess(); // ����3���մ���
	Com3_SlaveSend();  // ����3�ӻ�����

	Com4_RcvProcess(); // ����4���մ���
	Com4_SlaveSend();  // ����4�ӻ�����

	Com5_RcvProcess(); // ����5���մ���
	Com5_SlaveSend();  // ����5�ӻ�����

	Com6_RcvProcess(); // ����6���մ���
	Com6_SlaveSend();  // ����6�ӻ�����
}

//�������庯��
void Send_Pulse(unsigned char MotorID, uint32_t MaxPosition, unsigned char Dir, uint32_t MaxSpeed)
{
	switch (MotorID)
	{
	case 1:
		CalculateSModelLine(fre_MOTOR1, period_MOTOR1, ACCELERATED_SPEED_LENGTH, FRE_MAX, FRE_MIN, 1);
		__HAL_TIM_SET_AUTORELOAD(&htim2_MOTOR1, period_MOTOR1[0]);
		__HAL_TIM_SET_COMPARE(&htim2_MOTOR1, MOTOR1_TIM2_CHANNEL_x, period_MOTOR1[0] >> 1);
		Motor1_status = 1;
		Motor1_num = 0;

		step_to_run_MOTOR1 = MaxPosition;

		HAL_TIM_Base_Start(&htim2_MOTOR1);									   // ʹ�ܶ�ʱ��
		TIM_CCxChannelCmd(MOTOR1_TIM2, MOTOR1_TIM2_CHANNEL_x, TIM_CCx_ENABLE); // ʹ�ܶ�ʱ��ͨ��
		MOTOR1_OUTPUT_ENABLE();

		if (Dir == 0)
			MOTOR1_DIR_FORWARD();
		else
			MOTOR1_DIR_REVERSAL();

		break;
	case 2:
		CalculateSModelLine(fre_MOTOR2, period_MOTOR2, ACCELERATED_SPEED_LENGTH, FRE_MAX, FRE_MIN, 1);
		__HAL_TIM_SET_AUTORELOAD(&htim4_MOTOR2, period_MOTOR2[0]);
		__HAL_TIM_SET_COMPARE(&htim4_MOTOR2, MOTOR2_TIM4_CHANNEL_x, period_MOTOR2[0] >> 1);
		Motor2_status = 1;
		Motor2_num = 0;

		step_to_run_MOTOR2 = MaxPosition;

		HAL_TIM_Base_Start(&htim4_MOTOR2);									   // ʹ�ܶ�ʱ��
		TIM_CCxChannelCmd(MOTOR2_TIM4, MOTOR2_TIM4_CHANNEL_x, TIM_CCx_ENABLE); // ʹ�ܶ�ʱ��ͨ��
		MOTOR2_OUTPUT_ENABLE();
		if (Dir == 0)
			MOTOR2_DIR_FORWARD();
		else
			MOTOR2_DIR_REVERSAL();
		break;

	case 3:
		CalculateSModelLine(fre_MOTOR3, period_MOTOR3, ACCELERATED_SPEED_LENGTH, FRE_MAX, FRE_MIN, 4);
		__HAL_TIM_SET_AUTORELOAD(&htim8_MOTOR3, period_MOTOR3[0]);
		__HAL_TIM_SET_COMPARE(&htim8_MOTOR3, MOTOR3_TIM8_CHANNEL_x, period_MOTOR3[0] >> 1);
		Motor3_status = 1;
		Motor3_num = 0;

		step_to_run_MOTOR3 = MaxPosition;

		HAL_TIM_Base_Start(&htim8_MOTOR3);									   // ʹ�ܶ�ʱ��
		TIM_CCxChannelCmd(MOTOR3_TIM8, MOTOR3_TIM8_CHANNEL_x, TIM_CCx_ENABLE); // ʹ�ܶ�ʱ��ͨ��
		MOTOR3_OUTPUT_ENABLE();
		if (Dir == 0)
			MOTOR3_DIR_FORWARD();
		else
			MOTOR3_DIR_REVERSAL();
		break;

	case 4:
		CalculateSModelLine(fre_MOTOR4, period_MOTOR4, ACCELERATED_SPEED_LENGTH, FRE_MAX, FRE_MIN, 1);
		__HAL_TIM_SET_AUTORELOAD(&htim3_MOTOR4, period_MOTOR4[0]);
		__HAL_TIM_SET_COMPARE(&htim3_MOTOR4, MOTOR4_TIM3_CHANNEL_x, period_MOTOR4[0] >> 1);
		Motor4_status = 1;
		Motor4_num = 0;

		step_to_run_MOTOR4 = MaxPosition;

		HAL_TIM_Base_Start(&htim3_MOTOR4);									   // ʹ�ܶ�ʱ��
		TIM_CCxChannelCmd(MOTOR4_TIM3, MOTOR4_TIM3_CHANNEL_x, TIM_CCx_ENABLE); // ʹ�ܶ�ʱ��ͨ��
		MOTOR4_OUTPUT_ENABLE();
		if (Dir == 0)
			MOTOR4_DIR_FORWARD();
		else
			MOTOR4_DIR_REVERSAL();
		break;
	case 5:
		CalculateSModelLine(fre_MOTOR5, period_MOTOR5, ACCELERATED_SPEED_LENGTH, FRE_MAX, FRE_MIN, 1);
		__HAL_TIM_SET_AUTORELOAD(&htim1_MOTOR5, period_MOTOR5[0]);
		__HAL_TIM_SET_COMPARE(&htim1_MOTOR5, MOTOR5_TIM1_CHANNEL_x, period_MOTOR5[0] >> 1);
		Motor5_status = 1;
		Motor5_num = 0;

		step_to_run_MOTOR5 = MaxPosition;

		HAL_TIM_Base_Start(&htim1_MOTOR5);									   // ʹ�ܶ�ʱ��
		TIM_CCxChannelCmd(MOTOR5_TIM1, MOTOR5_TIM1_CHANNEL_x, TIM_CCx_ENABLE); // ʹ�ܶ�ʱ��ͨ��
		MOTOR5_OUTPUT_ENABLE();
		if (Dir == 0)
			MOTOR5_DIR_FORWARD();
		else
			MOTOR5_DIR_REVERSAL();
		break;

	case 6:
		CalculateSModelLine(fre_MOTOR6, period_MOTOR6, ACCELERATED_SPEED_LENGTH, FRE_MAX, FRE_MIN, 1);
		__HAL_TIM_SET_AUTORELOAD(&htim5_MOTOR6, period_MOTOR6[0]);
		__HAL_TIM_SET_COMPARE(&htim5_MOTOR6, MOTOR6_TIM5_CHANNEL_x, period_MOTOR6[0] >> 1);
		Motor6_status = 1;
		Motor6_num = 0;

		step_to_run_MOTOR6 = MaxPosition;

		HAL_TIM_Base_Start(&htim5_MOTOR6);									   // ʹ�ܶ�ʱ��
		TIM_CCxChannelCmd(MOTOR6_TIM5, MOTOR6_TIM5_CHANNEL_x, TIM_CCx_ENABLE); // ʹ�ܶ�ʱ��ͨ��
		MOTOR6_OUTPUT_ENABLE();
		if (Dir == 0)
			MOTOR6_DIR_FORWARD();
		else
			MOTOR6_DIR_REVERSAL();
	}
}

//�������巢��
void Test_Send_Pulse(void)
{
	if (Pw_SendPWM == 1)
	{
		// Send_Pulse(1, 1000, 0, 3000);
		Motor1_MotionStatus = STOP;
		MOTOR1_AxisMoveRel(10, 10, 10, 80);

		Motor2_MotionStatus = STOP;
		MOTOR2_AxisMoveRel(20, 10, 10, 50);

		Motor3_MotionStatus = STOP;
		MOTOR3_AxisMoveRel(3000, 10, 10, 300);

		Motor4_MotionStatus = STOP;
		MOTOR4_AxisMoveRel(3000, 10, 10, 300);

		Motor5_MotionStatus = STOP;
		MOTOR5_AxisMoveRel(3000, 10, 10, 300);

		Motor6_MotionStatus = STOP;
		MOTOR6_AxisMoveRel(3000, 10, 10, 300);
		Pw_SendPWM = 0;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/