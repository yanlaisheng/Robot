/**
  ******************************************************************************
  * 文件名程: bsp_MOTOR4.c
  * 作    者: QINGDAO SANLI
  * 版    本: V1.0
  * 编写日期: 2021-04-19
  * 功    能: 电机驱动器控制实现
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_MOTOR4.h"
#include "bsp_MOTOR3.h"
#include <math.h>
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/

TIM_HandleTypeDef htim3_MOTOR4;
speedRampData MOTOR4_srd = {STOP, CW, 0, 0, 0, 0, 0}; // 加减速曲线变量
__IO int32_t MOTOR4_step_position = 0;                // 当前位置
__IO uint8_t MOTOR4_MotionStatus = 0;                 //是否在运动？0：停止，1：运动

__IO uint8_t Motor4_status = 1;
__IO int Motor4_num = 0;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
//#define S_ACCEL 1
//#define T_ACCEL 0
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 驱动器相关GPIO初始化配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void MOTOR4_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* 引脚端口时钟使能 */
  MOTOR4_TIM3_GPIO_CLK_ENABLE();
  MOTOR4_DIR_GPIO_CLK_ENABLE();
  MOTOR4_ENA_GPIO_CLK_ENABLE();

  /* 驱动器脉冲控制引脚IO初始化 */
  GPIO_InitStruct.Pin = MOTOR4_TIM3_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; //GPIO_MODE_AF_PP
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; // GPIO引脚用做TIM复用功能
  HAL_GPIO_Init(MOTOR4_TIM3_PUL_PORT, &GPIO_InitStruct);

  /* 驱动器方向控制引脚IO初始化 */
  GPIO_InitStruct.Pin = MOTOR4_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO引脚用做系统默认功能
  HAL_GPIO_Init(MOTOR4_DIR_PORT, &GPIO_InitStruct);

  /* 驱动器脱机使能控制引脚IO初始化 */
  GPIO_InitStruct.Pin = MOTOR4_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO引脚用做系统默认功能
  HAL_GPIO_Init(MOTOR4_ENA_PORT, &GPIO_InitStruct);

  MOTOR4_DIR_FORWARD();
  MOTOR4_OUTPUT_DISABLE();
}

/**
  * 函数功能: 驱动器定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void MOTOR4_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig; // 定时器时钟
  TIM_OC_InitTypeDef sConfigOC;              // 定时器通道比较输出

  MOTOR4_TIM3_RCC_CLK_ENABLE();

  /* STEPMOTOR相关GPIO初始化配置 */
  MOTOR4_GPIO_Init();
  //
  /* 定时器基本环境配置 */
  htim3_MOTOR4.Instance = MOTOR4_TIM3;                      // 定时器编号
  htim3_MOTOR4.Init.Prescaler = MOTOR4_TIM_PRESCALER;       // 定时器预分频器
  htim3_MOTOR4.Init.CounterMode = TIM_COUNTERMODE_UP;       // 计数方向：向上计数
  htim3_MOTOR4.Init.Period = MOTOR4_TIM_PERIOD;             // 定时器周期
  htim3_MOTOR4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频
  HAL_TIM_Base_Init(&htim3_MOTOR4);

  /* 定时器时钟源配置 */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; // 使用内部时钟源
  HAL_TIM_ConfigClockSource(&htim3_MOTOR4, &sClockSourceConfig);

  /* 定时器比较输出配置 */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;            // 比较输出模式：反转输出
  sConfigOC.Pulse = 0xFFFF;                        // 脉冲数
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;       // 输出极性
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;     // 互补通道输出极性
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       // 快速模式
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;   // 空闲电平
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // 互补通道空闲电平
  HAL_TIM_OC_ConfigChannel(&htim3_MOTOR4, &sConfigOC, MOTOR4_TIM3_CHANNEL_x);
  /* 使能比较输出通道 */
  TIM_CCxChannelCmd(MOTOR4_TIM3, MOTOR4_TIM3_CHANNEL_x, TIM_CCx_DISABLE);

  /* 配置定时器中断优先级并使能 */
  HAL_NVIC_SetPriority(MOTOR4_TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTOR4_TIM3_IRQn);

  __HAL_TIM_CLEAR_FLAG(&htim3_MOTOR4, MOTOR4_TIM3_FLAG_CCx);
  /* 使能定时器比较输出 */
  __HAL_TIM_ENABLE_IT(&htim3_MOTOR4, MOTOR4_TIM3_IT_CCx);
  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&htim3_MOTOR4);
  //  HAL_TIM_Base_Start(&htim3_MOTOR4);// 使能定时器
}

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
