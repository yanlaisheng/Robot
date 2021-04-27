#ifndef __MOTOR5_TIM_H__
#define __MOTOR5_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "GlobalConst.h"

/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/
#define MOTOR5_TIM1 TIM1
#define MOTOR5_TIM1_RCC_CLK_ENABLE() __HAL_RCC_TIM1_CLK_ENABLE()
#define MOTOR5_TIM1_RCC_CLK_DISABLE() __HAL_RCC_TIM1_CLK_DISABLE()
#define MOTOR5_TIM1_IT_CCx TIM_IT_CC1
#define MOTOR5_TIM1_FLAG_CCx TIM_FLAG_CC1
#define MOTOR5_TIM1_IRQn TIM1_CC_IRQn
#define MOTOR5_TIM1_IRQHandler TIM1_CC_IRQHandler

#define MOTOR5_TIM1_CHANNEL_x TIM_CHANNEL_1
#define MOTOR5_TIM1_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE() // 输出控制脉冲给电机驱动器
#define MOTOR5_TIM1_PUL_PORT GPIOE                                 // 对应驱动器的PUL-（驱动器使用共阳接法）
#define MOTOR5_TIM1_PUL_PIN GPIO_PIN_9                             // 而PLU+直接接开发板的VCC

#define MOTOR5_DIR_GPIO_CLK_ENABLE() __HAL_RCC_GPIOF_CLK_ENABLE() // 电机旋转方向控制，如果悬空不接默认正转
#define MOTOR5_DIR_PORT GPIOF                                     // 对应驱动器的DIR-（驱动器使用共阳接法）
#define MOTOR5_DIR_PIN GPIO_PIN_15                                // 而DIR+直接接开发板的VCC
// #define GPIO_PIN_AF_AS_SYS GPIO_AF0_RTC_50Hz                      // 引脚不作为复用功能使用

#define MOTOR5_ENA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOG_CLK_ENABLE() // 电机脱机使能控制，如果悬空不接默认使能电机
#define MOTOR5_ENA_PORT GPIOG                                     // 对应驱动器的ENA-（驱动器使用共阳接法）
#define MOTOR5_ENA_PIN GPIO_PIN_0                                 // 而ENA+直接接开发板的VCC

#define MOTOR5_DIR_FORWARD() HAL_GPIO_WritePin(MOTOR5_DIR_PORT, MOTOR5_DIR_PIN, GPIO_PIN_RESET)
#define MOTOR5_DIR_REVERSAL() HAL_GPIO_WritePin(MOTOR5_DIR_PORT, MOTOR5_DIR_PIN, GPIO_PIN_SET)

#define MOTOR5_OUTPUT_ENABLE() HAL_GPIO_WritePin(MOTOR5_ENA_PORT, MOTOR5_ENA_PIN, GPIO_PIN_RESET)
#define MOTOR5_OUTPUT_DISABLE() HAL_GPIO_WritePin(MOTOR5_ENA_PORT, MOTOR5_ENA_PIN, GPIO_PIN_SET)

// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define MOTOR5_TIM_PERIOD 0xFFFF
// 定义高级定时器重复计数寄存器值
#define MOTOR5_TIM_REPETITIONCOUNTER 0

#define FALSE 0
#define TRUE 1
#define CW 0  // 顺时针
#define CCW 1 // 逆时针

// 数学常数
#define ALPHA ((float)(2 * 3.14159 / SPR)) // α= 2*pi/spr
#define A_T_x10_MOTOR5 ((float)(10 * ALPHA * T1_FREQ_MOTOR5))
#define T1_FREQ_148_MOTOR5 ((float)((T1_FREQ_MOTOR5 * 0.676) / 10)) // 0.676为误差修正值
#define A_SQ ((float)(2 * 100000 * ALPHA))
#define A_x200 ((float)(200 * ALPHA))

/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim1_MOTOR5;
extern __IO uint8_t Motor5_status;
extern __IO int Motor5_num;
extern __IO uint8_t Motor5_MotionStatus;

extern uint32_t step_to_run_MOTOR5;
extern unsigned short period_MOTOR5[ACCELERATED_SPEED_LENGTH]; //数组储存加速过程中每一步定时器的自动装载值
/* 函数声明 ------------------------------------------------------------------*/

void MOTOR5_TIMx_Init(void);
void MOTOR5_AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

extern void CalculateSModelLine(unsigned short period[], float len, float fre_max, float fre_min, float flexible);
extern void MOTOR5_AxisMoveRel_S(int32_t step, uint32_t speed, uint16_t Dir, uint16_t Acc_len);

#endif /* __MOTOR5_TIM_H__ */
/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
