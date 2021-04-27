#ifndef __MOTOR6_TIM_H__
#define __MOTOR6_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "GlobalConst.h"

/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/
#define MOTOR6_TIM5 TIM5
#define MOTOR6_TIM5_RCC_CLK_ENABLE() __HAL_RCC_TIM5_CLK_ENABLE()
#define MOTOR6_TIM5_RCC_CLK_DISABLE() __HAL_RCC_TIM5_CLK_DISABLE()
#define MOTOR6_TIM5_IT_CCx TIM_IT_CC1
#define MOTOR6_TIM5_FLAG_CCx TIM_FLAG_CC1
#define MOTOR6_TIM5_IRQn TIM5_IRQn
#define MOTOR6_TIM5_IRQHandler TIM5_IRQHandler

#define MOTOR6_TIM5_CHANNEL_x TIM_CHANNEL_1
#define MOTOR6_TIM5_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() // 输出控制脉冲给电机驱动器
#define MOTOR6_TIM5_PUL_PORT GPIOA                                 // 对应驱动器的PUL-（驱动器使用共阳接法）
#define MOTOR6_TIM5_PUL_PIN GPIO_PIN_0                             // 而PLU+直接接开发板的VCC

#define MOTOR6_DIR_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE() // 电机旋转方向控制，如果悬空不接默认正转
#define MOTOR6_DIR_PORT GPIOD                                     // 对应驱动器的DIR-（驱动器使用共阳接法）
#define MOTOR6_DIR_PIN GPIO_PIN_14                                // 而DIR+直接接开发板的VCC
// #define GPIO_PIN_AF_AS_SYS GPIO_AF0_RTC_50Hz                      // 引脚不作为复用功能使用

#define MOTOR6_ENA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE() // 电机脱机使能控制，如果悬空不接默认使能电机
#define MOTOR6_ENA_PORT GPIOD                                     // 对应驱动器的ENA-（驱动器使用共阳接法）
#define MOTOR6_ENA_PIN GPIO_PIN_15                                // 而ENA+直接接开发板的VCC

#define MOTOR6_DIR_FORWARD() HAL_GPIO_WritePin(MOTOR6_DIR_PORT, MOTOR6_DIR_PIN, GPIO_PIN_RESET)
#define MOTOR6_DIR_REVERSAL() HAL_GPIO_WritePin(MOTOR6_DIR_PORT, MOTOR6_DIR_PIN, GPIO_PIN_SET)

#define MOTOR6_OUTPUT_ENABLE() HAL_GPIO_WritePin(MOTOR6_ENA_PORT, MOTOR6_ENA_PIN, GPIO_PIN_RESET)
#define MOTOR6_OUTPUT_DISABLE() HAL_GPIO_WritePin(MOTOR6_ENA_PORT, MOTOR6_ENA_PIN, GPIO_PIN_SET)

// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define MOTOR6_TIM_PERIOD 0xFFFF
// 定义高级定时器重复计数寄存器值
#define MOTOR6_TIM_REPETITIONCOUNTER 0

#define FALSE 0
#define TRUE 1
#define CW 0  // 顺时针
#define CCW 1 // 逆时针

// 数学常数
#define ALPHA ((float)(2 * 3.14159 / SPR)) // α= 2*pi/spr
#define A_T_x10_MOTOR6 ((float)(10 * ALPHA * T1_FREQ_MOTOR6))
#define T1_FREQ_148_MOTOR6 ((float)((T1_FREQ_MOTOR6 * 0.676) / 10)) // 0.676为误差修正值
#define A_SQ ((float)(2 * 100000 * ALPHA))
#define A_x200 ((float)(200 * ALPHA))

/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim5_MOTOR6;
extern __IO uint8_t Motor6_status;
extern __IO int Motor6_num;
extern __IO uint8_t Motor6_MotionStatus;

extern uint32_t step_to_run_MOTOR6;
extern unsigned short period_MOTOR6[ACCELERATED_SPEED_LENGTH]; //数组储存加速过程中每一步定时器的自动装载值
/* 函数声明 ------------------------------------------------------------------*/

void MOTOR6_TIMx_Init(void);
void MOTOR6_AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

extern void CalculateSModelLine(unsigned short period[], float len, float fre_max, float fre_min, float flexible);
extern void MOTOR6_AxisMoveRel_S(int32_t step, uint32_t speed, uint16_t Dir, uint16_t Acc_len);

#endif /* __MOTOR6_TIM_H__ */
/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
