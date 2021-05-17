#ifndef __MOTOR3_TIM_H__
#define __MOTOR3_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
  __IO uint8_t run_state;    // 电机旋转状态
  __IO uint8_t dir;          // 电机旋转方向
  __IO int32_t step_delay;   // 下个脉冲周期（时间间隔），启动时为加速度
  __IO uint32_t decel_start; // 启动减速位置
  __IO int32_t decel_val;    // 减速阶段步数
  __IO int32_t min_delay;    // 最小脉冲周期(最大速度，即匀速段速度)
  __IO int32_t accel_count;  // 加减速阶段计数值
} speedRampData;

/* 宏定义 --------------------------------------------------------------------*/
#define MOTOR3_TIM8 TIM8
#define MOTOR3_TIM8_RCC_CLK_ENABLE() __HAL_RCC_TIM8_CLK_ENABLE()
#define MOTOR3_TIM8_RCC_CLK_DISABLE() __HAL_RCC_TIM8_CLK_DISABLE()
#define MOTOR3_TIM8_IT_CCx TIM_IT_CC1
#define MOTOR3_TIM8_FLAG_CCx TIM_FLAG_CC1
#define MOTOR3_TIM8_IRQn TIM8_UP_TIM13_IRQn
#define MOTOR3_TIM8_IRQHandler TIM8_CC_IRQHandler

#define MOTOR3_TIM8_CHANNEL_x TIM_CHANNEL_1
#define MOTOR3_TIM8_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE() // 输出控制脉冲给电机驱动器
#define MOTOR3_TIM8_PUL_PORT GPIOC                                 // 对应驱动器的PUL-（驱动器使用共阳接法）
#define MOTOR3_TIM8_PUL_PIN GPIO_PIN_6                             // 而PLU+直接接开发板的VCC

#define MOTOR3_DIR_GPIO_CLK_ENABLE() __HAL_RCC_GPIOG_CLK_ENABLE() // 电机旋转方向控制，如果悬空不接默认正转
#define MOTOR3_DIR_PORT GPIOG                                     // 对应驱动器的DIR-（驱动器使用共阳接法）
#define MOTOR3_DIR_PIN GPIO_PIN_5                                 // 而DIR+直接接开发板的VCC
//#define GPIO_PIN_AF_AS_SYS GPIO_AF0_RTC_50Hz                      // 引脚不作为复用功能使用

#define MOTOR3_ENA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOG_CLK_ENABLE() // 电机脱机使能控制，如果悬空不接默认使能电机
#define MOTOR3_ENA_PORT GPIOG                                     // 对应驱动器的ENA-（驱动器使用共阳接法）
#define MOTOR3_ENA_PIN GPIO_PIN_4                                 // 而ENA+直接接开发板的VCC

#define MOTOR3_DIR_FORWARD() HAL_GPIO_WritePin(MOTOR3_DIR_PORT, MOTOR3_DIR_PIN, GPIO_PIN_RESET)
#define MOTOR3_DIR_REVERSAL() HAL_GPIO_WritePin(MOTOR3_DIR_PORT, MOTOR3_DIR_PIN, GPIO_PIN_SET)

#define MOTOR3_OUTPUT_ENABLE() HAL_GPIO_WritePin(MOTOR3_ENA_PORT, MOTOR3_ENA_PIN, GPIO_PIN_RESET)
#define MOTOR3_OUTPUT_DISABLE() HAL_GPIO_WritePin(MOTOR3_ENA_PORT, MOTOR3_ENA_PIN, GPIO_PIN_SET)

// 定义定时器预分频，定时器实际时钟频率为：168MHz/（STEPMOTOR_TIMx_PRESCALER+1）
#define MOTOR3_TIM_PRESCALER 15 // 步进电机驱动器细分设置为：   64  细分

// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define MOTOR3_TIM_PERIOD 0xFFFF
// 定义高级定时器重复计数寄存器值
#define MOTOR3_TIM_REPETITIONCOUNTER 0

#define FALSE 0
#define TRUE 1
#define CW 0  // 顺时针
#define CCW 1 // 逆时针

#define STOP 0                                                        // 加减速曲线状态：停止
#define ACCEL 1                                                       // 加减速曲线状态：加速阶段
#define DECEL 2                                                       // 加减速曲线状态：减速阶段
#define RUN 3                                                         // 加减速曲线状态：匀速阶段
#define T1_FREQ_MOTOR3 (SystemCoreClock / (MOTOR3_TIM_PRESCALER + 1)) // 频率ft值
#define FSPR 200                                                      //步进电机单圈步数
#define MICRO_STEP 64                                                 // 步进电机驱动器细分数
#define SPR (FSPR * MICRO_STEP)                                       // 旋转一圈需要的脉冲数

// 数学常数
#define ALPHA ((float)(2 * 3.14159 / SPR)) // α= 2*pi/spr
#define A_T_x10 ((float)(10 * ALPHA * T1_FREQ_MOTOR3))
#define T1_FREQ_148 ((float)((T1_FREQ_MOTOR3 * 0.676) / 10)) // 0.676为误差修正值
#define A_SQ ((float)(2 * 100000 * ALPHA))
#define A_x200 ((float)(200 * ALPHA))

/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim8_MOTOR3;
extern __IO uint8_t Motor3_status;
extern __IO int Motor3_num;
/* 函数声明 ------------------------------------------------------------------*/

void MOTOR3_TIMx_Init(void);
void MOTOR3_AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif /* __MOTOR3_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
