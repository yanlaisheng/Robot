/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x_it.h"
//#include "stm32f10x_it.h"
//#include "STM32F10X.h"
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"
#include "include.h"
#include "typedef.h"
#include "bsp_MOTOR1.h"
#include "bsp_MOTOR2.h"
#include "bsp_MOTOR3.h"
#include "bsp_MOTOR4.h"
#include "bsp_MOTOR5.h"
#include "bsp_MOTOR6.h"

/*see 
http://picprog.strongedge.net/step_prof/step-profile.html
*/

/*电机S型曲线算法公共处理函数*/
void TIMX_UP_IRQHandler_S(MOTOR_CONTROL_S *pmotor)
{
	if (1 == pmotor->en)
	{
		//位置计算
		if (pmotor->clockwise == pmotor->dir)
		{
			pmotor->CurrentPosition_Pulse++;
			if (pmotor->CurrentPosition_Pulse >= pmotor->MaxPosition_Pulse)
			{
				pmotor->CurrentPosition_Pulse = 0;
			}
			pmotor->target_pos++;
			if (pmotor->target_pos >= pmotor->MaxPosition_Pulse)
			{
				pmotor->target_pos = 0;
			}
		}
		else
		{
			pmotor->CurrentPosition_Pulse--;
			if (pmotor->CurrentPosition_Pulse == 0xffffffff)
			{
				pmotor->CurrentPosition_Pulse = pmotor->MaxPosition_Pulse - 1;
			}
			pmotor->target_pos--;
			if (pmotor->target_pos == 0xffffffff)
			{
				pmotor->target_pos = pmotor->MaxPosition_Pulse - 1;
			}
		}
		pmotor->CurrentPosition = pmotor->CurrentPosition_Pulse / pmotor->divnum;

		//速度控制
		if (pmotor->speedenbale && (pmotor->CurrentIndex == pmotor->TargetIndex || pmotor->TargetIndex + pmotor->CurrentIndex == pmotor->StartTableLength + pmotor->StopTableLength - 1))
		{
			return;
		}
		pmotor->PulsesHaven++; //总脉冲个数
		pmotor->pulsecount++;  //以该频率脉冲输出的脉冲个数，当前位置段脉冲个数

		//对称反转
		if (pmotor->RevetDot == pmotor->PulsesHaven)
		{
			pmotor->pulsecount = pmotor->Step_Table[pmotor->CurrentIndex];
		}
		if (pmotor->pulsecount >= pmotor->Step_Table[pmotor->CurrentIndex])
		{
			if (pmotor->PulsesHaven <= pmotor->StartSteps)
			{
				//起步阶段
				if (pmotor->CurrentIndex < pmotor->StartTableLength - 1)
				{
					pmotor->CurrentIndex++;
					pmotor->pulsecount = 0;
					if (pmotor->CurrentIndex >= pmotor->StartTableLength)
						pmotor->CurrentIndex = pmotor->StartTableLength;
				}
			}
			//对于速度控制，此处不能判断pmotor->PulsesHaven>=(pmotor->PulsesGiven>>1)
			//if(pmotor->PulsesGiven-pmotor->PulsesHaven<=pmotor->StopSteps&&pmotor->PulsesHaven>=(pmotor->PulsesGiven>>1))
			if ((pmotor->PulsesGiven - pmotor->PulsesHaven <= pmotor->StopSteps && pmotor->speedenbale == 1) ||
				(pmotor->PulsesGiven - pmotor->PulsesHaven <= pmotor->StopSteps && pmotor->speedenbale == 0 && pmotor->PulsesHaven >= (pmotor->PulsesGiven >> 1)))
			{
				//停止阶段
				if (pmotor->CurrentIndex < pmotor->StartTableLength - 1)
				{
					pmotor->CurrentIndex = pmotor->StartTableLength + pmotor->StopTableLength - pmotor->CurrentIndex;
				}
				pmotor->CurrentIndex++;
				pmotor->pulsecount = 0;
				if (pmotor->CurrentIndex >= pmotor->StartTableLength + pmotor->StopTableLength)
					pmotor->CurrentIndex = pmotor->StartTableLength + pmotor->StopTableLength - 1;
			}
			pmotor->TIMx->ARR = pmotor->Counter_Table[pmotor->CurrentIndex]; //设置周期
			if (pmotor->TIMx == TIM2)
			{
				TIM2->CCR2 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
				TIM2->CNT = 0;
			}
			else if (pmotor->TIMx == TIM4)
			{
				TIM4->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
				TIM4->CNT = 0;
			}
			else if (pmotor->TIMx == TIM8)
			{
				TIM8->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
				TIM8->CNT = 0;
			}
			if (pmotor->TIMx == TIM3)
			{
				TIM3->CCR4 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
				TIM3->CNT = 0;
			}
			else if (pmotor->TIMx == TIM1)
			{
				TIM1->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
				TIM1->CNT = 0;
			}
			else if (pmotor->TIMx == TIM5)
			{
				TIM5->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //设置占空比
				TIM5->CNT = 0;
			}
		}
		//旋转预定脉冲数，停止，running=0，可以进行下一次旋转
		if (pmotor->PulsesHaven >= pmotor->PulsesGiven && pmotor->PulsesHaven > 3)
		{
			pmotor->en = 0;
			pmotor->running = 0;
			pmotor->CurrentIndex = 0;
			// TIM_Cmd(pmotor->TIMx, DISABLE); //DISABLE
			if (pmotor->TIMx == TIM2)
			{
				TIM2->CR1 &= ~(TIM_CR1_CEN);
			}
			else if (pmotor->TIMx == TIM4)
			{
				TIM4->CR1 &= ~(TIM_CR1_CEN);
			}
			else if (pmotor->TIMx == TIM8)
			{
				TIM8->CR1 &= ~(TIM_CR1_CEN);
			}
			if (pmotor->TIMx == TIM3)
			{
				TIM3->CR1 &= ~(TIM_CR1_CEN);
			}
			else if (pmotor->TIMx == TIM1)
			{
				TIM1->CR1 &= ~(TIM_CR1_CEN);
			}
			else if (pmotor->TIMx == TIM5)
			{
				TIM5->CR1 &= ~(TIM_CR1_CEN);
			}
		}
		else
		{
			pmotor->Time_Cost_Act += pmotor->TIMx->ARR;
		}
	}
}
