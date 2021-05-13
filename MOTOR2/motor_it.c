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

#ifdef OUTPUT_DATA
char tmp[64];
float timecnt = 0;
float periodcnt = 0;
#endif

/*see 
http://picprog.strongedge.net/step_prof/step-profile.html
*/

/*���SPTA�㷨������������*/
void TIMX_IRQHandler_SPTA(MOTOR_CONTROL_SPTA *pmotor)
{
	int carry = 0;
#ifdef OUTPUT_DATA
	timecnt++;
	periodcnt++;
#endif
	/*���������ź�*/
	// pmotor->GPIOBASE->BSRR = (uint32_t)pmotor->PWMGPIO << 16U; //pmotor->PWMGPIO YLS 2021.04.29
	pmotor->GPIOBASE->BSRR = pmotor->PWMGPIO;

	/*�����ٶ��ۼ����Ƿ�����������Ƿ����һ����������*/
	pmotor->step_frac += pmotor->step_speed; //�����ٶ�
	carry = pmotor->step_frac >> 16;
	pmotor->step_frac -= carry << 16;
	//carry=1˵�����
	if (carry != 0)
	{
		//��������һ������
		pmotor->step_count += 1;
		/*���������źŲ���һ����������*/
		//		pmotor->GPIOBASE->BSRR = pmotor->PWMGPIO;
		pmotor->GPIOBASE->BSRR = (uint32_t)pmotor->PWMGPIO << 16U; //pmotor->PWMGPIO YLS 2021.04.29

		//λ�ü���,//�������壬����Ҫλ�ü��㣬���ݷ������λ��
		if (pmotor->clockwise == pmotor->dir)
		{
			pmotor->CurrentPosition_Pulse++;
			if (pmotor->CurrentPosition_Pulse >= pmotor->MaxPosition_Pulse)
			{
				pmotor->CurrentPosition_Pulse = 0;
			}
		}
		else
		{
			pmotor->CurrentPosition_Pulse--;
			if (pmotor->CurrentPosition_Pulse == 0xffffffff)
			{
				pmotor->CurrentPosition_Pulse = pmotor->MaxPosition_Pulse - 1;
			}
		}
		pmotor->CurrentPosition = pmotor->CurrentPosition_Pulse / pmotor->divnum;
#ifdef OUTPUT_DATA

		//ÿ��ʱ�̶�Ӧ�ĸߵ�ƽ��͵�ƽ
		sprintf(tmp, "%f,0\r\n%f,1\r\n", timecnt - 1, timecnt);

		USART1_Printfstr(tmp);
		periodcnt = 0;
#endif
	}

	//�ٶȿ���-�ѹر�
	if (pmotor->speedenbale)
	{
		if ((pmotor->step_speed >= pmotor->step_spmax && pmotor->step_speed - pmotor->step_spmax <= 3) ||
			(pmotor->step_speed <= pmotor->step_spmax && pmotor->step_spmax - pmotor->step_speed <= 3))
		{
			return;
		}
	}
	/*���ݵ����״̬����״̬ת���Լ������任*/
	switch (pmotor->step_state)
	{
	//����
	case ACCELERATING:
		//������ٽ׶εĲ���
		if (carry)
		{
			pmotor->step_acced++;
		}
		//���ٽ׶Σ��ٶ��ۼ���Ҫ�����趨�ļ��ٶ�ֵ�����ۼ�
		pmotor->speed_frac += pmotor->step_accel;
		//����ж�
		carry = pmotor->speed_frac >> 17;
		pmotor->speed_frac -= carry << 17;
		if (carry)
		{
			//����ٶ��ۼ�������������ٶ�����Ҫ���ϸ�ֵ���Ա��ƶ�
			//�����ٶ�����������Ϊ�����ۼ������׼��
			pmotor->step_speed += carry;
		}
		if (!pmotor->speedenbale)
		{
			/*although we are ACCELERATING,but middle point reached,we need DECELERATING*/
			//��������ֵ��Ҫ��תΪ����
			if (pmotor->step_middle != 0)
			{
				if (pmotor->step_count == pmotor->step_middle)
				{
					pmotor->step_state = DECELERATING;
				}
			}
			else if (pmotor->step_count > 0)
			{
				pmotor->step_state = DECELERATING;
			}
		}
		if (pmotor->step_speed >= pmotor->step_spmax)
		{
			//���ٽ׶ε���һ����ʱ�̣��ʹﵽ���趨������ٶ�
			pmotor->step_speed = pmotor->step_spmax;
			//תΪ����ٶ�״̬
			pmotor->step_state = AT_MAX;
		}
		break;
		// �ﵽ����ٶ�
	case AT_MAX:
		if (pmotor->step_move - pmotor->step_count <= pmotor->step_acced)
		{
			pmotor->step_state = DECELERATING;
		}
		break;
		// ����
	case DECELERATING:
		//���ٽ׶�����ٽ׶����෴�Ĺ��̣�ԭ��Ҳ��ͬ
		if (carry && pmotor->step_acced > 0)
		{
			pmotor->step_acced--;
		}
		pmotor->speed_frac += pmotor->step_accel;
		carry = pmotor->speed_frac >> 17;
		pmotor->speed_frac -= carry << 17;
		if (carry && pmotor->step_speed > carry)
		{
			pmotor->step_speed -= carry;
		}
		if (!pmotor->speedenbale)
		{
			//�������趨�����Ժ�ֹͣ����
			if (pmotor->step_count >= pmotor->step_move)
			{
				pmotor->step_state = IDLE;
				pmotor->running = 0;
				pmotor->step_spmax = 0;
				//						TIM_Cmd(pmotor->TIMx, DISABLE);
				htim3_MOTOR4.Instance = pmotor->TIMx;
				HAL_TIM_Base_Stop(&htim3_MOTOR4); // ��ֹʹ�ܶ�ʱ��     YLS 04.22
#ifdef OUTPUT_DATA
				timecnt = 0;
#endif
			}
		}
		break;
	}
}
// void TIM2_IRQHandler(void)
// {
// 	/*����ж�*/
// 	// TIM2->SR = (u16)~TIM_FLAG_Update;
// 	TIMX_IRQHandler_S(&motor1);
// }

/*���S�������㷨������������*/
void TIMX_UP_IRQHandler_S(MOTOR_CONTROL_S *pmotor)
{
	if (1 == pmotor->en)
	{
		//λ�ü���
		if (pmotor->clockwise == pmotor->dir)
		{
			pmotor->CurrentPosition_Pulse++;
			if (pmotor->CurrentPosition_Pulse >= pmotor->MaxPosition_Pulse)
			{
				pmotor->CurrentPosition_Pulse = 0;
			}
		}
		else
		{
			pmotor->CurrentPosition_Pulse--;
			if (pmotor->CurrentPosition_Pulse == 0xffffffff)
			{
				pmotor->CurrentPosition_Pulse = pmotor->MaxPosition_Pulse - 1;
			}
		}
		pmotor->CurrentPosition = pmotor->CurrentPosition_Pulse / pmotor->divnum;

		//�ٶȿ���
		if (pmotor->speedenbale && (pmotor->CurrentIndex == pmotor->TargetIndex || pmotor->TargetIndex + pmotor->CurrentIndex == pmotor->StartTableLength + pmotor->StopTableLength - 1))
		{
			return;
		}
		pmotor->PulsesHaven++; //���������
		pmotor->pulsecount++;  //�Ը�Ƶ����������������������ǰλ�ö��������

		//�ԳƷ�ת
		if (pmotor->RevetDot == pmotor->PulsesHaven)
		{
			pmotor->pulsecount = pmotor->Step_Table[pmotor->CurrentIndex];
		}
		if (pmotor->pulsecount >= pmotor->Step_Table[pmotor->CurrentIndex])
		{
			if (pmotor->PulsesHaven <= pmotor->StartSteps)
			{
				//�𲽽׶�
				if (pmotor->CurrentIndex < pmotor->StartTableLength - 1)
				{
					pmotor->CurrentIndex++;
					pmotor->pulsecount = 0;
					if (pmotor->CurrentIndex >= pmotor->StartTableLength)
						pmotor->CurrentIndex = pmotor->StartTableLength;
				}
			}
			//�����ٶȿ��ƣ��˴������ж�pmotor->PulsesHaven>=(pmotor->PulsesGiven>>1)
			//if(pmotor->PulsesGiven-pmotor->PulsesHaven<=pmotor->StopSteps&&pmotor->PulsesHaven>=(pmotor->PulsesGiven>>1))
			if ((pmotor->PulsesGiven - pmotor->PulsesHaven <= pmotor->StopSteps && pmotor->speedenbale == 1) ||
				(pmotor->PulsesGiven - pmotor->PulsesHaven <= pmotor->StopSteps && pmotor->speedenbale == 0 && pmotor->PulsesHaven >= (pmotor->PulsesGiven >> 1)))
			{
				//ֹͣ�׶�
				if (pmotor->CurrentIndex < pmotor->StartTableLength - 1)
				{
					pmotor->CurrentIndex = pmotor->StartTableLength + pmotor->StopTableLength - pmotor->CurrentIndex;
				}
				pmotor->CurrentIndex++;
				pmotor->pulsecount = 0;
				if (pmotor->CurrentIndex >= pmotor->StartTableLength + pmotor->StopTableLength)
					pmotor->CurrentIndex = pmotor->StartTableLength + pmotor->StopTableLength - 1;
			}
			pmotor->TIMx->ARR = pmotor->Counter_Table[pmotor->CurrentIndex]; //��������
			if (pmotor->TIMx == TIM2)
			{
				pmotor->TIMx->CCR2 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //����ռ�ձ�
				pmotor->TIMx->CNT = 0;
			}
			else if (pmotor->TIMx == TIM4)
			{
				pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //����ռ�ձ�
				pmotor->TIMx->CNT = 0;
			}
			else if (pmotor->TIMx == TIM8)
			{
				pmotor->TIMx->CCR1 = (pmotor->Counter_Table[pmotor->CurrentIndex]) >> 1; //����ռ�ձ�
				pmotor->TIMx->CNT = 0;
			}
		}
		//��תԤ����������ֹͣ��running=0�����Խ�����һ����ת
		if (pmotor->PulsesHaven >= pmotor->PulsesGiven && pmotor->PulsesHaven > 3)
		{
			pmotor->en = 0;
			pmotor->running = 0;
			pmotor->CurrentIndex = 0;
			// TIM_Cmd(pmotor->TIMx, DISABLE); //DISABLE
			if (pmotor->TIMx == TIM2)
			{
				htim2_MOTOR1.Instance = pmotor->TIMx;
				HAL_TIM_Base_Stop(&htim2_MOTOR1);
				TIM2->CR1 &= ~(TIM_CR1_CEN);
				TIM2->CNT = 0;
			}
			else if (pmotor->TIMx == TIM4)
			{
				htim4_MOTOR2.Instance = pmotor->TIMx;
				HAL_TIM_Base_Stop(&htim4_MOTOR2);
				TIM4->CR1 &= ~(TIM_CR1_CEN);
				TIM4->CNT = 0;
			}
			else if (pmotor->TIMx == TIM8)
			{
				htim8_MOTOR3.Instance = pmotor->TIMx;
				HAL_TIM_Base_Stop(&htim8_MOTOR3);
				TIM8->CR1 &= ~(TIM_CR1_CEN);
				TIM8->CNT = 0;
			}
//			USART1_Printfstr("1\r\n");
#ifdef OUTPUT_DATA
			timecnt = 0;
#endif
		}
		else
		{
			pmotor->Time_Cost_Act += pmotor->TIMx->ARR;
		}
	}
#ifdef OUTPUT_DATA

	//ÿ��ʱ�̶�Ӧ��Ƶ��
	//sprintf(tmp,"%f\t%f\r\n",timecnt,1.0/TIM1->CCR1);

	//ÿ��ʱ�̶�Ӧ�ĸߵ�ƽ��͵�ƽ
	sprintf(tmp, "%f,0\r\n%f,1\r\n", timecnt, timecnt + pmotor->TIMx->CCR1);

	USART1_Printfstr(tmp);
	timecnt += pmotor->TIMx->ARR;
#endif
}

//void TIM2_IRQHandler(void)
//{
//	TIM2->SR = (u16)~TIM_FLAG_UPDATE;
//	TIMX_UP_IRQHandler_S(&motor1);
//}

//void TIM3_IRQHandler(void)
//{
//	TIM3->SR = (u16)~TIM_FLAG_Update;
//	TIMX_UP_IRQHandler_S(&motor3);
//}
//
//void EXTI4_IRQHandler(void)
//{
//   if(EXTI_GetITStatus(EXTI_Line4)!= RESET)
//   {
//			EXTI_ClearITPendingBit(EXTI_Line4);
//			if(motor1.rstflg==1)
//			{
//				motor1.speedenbale=0;
//				motor1.PulsesHaven=motor1.PulsesGiven-128;
//				motor1.dir=M1_UNCLOCKWISE;
//				motor1.CurrentPosition_Pulse=128;
//				motor1.rstflg=0;
//			}
//  }
//}