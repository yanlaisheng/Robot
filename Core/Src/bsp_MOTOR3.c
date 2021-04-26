/**
  ******************************************************************************
  * �ļ�����: bsp_STEPMOTOR.c
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-06-03
  * ��    ��: �����������������ʵ��
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_MOTOR3.h"
#include "GlobalConst.h"
#include <math.h>
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htim8_MOTOR3;
speedRampData Motor3_srd = {STOP, CW, 0, 0, 0, 0, 0}; // �Ӽ������߱���
__IO int32_t Motor3_step_position = 0;                // ��ǰλ��
__IO uint8_t Motor3_MotionStatus = 0;                 //�Ƿ����˶���0��ֹͣ��1���˶�

__IO uint8_t Motor3_status = 1;
__IO int Motor3_num = 0;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ���������GPIO��ʼ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void MOTOR3_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* ���Ŷ˿�ʱ��ʹ�� */
  MOTOR3_TIM8_GPIO_CLK_ENABLE();
  MOTOR3_DIR_GPIO_CLK_ENABLE();
  MOTOR3_ENA_GPIO_CLK_ENABLE();

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR3_TIM8_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8; // GPIO��������TIM���ù���
  HAL_GPIO_Init(MOTOR3_TIM8_PUL_PORT, &GPIO_InitStruct);

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR3_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR3_DIR_PORT, &GPIO_InitStruct);

  /* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR3_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR3_ENA_PORT, &GPIO_InitStruct);

  MOTOR3_DIR_FORWARD();
  MOTOR3_OUTPUT_DISABLE();
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void MOTOR3_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig; // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;              // ��ʱ��ͨ���Ƚ����

  MOTOR3_TIM8_RCC_CLK_ENABLE();

  /* STEPMOTOR���GPIO��ʼ������ */
  MOTOR3_GPIO_Init();
  //
  /* ��ʱ�������������� */
  htim8_MOTOR3.Instance = MOTOR3_TIM8;                      // ��ʱ�����
  htim8_MOTOR3.Init.Prescaler = MOTOR3_TIM_PRESCALER;       // ��ʱ��Ԥ��Ƶ��
  htim8_MOTOR3.Init.CounterMode = TIM_COUNTERMODE_UP;       // �����������ϼ���
  htim8_MOTOR3.Init.Period = MOTOR3_TIM_PERIOD;             // ��ʱ������
  htim8_MOTOR3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
  HAL_TIM_Base_Init(&htim8_MOTOR3);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htim8_MOTOR3, &sClockSourceConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;            // �Ƚ����ģʽ����ת���
  sConfigOC.Pulse = 0xFFFF;                        // ������
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;       // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;     // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;   // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htim8_MOTOR3, &sConfigOC, MOTOR3_TIM8_CHANNEL_x);
  /* ʹ�ܱȽ����ͨ�� */
  TIM_CCxChannelCmd(MOTOR3_TIM8, MOTOR3_TIM8_CHANNEL_x, TIM_CCx_DISABLE);

  /* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(MOTOR3_TIM8_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTOR3_TIM8_IRQn);

  __HAL_TIM_CLEAR_FLAG(&htim8_MOTOR3, MOTOR3_TIM8_FLAG_CCx);
  /* ʹ�ܶ�ʱ���Ƚ���� */
  __HAL_TIM_ENABLE_IT(&htim8_MOTOR3, MOTOR3_TIM8_IT_CCx);
  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&htim8_MOTOR3);
  // HAL_TIM_Base_Start(&htim8_MOTOR3); // ʹ�ܶ�ʱ��
}

/**
  * ��������: ������ʱ��Ӳ����ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
//{
//
//  if(htim_base->Instance==MOTOR3_TIM8)
//  {
//    /* ������ʱ������ʱ��ʹ�� */
//    MOTOR3_TIM8_RCC_CLK_ENABLE();
//  }
//}

/**
  * ��������: ������ʱ��Ӳ������ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
//void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
//{
//
//  if(htim_base->Instance==MOTOR3_TIM8)
//  {
//    /* ������ʱ������ʱ�ӽ��� */
//    MOTOR3_TIM8_RCC_CLK_DISABLE();
//    HAL_GPIO_DeInit(MOTOR3_TIM8_PUL_PORT,MOTOR3_TIM8_PUL_PIN);
//    HAL_GPIO_DeInit(MOTOR3_DIR_PORT,MOTOR3_DIR_PIN);
//    HAL_GPIO_DeInit(MOTOR3_ENA_PORT,MOTOR3_ENA_PIN);
//
//    HAL_NVIC_DisableIRQ(MOTOR3_TIM8_IRQn);
//  }
//}
/**
  * ��������: ���λ���˶����˶������Ĳ���
  * �������: step���ƶ��Ĳ��� (����Ϊ˳ʱ�룬����Ϊ��ʱ��).
              accel  ���ٶ�,ʵ��ֵΪaccel*0.1*rad/sec^2
              decel  ���ٶ�,ʵ��ֵΪdecel*0.1*rad/sec^2
              speed  ����ٶ�,ʵ��ֵΪspeed*0.1*rad/sec
  * �� �� ֵ: ��
  * ˵    ��: �Ը����Ĳ����ƶ�����������ȼ��ٵ�����ٶȣ�Ȼ���ں���λ�ÿ�ʼ
  *           ������ֹͣ��ʹ�������˶�����Ϊָ���Ĳ���������Ӽ��ٽ׶κ̲ܶ���
  *           �ٶȺ������ǻ�û�ﵽ����ٶȾ�Ҫ��ʼ����
  */
void MOTOR3_AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
  __IO uint16_t tim_count;
  // �ﵽ����ٶ�ʱ�Ĳ���
  __IO uint32_t max_s_lim;
  // ����Ҫ��ʼ���ٵĲ������������û�дﵽ����ٶȣ�
  __IO uint32_t accel_lim;

  if (Motor3_MotionStatus != STOP) // ֻ�������������ֹͣ��ʱ��ż���
    return;
  if (step < 0) // ����Ϊ����
  {
    Motor3_srd.dir = CCW; // ��ʱ�뷽����ת
    MOTOR3_DIR_REVERSAL();
    step = -step; // ��ȡ��������ֵ
  }
  else
  {
    Motor3_srd.dir = CW; // ˳ʱ�뷽����ת
    MOTOR3_DIR_FORWARD();
  }

  if (step == 1) // ����Ϊ1
  {
    Motor3_srd.accel_count = -1;  // ֻ�ƶ�һ��
    Motor3_srd.run_state = DECEL; // ����״̬.
    Motor3_srd.step_delay = 1000; // ����ʱ
  }
  else if (step != 0) // ���Ŀ���˶�������Ϊ0
  {
    // ���ǵĵ������ר��ָ���ֲ�����ϸ�ļ��㼰�Ƶ�����

    // ��������ٶȼ���, ����õ�min_delay���ڶ�ʱ���ļ�������ֵ��
    // min_delay = (alpha / tt)/ w
    Motor3_srd.min_delay = (int32_t)(A_T_x10 / speed);

    // ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶȣ�����accel��λΪ0.1rad/sec^2
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
    Motor3_srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel)) / 10);

    // ������ٲ�֮��ﵽ����ٶȵ�����
    //     max_s_lim = speed*speed / (20*ALPHA*accel);
    max_s_lim = (uint32_t)(speed * speed / (A_x200 * accel / 10));

    // ����ﵽ����ٶ�С��0.5�������ǽ���������Ϊ0
    // ��ʵ�����Ǳ����ƶ�����һ�����ܴﵽ��Ҫ���ٶ�
    if (max_s_lim == 0)
    {
      max_s_lim = 1;
    }

    // ������ٲ�֮�����Ǳ��뿪ʼ����
    // n1 = (n1+n2)decel / (accel + decel)
    accel_lim = (uint32_t)(step * decel / (accel + decel));
    // ���Ǳ����������1�����ܲ��ܿ�ʼ����.
    if (accel_lim == 0)
    {
      accel_lim = 1;
    }

    // ʹ�������������ǿ��Լ�������ٽ׶β���
    if (accel_lim <= max_s_lim)
    {
      Motor3_srd.decel_val = accel_lim - step;
    }
    else
    {
      Motor3_srd.decel_val = -(max_s_lim * accel / decel);
    }
    // ��ֻʣ��һ�����Ǳ������
    if (Motor3_srd.decel_val == 0)
    {
      Motor3_srd.decel_val = -1;
    }
    Motor3_srd.min_delay = (int32_t)(A_T_x10 / speed);
    Motor3_srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel)) / 10);

    //     max_s_lim = 0x18de3;//9B6CC4;
    //     accel_lim = 0x82355;
    //     Motor3_srd.decel_val = accel_lim - step;
    // ���㿪ʼ����ʱ�Ĳ���
    Motor3_srd.decel_start = step + Motor3_srd.decel_val;

    // �������ٶȺ��������ǾͲ���Ҫ���м����˶�
    if (Motor3_srd.step_delay <= Motor3_srd.min_delay)
    {
      Motor3_srd.step_delay = Motor3_srd.min_delay;
      Motor3_srd.run_state = RUN;
    }
    else
    {
      Motor3_srd.run_state = ACCEL;
    }
    // ��λ���ٶȼ���ֵ
    Motor3_srd.accel_count = 0;
  }
  Motor3_MotionStatus = 1; // ���Ϊ�˶�״̬
  tim_count = __HAL_TIM_GET_COUNTER(&htim8_MOTOR3);
  __HAL_TIM_SET_COMPARE(&htim8_MOTOR3, MOTOR3_TIM8_CHANNEL_x, tim_count + Motor3_srd.step_delay); // ���ö�ʱ���Ƚ�ֵ
  TIM_CCxChannelCmd(MOTOR3_TIM8, MOTOR3_TIM8_CHANNEL_x, TIM_CCx_ENABLE);                          // ʹ�ܶ�ʱ��ͨ��
  HAL_TIM_Base_Start(&htim8_MOTOR3);                                                              // ʹ�ܶ�ʱ��
  MOTOR3_OUTPUT_ENABLE();
}
#if S_ACCEL
extern uint32_t step_to_run_MOTOR3;
extern float fre_MOTOR3[ACCELERATED_SPEED_LENGTH];             //����洢���ٹ�����ÿһ����Ƶ��
extern unsigned short period_MOTOR3[ACCELERATED_SPEED_LENGTH]; //���鴢����ٹ�����ÿһ����ʱ�����Զ�װ��ֵ

#endif

/**
  * ��������: ��ʱ���жϷ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ּӼ��ٹ���
  */
void MOTOR3_TIM8_IRQHandler(void) //��ʱ���жϴ���
{
  __IO uint16_t tim_count = 0;
  // �����£��£�һ����ʱ����
#if T_ACCEL
  uint16_t new_step_delay = 0;
#endif
  // ���ٹ��������һ����ʱ���������ڣ�.
  __IO static uint16_t last_accel_delay = 0;
  // ���ƶ�����������
  __IO static uint32_t step_count = 0;
  // ��¼new_step_delay�е������������һ������ľ���
  __IO static int32_t rest = 0;
  //��ʱ��ʹ�÷�תģʽ����Ҫ���������жϲ����һ����������
  __IO static uint32_t i = 0;

  if (__HAL_TIM_GET_IT_SOURCE(&htim8_MOTOR3, MOTOR3_TIM8_IT_CCx) != RESET)
  {
    // �����ʱ���ж�
    __HAL_TIM_CLEAR_IT(&htim8_MOTOR3, MOTOR3_TIM8_IT_CCx);
    i++;
#if S_ACCEL
    if (i % 2 == 0) //ÿ���������жϲ���һ����������
    {
      switch (Motor3_status)
      {
      case ACCEL: //����
        __HAL_TIM_SET_AUTORELOAD(&htim8_MOTOR3, period_MOTOR3[Motor3_num]);
        __HAL_TIM_SET_COMPARE(&htim8_MOTOR3, MOTOR3_TIM8_CHANNEL_x, period_MOTOR3[Motor3_num] / 2);
        Motor3_num++;
        if (Motor3_num >= ACCELERATED_SPEED_LENGTH)
        {
          Motor3_status = 3;
        }
        break;
      case RUN: //����
        step_to_run_MOTOR3--;
        if (step_to_run_MOTOR3 < 1)
          Motor3_status = 2;
        break;
      case DECEL: //����
        Motor3_num--;
        __HAL_TIM_SET_AUTORELOAD(&htim8_MOTOR3, period_MOTOR3[Motor3_num]);
        __HAL_TIM_SET_COMPARE(&htim8_MOTOR3, MOTOR3_TIM8_CHANNEL_x, period_MOTOR3[Motor3_num] / 2);
        if (Motor3_num < 1)
          Motor3_status = 0;
        break;
      case STOP: //ֹͣ
                 // �ر�ͨ��
        TIM_CCxChannelCmd(MOTOR3_TIM8, MOTOR3_TIM8_CHANNEL_x, TIM_CCx_DISABLE);
        __HAL_TIM_CLEAR_FLAG(&htim8_MOTOR3, MOTOR3_TIM8_FLAG_CCx);
        MOTOR3_OUTPUT_DISABLE();
        break;
      }
    }
#endif

#if T_ACCEL
    // ���ñȽ�ֵ
    tim_count = __HAL_TIM_GET_COUNTER(&htim8_MOTOR3);
    __HAL_TIM_SET_COMPARE(&htim8_MOTOR3, MOTOR3_TIM8_CHANNEL_x, tim_count + Motor3_srd.step_delay);

    i++;        // ��ʱ���жϴ�������ֵ
    if (i == 2) // 2�Σ�˵���Ѿ����һ����������
    {
      i = 0;                        // ���㶨ʱ���жϴ�������ֵ
      switch (Motor3_srd.run_state) // �Ӽ������߽׶�
      {
      case STOP:
        step_count = 0; // ���㲽��������
        rest = 0;       // ������ֵ
        // �ر�ͨ��
        TIM_CCxChannelCmd(MOTOR3_TIM8, MOTOR3_TIM8_CHANNEL_x, TIM_CCx_DISABLE);
        __HAL_TIM_CLEAR_FLAG(&htim8_MOTOR3, MOTOR3_TIM8_FLAG_CCx);
        MOTOR3_OUTPUT_DISABLE();
        Motor3_MotionStatus = 0; //  ���Ϊֹͣ״̬
        break;

      case ACCEL:
        step_count++; // ������1
        if (Motor3_srd.dir == CW)
        {
          Motor3_step_position++; // ����λ�ü�1
        }
        else
        {
          Motor3_step_position--; // ����λ�ü�1
        }
        Motor3_srd.accel_count++;                                                                                           // ���ټ���ֵ��1
        new_step_delay = Motor3_srd.step_delay - (((2 * Motor3_srd.step_delay) + rest) / (4 * Motor3_srd.accel_count + 1)); //������(��)һ����������(ʱ����)
        rest = ((2 * Motor3_srd.step_delay) + rest) % (4 * Motor3_srd.accel_count + 1);                                     // �����������´μ��㲹���������������
        if (step_count >= Motor3_srd.decel_start)                                                                           // ����ǹ�Ӧ�ÿ�ʼ����
        {
          Motor3_srd.accel_count = Motor3_srd.decel_val; // ���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
          Motor3_srd.run_state = DECEL;                  // �¸����������ٽ׶�
        }
        else if (new_step_delay <= Motor3_srd.min_delay) // ����Ƿ񵽴�����������ٶ�
        {
          last_accel_delay = new_step_delay;     // ������ٹ��������һ����ʱ���������ڣ�
          new_step_delay = Motor3_srd.min_delay; // ʹ��min_delay����Ӧ����ٶ�speed��
          rest = 0;                              // ������ֵ
          Motor3_srd.run_state = RUN;            // ����Ϊ��������״̬
        }
        break;

      case RUN:
        step_count++; // ������1
        if (Motor3_srd.dir == CW)
        {
          Motor3_step_position++; // ����λ�ü�1
        }
        else
        {
          Motor3_step_position--; // ����λ�ü�1
        }
        new_step_delay = Motor3_srd.min_delay;    // ʹ��min_delay����Ӧ����ٶ�speed��
        if (step_count >= Motor3_srd.decel_start) // ��Ҫ��ʼ����
        {
          Motor3_srd.accel_count = Motor3_srd.decel_val; // ���ٲ�����Ϊ���ټ���ֵ
          new_step_delay = last_accel_delay;             // �ӽ׶�������ʱ��Ϊ���ٽ׶ε���ʼ��ʱ(��������)
          Motor3_srd.run_state = DECEL;                  // ״̬�ı�Ϊ����
        }
        break;

      case DECEL:
        step_count++; // ������1
        if (Motor3_srd.dir == CW)
        {
          Motor3_step_position++; // ����λ�ü�1
        }
        else
        {
          Motor3_step_position--; // ����λ�ü�1
        }
        Motor3_srd.accel_count++;
        new_step_delay = Motor3_srd.step_delay - (((2 * Motor3_srd.step_delay) + rest) / (4 * Motor3_srd.accel_count + 1)); //������(��)һ����������(ʱ����)
        rest = ((2 * Motor3_srd.step_delay) + rest) % (4 * Motor3_srd.accel_count + 1);                                     // �����������´μ��㲹���������������

        //����Ƿ�Ϊ���һ��
        if (Motor3_srd.accel_count >= 0)
        {
          Motor3_srd.run_state = STOP;
        }
        break;
      }
      Motor3_srd.step_delay = new_step_delay; // Ϊ�¸�(�µ�)��ʱ(��������)��ֵ
    }
#endif
  }
}
/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/