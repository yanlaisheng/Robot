/**
  ******************************************************************************
  * �ļ�����: bsp_MOTOR6.c
  * ��    ��: QINGDAO SANLI
  * ��    ��: V1.0
  * ��д����: 2021-04-19
  * ��    ��: �������������ʵ��
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_MOTOR6.h"
#include "bsp_MOTOR3.h"
#include "GlobalConst.h"
#include <math.h>
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/

TIM_HandleTypeDef htim5_MOTOR6;
speedRampData MOTOR6_srd = {STOP, CW, 0, 0, 0, 0, 0}; // �Ӽ������߱���
__IO int32_t MOTOR6_step_position = 0;                // ��ǰλ��
__IO uint8_t Motor6_MotionStatus = 0;                 //�Ƿ����˶���0��ֹͣ��1���˶�

__IO uint8_t Motor6_status = 1;
__IO int Motor6_num = 0;

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim5_MOTOR6;
/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ���������GPIO��ʼ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void MOTOR6_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* ���Ŷ˿�ʱ��ʹ�� */
  MOTOR6_TIM5_GPIO_CLK_ENABLE();
  MOTOR6_DIR_GPIO_CLK_ENABLE();
  MOTOR6_ENA_GPIO_CLK_ENABLE();

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR6_TIM5_PUL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5; // GPIO��������TIM���ù���
  HAL_GPIO_Init(MOTOR6_TIM5_PUL_PORT, &GPIO_InitStruct);

  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR6_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR6_DIR_PORT, &GPIO_InitStruct);

  /* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = MOTOR6_ENA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE; // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(MOTOR6_ENA_PORT, &GPIO_InitStruct);

  MOTOR6_DIR_FORWARD();
  MOTOR6_OUTPUT_DISABLE();
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void MOTOR6_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig; // ��ʱ��ʱ��
  TIM_OC_InitTypeDef sConfigOC;              // ��ʱ��ͨ���Ƚ����

  MOTOR6_TIM5_RCC_CLK_ENABLE();

  /* STEPMOTOR���GPIO��ʼ������ */
  MOTOR6_GPIO_Init();
  //
  /* ��ʱ�������������� */
  htim5_MOTOR6.Instance = MOTOR6_TIM5;                      // ��ʱ�����
  htim5_MOTOR6.Init.Prescaler = MOTOR6_TIM_PRESCALER;       // ��ʱ��Ԥ��Ƶ��
  htim5_MOTOR6.Init.CounterMode = TIM_COUNTERMODE_UP;       // �����������ϼ���
  htim5_MOTOR6.Init.Period = MOTOR6_TIM_PERIOD;             // ��ʱ������
  htim5_MOTOR6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // ʱ�ӷ�Ƶ
  HAL_TIM_Base_Init(&htim5_MOTOR6);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htim5_MOTOR6, &sClockSourceConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;            // �Ƚ����ģʽ����ת���
  sConfigOC.Pulse = 0xFFFF;                        // ������
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;       // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;     // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;   // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htim5_MOTOR6, &sConfigOC, MOTOR6_TIM5_CHANNEL_x);
  /* ʹ�ܱȽ����ͨ�� */
  TIM_CCxChannelCmd(MOTOR6_TIM5, MOTOR6_TIM5_CHANNEL_x, TIM_CCx_DISABLE);

  /* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(MOTOR6_TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTOR6_TIM5_IRQn);

  __HAL_TIM_CLEAR_FLAG(&htim5_MOTOR6, MOTOR6_TIM5_FLAG_CCx);
  /* ʹ�ܶ�ʱ���Ƚ���� */
  __HAL_TIM_ENABLE_IT(&htim5_MOTOR6, MOTOR6_TIM5_IT_CCx);
  /* Enable the main output */
  __HAL_TIM_MOE_ENABLE(&htim5_MOTOR6);
  //  HAL_TIM_Base_Start(&htim5_MOTOR6);// ʹ�ܶ�ʱ��
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
//  if(htim_base->Instance==MOTOR6_TIM5)
//  {
//    /* ������ʱ������ʱ��ʹ�� */
//    MOTOR6_TIM5_RCC_CLK_ENABLE();
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
//  if(htim_base->Instance==MOTOR6_TIM5)
//  {
//    /* ������ʱ������ʱ�ӽ��� */
//    MOTOR6_TIM5_RCC_CLK_DISABLE();
//    HAL_GPIO_DeInit(MOTOR6_TIM5_PUL_PORT,MOTOR6_TIM5_PUL_PIN);
//    HAL_GPIO_DeInit(MOTOR6_DIR_PORT,MOTOR6_DIR_PIN);
//    HAL_GPIO_DeInit(MOTOR6_ENA_PORT,MOTOR6_ENA_PIN);
//
//    HAL_NVIC_DisableIRQ(MOTOR6_TIM5_IRQn);
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
void MOTOR6_AxisMoveRel(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
  __IO uint16_t tim_count;
  // �ﵽ����ٶ�ʱ�Ĳ���
  __IO uint32_t max_s_lim;
  // ����Ҫ��ʼ���ٵĲ������������û�дﵽ����ٶȣ�
  __IO uint32_t accel_lim;

  if (Motor6_MotionStatus != STOP) // ֻ�������������ֹͣ��ʱ��ż���
    return;
  if (step < 0) // ����Ϊ����
  {
    MOTOR6_srd.dir = CCW; // ��ʱ�뷽����ת
    MOTOR6_DIR_REVERSAL();
    step = -step; // ��ȡ��������ֵ
  }
  else
  {
    MOTOR6_srd.dir = CW; // ˳ʱ�뷽����ת
    MOTOR6_DIR_FORWARD();
  }

  if (step == 1) // ����Ϊ1
  {
    MOTOR6_srd.accel_count = -1;  // ֻ�ƶ�һ��
    MOTOR6_srd.run_state = DECEL; // ����״̬.
    MOTOR6_srd.step_delay = 1000; // ����ʱ
  }
  else if (step != 0) // ���Ŀ���˶�������Ϊ0
  {
    // ���ǵĵ������ר��ָ���ֲ�����ϸ�ļ��㼰�Ƶ�����

    // ��������ٶȼ���, ����õ�min_delay���ڶ�ʱ���ļ�������ֵ��
    // min_delay = (alpha / tt)/ w
    MOTOR6_srd.min_delay = (int32_t)(A_T_x10_MOTOR6 / speed);

    // ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶȣ�����accel��λΪ0.1rad/sec^2
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
    MOTOR6_srd.step_delay = (int32_t)((T1_FREQ_148_MOTOR6 * sqrt(A_SQ / accel)) / 10);

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
      MOTOR6_srd.decel_val = accel_lim - step;
    }
    else
    {
      MOTOR6_srd.decel_val = -(max_s_lim * accel / decel);
    }
    // ��ֻʣ��һ�����Ǳ������
    if (MOTOR6_srd.decel_val == 0)
    {
      MOTOR6_srd.decel_val = -1;
    }
    MOTOR6_srd.min_delay = (int32_t)(A_T_x10_MOTOR6 / speed);
    MOTOR6_srd.step_delay = (int32_t)((T1_FREQ_148_MOTOR6 * sqrt(A_SQ / accel)) / 10);

    //     max_s_lim = 0x18de3;//9B6CC4;
    //     accel_lim = 0x82355;
    //     MOTOR6_srd.decel_val = accel_lim - step;
    // ���㿪ʼ����ʱ�Ĳ���
    MOTOR6_srd.decel_start = step + MOTOR6_srd.decel_val;

    // �������ٶȺ��������ǾͲ���Ҫ���м����˶�
    if (MOTOR6_srd.step_delay <= MOTOR6_srd.min_delay)
    {
      MOTOR6_srd.step_delay = MOTOR6_srd.min_delay;
      MOTOR6_srd.run_state = RUN;
    }
    else
    {
      MOTOR6_srd.run_state = ACCEL;
    }
    // ��λ���ٶȼ���ֵ
    MOTOR6_srd.accel_count = 0;
  }
  Motor6_MotionStatus = 1; // ���Ϊ�˶�״̬
  tim_count = __HAL_TIM_GET_COUNTER(&htim5_MOTOR6);
  __HAL_TIM_SET_COMPARE(&htim5_MOTOR6, MOTOR6_TIM5_CHANNEL_x, tim_count + MOTOR6_srd.step_delay); // ���ö�ʱ���Ƚ�ֵ
  TIM_CCxChannelCmd(MOTOR6_TIM5, MOTOR6_TIM5_CHANNEL_x, TIM_CCx_ENABLE);                          // ʹ�ܶ�ʱ��ͨ��
  HAL_TIM_Base_Start(&htim5_MOTOR6);                                                              // ʹ�ܶ�ʱ��
  MOTOR6_OUTPUT_ENABLE();
}
#if S_ACCEL
extern uint32_t step_to_run_MOTOR6;
extern unsigned short period_MOTOR6[ACCELERATED_SPEED_LENGTH]; //���鴢����ٹ�����ÿһ����ʱ�����Զ�װ��ֵ
extern float fre_MOTOR6[ACCELERATED_SPEED_LENGTH];             //����洢���ٹ�����ÿһ����Ƶ��

#endif

/**
  * ��������: ��ʱ���жϷ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ּӼ��ٹ���
  */
void MOTOR6_TIM5_IRQHandler(void) //��ʱ���жϴ���
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

  if (__HAL_TIM_GET_IT_SOURCE(&htim5_MOTOR6, MOTOR6_TIM5_IT_CCx) != RESET)
  {
    // �����ʱ���ж�
    __HAL_TIM_CLEAR_IT(&htim5_MOTOR6, MOTOR6_TIM5_IT_CCx);
    i++;
#if S_ACCEL
    if (i % 2 == 0) //ÿ���������жϲ���һ����������
    {
      switch (Motor6_status)
      {
      case ACCEL: //����
        __HAL_TIM_SET_AUTORELOAD(&htim5_MOTOR6, period_MOTOR6[Motor6_num]);
        __HAL_TIM_SET_COMPARE(&htim5_MOTOR6, MOTOR6_TIM5_CHANNEL_x, period_MOTOR6[Motor6_num] / 2);
        Motor6_num++;
        if (Motor6_num >= ACCELERATED_SPEED_LENGTH)
        {
          Motor6_status = 3;
        }
        break;
      case RUN: //����
        step_to_run_MOTOR6--;
        if (step_to_run_MOTOR6 < 1)
          Motor6_status = 2;
        break;
      case DECEL: //����
        Motor6_num--;
        __HAL_TIM_SET_AUTORELOAD(&htim5_MOTOR6, period_MOTOR6[Motor6_num]);
        __HAL_TIM_SET_COMPARE(&htim5_MOTOR6, MOTOR6_TIM5_CHANNEL_x, period_MOTOR6[Motor6_num] / 2);
        if (Motor6_num < 1)
          Motor6_status = 0;
        break;
      case STOP: //ֹͣ
                 // �ر�ͨ��
        TIM_CCxChannelCmd(MOTOR6_TIM5, MOTOR6_TIM5_CHANNEL_x, TIM_CCx_DISABLE);
        __HAL_TIM_CLEAR_FLAG(&htim5_MOTOR6, MOTOR6_TIM5_FLAG_CCx);
        MOTOR6_OUTPUT_DISABLE();
        break;
      }
    }
#endif

#if T_ACCEL
    // ���ñȽ�ֵ
    tim_count = __HAL_TIM_GET_COUNTER(&htim5_MOTOR6);
    __HAL_TIM_SET_COMPARE(&htim5_MOTOR6, MOTOR6_TIM5_CHANNEL_x, tim_count + MOTOR6_srd.step_delay);

    i++;        // ��ʱ���жϴ�������ֵ
    if (i == 2) // 2�Σ�˵���Ѿ����һ����������
    {
      i = 0;                        // ���㶨ʱ���жϴ�������ֵ
      switch (MOTOR6_srd.run_state) // �Ӽ������߽׶�
      {
      case STOP:
        step_count = 0; // ���㲽��������
        rest = 0;       // ������ֵ
        // �ر�ͨ��
        TIM_CCxChannelCmd(MOTOR6_TIM5, MOTOR6_TIM5_CHANNEL_x, TIM_CCx_DISABLE);
        __HAL_TIM_CLEAR_FLAG(&htim5_MOTOR6, MOTOR6_TIM5_FLAG_CCx);
        MOTOR6_OUTPUT_DISABLE();
        Motor6_MotionStatus = 0; //  ���Ϊֹͣ״̬
        break;

      case ACCEL:
        step_count++; // ������1
        if (MOTOR6_srd.dir == CW)
        {
          MOTOR6_step_position++; // ����λ�ü�1
        }
        else
        {
          MOTOR6_step_position--; // ����λ�ü�1
        }
        MOTOR6_srd.accel_count++;                                                                                           // ���ټ���ֵ��1
        new_step_delay = MOTOR6_srd.step_delay - (((2 * MOTOR6_srd.step_delay) + rest) / (4 * MOTOR6_srd.accel_count + 1)); //������(��)һ����������(ʱ����)
        rest = ((2 * MOTOR6_srd.step_delay) + rest) % (4 * MOTOR6_srd.accel_count + 1);                                     // �����������´μ��㲹���������������
        if (step_count >= MOTOR6_srd.decel_start)                                                                           // ����ǹ�Ӧ�ÿ�ʼ����
        {
          MOTOR6_srd.accel_count = MOTOR6_srd.decel_val; // ���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
          MOTOR6_srd.run_state = DECEL;                  // �¸����������ٽ׶�
        }
        else if (new_step_delay <= MOTOR6_srd.min_delay) // ����Ƿ񵽴�����������ٶ�
        {
          last_accel_delay = new_step_delay;     // ������ٹ��������һ����ʱ���������ڣ�
          new_step_delay = MOTOR6_srd.min_delay; // ʹ��min_delay����Ӧ����ٶ�speed��
          rest = 0;                              // ������ֵ
          MOTOR6_srd.run_state = RUN;            // ����Ϊ��������״̬
        }
        break;

      case RUN:
        step_count++; // ������1
        if (MOTOR6_srd.dir == CW)
        {
          MOTOR6_step_position++; // ����λ�ü�1
        }
        else
        {
          MOTOR6_step_position--; // ����λ�ü�1
        }
        new_step_delay = MOTOR6_srd.min_delay;    // ʹ��min_delay����Ӧ����ٶ�speed��
        if (step_count >= MOTOR6_srd.decel_start) // ��Ҫ��ʼ����
        {
          MOTOR6_srd.accel_count = MOTOR6_srd.decel_val; // ���ٲ�����Ϊ���ټ���ֵ
          new_step_delay = last_accel_delay;             // �ӽ׶�������ʱ��Ϊ���ٽ׶ε���ʼ��ʱ(��������)
          MOTOR6_srd.run_state = DECEL;                  // ״̬�ı�Ϊ����
        }
        break;

      case DECEL:
        step_count++; // ������1
        if (MOTOR6_srd.dir == CW)
        {
          MOTOR6_step_position++; // ����λ�ü�1
        }
        else
        {
          MOTOR6_step_position--; // ����λ�ü�1
        }
        MOTOR6_srd.accel_count++;
        new_step_delay = MOTOR6_srd.step_delay - (((2 * MOTOR6_srd.step_delay) + rest) / (4 * MOTOR6_srd.accel_count + 1)); //������(��)һ����������(ʱ����)
        rest = ((2 * MOTOR6_srd.step_delay) + rest) % (4 * MOTOR6_srd.accel_count + 1);                                     // �����������´μ��㲹���������������

        //����Ƿ�Ϊ���һ��
        if (MOTOR6_srd.accel_count >= 0)
        {
          MOTOR6_srd.run_state = STOP;
        }
        break;
      }
      MOTOR6_srd.step_delay = new_step_delay; // Ϊ�¸�(�µ�)��ʱ(��������)��ֵ
    }
#endif
  }
}
/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/