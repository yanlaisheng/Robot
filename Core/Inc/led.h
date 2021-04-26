#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
 extern "C" {
#endif  

#include "stm32f4xx.h"

//LED�˿ڶ���
//���������õ��Ǻ�������ɫ�ƣ�ͬʱֻ�ܵ���һ��Ԫɫ����Ӧ���Ӿ�����ԭ��ʵ�ָ���ɫ��
//GPIO_PIN_RESET ����ON�� GPIO_PIN_SET ����OFF
  

#define LED_Blue(n)		(n?HAL_GPIO_WritePin(LED_H63_GPIO_Port,LED_H63_Pin,GPIO_PIN_RESET):HAL_GPIO_WritePin(LED_H63_GPIO_Port,LED_H63_Pin,GPIO_PIN_SET))
#define LED_Blue_Toggle (HAL_GPIO_TogglePin(LED_H63_GPIO_Port, LED_H63_Pin)) //_Blue�����ƽ��ת



void LED_Init(void);

void LED_Reset(void);  //LED��λ��ȫ��

void Led_B(uint8_t level);

void LED_Blue_Flash(uint16_t nms); //������nms

void LED_Test(void); //�ϣ��ƣ������ε��� ������ɫ���Ƿ���������� ʱ��1.5s

#ifdef __cplusplus
}
#endif

#endif