#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
 extern "C" {
#endif  

#include "stm32f4xx.h"

//LED端口定义
//开发板上用的是红蓝绿三色灯，同时只能点亮一种元色；可应用视觉暂留原理实现复合色；
//GPIO_PIN_RESET 灯亮ON； GPIO_PIN_SET 灯灭OFF
  

#define LED_Blue(n)		(n?HAL_GPIO_WritePin(LED_H63_GPIO_Port,LED_H63_Pin,GPIO_PIN_RESET):HAL_GPIO_WritePin(LED_H63_GPIO_Port,LED_H63_Pin,GPIO_PIN_SET))
#define LED_Blue_Toggle (HAL_GPIO_TogglePin(LED_H63_GPIO_Port, LED_H63_Pin)) //_Blue输出电平翻转



void LED_Init(void);

void LED_Reset(void);  //LED复位灯全灭

void Led_B(uint8_t level);

void LED_Blue_Flash(uint16_t nms); //蓝灯闪nms

void LED_Test(void); //紫，黄，青依次点亮 测试三色灯是否可正常工作 时间1.5s

#ifdef __cplusplus
}
#endif

#endif
