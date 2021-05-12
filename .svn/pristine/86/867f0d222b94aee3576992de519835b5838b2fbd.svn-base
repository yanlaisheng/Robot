//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//LED驱动代码	   								  
////////////////////////////////////////////////////////////////////////////////// 

#include "led.h"
#include "main.h"

//LED初始化函数
void LED_Init(void)
{

}

void LED_Reset()  
  {
    HAL_GPIO_WritePin(LED_H63_GPIO_Port,LED_H63_Pin,GPIO_PIN_SET);
  }

void Led_B(uint8_t level)
{
	if(level == 1)
	{
		LED_Reset();
		HAL_GPIO_WritePin(LED_H63_GPIO_Port, LED_H63_Pin, GPIO_PIN_RESET);
	}
	else if(level == 0)
	{
		HAL_GPIO_WritePin(LED_H63_GPIO_Port, LED_H63_Pin, GPIO_PIN_SET);
	}
}


void LED_Blue_Flash(uint16_t nms)  //蓝灯闪nms
{
    uint16_t temp=nms>>8; //右移8位，等价于除以2^8=256，即每隔256ms闪烁一次  
    LED_Reset();
    for(uint8_t i=0;i<temp;i++)
    {
        LED_Blue(1);
        HAL_Delay(1<<7); //左移8位，即间隔128ms亮        
        
        LED_Reset();
        HAL_Delay(1<<7); //左移8位，即间隔128ms灭
       
    }    		
}

void LED_Test() 
{
 //紫，黄，青依次点亮 测试三色灯是否可正常工作 时间1.5s   
    LED_Blue_Flash(1000);    
     
}


