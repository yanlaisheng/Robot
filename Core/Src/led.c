//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//LED��������	   								  
////////////////////////////////////////////////////////////////////////////////// 

#include "led.h"
#include "main.h"

//LED��ʼ������
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


void LED_Blue_Flash(uint16_t nms)  //������nms
{
    uint16_t temp=nms>>8; //����8λ���ȼ��ڳ���2^8=256����ÿ��256ms��˸һ��  
    LED_Reset();
    for(uint8_t i=0;i<temp;i++)
    {
        LED_Blue(1);
        HAL_Delay(1<<7); //����8λ�������128ms��        
        
        LED_Reset();
        HAL_Delay(1<<7); //����8λ�������128ms��
       
    }    		
}

void LED_Test() 
{
 //�ϣ��ƣ������ε��� ������ɫ���Ƿ���������� ʱ��1.5s   
    LED_Blue_Flash(1000);    
     
}

