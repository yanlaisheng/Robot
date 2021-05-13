/******************** (C) COPYRIGHT 2020-2021 QINGDAO SANLI ********************
* File Name          : spi_flash.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides a set of functions needed to manage the
*                      communication between SPI peripheral and SPI M25P64 FLASH.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, QINGDAO SANLI SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "spi_flash.h"
#include "GlobalV_Extern.h" // 全局变量声明
#include "GlobalConst.h"
#include "stm32f4xx_hal_gpio.h"

/* Private typedef -----------------------------------------------------------*/
#define SPI_FLASH_PageSize 0x100

/* Private define ------------------------------------------------------------*/
#define WRITE 0x02 /* Write to Memory instruction */      //写指令
#define WRSR 0x01 /* Write Status Register instruction */ //写状态寄存器指令
#define WREN 0x06 /* Write enable instruction */          //写使能指令

#define READ 0x03 /* Read from Memory instruction */      //读指令
#define RDSR 0x05 /* Read Status Register instruction  */ //读状态寄存器指令
#define RDID 0x9F /* Read identification */               //读ID标示符
#define SE 0xD8 /* Sector Erase instruction */            //扇区擦除指令
#define BE 0xC7 /* Bulk Erase instruction */              //全部擦除指令

#define WIP_Flag 0x01 /* Write In Progress (WIP) flag */ //在编程写标志

#define Dummy_Byte 0xA5 //伪字节

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u8 rch = 0; //BU6929 语音播放使用
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : SPI_FLASH_Init
* Description    : Initializes the peripherals used by the SPI FLASH driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

//void SPI_FLASH_Init(void)
//{
//  SPI_InitTypeDef  SPI_InitStructure;
//  GPIO_InitTypeDef GPIO_InitStructure;

//  /* SPI2 Periph clock enable */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
//	/* Enable  clocks */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
//         | RCC_APB2Periph_GPIOE , ENABLE);
//
//  /* Configure SPI2 pins: SCK, MISO and MOSI */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);

//  /* Configure I/O for Flash1 Chip select */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;					//	/CS-M25P32
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//	/* Configure I/O for FM25L16 Chip select */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				// /CS-FM25L16
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//

//  /* Deselect the FLASH: Chip Select high */
//  SPI_FLASH_CS_HIGH();

//  /* SPI2 configuration */
//  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; 		//SPI_CPOL_High;  0,0(模式0); 1,1(模式3)
//  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;		//SPI_CPHA_2Edge;
//  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;				//软件选择么？
//  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
//  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  //SPI_FirstBit_MSB 	和	SPI_FirstBit_LSB
//  SPI_InitStructure.SPI_CRCPolynomial = 7;
//  SPI_Init(SPI2, &SPI_InitStructure);

//  /* Enable SPI2  */
//  SPI_Cmd(SPI2, ENABLE);
//}

/*******************************************************************************
* Function Name  : SPI_FLASH_SectorErase			SPI FALSH扇区擦除
* Description    : Erases the specified FLASH sector.
* Input          : SectorAddr: address of the sector to erase.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_SectorErase(u32 SectorAddr)
{
  /* Send write enable instruction */ //1. 写使能指令
  SPI_FLASH_WriteEnable();            //WREN

  /* Sector Erase */
  /* Select the FLASH: Chip Select low */ //2. 使能片选
  SPI_FLASH_CS_LOW();
  /* Send Sector Erase instruction */ //3. 发送全部擦除指令
  SPI_FLASH_SendByte(SE);
  /* Send SectorAddr high nibble address byte */ //4. 发送高位字节地址
  SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  /* Send SectorAddr medium nibble address byte */ //5. 发送中间位字节地址
  SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  /* Send SectorAddr low nibble address byte */ //6. 发送低位字节地址
  SPI_FLASH_SendByte(SectorAddr & 0xFF);
  /* Deselect the FLASH: Chip Select high */ //7.禁止FLASH片选
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */ //8. 等待FLASH写结束
  SPI_FLASH_WaitForWriteEnd();        //周成磊记号：写完检查写结束
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BulkErase  全部擦除
* Description    : Erases the entire FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BulkErase(void)
{
  /* Send write enable instruction */ //1. 写使能指令
  SPI_FLASH_WriteEnable();

  /* Bulk Erase */
  /* Select the FLASH: Chip Select low */ //2. 使能片选
  SPI_FLASH_CS_LOW();
  /* Send Bulk Erase instruction  */ //3. 发送全部擦除指令
  SPI_FLASH_SendByte(BE);
  /* Deselect the FLASH: Chip Select high */ //4. 禁止片选
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */ //5. 等待FLASH写结束
  SPI_FLASH_WaitForWriteEnd();        //周成磊记号：写完检查写结束
}

/*******************************************************************************
* Function Name  : SPI_FLASH_PageWrite		页写
* Description    : Writes more than one byte to the FLASH with a single WRITE
*                  cycle(Page WRITE sequence). The number of byte can't exceed
*                  the FLASH page size.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH,
*                    must be equal or less than "SPI_FLASH_PageSize" value.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_PageWrite(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
  /* Enable the write access to the FLASH */ //0. 使能FLASH写使能
  SPI_FLASH_WriteEnable();

  /* Select the FLASH: Chip Select low */ //1. 使能FLASH片选
  SPI_FLASH_CS_LOW();
  /* Send "Write to Memory " instruction */ //2. 发送写指令
  SPI_FLASH_SendByte(WRITE);
  /* Send WriteAddr high nibble address byte to write to */ //3.发送高字节地址
  SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
  /* Send WriteAddr medium nibble address byte to write to */ //4.发送中间字节地址
  SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  /* Send WriteAddr low nibble address byte to write to */ //5.发送低字节地址
  SPI_FLASH_SendByte(WriteAddr & 0xFF);

  /* while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /* Send the current byte */
    SPI_FLASH_SendByte(*pBuffer); //6. 发送字节
    /* Point on the next byte to be written */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */ //7.禁止FLASH片选
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */ //8. 等待FLASH写结束
  SPI_FLASH_WaitForWriteEnd();        //周成磊记号：写完检查写结束
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferWrite  缓存写，稍复杂写，分情况写，也不难
* Description    : Writes block of data to the FLASH. In this function, the
*                  number of WRITE cycles are reduced, using Page WRITE sequence.  使用页写顺序
* Input          : - pBuffer : pointer to the buffer  containing the data to be 
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH.
* Output         : None
* Return         : None
* 在AAAA处应该是正确的，I2C例程中这个地方好像不对。
*******************************************************************************/
void SPI_FLASH_BufferWrite(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % SPI_FLASH_PageSize;             //求余，如:5%4=1
  count = SPI_FLASH_PageSize - Addr;                 //4-1=3;　本页空闲空间数
  NumOfPage = NumByteToWrite / SPI_FLASH_PageSize;   //26/4=6  整数页
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize; //26%4=2	不足一页的字节数

  if (Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */ //一、具有整页的起始点
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize); //写整数页
        WriteAddr += SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle); //写不足一页的剩余部分
    }
  }
  else /* WriteAddr is not SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */ //此处跟I2C不一样
      {
        temp = NumOfSingle - count; //count：1.本页空闲字节数

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, count); //2.写这页指定地址到页尾
        WriteAddr += count;
        pBuffer += count;

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp); //3.紧跟下一页写剩余部分
      }
      else
      {
        //此处跟I2C不一样，就这一段 ！！！ AAAA
        //要写的字节数小于本页剩余字节，可以直接写
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      NumByteToWrite -= count;                           //count：1.本页需要写的剩余数
      NumOfPage = NumByteToWrite / SPI_FLASH_PageSize;   //2.整页数
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize; //3.还需要写的不足一页的剩余数

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, count); //1.写本页剩余字节（字节数：COUNT）
      WriteAddr += count;
      pBuffer += count;

      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize); //2.紧跟写整页
        WriteAddr += SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      if (NumOfSingle != 0)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle); //3.写不足一页的剩余部分
      }
    }
  }
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferRead		缓存读 很简单
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BufferRead(u8 *pBuffer, u32 ReadAddr, u16 NumByteToRead)
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */
  SPI_FLASH_SendByte(READ);

  /* Send ReadAddr high nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr & 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte to read from */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);

  while (NumByteToRead--) /* while there is data to be read */
  {
    /* Read a byte from the FLASH */
    *pBuffer = SPI_FLASH_SendByte(Dummy_Byte);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadID  读FLASH标示符ID
* Description    : Reads FLASH identification.
* Input          : None
* Output         : None
* Return         : FLASH identification
*******************************************************************************/
u32 SPI_FLASH_ReadID(void)
{
  u32 Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

  /* Select the FLASH: Chip Select low */ //1. 使能FLASH片选
  SPI_FLASH_CS_LOW();

  /* Send "RDID " instruction */ //2. 发送读ID指令
  SPI_FLASH_SendByte(0x9F);

  /* Read a byte from the FLASH */ //3. 读ID高字节
  Temp0 = SPI_FLASH_SendByte(Dummy_Byte);

  /* Read a byte from the FLASH */ //4. 读ID中间字节
  Temp1 = SPI_FLASH_SendByte(Dummy_Byte);

  /* Read a byte from the FLASH */ //5. 读ID低字节
  Temp2 = SPI_FLASH_SendByte(Dummy_Byte);

  /* Deselect the FLASH: Chip Select high */ //6. 禁止FLASH片选
  SPI_FLASH_CS_HIGH();
  //7. 三个字节合成24位的ID
  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2; //可以看出ID是一个24位的数

  return Temp;
}

/*******************************************************************************
* Function Name  : SPI_FLASH_StartReadSequence	启动读顺序
* Description    : Initiates a read data byte (READ) sequence from the Flash.
*                  This is done by driving the /CS line low to select the device,
*                  then the READ instruction is transmitted followed by 3 bytes
*                  address. This function exit and keep the /CS line low, so the
*                  Flash still being selected. With this technique the whole
*                  content of the Flash is read with a single READ instruction.
* Input          : - ReadAddr : FLASH's internal address to read from.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_StartReadSequence(u32 ReadAddr)
{
  /* Select the FLASH: Chip Select low */ //1. 使能FLASH片选
  SPI_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */ //2. 发送读指令
  SPI_FLASH_SendByte(READ);

  /* Send the 24-bit address of the address to read from */ //3. 发送24位地址
  /* Send ReadAddr high nibble address byte */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte */
  SPI_FLASH_SendByte((ReadAddr & 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadByte		读字节
* Description    : Reads a byte from the SPI Flash.
*                  This function must be used only if the Start_Read_Sequence
*                  function has been previously called.
* Input          : None
* Output         : None
* Return         : Byte Read from the SPI Flash.
*******************************************************************************/
u8 SPI_FLASH_ReadByte(void)
{
  return (SPI_FLASH_SendByte(Dummy_Byte)); // 用发送伪字节后的返回值来读
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte		SPI发送字节
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
u8 SPI_FLASH_SendByte(u8 byte)
{
  /* Loop while DR register in not emplty */ //1. 检测：DR寄存器是空，可以发送
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    ;

  /* Send byte through the SPI2 peripheral */ //2. 发送
  SPI_I2S_SendData(SPI2, byte);

  /* Wait to receive a byte */ //3. 等待接收字节完成
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    ;

  /* Return the byte read from the SPI bus */ //4. 返回接收的字节
  return SPI_I2S_ReceiveData(SPI2);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendHalfWord		SPI发送半字（这里的一个字32位）
* Description    : Sends a Half Word through the SPI interface and return the
*                  Half Word received from the SPI bus.
* Input          : Half Word : Half Word to send.
* Output         : None
* Return         : The value of the received Half Word.
*******************************************************************************/
u16 SPI_FLASH_SendHalfWord(u16 HalfWord)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    ;

  /* Send Half Word through the SPI2 peripheral */
  SPI_I2S_SendData(SPI2, HalfWord);

  /* Wait to receive a Half Word */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    ;

  /* Return the Half Word read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI2);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WriteEnable		FLASH写使能指令
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WriteEnable(void)
{
  /* Select the FLASH: Chip Select low */ //1. 使能FLASH片选
  SPI_FLASH_CS_LOW();

  /* Send "Write Enable" instruction */ //2. 发送WREN指令
  SPI_FLASH_SendByte(WREN);

  /* Deselect the FLASH: Chip Select high */ //3. 禁止FLASH片选
  SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WaitForWriteEnd		等待写结束
* Description    : Polls the status of the Write In Progress (WIP) flag in the  
*                  FLASH's status  register  and  loop  until write  opertaion
*                  has completed.  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WaitForWriteEnd(void)
{
  u8 FLASH_Status = 0;

  /* Select the FLASH: Chip Select low */ //1. 使能FLASH片选
  SPI_FLASH_CS_LOW();

  /* Send "Read Status Register" instruction */ //2. 发送RDSR指令
  SPI_FLASH_SendByte(RDSR);

  /* Loop as long as the memory is busy with a write cycle */
  do
  {
    /* Send a dummy byte to generate the clock needed by the FLASH 
    and put the value of the status register in FLASH_Status variable */
    FLASH_Status = SPI_FLASH_SendByte(Dummy_Byte); //3. 发送伪字节，读状态寄存器值到FLASH_Status

  } while ((FLASH_Status & WIP_Flag) == SET); /* Write in progress */ //FLASH_Status=0,说明写完

  /* Deselect the FLASH: Chip Select high */ //4. 禁止FLASH片选
  SPI_FLASH_CS_HIGH();
}

// 周成磊增加函数
void SPI_FLASH_CS_LOW(void) //片选低
{
  if (B_SelCS == CS_Flash1)
    GPIO_ResetBits(GPIOC, GPIO_Pin_6);
  else if (B_SelCS == CS_FMRAM1)
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

void SPI_FLASH_CS_HIGH(void) //片选高
{
  GPIO_SetBits(GPIOC, GPIO_Pin_6);
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

//下面为铁电RAM FM25L16 程序-------------------------------------------------------------------------
/*******************************************************************************
* Function Name  : SPI_FMRAM_BufferWrite  铁电缓存写-可以连续写
* Description    : Writes block of data to the FLASH. In this function, the
*                  number of WRITE cycles are reduced, using Page WRITE sequence.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FMRAM_BufferWrite(u8 *pBuffer, u16 WriteAddr, u16 NumByteToWrite)
{
  /* Enable the write access to the FLASH */ //0. 使能FLASH写使能
  SPI_FLASH_WriteEnable();

  /* Select the FLASH: Chip Select low */ //1. 使能FLASH片选
  SPI_FLASH_CS_LOW();
  /* Send "Write to Memory " instruction */ //2. 发送写指令
  SPI_FLASH_SendByte(WRITE);
  /* Send WriteAddr medium nibble address byte to write to */ //3.发送中间字节地址(没有高字节)
  SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  /* Send WriteAddr low nibble address byte to write to */ //4.发送低字节地址
  SPI_FLASH_SendByte(WriteAddr & 0xFF);

  /* while there is data to be written on the FLASH */
  while (NumByteToWrite--) // 5. 循环写
  {
    /* Send the current byte */
    SPI_FLASH_SendByte(*pBuffer); //6. 发送字节
    /* Point on the next byte to be written */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */ //7.禁止FLASH片选
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */ //8. 等待FLASH写结束
  SPI_FLASH_WaitForWriteEnd();        //周成磊记号：写完检查写结束
}

/*******************************************************************************
* Function Name  : SPI_FMRAM_BufferRead     铁电缓存读-比较简单
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FMRAM_BufferRead(u8 *pBuffer, u16 ReadAddr, u16 NumByteToRead)
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */ //发送读指令
  SPI_FLASH_SendByte(READ);

  /* Send ReadAddr medium nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr & 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte to read from */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);

  while (NumByteToRead--) /* while there is data to be read */
  {
    /* Read a byte from the FLASH */
    *pBuffer = SPI_FLASH_SendByte(Dummy_Byte);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}

/******************* (C) COPYRIGHT 2020-2021 QINGDAO SANLI *****END OF FILE****/
