
#include "w25qxx.h"
#include "w25qxxConf.h"
#include "typedef.h"
#include "GlobalV_Extern.h"			// 全局变量声明

#if (_W25QXX_DEBUG==1)
#include <stdio.h>
#endif

#define W25QXX_DUMMY_BYTE       0xA5
#define SPI_FLASH_PageSize    	256

/* Private define ------------------------------------------------------------*/
#define WRITE      0x02  /* Write to Memory instruction */			//写指令
#define WRSR       0x01  /* Write Status Register instruction */		//写状态寄存器指令 
#define WREN       0x06  /* Write enable instruction */					//写使能指令

#define READ       0x03  /* Read from Memory instruction */			//读指令
#define RDSR       0x05  /* Read Status Register instruction  */		//读状态寄存器指令
#define RDID       0x9F  /* Read identification */							//读ID标示符
#define SE         0xD8  /* Sector Erase instruction */					//扇区擦除指令
#define BE         0xC7  /* Bulk Erase instruction */						//全部擦除指令

#define WIP_Flag   0x01  /* Write In Progress (WIP) flag */			//在编程写标志

#define Dummy_Byte 0xA5		//伪字节

w25qxx_t	w25qxx;

#if (_W25QXX_USE_FREERTOS==1)
#define	W25qxx_Delay(delay)		osDelay(delay)
#include "cmsis_os.h"
#else
#define	W25qxx_Delay(delay)		HAL_Delay(delay)
#endif
//###################################################################################################################
uint8_t	W25qxx_Spi(uint8_t	Data)
{
	uint8_t	ret;
	HAL_SPI_TransmitReceive(&_W25QXX_SPI,&Data,&ret,1,100);
	return ret;	
}
//###################################################################################################################
uint32_t W25qxx_ReadID(void)
{
  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
//#define READ_JEDEC_ID_CMD                    0x9F
  W25qxx_Spi(0x9F);								
  Temp0 = W25qxx_Spi(W25QXX_DUMMY_BYTE);	//0xA5
  Temp1 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
  Temp2 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
  return Temp;
}
//###################################################################################################################
void W25qxx_ReadUniqID(void)
{
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
//#define Read Unique ID 0x4B
  W25qxx_Spi(0x4B);
	for(uint8_t	i=0;i<4;i++)
		W25qxx_Spi(W25QXX_DUMMY_BYTE);
	for(uint8_t	i=0;i<8;i++)
		w25qxx.UniqID[i] = W25qxx_Spi(W25QXX_DUMMY_BYTE);
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
}
//###################################################################################################################
void W25qxx_WriteEnable(void)
{
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
//WRITE_ENABLE_CMD 0x06
  W25qxx_Spi(0x06);
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
	W25qxx_Delay(1);
}
//###################################################################################################################
void W25qxx_WriteDisable(void)
{
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
	//#define WRITE_DISABLE_CMD                    0x04
  W25qxx_Spi(0x04);
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
	W25qxx_Delay(1);
}
//###################################################################################################################
uint8_t W25qxx_ReadStatusRegister(uint8_t	SelectStatusRegister_1_2_3)
{
	uint8_t	status=0;
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
	if(SelectStatusRegister_1_2_3==1)
	{
		//#define READ_STATUS_REG1_CMD                  0x05		
		W25qxx_Spi(0x05);
		status=W25qxx_Spi(W25QXX_DUMMY_BYTE);	
		w25qxx.StatusRegister1 = status;
	}
	else if(SelectStatusRegister_1_2_3==2)
	{
		//#define READ_STATUS_REG2_CMD                  0x35		
		W25qxx_Spi(0x35);
		status=W25qxx_Spi(W25QXX_DUMMY_BYTE);	
		w25qxx.StatusRegister2 = status;
	}
	else
	{
		//#define READ_STATUS_REG3_CMD                  0x15
		W25qxx_Spi(0x15);
		status=W25qxx_Spi(W25QXX_DUMMY_BYTE);	
		w25qxx.StatusRegister3 = status;
	}	
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
	return status;
}
//###################################################################################################################
void W25qxx_WriteStatusRegister(uint8_t	SelectStatusRegister_1_2_3,uint8_t Data)
{
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
	if(SelectStatusRegister_1_2_3==1)
	{
		//#define WRITE_STATUS_REG1_CMD                 0x01		
		W25qxx_Spi(0x01);
		w25qxx.StatusRegister1 = Data;
	}
	else if(SelectStatusRegister_1_2_3==2)
	{
		//#define WRITE_STATUS_REG2_CMD                 0x31
		W25qxx_Spi(0x31);
		w25qxx.StatusRegister2 = Data;
	}
	else
	{
		//#define WRITE_STATUS_REG3_CMD                 0x11
		W25qxx_Spi(0x11);
		w25qxx.StatusRegister3 = Data;
	}
	W25qxx_Spi(Data);
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
}
//###################################################################################################################
void W25qxx_WaitForWriteEnd(void)
{
	W25qxx_Delay(1);
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
	//#define READ_STATUS_REG1_CMD                  0x05
	W25qxx_Spi(0x05);
  do
  {
    w25qxx.StatusRegister1 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
		W25qxx_Delay(1);
  }
  while ((w25qxx.StatusRegister1 & 0x01) == 0x01);
 HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
}
//###################################################################################################################
bool	W25qxx_Init(void)
{
	w25qxx.Lock=1;	
	while(HAL_GetTick()<100)
		W25qxx_Delay(1);
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
  W25qxx_Delay(100);
	uint32_t	id;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx Init Begin...\r\n");
	#endif
	//READ_JEDEC_ID_CMD  0x9F
	id=W25qxx_ReadID();
	
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx ID:0x%X\r\n",id);
	#endif
	switch(id&0x0000FFFF)
	{
		case 0x401A:	// 	w25q512
			w25qxx.ID=W25Q512;
			w25qxx.BlockCount=1024;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q512\r\n");
			#endif
		break;
		case 0x4019:	// 	w25q256
			w25qxx.ID=W25Q256;
			w25qxx.BlockCount=512;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q256\r\n");
			#endif
		break;
		case 0x4018:	// 	w25q128
			w25qxx.ID=W25Q128;
			w25qxx.BlockCount=256;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q128\r\n");
			#endif
		break;
		case 0x4017:	//	w25q64
			w25qxx.ID=W25Q64;
			w25qxx.BlockCount=128;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q64\r\n");
			#endif
		break;
		case 0x4016:	//	w25q32
			w25qxx.ID=W25Q32;
			w25qxx.BlockCount=64;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q32\r\n");
			#endif
		break;
		case 0x4015:	//	w25q16
			w25qxx.ID=W25Q16;
			w25qxx.BlockCount=32;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q16\r\n");
			#endif
		break;
		case 0x4014:	//	w25q80
			w25qxx.ID=W25Q80;
			w25qxx.BlockCount=16;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q80\r\n");
			#endif
		break;
		case 0x4013:	//	w25q40
			w25qxx.ID=W25Q40;
			w25qxx.BlockCount=8;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q40\r\n");
			#endif
		break;
		case 0x4012:	//	w25q20
			w25qxx.ID=W25Q20;
			w25qxx.BlockCount=4;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q20\r\n");
			#endif
		break;
		case 0x4011:	//	w25q10
			w25qxx.ID=W25Q10;
			w25qxx.BlockCount=2;
			#if (_W25QXX_DEBUG==1)
			printf("w25qxx Chip: w25q10\r\n");
			#endif
		break;
		default:
				#if (_W25QXX_DEBUG==1)
				printf("w25qxx Unknown ID\r\n");
				#endif
			w25qxx.Lock=0;	
			return false;
				
	}		
	w25qxx.PageSize=256;
	w25qxx.SectorSize=0x1000;
	w25qxx.SectorCount=w25qxx.BlockCount*16;
	w25qxx.PageCount=(w25qxx.SectorCount*w25qxx.SectorSize)/w25qxx.PageSize;
	w25qxx.BlockSize=w25qxx.SectorSize*16;
	w25qxx.CapacityInKiloByte=(w25qxx.SectorCount*w25qxx.SectorSize)/1024;
	W25qxx_ReadUniqID();
	W25qxx_ReadStatusRegister(1);
	W25qxx_ReadStatusRegister(2);
	W25qxx_ReadStatusRegister(3);
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx Page Size: %d Bytes\r\n",w25qxx.PageSize);
	printf("w25qxx Page Count: %d\r\n",w25qxx.PageCount);
	printf("w25qxx Sector Size: %d Bytes\r\n",w25qxx.SectorSize);
	printf("w25qxx Sector Count: %d\r\n",w25qxx.SectorCount);
	printf("w25qxx Block Size: %d Bytes\r\n",w25qxx.BlockSize);
	printf("w25qxx Block Count: %d\r\n",w25qxx.BlockCount);
	printf("w25qxx Capacity: %d KiloBytes\r\n",w25qxx.CapacityInKiloByte);
	printf("w25qxx Init Done\r\n");
	#endif
	w25qxx.Lock=0;	
	return true;
}	
//###################################################################################################################
//擦除整个芯片		  
//等待时间超长...
void	W25qxx_EraseChip(void)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;	
	#if (_W25QXX_DEBUG==1)
	uint32_t	StartTime=HAL_GetTick();	
	printf("w25qxx EraseChip Begin...\r\n");
	#endif
	W25qxx_WriteEnable();			//允许写入
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
	
//#define CHIP_ERASE_CMD                       0xC7
  W25qxx_Spi(0xC7);
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
	W25qxx_WaitForWriteEnd();			//等待擦除/写入完成
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx EraseBlock done after %d ms!\r\n",HAL_GetTick()-StartTime);
	#endif
	W25qxx_Delay(10);
	w25qxx.Lock=0;	
}
//###################################################################################################################
//擦除一个扇区
//SectorAddr:扇区地址 根据实际容量设置
//例如擦除扇区1，即SectorAddr=1，则实际地址要乘以4096
//擦除一个扇区的最少时间:150ms
void W25qxx_EraseSector(uint32_t SectorAddr)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;	
	#if (_W25QXX_DEBUG==1)
	uint32_t	StartTime=HAL_GetTick();	
	printf("w25qxx EraseSector %d Begin...\r\n",SectorAddr);
	#endif
	W25qxx_WaitForWriteEnd();			//等待擦除/写入完成	
	SectorAddr = SectorAddr * w25qxx.SectorSize;	//注意：地址要乘以4096
  W25qxx_WriteEnable();
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
	//#define SECTOR_ERASE_CMD                     0x20
  W25qxx_Spi(0x20);						//先擦除扇区，命令0x20+要擦除的扇区地址
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((SectorAddr & 0xFF000000) >> 24);
  W25qxx_Spi((SectorAddr & 0xFF0000) >> 16);
  W25qxx_Spi((SectorAddr & 0xFF00) >> 8);
  W25qxx_Spi(SectorAddr & 0xFF);
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
  W25qxx_WaitForWriteEnd();				//等待擦除/写入完成	
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx EraseSector done after %d ms\r\n",HAL_GetTick()-StartTime);
	#endif
	W25qxx_Delay(1);
	w25qxx.Lock=0;
}
//###################################################################################################################
void W25qxx_EraseBlock(uint32_t BlockAddr)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;	
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx EraseBlock %d Begin...\r\n",BlockAddr);
	W25qxx_Delay(100);
	uint32_t	StartTime=HAL_GetTick();	
	#endif
	W25qxx_WaitForWriteEnd();
	BlockAddr = BlockAddr * w25qxx.SectorSize*16;
  W25qxx_WriteEnable();
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
	//Block Erase(64KB)  0xD8
  W25qxx_Spi(0xD8);
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((BlockAddr & 0xFF000000) >> 24);
  W25qxx_Spi((BlockAddr & 0xFF0000) >> 16);
  W25qxx_Spi((BlockAddr & 0xFF00) >> 8);
  W25qxx_Spi(BlockAddr & 0xFF);
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
  W25qxx_WaitForWriteEnd();
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx EraseBlock done after %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif
	W25qxx_Delay(1);
	w25qxx.Lock=0;
}
//###################################################################################################################
uint32_t	W25qxx_PageToSector(uint32_t	PageAddress)
{
	return ((PageAddress*w25qxx.PageSize)/w25qxx.SectorSize);
}
//###################################################################################################################
uint32_t	W25qxx_PageToBlock(uint32_t	PageAddress)
{
	return ((PageAddress*w25qxx.PageSize)/w25qxx.BlockSize);
}
//###################################################################################################################
uint32_t	W25qxx_SectorToBlock(uint32_t	SectorAddress)
{
	return ((SectorAddress*w25qxx.SectorSize)/w25qxx.BlockSize);
}
//###################################################################################################################
uint32_t	W25qxx_SectorToPage(uint32_t	SectorAddress)
{
	return (SectorAddress*w25qxx.SectorSize)/w25qxx.PageSize;
}
//###################################################################################################################
uint32_t	W25qxx_BlockToPage(uint32_t	BlockAddress)
{
	return (BlockAddress*w25qxx.BlockSize)/w25qxx.PageSize;
}
//###################################################################################################################
bool 	W25qxx_IsEmptyPage(uint32_t Page_Address,uint32_t OffsetInByte,uint32_t NumByteToCheck_up_to_PageSize)
{
	while(w25qxx.Lock==1)
	W25qxx_Delay(1);
	w25qxx.Lock=1;	
	if(((NumByteToCheck_up_to_PageSize+OffsetInByte)>w25qxx.PageSize)||(NumByteToCheck_up_to_PageSize==0))
		NumByteToCheck_up_to_PageSize=w25qxx.PageSize-OffsetInByte;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckPage:%d, Offset:%d, Bytes:%d begin...\r\n",Page_Address,OffsetInByte,NumByteToCheck_up_to_PageSize);
	W25qxx_Delay(100);
	uint32_t	StartTime=HAL_GetTick();
	#endif		
	uint8_t		pBuffer[32];
	uint32_t	WorkAddress;
	uint32_t	i;
	for(i=OffsetInByte; i<w25qxx.PageSize; i+=sizeof(pBuffer))
	{
		HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
		WorkAddress=(i+Page_Address*w25qxx.PageSize);
		//#define FAST_READ_CMD                        0x0B
		W25qxx_Spi(0x0B);
		if(w25qxx.ID>=W25Q256)
			W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
		W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(WorkAddress & 0xFF);
		W25qxx_Spi(0);
		HAL_SPI_Receive(&_W25QXX_SPI,pBuffer,sizeof(pBuffer),100);	
		HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);	
		for(uint8_t x=0;x<sizeof(pBuffer);x++)
		{
			if(pBuffer[x]!=0xFF)
				goto NOT_EMPTY;		
		}			
	}	
	if((w25qxx.PageSize+OffsetInByte)%sizeof(pBuffer)!=0)
	{
		i-=sizeof(pBuffer);
		for( ; i<w25qxx.PageSize; i++)
		{
			HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
			WorkAddress=(i+Page_Address*w25qxx.PageSize);
			W25qxx_Spi(0x0B);
			if(w25qxx.ID>=W25Q256)
				W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
			W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(WorkAddress & 0xFF);
			W25qxx_Spi(0);
			HAL_SPI_Receive(&_W25QXX_SPI,pBuffer,1,100);	
			HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);	
			if(pBuffer[0]!=0xFF)
				goto NOT_EMPTY;
		}
	}	
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckPage is Empty in %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif	
	w25qxx.Lock=0;
	return true;	
	NOT_EMPTY:
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckPage is Not Empty in %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif	
	w25qxx.Lock=0;
	return false;
}
//###################################################################################################################
bool 	W25qxx_IsEmptySector(uint32_t Sector_Address,uint32_t OffsetInByte,uint32_t NumByteToCheck_up_to_SectorSize)
{
	while(w25qxx.Lock==1)
	W25qxx_Delay(1);
	w25qxx.Lock=1;	
	if((NumByteToCheck_up_to_SectorSize>w25qxx.SectorSize)||(NumByteToCheck_up_to_SectorSize==0))
		NumByteToCheck_up_to_SectorSize=w25qxx.SectorSize;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckSector:%d, Offset:%d, Bytes:%d begin...\r\n",Sector_Address,OffsetInByte,NumByteToCheck_up_to_SectorSize);
	W25qxx_Delay(100);
	uint32_t	StartTime=HAL_GetTick();
	#endif		
	uint8_t		pBuffer[32];
	uint32_t	WorkAddress;
	uint32_t	i;
	for(i=OffsetInByte; i<w25qxx.SectorSize; i+=sizeof(pBuffer))
	{
		HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
		WorkAddress=(i+Sector_Address*w25qxx.SectorSize);
		W25qxx_Spi(0x0B);
		if(w25qxx.ID>=W25Q256)
			W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
		W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(WorkAddress & 0xFF);
		W25qxx_Spi(0);
		HAL_SPI_Receive(&_W25QXX_SPI,pBuffer,sizeof(pBuffer),100);	
		HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);	
		for(uint8_t x=0;x<sizeof(pBuffer);x++)
		{
			if(pBuffer[x]!=0xFF)
				goto NOT_EMPTY;		
		}			
	}	
	if((w25qxx.SectorSize+OffsetInByte)%sizeof(pBuffer)!=0)
	{
		i-=sizeof(pBuffer);
		for( ; i<w25qxx.SectorSize; i++)
		{
			HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
			WorkAddress=(i+Sector_Address*w25qxx.SectorSize);
			W25qxx_Spi(0x0B);
			if(w25qxx.ID>=W25Q256)
				W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
			W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(WorkAddress & 0xFF);
			W25qxx_Spi(0);
			HAL_SPI_Receive(&_W25QXX_SPI,pBuffer,1,100);	
			HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);	
			if(pBuffer[0]!=0xFF)
				goto NOT_EMPTY;
		}
	}	
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckSector is Empty in %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif	
	w25qxx.Lock=0;
	return true;	
	NOT_EMPTY:
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckSector is Not Empty in %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif	
	w25qxx.Lock=0;
	return false;
}
//###################################################################################################################
bool 	W25qxx_IsEmptyBlock(uint32_t Block_Address,uint32_t OffsetInByte,uint32_t NumByteToCheck_up_to_BlockSize)
{
	while(w25qxx.Lock==1)
	W25qxx_Delay(1);
	w25qxx.Lock=1;	
	if((NumByteToCheck_up_to_BlockSize>w25qxx.BlockSize)||(NumByteToCheck_up_to_BlockSize==0))
		NumByteToCheck_up_to_BlockSize=w25qxx.BlockSize;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckBlock:%d, Offset:%d, Bytes:%d begin...\r\n",Block_Address,OffsetInByte,NumByteToCheck_up_to_BlockSize);
	W25qxx_Delay(100);
	uint32_t	StartTime=HAL_GetTick();
	#endif		
	uint8_t		pBuffer[32];
	uint32_t	WorkAddress;
	uint32_t	i;
	for(i=OffsetInByte; i<w25qxx.BlockSize; i+=sizeof(pBuffer))
	{
		HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
		WorkAddress=(i+Block_Address*w25qxx.BlockSize);
		W25qxx_Spi(0x0B);
		if(w25qxx.ID>=W25Q256)
			W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
		W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
		W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
		W25qxx_Spi(WorkAddress & 0xFF);
		W25qxx_Spi(0);
		HAL_SPI_Receive(&_W25QXX_SPI,pBuffer,sizeof(pBuffer),100);	
		HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);	
		for(uint8_t x=0;x<sizeof(pBuffer);x++)
		{
			if(pBuffer[x]!=0xFF)
				goto NOT_EMPTY;		
		}			
	}	
	if((w25qxx.BlockSize+OffsetInByte)%sizeof(pBuffer)!=0)
	{
		i-=sizeof(pBuffer);
		for( ; i<w25qxx.BlockSize; i++)
		{
			HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
			WorkAddress=(i+Block_Address*w25qxx.BlockSize);
			W25qxx_Spi(0x0B);
			if(w25qxx.ID>=W25Q256)
				W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
			W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
			W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
			W25qxx_Spi(WorkAddress & 0xFF);
			W25qxx_Spi(0);
			HAL_SPI_Receive(&_W25QXX_SPI,pBuffer,1,100);	
			HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);	
			if(pBuffer[0]!=0xFF)
				goto NOT_EMPTY;
		}
	}	
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckBlock is Empty in %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif	
	w25qxx.Lock=0;
	return true;	
	NOT_EMPTY:
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx CheckBlock is Not Empty in %d ms\r\n",HAL_GetTick()-StartTime);
	W25qxx_Delay(100);
	#endif	
	w25qxx.Lock=0;
	return false;
}

//###################################################################################################################
//从地址WriteAddr_inBytes写入1个字节
//必须确保所写的地址的数据为0XFF，否则在非0XFF处写入的数据将失败!
void W25qxx_WriteByte(uint8_t pBuffer, uint32_t WriteAddr_inBytes)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;
	#if (_W25QXX_DEBUG==1)
	uint32_t	StartTime=HAL_GetTick();
	printf("w25qxx WriteByte 0x%02X at address %d begin...",pBuffer,WriteAddr_inBytes);
	#endif
	W25qxx_WaitForWriteEnd();
  W25qxx_WriteEnable();
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
  //#define PAGE_PROG_CMD                        0x02
  W25qxx_Spi(0x02);
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((WriteAddr_inBytes & 0xFF000000) >> 24);
  W25qxx_Spi((WriteAddr_inBytes & 0xFF0000) >> 16);
  W25qxx_Spi((WriteAddr_inBytes & 0xFF00) >> 8);
  W25qxx_Spi(WriteAddr_inBytes & 0xFF);
  W25qxx_Spi(pBuffer);
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
  W25qxx_WaitForWriteEnd();
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx WriteByte done after %d ms\r\n",HAL_GetTick()-StartTime);
	#endif
	w25qxx.Lock=0;
}

//###################################################################################################################
//SPI在一页内写入少于256个字节的数据
//在指定地址开始写入最大256字节的数据
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!
//不校验是否为0xFF，直接写
void W25qxx_WriteBytes_Page(u8* pBuffer, u32 WriteAddr_inBytes, u16 NumByteToWrite)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;
	#if (_W25QXX_DEBUG==1)
	uint32_t	StartTime=HAL_GetTick();
	printf("w25qxx WriteByte 0x%02X at address %d begin...",pBuffer,WriteAddr_inBytes);
	#endif
	W25qxx_WaitForWriteEnd();
    W25qxx_WriteEnable();
    HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
    //#define PAGE_PROG_CMD                        0x02
    W25qxx_Spi(0x02);
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((WriteAddr_inBytes & 0xFF000000) >> 24);
    W25qxx_Spi((WriteAddr_inBytes & 0xFF0000) >> 16);
    W25qxx_Spi((WriteAddr_inBytes & 0xFF00) >> 8);
    W25qxx_Spi(WriteAddr_inBytes & 0xFF);
	
    HAL_SPI_Transmit(&_W25QXX_SPI,pBuffer,NumByteToWrite,100);
    HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
    W25qxx_WaitForWriteEnd();
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx WriteByte done after %d ms\r\n",HAL_GetTick()-StartTime);
	#endif
	w25qxx.Lock=0;
}

//无检验写SPI FLASH 
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能 
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 			 		 
	u16 pageremain;	   
	pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
	while(1)
	{	   
		W25qxx_WriteBytes_Page(pBuffer,WriteAddr,pageremain);	//只能按页写
		if(NumByteToWrite==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
			else pageremain=NumByteToWrite; 	  //不够256个字节了
		}
	};	    
} 

//###################################################################################################################
//根据页号来写
//最大256个字节
//偏移量+要写入的数量不能大于256
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
void 	W25qxx_WritePage(uint8_t *pBuffer	,uint32_t Page_Address,uint32_t OffsetInByte,uint32_t NumByteToWrite_up_to_PageSize)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;
	if(((NumByteToWrite_up_to_PageSize+OffsetInByte)>w25qxx.PageSize)||(NumByteToWrite_up_to_PageSize==0))
		NumByteToWrite_up_to_PageSize=w25qxx.PageSize-OffsetInByte;
	if((OffsetInByte+NumByteToWrite_up_to_PageSize) > w25qxx.PageSize)
		NumByteToWrite_up_to_PageSize = w25qxx.PageSize-OffsetInByte;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx WritePage:%d, Offset:%d ,Writes %d Bytes, begin...\r\n",Page_Address,OffsetInByte,NumByteToWrite_up_to_PageSize);
	W25qxx_Delay(100);
	uint32_t	StartTime=HAL_GetTick();
	#endif	
	W25qxx_WaitForWriteEnd();
  W25qxx_WriteEnable();
  HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
  W25qxx_Spi(0x02);
	Page_Address = (Page_Address*w25qxx.PageSize)+OffsetInByte;	
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((Page_Address & 0xFF000000) >> 24);
  W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
  W25qxx_Spi((Page_Address & 0xFF00) >> 8);
  W25qxx_Spi(Page_Address&0xFF);
	HAL_SPI_Transmit(&_W25QXX_SPI,pBuffer,NumByteToWrite_up_to_PageSize,100);	
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
  W25qxx_WaitForWriteEnd();
	#if (_W25QXX_DEBUG==1)
	StartTime = HAL_GetTick()-StartTime; 
	for(uint32_t i=0;i<NumByteToWrite_up_to_PageSize ; i++)
	{
		if((i%8==0)&&(i>2))
		{
			printf("\r\n");
			W25qxx_Delay(10);			
		}
		printf("0x%02X,",pBuffer[i]);		
	}	
	printf("\r\n");
	printf("w25qxx WritePage done after %d ms\r\n",StartTime);
	W25qxx_Delay(100);
	#endif	
	W25qxx_Delay(1);
	w25qxx.Lock=0;
}
//###################################################################################################################
//按扇区号来写

void 	W25qxx_WriteSector(uint8_t *pBuffer	,uint32_t Sector_Address,uint32_t OffsetInByte	,uint32_t NumByteToWrite_up_to_SectorSize)
{
	if((NumByteToWrite_up_to_SectorSize>w25qxx.SectorSize)||(NumByteToWrite_up_to_SectorSize==0))
		NumByteToWrite_up_to_SectorSize=w25qxx.SectorSize;
	#if (_W25QXX_DEBUG==1)
	printf("+++w25qxx WriteSector:%d, Offset:%d ,Write %d Bytes, begin...\r\n",Sector_Address,OffsetInByte,NumByteToWrite_up_to_SectorSize);
	W25qxx_Delay(100);
	#endif	
	if(OffsetInByte>=w25qxx.SectorSize)
	{
		#if (_W25QXX_DEBUG==1)
		printf("---w25qxx WriteSector Faild!\r\n");
		W25qxx_Delay(100);
		#endif	
		return;
	}	
	uint32_t	StartPage;
	int32_t		BytesToWrite;
	uint32_t	LocalOffset;	
	if((OffsetInByte+NumByteToWrite_up_to_SectorSize) > w25qxx.SectorSize)
		BytesToWrite = w25qxx.SectorSize-OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_SectorSize;	
	StartPage = W25qxx_SectorToPage(Sector_Address)+(OffsetInByte/w25qxx.PageSize);
	LocalOffset = OffsetInByte%w25qxx.PageSize;	
	do
	{		
		W25qxx_WritePage(pBuffer,StartPage,LocalOffset,BytesToWrite);
		StartPage++;
		BytesToWrite-=w25qxx.PageSize-LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset=0;
	}while(BytesToWrite>0);		
	#if (_W25QXX_DEBUG==1)
	printf("---w25qxx WriteSector Done\r\n");
	W25qxx_Delay(100);
	#endif	
}
//###################################################################################################################
void 	W25qxx_WriteBlock	(uint8_t* pBuffer ,uint32_t Block_Address	,uint32_t OffsetInByte	,uint32_t	NumByteToWrite_up_to_BlockSize)
{
	if((NumByteToWrite_up_to_BlockSize>w25qxx.BlockSize)||(NumByteToWrite_up_to_BlockSize==0))
		NumByteToWrite_up_to_BlockSize=w25qxx.BlockSize;
	#if (_W25QXX_DEBUG==1)
	printf("+++w25qxx WriteBlock:%d, Offset:%d ,Write %d Bytes, begin...\r\n",Block_Address,OffsetInByte,NumByteToWrite_up_to_BlockSize);
	W25qxx_Delay(100);
	#endif	
	if(OffsetInByte>=w25qxx.BlockSize)
	{
		#if (_W25QXX_DEBUG==1)
		printf("---w25qxx WriteBlock Faild!\r\n");
		W25qxx_Delay(100);
		#endif	
		return;
	}	
	uint32_t	StartPage;
	int32_t		BytesToWrite;
	uint32_t	LocalOffset;	
	if((OffsetInByte+NumByteToWrite_up_to_BlockSize) > w25qxx.BlockSize)
		BytesToWrite = w25qxx.BlockSize-OffsetInByte;
	else
		BytesToWrite = NumByteToWrite_up_to_BlockSize;	
	StartPage = W25qxx_BlockToPage(Block_Address)+(OffsetInByte/w25qxx.PageSize);
	LocalOffset = OffsetInByte%w25qxx.PageSize;	
	do
	{		
		W25qxx_WritePage(pBuffer,StartPage,LocalOffset,BytesToWrite);
		StartPage++;
		BytesToWrite-=w25qxx.PageSize-LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset=0;
	}while(BytesToWrite>0);		
	#if (_W25QXX_DEBUG==1)
	printf("---w25qxx WriteBlock Done\r\n");
	W25qxx_Delay(100);
	#endif	
}
//###################################################################################################################
void 	W25qxx_ReadByte(uint8_t *pBuffer,uint32_t Bytes_Address)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;
	#if (_W25QXX_DEBUG==1)
	uint32_t	StartTime=HAL_GetTick();
	printf("w25qxx ReadByte at address %d begin...\r\n",Bytes_Address);
	#endif
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
	//快速读命令 0x0B
    W25qxx_Spi(0x0B);
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((Bytes_Address & 0xFF000000) >> 24);
    W25qxx_Spi((Bytes_Address & 0xFF0000) >> 16);
    W25qxx_Spi((Bytes_Address& 0xFF00) >> 8);
    W25qxx_Spi(Bytes_Address & 0xFF);
	W25qxx_Spi(0);
	*pBuffer = W25qxx_Spi(W25QXX_DUMMY_BYTE);
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);	
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx ReadByte 0x%02X done after %d ms\r\n",*pBuffer,HAL_GetTick()-StartTime);
	#endif
	w25qxx.Lock=0;
}
//###################################################################################################################
//读多个字节，最多到整个芯片容量
void W25qxx_ReadBytes(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;
	#if (_W25QXX_DEBUG==1)
	uint32_t	StartTime=HAL_GetTick();
	printf("w25qxx ReadBytes at Address:%d, %d Bytes  begin...\r\n",ReadAddr,NumByteToRead);
	#endif	
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
	W25qxx_Spi(0x0B);
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((ReadAddr & 0xFF000000) >> 24);
    W25qxx_Spi((ReadAddr & 0xFF0000) >> 16);
    W25qxx_Spi((ReadAddr& 0xFF00) >> 8);
    W25qxx_Spi(ReadAddr & 0xFF);
	W25qxx_Spi(0);
	HAL_SPI_Receive(&_W25QXX_SPI,pBuffer,NumByteToRead,2000);	
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
	#if (_W25QXX_DEBUG==1)
	StartTime = HAL_GetTick()-StartTime; 
	for(uint32_t i=0;i<NumByteToRead ; i++)
	{
		if((i%8==0)&&(i>2))
		{
			printf("\r\n");
			W25qxx_Delay(10);
		}
		printf("0x%02X,",pBuffer[i]);		
	}
	printf("\r\n");
	printf("w25qxx ReadBytes done after %d ms\r\n",StartTime);
	W25qxx_Delay(100);
	#endif	
	W25qxx_Delay(1);
	w25qxx.Lock=0;
}
//###################################################################################################################
//按页号来读
//只读本页内，最多256个字节

void 	W25qxx_ReadPage(uint8_t *pBuffer,uint32_t Page_Address,uint32_t OffsetInByte,uint32_t NumByteToRead_up_to_PageSize)
{
	while(w25qxx.Lock==1)
		W25qxx_Delay(1);
	w25qxx.Lock=1;
	if((NumByteToRead_up_to_PageSize>w25qxx.PageSize)||(NumByteToRead_up_to_PageSize==0))
		NumByteToRead_up_to_PageSize=w25qxx.PageSize;
	if((OffsetInByte+NumByteToRead_up_to_PageSize) > w25qxx.PageSize)
		NumByteToRead_up_to_PageSize = w25qxx.PageSize-OffsetInByte;
	#if (_W25QXX_DEBUG==1)
	printf("w25qxx ReadPage:%d, Offset:%d ,Read %d Bytes, begin...\r\n",Page_Address,OffsetInByte,NumByteToRead_up_to_PageSize);
	W25qxx_Delay(100);
	uint32_t	StartTime=HAL_GetTick();
	#endif	
	Page_Address = Page_Address*w25qxx.PageSize+OffsetInByte;
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);
	W25qxx_Spi(0x0B);
	if(w25qxx.ID>=W25Q256)
		W25qxx_Spi((Page_Address & 0xFF000000) >> 24);
    W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
    W25qxx_Spi((Page_Address& 0xFF00) >> 8);
    W25qxx_Spi(Page_Address & 0xFF);
	W25qxx_Spi(0);
	HAL_SPI_Receive(&_W25QXX_SPI,pBuffer,NumByteToRead_up_to_PageSize,100);	
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);
	#if (_W25QXX_DEBUG==1)
	StartTime = HAL_GetTick()-StartTime; 
	for(uint32_t i=0;i<NumByteToRead_up_to_PageSize ; i++)
	{
		if((i%8==0)&&(i>2))
		{
			printf("\r\n");
			W25qxx_Delay(10);
		}
		printf("0x%02X,",pBuffer[i]);		
	}	
	printf("\r\n");
	printf("w25qxx ReadPage done after %d ms\r\n",StartTime);
	W25qxx_Delay(100);
	#endif	
	W25qxx_Delay(1);
	w25qxx.Lock=0;
}
//###################################################################################################################
void 	W25qxx_ReadSector(uint8_t *pBuffer,uint32_t Sector_Address,uint32_t OffsetInByte,uint32_t NumByteToRead_up_to_SectorSize)
{	
	if((NumByteToRead_up_to_SectorSize>w25qxx.SectorSize)||(NumByteToRead_up_to_SectorSize==0))
		NumByteToRead_up_to_SectorSize=w25qxx.SectorSize;
	#if (_W25QXX_DEBUG==1)
	printf("+++w25qxx ReadSector:%d, Offset:%d ,Read %d Bytes, begin...\r\n",Sector_Address,OffsetInByte,NumByteToRead_up_to_SectorSize);
	W25qxx_Delay(100);
	#endif	
	if(OffsetInByte>=w25qxx.SectorSize)
	{
		#if (_W25QXX_DEBUG==1)
		printf("---w25qxx ReadSector Faild!\r\n");
		W25qxx_Delay(100);
		#endif	
		return;
	}	
	uint32_t	StartPage;
	int32_t		BytesToRead;
	uint32_t	LocalOffset;	
	if((OffsetInByte+NumByteToRead_up_to_SectorSize) > w25qxx.SectorSize)
		BytesToRead = w25qxx.SectorSize-OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_SectorSize;	
	StartPage = W25qxx_SectorToPage(Sector_Address)+(OffsetInByte/w25qxx.PageSize);
	LocalOffset = OffsetInByte%w25qxx.PageSize;	
	do
	{		
		W25qxx_ReadPage(pBuffer,StartPage,LocalOffset,BytesToRead);
		StartPage++;
		BytesToRead-=w25qxx.PageSize-LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset=0;
	}while(BytesToRead>0);		
	#if (_W25QXX_DEBUG==1)
	printf("---w25qxx ReadSector Done\r\n");
	W25qxx_Delay(100);
	#endif	
}
//###################################################################################################################
void 	W25qxx_ReadBlock(uint8_t* pBuffer,uint32_t Block_Address,uint32_t OffsetInByte,uint32_t	NumByteToRead_up_to_BlockSize)
{
	if((NumByteToRead_up_to_BlockSize>w25qxx.BlockSize)||(NumByteToRead_up_to_BlockSize==0))
		NumByteToRead_up_to_BlockSize=w25qxx.BlockSize;
	#if (_W25QXX_DEBUG==1)
	printf("+++w25qxx ReadBlock:%d, Offset:%d ,Read %d Bytes, begin...\r\n",Block_Address,OffsetInByte,NumByteToRead_up_to_BlockSize);
	W25qxx_Delay(100);
	#endif	
	if(OffsetInByte>=w25qxx.BlockSize)
	{
		#if (_W25QXX_DEBUG==1)
		printf("w25qxx ReadBlock Faild!\r\n");
		W25qxx_Delay(100);
		#endif	
		return;
	}	
	uint32_t	StartPage;
	int32_t		BytesToRead;
	uint32_t	LocalOffset;	
	if((OffsetInByte+NumByteToRead_up_to_BlockSize) > w25qxx.BlockSize)
		BytesToRead = w25qxx.BlockSize-OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_BlockSize;	
	StartPage = W25qxx_BlockToPage(Block_Address)+(OffsetInByte/w25qxx.PageSize);
	LocalOffset = OffsetInByte%w25qxx.PageSize;	
	do
	{		
		W25qxx_ReadPage(pBuffer,StartPage,LocalOffset,BytesToRead);
		StartPage++;
		BytesToRead-=w25qxx.PageSize-LocalOffset;
		pBuffer += w25qxx.PageSize - LocalOffset;
		LocalOffset=0;
	}while(BytesToRead>0);		
	#if (_W25QXX_DEBUG==1)
	printf("---w25qxx ReadBlock Done\r\n");
	W25qxx_Delay(100);
	#endif	
}
//###################################################################################################################
//写SPI FLASH  
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)						
//NumByteToWrite:要写入的字节数(最大65535)   
//花费的时间比较长：即使写1个字节，也要整个扇区全部读出来，然后每个字节判断是否为0xFF，然后擦除扇区，然后再写入整个扇区
u8 W25QXX_BUFFER[4096];		 
void W25QXX_Write_WithErase(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 
	u32 secpos;
	u16 secoff;
	u16 secremain;	   
 	u16 i;    
	u8 * W25QXX_BUF;	  
   	W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;//扇区地址  
	secoff=WriteAddr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//测试用
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//不大于4096个字节
	while(1) 
	{	
		W25qxx_ReadBytes(W25QXX_BUF,secpos*4096,4096);	//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(W25QXX_BUF[secoff+i]!=0XFF)break;		//判断是否为空  	  
		}
		if(i<secremain)									//需要擦除
		{
			W25qxx_EraseSector(secpos);					//擦除这个扇区
			for(i=0;i<secremain;i++)	   				//复制
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//写入整个扇区  
		}
		else 
			W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间. 	
		
		if(NumByteToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;//扇区地址增1
			secoff=0;//偏移位置为0 	 

		   	pBuffer+=secremain;  //指针偏移
			WriteAddr+=secremain;//写地址偏移	   
		   	NumByteToWrite-=secremain;				//字节数递减
			if(NumByteToWrite>4096)secremain=4096;	//下一个扇区还是写不完
			else secremain=NumByteToWrite;			//下一个扇区可以写完了
		}	 
	};	 
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
*******************************************************************************/
//直接写，不检查是否为空，最多写65535个字节
//花费时间要少一些，省去了读出整个扇区，然后每个字节判断的过程
//需求提前擦除要写的扇区
void SPI_FLASH_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
 u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % SPI_FLASH_PageSize;			//求余，如:5%4=1
  count = SPI_FLASH_PageSize - Addr;					//4-1=3;　本页空闲空间数
  NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;		//26/4=6  整数页
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;	//26%4=2	不足一页的字节数
  
  if(Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */		//一、具有整页的起始点
  {
    if(NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */	//表示要写的字节数不到一页（256个字节）
    {
      W25qxx_WriteBytes_Page(pBuffer, WriteAddr, NumByteToWrite);		//则可以直接写入（不管是否已经擦除，所以在执行这个函数前要先擦除）
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */ 
    {
      while(NumOfPage--)											//如果超过一页
      {
        W25qxx_WriteBytes_Page(pBuffer, WriteAddr, SPI_FLASH_PageSize);	//写整数页
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;  
      }    
     
      W25qxx_WriteBytes_Page(pBuffer, WriteAddr, NumOfSingle);			//写不足一页的剩余部分
    }
  }
  else /* WriteAddr is not SPI_FLASH_PageSize aligned  */			//如果不是从页的起始点开始写
  {
    if(NumOfPage== 0) /* NumByteToWrite < SPI_FLASH_PageSize */		//如果不足一页
    {
	  //如果要写的字节数超出本页剩余空间的字节数
      if(NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */		//此处跟I2C不一样
      {
        temp = NumOfSingle - count;							//count：1.本页空闲字节数
      
        W25qxx_WriteBytes_Page(pBuffer, WriteAddr, count);		//2.写这页指定地址到页尾
        WriteAddr +=  count;
        pBuffer += count; 
        
        W25qxx_WriteBytes_Page(pBuffer, WriteAddr, temp);		//3.紧跟下一页写剩余部分
      }
      else
      {			
        W25qxx_WriteBytes_Page(pBuffer, WriteAddr, NumByteToWrite);	//要写的字节数小于本页剩余字节，可以直接写							
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      NumByteToWrite -= count;									//count：1.本页需要写的剩余数
      NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;			//2.整页数
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;		//3.还需要写的不足一页的剩余数
      
      W25qxx_WriteBytes_Page(pBuffer, WriteAddr, count);				//1.写本页剩余字节（字节数：COUNT）
      WriteAddr +=  count;
      pBuffer += count;  
     
      while(NumOfPage--)
      {
        W25qxx_WriteBytes_Page(pBuffer, WriteAddr, SPI_FLASH_PageSize);	//2.紧跟写整页
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }
      
      if(NumOfSingle != 0)
      {
        W25qxx_WriteBytes_Page(pBuffer, WriteAddr, NumOfSingle);			//3.写不足一页的剩余部分
      }
    }
  }
}


//下面为铁电RAM FM25L16 程序-------------------------------------------------------------------------
//###################################################################################################################
uint8_t	SPI_FLASH_SendByte(uint8_t	Data)
{
	uint8_t	ret;
	HAL_SPI_TransmitReceive(&_W25QXX_SPI,&Data,&ret,1,100);
	return ret;	
}

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
void SPI_FMRAM_BufferWrite(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite)
{
  /* Enable the write access to the FLASH */	//0. 使能FLASH写使能	
  SPI_FLASH_WriteEnable();

  /* Select the FLASH: Chip Select low */			//1. 使能FLASH片选
  B_SelCS=CS_FMRAM1;			//ZCL 2020.3.18		
  SPI_FLASH_CS_LOW();
  /* Send "Write to Memory " instruction */		//2. 发送写指令
  SPI_FLASH_SendByte(WRITE);
  /* Send WriteAddr medium nibble address byte to write to */		//3.发送中间字节地址(没有高字节)
  SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  /* Send WriteAddr low nibble address byte to write to */			//4.发送低字节地址
  SPI_FLASH_SendByte(WriteAddr & 0xFF);

  /* while there is data to be written on the FLASH */
  while (NumByteToWrite--)      // 5. 循环写
  {
    /* Send the current byte */
    SPI_FLASH_SendByte(*pBuffer);								//6. 发送字节
    /* Point on the next byte to be written */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */		//7.禁止FLASH片选
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */				//8. 等待FLASH写结束
  SPI_FLASH_WaitForWriteEnd();				//周成磊记号：写完检查写结束
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
void SPI_FMRAM_BufferRead(u8* pBuffer, u16 ReadAddr, u16 NumByteToRead)
{
  /* Select the FLASH: Chip Select low */
  B_SelCS=CS_FMRAM1;			//ZCL 2020.3.18		
  SPI_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */      //发送读指令
  SPI_FLASH_SendByte(READ);

  /* Send ReadAddr medium nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
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
* Function Name  : SPI_FLASH_WriteEnable		FLASH写使能指令
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WriteEnable(void)
{
  /* Select the FLASH: Chip Select low */		//1. 使能FLASH片选
  SPI_FLASH_CS_LOW();
  
  /* Send "Write Enable" instruction */			//2. 发送WREN指令
  SPI_FLASH_SendByte(WREN);
  
  /* Deselect the FLASH: Chip Select high */	//3. 禁止FLASH片选
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
  
  /* Select the FLASH: Chip Select low */		//1. 使能FLASH片选
  SPI_FLASH_CS_LOW();
  
  /* Send "Read Status Register" instruction */		//2. 发送RDSR指令
  SPI_FLASH_SendByte(RDSR);
  
  /* Loop as long as the memory is busy with a write cycle */
  do
  {
    /* Send a dummy byte to generate the clock needed by the FLASH 
    and put the value of the status register in FLASH_Status variable */
    FLASH_Status = SPI_FLASH_SendByte(Dummy_Byte);		//3. 发送伪字节，读状态寄存器值到FLASH_Status

  } while((FLASH_Status & WIP_Flag) == SET); /* Write in progress */		//FLASH_Status=0,说明写完

  /* Deselect the FLASH: Chip Select high */		//4. 禁止FLASH片选
  SPI_FLASH_CS_HIGH();
}

void SPI_FLASH_CS_LOW(void)						//片选低
{
	if (B_SelCS==CS_Flash1)
		HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET);	
	else if (B_SelCS==CS_FMRAM1)	
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
}

void SPI_FLASH_CS_HIGH(void)					//片选高
{
	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET);		//W25Q16
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);						//FM25L16
}

