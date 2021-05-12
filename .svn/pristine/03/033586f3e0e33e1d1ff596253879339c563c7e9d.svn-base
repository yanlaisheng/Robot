/***************************************************************************/
//#include "delay.h"本文件为WK2XXX系列串口扩展芯片的设备驱动程序,作为驱动参开demo。使用者可以根据自身的情况加以修改，完善。

/***************************************************************************/
#include "wk2xxx.h"
#include "spi.h"
#include "usart.h"
#include "tim.h"
#include "typedef.h"


//void WK2XXX_RST_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};

//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOA_CLK_ENABLE();

//  /*Configure GPIO pin : PtPin */
//  GPIO_InitStruct.Pin = GPIO_PIN_3;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);    

//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); //PA.3 输出高	
//  					 
//}
//void WK2XXX_Reset_Init(void)
//{
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); //1
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); //0
//    delay_ms(10);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); //1	
//    delay_ms(100);
//	
//}
/*************************************************************************/
//函数功能：初始化SPI片选信号CS,并把CS的默认状态设置为高电平
//
//
/*************************************************************************/
void SPI_CS_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;    
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);   
 
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 					 //PA.4 输出高	
}
/*************************************************************************/
//函数功能：初始化SPI总线，设置SPI总线为0模式
/*************************************************************************/
void SPI_BUS_Init(void)
{

	MX_SPI1_Init();		   //初始化SPI

}
/*************************************************************************/
//函数功能：设置CS信号为高电平
/*************************************************************************/
void SPI_CS_H(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
/*************************************************************************/
//函数功能：设置CS信号为低电平
/*************************************************************************/
void SPI_CS_L(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
/*************************************************************************/
//函数功能：初始化SPI接口
/*************************************************************************/
void WK2XXX_SPI_Init(void)
{
	SPI_CS_Init();
	SPI_BUS_Init();	
	
}

/*************************************************************************/
//函数功能：写寄存器函数（前提是该寄存器可写，某些寄存器如果你写1，可能会自动置1，具体见数据手册)
//参数：port:为子串口的数(C0C1)
//      reg:为寄存器的地址(A3A2A1A0)
//      dat:为写入寄存器的数据
//注意：在子串口被打通的情况下，向FDAT写入的数据会通过TX引脚输出
//*************************************************************************/
void Wk2xxxWriteReg(unsigned char port,unsigned char reg,unsigned char dat)
{	 

     SPI_CS_L();//片选使能
	 SPI1_ReadWriteByte(((port-1)<<4)+reg); //写控制字节
	 SPI1_ReadWriteByte(dat); //写数据
	 SPI_CS_H();//片选无效

}


/*************************************************************************/
//函数功能：读寄存器函数
//参数：port:为子串口的数(C0C1)
//      reg:为寄存器的地址(A3A2A1A0)
//      rec_data:为读取到的寄存器值
//注意：在子串口被打通的情况下，读FDAT，实际上就是读取uart的rx接收的数据
/*************************************************************************/
unsigned char Wk2xxxReadReg(unsigned char port,unsigned char reg)
{	
	unsigned char rec_data; 
	SPI_CS_L();	//片选使能
	SPI1_ReadWriteByte(0x40+((port-1)<<4)+reg);//写控制字节，控制命令构成见数据手册
	rec_data=SPI1_ReadWriteByte(0);//接收返回的数据
	SPI_CS_H();	//片选无效	
	return rec_data;
}
/**************************** Wk2xxxWriteFifo*********************************************/
//函数功能：该函数为写FIFO函数，通过该函数写入的数据会直接进入子串口的发送FIFO，然后通过TX引脚发送
//参数：port：为子串口的端口号(C0\C1)
//      *wbuf:写入数据部分
//      len：  写入数据长度
//
/*************************************************************************/
void Wk2xxxWriteFifo(unsigned char port,unsigned char *wbuf,unsigned int len)
{	 unsigned char n;
	 SPI_CS_L(); // 片选有效
	 SPI1_ReadWriteByte(0x80+((port-1)<<4)); //写FIFO控制指令
	  for(n=0;n<len;n++)
	    {
	     SPI1_ReadWriteByte(*(wbuf+n));
		} 
	 SPI_CS_H();	//片选无效

}

/**************************** Wk2xxxReadFifo*********************************************/
//函数功能：该函数为读FIFO函数，通过该函数可以一次读出多个接收FIFO中的数据，最多256个字节
//参数：port：为子串口的端口号(C0\C1)
//      *rbuf:写入数据部分
//      len：  写入数据长度
//
/*************************************************************************/
void Wk2xxxReadFifo(unsigned char port,unsigned char *rbuf,unsigned int len)
{	 unsigned char n;
	 SPI_CS_L();//片选有效
	 SPI1_ReadWriteByte(0xc0+((port-1)<<4));	//写读fifo控制指令
	 for(n=0;n<len;n++)
	   {
		*(rbuf+n)=SPI1_ReadWriteByte(0); 
	   }
	 SPI_CS_H();//片选无效										
	 //return 0;
}

/*************************************************************************/
//函数功能:此函数主要是通过读写wk2xxx的寄存器来判断主接口的通信时序是否有问题
//参数：无
//返回值：rv表示返回值，0成功  
/*************************************************************************/
// unsigned char Wk2xxxTest(void)
// {
//	unsigned char rec_data,rv;
////主接口为SPI	
//	rec_data=Wk2xxxReadReg(WK2XXX_GPORT,WK2XXX_GENA);
//	if(rec_data==0x30)
//		return rv;
//	else
//		{
//			rv=1;
//			return rv;
//		}
//
// }
/******************************Wk2xxxInit*******************************************/
//函数功能：本函数主要会初始化一些芯片基本寄存器；
/*********************************************************************************/
void Wk2xxxInit(unsigned char port)
{
    unsigned char gena,grst,gier,sier,scr,lcr;
	//使能子串口时钟
    gena=Wk2xxxReadReg(WK2XXX_GPORT,WK2XXX_GENA); 
	switch (port)
    {
          case 1://使能子串口1的时钟
              gena|=WK2XXX_UT1EN;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
		  case 2://使能子串口2的时钟
              gena|=WK2XXX_UT2EN;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
		   case 3://使能子串口3的时钟
              gena|=WK2XXX_UT3EN;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
		   case 4://使能子串口4的时钟
              gena|=WK2XXX_UT4EN;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
	 }	
	//软件复位子串口
	grst=Wk2xxxReadReg(WK2XXX_GPORT,WK2XXX_GRST); 
	switch (port)
    {
          case 1://软件复位子串口1
              grst|=WK2XXX_UT1RST;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
		  case 2://软件复位子串口2
              grst|=WK2XXX_UT2RST;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
		   case 3://软件复位子串口3
              grst|=WK2XXX_UT3RST;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
		   case 4://软件复位子串口4
             grst|=WK2XXX_UT4RST;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
	 }	
  //使能子串口中断，包括子串口总中断和子串口内部的接收中断，和设置中断触点
	gier=Wk2xxxReadReg(WK2XXX_GPORT,WK2XXX_GIER); 
	switch (port)
    {
          case 1://软件复位子串口1
              gier|=WK2XXX_UT1RST;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GIER,gier);
              break;
		  case 2://软件复位子串口2
              gier|=WK2XXX_UT2RST;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GIER,gier);
              break;
		   case 3://软件复位子串口3
              gier|=WK2XXX_UT3RST;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GIER,gier);
              break;
		   case 4://软件复位子串口4
              gier|=WK2XXX_UT4RST;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GIER,gier);
              break;
	 }	 
	 //使能子串口接收触点中断和超时中断
	 sier=Wk2xxxReadReg(port,WK2XXX_SIER); 
	 sier |= WK2XXX_RFTRIG_IEN|WK2XXX_RXOVT_IEN;
	 Wk2xxxWriteReg(port,WK2XXX_SIER,sier);
	 // 初始化FIFO和设置固定中断触点
	 Wk2xxxWriteReg(port,WK2XXX_FCR,0XFF);
	 //设置任意中断触点，如果下面的设置有效，那么上面FCR寄存器中断的固定中断触点将失效
	 Wk2xxxWriteReg(port,WK2XXX_SPAGE,1);//切换到page1
	 Wk2xxxWriteReg(port,WK2XXX_RFTL,0X40);//设置接收触点为64个字节
	 Wk2xxxWriteReg(port,WK2XXX_TFTL,0X10);//设置发送触点为16个字节
	 Wk2xxxWriteReg(port,WK2XXX_SPAGE,0);//切换到page0 
	 //使能子串口的发送和接收使能
	 scr=Wk2xxxReadReg(port,WK2XXX_SCR); 
	 scr|=WK2XXX_TXEN|WK2XXX_RXEN;
	 Wk2xxxWriteReg(port,WK2XXX_SCR,scr);
	 
	 //设置校验位
	 lcr=Wk2xxxReadReg(port,WK2XXX_LCR);
	 lcr&=~WK2XXX_STPL;					//=0,1bit
	 lcr&=~WK2XXX_PAEN;					//=0,无校验位(8个数据位)
	 Wk2xxxWriteReg(port,WK2XXX_LCR,lcr);
}

/******************************Wk2xxxClose*******************************************/
//函数功能：本函数会关闭当前子串口，和复位初始值；
/*********************************************************************************/

void Wk2xxxClose(unsigned char port)
{
    unsigned char gena,grst;
	//复位子串口
	grst=Wk2xxxReadReg(WK2XXX_GPORT,WK2XXX_GRST); 
	switch (port)
    {
          case 1://软件复位子串口1
              grst|=WK2XXX_UT1RST;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
		  case 2://软件复位子串口2
              grst|=WK2XXX_UT2RST;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
		   case 3://软件复位子串口3
              grst|=WK2XXX_UT3RST;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
		   case 4://软件复位子串口4
              grst|=WK2XXX_UT4RST;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GRST,grst);
              break;
	 }	
	//关闭子串口时钟
    gena=Wk2xxxReadReg(WK2XXX_GPORT,WK2XXX_GENA); 
	switch (port)
    {
          case 1://使能子串口1的时钟
              gena&=~WK2XXX_UT1EN;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
		  case 2://使能子串口2的时钟
              gena&=~WK2XXX_UT2EN;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
		   case 3://使能子串口3的时钟
              gena&=~WK2XXX_UT3EN;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
		   case 4://使能子串口4的时钟
              gena&=~WK2XXX_UT4EN;
		      Wk2xxxWriteReg(WK2XXX_GPORT,WK2XXX_GENA,gena);
              break;
	 }	
}






/**************************Wk2xxxSetBaud*******************************************************/
//函数功能：设置子串口波特率函数、此函数中波特率的匹配值是根据18.432MHz下的外部晶振计算的
// port:子串口号
// baud:波特率大小.波特率表示方式，
//
/**************************Wk2xxxSetBaud*******************************************************/
void Wk2xxxSetBaud(unsigned char port,int baud)
{  
	unsigned char baud1,baud0,pres,scr;
	//如下波特率相应的寄存器值，是在外部时钟为18.432MHz的情况下计算所得，如果使用其他晶振，需要重新计算
	switch (baud) 
	{
      case B600:
			baud1=0x07;
			baud0=0x7f;
			pres=0;
			break;
      case B1200:
			baud1=0x3;
			baud0=0x3F;
			pres=0;
			break;
      case B2400:
			baud1=0x1;
			baud0=0xdf;
			pres=0;
			break;
      case B4800:
			baud1=0x00;
			baud0=0xef;
			pres=0;
			break;
      case B9600:
			baud1=0x00;
			baud0=0x77;
			pres=0;
			break;
      case B19200:
			baud1=0x00;
			baud0=0x3b;
			pres=0;
			break;
      case B38400:
			baud1=0x00;
			baud0=0x1d;
			pres=0;
			break;
			
      case B76800:
			baud1=0x00;
			baud0=0x0e;
			pres=0;
			break; 
       
      case B1800:
			baud1=0x02;
			baud0=0x7f;
			pres=0;
			break;
      case B3600:
			baud1=0x01;
			baud0=0x3f;
			pres=0;
			break;
      case B7200:
			baud1=0x00;
			baud0=0x9f;
			pres=0;
			break;
      case B14400:
			baud1=0x00;
			baud0=0x4f;
			pres=0;
			break;
      case B28800:
			baud1=0x00;
			baud0=0x27;
			pres=0;
			break;
      case B57600:
			baud1=0x00;
			baud0=0x13;
			pres=0;
			break;
      case B115200:
			baud1=0x00;
			baud0=0x09;
			pres=0;
			break;
      case B230400:
			baud1=0x00;
			baud0=0x04;
			pres=0;
			break;
      default:
			baud1=0x00;
			baud0=0x00;
			pres=0;
    }
	//关掉子串口收发使能
	scr=Wk2xxxReadReg(port,WK2XXX_SCR); 
	Wk2xxxWriteReg(port,WK2XXX_SCR,0);
	//设置波特率相关寄存器
	Wk2xxxWriteReg(port,WK2XXX_SPAGE,1);//切换到page1
	Wk2xxxWriteReg(port,WK2XXX_BAUD1,baud1);
	Wk2xxxWriteReg(port,WK2XXX_BAUD0,baud0);
	Wk2xxxWriteReg(port,WK2XXX_PRES,pres);
	Wk2xxxWriteReg(port,WK2XXX_SPAGE,0);//切换到page0 
	//使能子串口收发使能
	Wk2xxxWriteReg(port,WK2XXX_SCR,scr);
	
	
}
/*****************************Wk2xxxSendBuf****************************************/
//本函数为子串口发送数据的函数，发送数据到子串口的FIFO.然后通过再发送
//参数说明：port：子串口端口号
//          *sendbuf:需要发送的数据buf
//          len：需要发送数据的长度
// 函数返回值：实际成功发送的数据
//说明：调用此函数只是把数据写入子串口的发送FIFO，然后再发送。1、首先确认子串口的发送FIFO有多少数据，根据具体情况、
//确定写入FIFO数据的个数，
/*********************************************************************/
unsigned int Wk2xxxSendBuf(unsigned char port,unsigned char *sendbuf,unsigned int len)
{
	 unsigned int ret,tfcnt,sendlen;
	 unsigned char  fsr;
	  
	 fsr=Wk2xxxReadReg(port,WK2XXX_FSR);
	 if(~fsr&WK2XXX_TFULL )//子串口发送FIFO未满
	 {

	     tfcnt=Wk2xxxReadReg(port,WK2XXX_TFCNT);//读子串口发送fifo中数据个数
		 sendlen=256-tfcnt;//FIFO能写入的最多字节数
		 
		 if(sendlen<len)
		 {
			ret=sendlen; 
			Wk2xxxWriteFifo(port,sendbuf,sendlen);
		 }
		 else
		 {
			 Wk2xxxWriteFifo(port,sendbuf,len);
			 ret=len;
		 }
	  }
	 
	 return ret;
}

/*****************************Wk2xxxGetBuf****************************************/
//本函数为子串口接收数据函数
//参数说明：port：子串口端口号
//          *getbuf:接收到的数据buf
// 函数返回值：实际接收到的数据个数
/*********************************************************************/
unsigned int Wk2xxxGetBuf(unsigned char port,unsigned char *getbuf)
{
	unsigned int ret=0,rfcnt;
	unsigned char fsr;
	fsr=Wk2xxxReadReg(port,WK2XXX_FSR);
	if(fsr&WK2XXX_RDAT )//子串口接收FIFO未空
	  {
	     rfcnt=Wk2xxxReadReg(port,WK2XXX_RFCNT);//读子串口发送fifo中数据个数
		 if(rfcnt==0)//当RFCNT寄存器为0的时候，有两种情况，可能是256或者是0，这个时候通过FSR来判断，如果FSR显示接收FIFO不为空，就为256个字节
		 {rfcnt=256;}
		 Wk2xxxReadFifo(port,getbuf,rfcnt);
		 ret=rfcnt;
	   }
	 return ret;	
}


/**************************WK_RxChars*******************************************/
//函数功能:读取子串口fifo中的数据
// port:端口号
// recbuf:接收到的数据
// 返回值：接收数据的长度
/**************************WK_RxChars********************************************/
int wk_RxChars(u8 port,u8 *recbuf)
{
	u8  fsr=0,rfcnt=0,rfcnt2=0,sifr=0;
  int len=0;
	sifr=Wk2xxxReadReg(port,WK2XXX_SIFR);
	

	if((sifr&WK2XXX_RFTRIG_INT)||(sifr&WK2XXX_RXOVT_INT))//有接收中断和接收超时中断
	{ 
			fsr  =Wk2xxxReadReg(port,WK2XXX_FSR);
			rfcnt=Wk2xxxReadReg(port,WK2XXX_RFCNT);
			rfcnt2=Wk2xxxReadReg(port,WK2XXX_RFCNT);
			//printf("rfcnt=0x%x.\n",rfcnt);
			/*判断fifo中数据个数*/
			if(fsr& WK2XXX_RDAT)
			{ 
				if(!(rfcnt2>=rfcnt))
				{
					rfcnt=rfcnt2;
				}
				len=(rfcnt==0)?256:rfcnt;
			}
		#if 1
			Wk2xxxReadFifo(port,recbuf,len);
		#else
			for(n=0;n<len;n++)
			 *(recbuf+n)=WkReadSReg(port,WK2XXX_FDAT);
		#endif	
			return len;
  }
	else
	{
		len=0;
		return len;
	}
}


/**************************WK_TxChars*******************************************/
//函数功能:通过子串口发送固定长度数据
// port:端口号
// len:单次发送长度不超过256
// 
/**************************WK_TxChars********************************************/
int wk_TxChars(u8 port,int len,u8 *sendbuf)
{
	
#if 1
	Wk2xxxWriteFifo(port,sendbuf,len);//通过fifo方式发送数据
#else
	int num=len;
	for(num=0;num<len;num++)
	{
		WkWriteSReg(port,WK2XXX_FDAT,*(sendbuf+num));
	}
#endif	
	return 0;
}


