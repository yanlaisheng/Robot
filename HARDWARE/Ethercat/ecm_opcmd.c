#include "main.h"
#include "ecm_define.h"

#define Printf_ON
uint8_t spi3_SendByte(uint8_t byte);
#define Master_BUSY HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_15)
extern uint8_t			bNetworkMap[42];
struct
{
		uint16_t 	CMD;
		uint16_t	Parm;
		int32_t	  Data1;
	  int32_t	  Data2;
}sbuff[DEF_MA_MAX]; /* Send Buffer */
struct	rbuff_st
{
		uint16_t 	CMD;
		uint16_t	Parm;
		int32_t	  Data1;
		int32_t	  Data2;
}rbuff[DEF_MA_MAX]; /* Receive Buffer */
struct timeval {
  int32_t      tv_sec;
  int32_t      tv_usec;
};

uint8_t  g_u8TEST_COUNTByN = 1;
uint8_t *g_p8SourceData;
uint8_t *g_p8DestinationData;
uint16_t *g_p16SourceData;
uint16_t *g_p16DestinationData;


uint32_t g_au32TimeStamp[10];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;
volatile uint32_t Timer_sec = 0;
volatile uint32_t Timer_usec = 0;

volatile int8_t b_SDOWrite = 0;
volatile int8_t bSPIDMADone = 1;
volatile uint32_t k = 0;
/** Possible EtherCAT slave states */

uint16_t		CMD_Phase[DEF_MA_MAX] = {0};	
int32_t     CMD_Pulse[DEF_MA_MAX] = {0};
int32_t     CMD_Pulse2[DEF_MA_MAX] = {0};
int32_t     CMD_Velocity[DEF_MA_MAX] = {0};
int32_t     CMD_Torque[DEF_MA_MAX] = {0};
int32_t     CMD_Parameter[DEF_MA_MAX] = {0};
uint16_t    SDOch = 0;
uint32_t    CMDindex = 0;
uint32_t    CMDvalue = 0;
uint16_t    LIO_SetData = 0;
uint16_t    LIO_WriteData = 0;
uint16_t    CMD_Parameter_Size[DEF_MA_MAX] = {0};
int32_t     CMD_Parameter_Value[DEF_MA_MAX] = {0};
uint8_t			HomeMethod[DEF_MA_MAX] = {0};
int32_t			HomeOffset[DEF_MA_MAX] = {0};
int32_t			HomeSwitchSpeed[DEF_MA_MAX] = {0};
int32_t			HomeZeroSpeed[DEF_MA_MAX] = {0};	
uint8_t			bHomeEnable[DEF_MA_MAX] = {0};
uint8_t     State = NIC_INIT;
int32_t     OP_Mode[DEF_MA_MAX] = {0};
int32_t     DRIVE_TYPE[DEF_MA_MAX] = {0};
uint32_t		IO_Input = 0;
uint32_t		IO_Output[DEF_MA_MAX] = {0};
uint32_t		IO_Output2[DEF_MA_MAX] = {0};
int8_t      Drive_Axis = 0;
int8_t      IO_Axis = 0;
int16_t     AxisGroup = 0;
uint32_t    Topology1 = 0;
uint32_t    Topology2 = 0;
uint32_t    CycleTime = 0;	//us
uint32_t    DC_Offset = 0;  //us
uint8_t     RetryNum = 0;

uint32_t Output_test = 0;


void send_cmd_make(uint8_t MaxAxis)
{
	uint8_t i;
	for(i=0;i< MaxAxis;i++){
		switch(CMD_Phase[i]){
			case GET_STATUS:			    //0x00
				sbuff[i].CMD = GET_STATUS;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = 0;
				sbuff[i].Data2 = 0;
				break;			
			case SET_STATE:	//0x01
				sbuff[i].CMD = SET_STATE;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = State;
				sbuff[i].Data2 = 0;
				break;			
			case SET_AXIS:	//0x02
				sbuff[i].CMD = SET_AXIS;
				sbuff[i].Parm  = AxisGroup;
				sbuff[i].Data1 = Topology1;
				sbuff[i].Data2 = Topology2;
				break;				
			case SET_DC:	//0x03
				sbuff[i].CMD = SET_DC;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = CycleTime;
				sbuff[i].Data2 = DC_Offset;
				break;			
			case SDO_RD:			//0x07
				sbuff[i].CMD = SDO_RD;
				sbuff[i].Parm  = SDOch;
				sbuff[i].Data1 = CMDindex;
			   sbuff[i].Data2 = 0;
				break;			
			case SDO_WR:			//0x05
				sbuff[i].CMD = SDO_WR;
				sbuff[i].Parm  = SDOch;
				sbuff[i].Data1 = CMDindex;
				sbuff[i].Data2 = CMDvalue;
				break;						
			case DRIVE_MODE:	//0x06
				sbuff[i].CMD = DRIVE_MODE;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = OP_Mode[i];
				sbuff[i].Data2 = DRIVE_TYPE[i];
				break;			
			case ALM_CLR:			//0x10
				sbuff[i].CMD = ALM_CLR;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = 0;
				sbuff[i].Data2 = 0;
				break;			
			case SV_ON:			  //0x11
				sbuff[i].CMD = SV_ON;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = 0;
				sbuff[i].Data2 = 0;
				break;			
			case SV_OFF:			//0x12
				sbuff[i].CMD = SV_OFF;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = 0;
				sbuff[i].Data2 = 0;
				break;
			case IO_RD:			  //0x13
				sbuff[i].CMD = IO_RD;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = 0;
//			  IO_Input	= sbuff[i].Data1;
				sbuff[i].Data2 = 0;
				break;			
			case IO_WR:		  	//0x14
				sbuff[i].CMD = IO_WR;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = IO_Output[i];
				sbuff[i].Data2 = 0;
				break;			
			case CSP:			    //0x15
				sbuff[i].CMD = CSP;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = CMD_Pulse[i];
				sbuff[i].Data2 = 0;
				break;			
			case CSV:			    //0x16
				sbuff[i].CMD = CSV;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = CMD_Velocity[i];
				sbuff[i].Data2 = 0;
			  break;			
			case CST:			    //0x17
				sbuff[i].CMD = CST;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = CMD_Torque[i];
				sbuff[i].Data2 = 0;
				break;			
			case GO_HOME:			  //0x18
				sbuff[i].CMD = GO_HOME;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = 0;
				sbuff[i].Data2 = 0;
				break;
			case LIO_RD:			  //0x21
				sbuff[i].CMD = LIO_RD;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = 0;
				sbuff[i].Data2 = 0;
				break;
			case LIO_WR:			  //0x22
				sbuff[i].CMD = LIO_WR;
				sbuff[i].Parm  = 0;
				sbuff[i].Data1 = LIO_WriteData;
				sbuff[i].Data2 = 0;
				break;
		}
	}
}

void CMD00_GET_STATUS(uint16_t Channel)
{
	CMD_Phase[Channel] = GET_STATUS;
}

void CMD00_ALL_GET_STATUS()
{
	int i;
	for (i=0 ; i<DEF_MA_MAX ; i++){
		CMD00_GET_STATUS(i);
	}
}

void CMD01_SET_STATE(uint8_t state)
{
	uint8_t i;
	CMD_Phase[0] = SET_STATE;
	State = state;
	for(i=1;i< DEF_MA_MAX;i++){
		CMD_Phase[i] = GET_STATUS;
	}
}

void CMD02_SET_AXIS(uint8_t Group, uint8_t* pTopology)
{
	uint8_t i;
	CMD_Phase[0] = SET_AXIS;
	AxisGroup = Group;
	Topology1 = (pTopology[7] << 28) | (pTopology[6] << 24) | (pTopology[5] << 20) | (pTopology[4] << 16) | (pTopology[3] << 12) | (pTopology[2] << 8) | (pTopology[1] << 4) | pTopology[0];
	for(i=1;i< DEF_MA_MAX;i++){
		CMD_Phase[i] = GET_STATUS;
	}
}

void CMD03_SET_DC(uint32_t Time, uint32_t Offset)
{
	uint8_t i;
	CMD_Phase[0] = SET_DC;
	CycleTime = Time;
	DC_Offset = Offset;
	for(i=1;i< DEF_MA_MAX;i++){
		CMD_Phase[i] = GET_STATUS;
	}
}

void CMD06_DRIVE_MODE(uint16_t Channel, uint16_t Mode, uint16_t Type)
{
	CMD_Phase[Channel] = DRIVE_MODE;
	OP_Mode[Channel] = Mode;
	DRIVE_TYPE[Channel] = Type;
	CMD_Phase[0] = GET_STATUS;
	CMD_Phase[SDO_CH] = GET_STATUS;
}

void CMD07_SDO_RD(uint16_t Channel, uint32_t Para)
{
	uint8_t i;
	CMD_Phase[DEF_MA_MAX-1] = SDO_RD;
	SDOch = Channel;
	CMDindex = Para;
	for(i=0;i< (DEF_MA_MAX-1);i++){
		CMD_Phase[i] = GET_STATUS;
	}
}

void CMD08_SDO_WR(uint8_t Channel, uint32_t Para, int8_t Size, int32_t Value)
{
	uint8_t i;
	CMD_Phase[DEF_MA_MAX-1] = SDO_WR;
	SDOch = (Size << 8) | Channel;
	CMDindex = Para;
	CMDvalue = Value;
	for(i=0;i< DEF_MA_MAX-1;i++){
		CMD_Phase[i] = GET_STATUS;
	}
}

void CMD10_ALMCLR(uint16_t Channel)
{
	CMD_Phase[Channel] = ALM_CLR;
	CMD_Phase[0] = GET_STATUS;
	CMD_Phase[SDO_CH] = GET_STATUS;
}

void CMD11_SVON(uint16_t Channel)
{
	CMD_Phase[Channel] = SV_ON;
	CMD_Phase[0] = GET_STATUS;
	CMD_Phase[SDO_CH] = GET_STATUS;
}

void CMD12_SVOFF(uint16_t Channel)
{
	CMD_Phase[Channel] = SV_OFF;
	CMD_Phase[0] = GET_STATUS;
	CMD_Phase[SDO_CH] = GET_STATUS;
}

void CMD13_IORD(uint16_t Channel)
{
	CMD_Phase[Channel] = IO_RD;
	CMD_Phase[0] = GET_STATUS;
	CMD_Phase[SDO_CH] = GET_STATUS;
}

void CMD14_IOWR(uint16_t Channel, uint32_t Output)
{
	CMD_Phase[Channel] = IO_WR;
	IO_Output[Channel] = Output;
	CMD_Phase[0] = GET_STATUS;
	CMD_Phase[SDO_CH] = GET_STATUS;
}

void CMD15_CSP(uint16_t Channel, int32_t Pulse)
{
	CMD_Phase[Channel] = CSP;
	CMD_Pulse[Channel] = Pulse;
	CMD_Phase[0] = GET_STATUS;
	CMD_Phase[SDO_CH] = GET_STATUS;
}

void CMD16_CSV(uint16_t Channel, int32_t Velocity)
{
	CMD_Phase[Channel] = CSV;
	CMD_Velocity[Channel] = Velocity;
	CMD_Phase[0] = GET_STATUS;
	CMD_Phase[SDO_CH] = GET_STATUS;
	CMD_Phase[2] = IO_WR;
	IO_Output[2] = 0Xffffffff;
}

void CMD17_CST(uint16_t Channel, int32_t Torque)
{
	CMD_Phase[Channel] = CST;
	CMD_Torque[Channel] = Torque;
	CMD_Phase[0] = GET_STATUS;
	CMD_Phase[SDO_CH] = GET_STATUS;
}

void CMD18_GO_HOME(uint16_t Channel)
{
	CMD_Phase[Channel] = GO_HOME;
	CMD_Phase[0] = GET_STATUS;
	CMD_Phase[SDO_CH] = GET_STATUS;
}

void CMD21_LIO_RD(void)
{
	int i;
	CMD_Phase[0] = LIO_RD;
	for(i=1;i< DEF_MA_MAX-1;i++){
		CMD_Phase[i] = GET_STATUS;
	}
}

void CMD22_LIO_WR(uint16_t Data)
{
	int i;
	CMD_Phase[0] = LIO_WR;
	LIO_WriteData = Data;
	for(i=1;i< DEF_MA_MAX-1;i++){
		CMD_Phase[i] = GET_STATUS;
	}
}

void SPI_exchange()
{         
	send_cmd_make(DEF_MA_MAX);
	ECMSPIReadWrite((uint8_t *)rbuff, (uint8_t *)sbuff, DEF_MA_MAX*SPI_DG_LEN*4);
}

void SPI_exchange_polling()
{
	send_cmd_make(DEF_MA_MAX);
	while(Master_BUSY);//while(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_15)); //Busy Pin Input status£¬SPI-BUSYÐÅºÅ
	ECMSPIReadWrite((uint8_t *)rbuff, (uint8_t *)sbuff, DEF_MA_MAX*SPI_DG_LEN*4);
//	HAL_Delay(30);
	/*
	for(uint8_t i=1;i< DEF_MA_MAX-1;i++)//¶ÁÈ¡IOÊäÈë×´Ì¬
	{
		if (bNetworkMap[i-1] == IO)
		{
				IO_Input=rbuff[i].Data1;
				#ifndef Printf_ON
				printf("IO_Input =%x\n\r", IO_Input);
				#endif
		}
	}*/
}

void DriveGoHome(void)
{
	uint32_t i;	
	for(i=1;i< DEF_MA_MAX-1;i++){
		CMD08_SDO_WR(i, 0x60600000, 0x1, HOMING_MODE);
		SPI_exchange_polling();
		CMD08_SDO_WR(i, 0x60980000, 0x1, 35);
		SPI_exchange_polling();
		CMD08_SDO_WR(i, 0x607C0000, 0x4, 0); 
		SPI_exchange_polling();
	}
}

uint8_t spi3_SendByte(uint8_t byte)
{
	uint16_t retry = 0;
	while ((SPI3->SR & SPI_FLAG_TXE) == (uint16_t)RESET){
	retry++;
    if (retry > 1000)			//??
      return 0;
	}
  SPI3->DR = byte;
	retry = 0;
  while ((SPI3->SR & SPI_FLAG_RXNE) == (uint16_t)RESET){
	retry++;
    if(retry > 1000)				//??
      return 0;
	}
  return SPI3->DR;
}

