#ifndef __ECM_DEFINE_H
#define __ECM_DEFINE_H

#include "stdint.h"
enum
{
   NIC_INIT             =			0x00,
   STATE_INIT           =			0x01,/* Init state */
   STATE_PRE_OP         =			0x02,/* Pre-operational */
   STATE_SAFE_OP        =			0x04,/* Safe-operational */
   STATE_OPERATIONAL    =			0x08,/* Operational */
};

#define CYCLETIME   					8000
#define DC_OFFSET   					0xffff

#define DEF_MA_MAX   					42 /*32 for 16Byte*/
#define SDO_CH     						(DEF_MA_MAX-1)
#define SPI_DG_LEN   					3 /*4 for 16Byte*/

#define GET_STATUS     					0x00
#define SET_STATE   					0x01
#define SET_AXIS    					0x02
#define SET_DC      					0x03
#define DRIVE_MODE  					0x06
#define SDO_RD      					0x07
#define SDO_WR      					0x08
#define ALM_CLR     					0x10
#define SV_ON       					0x11
#define SV_OFF      					0x12
#define IO_RD       					0x13
#define IO_WR       					0x14
#define CSP         					0x15
#define CSV         					0x16
#define CST         					0x17
#define GO_HOME     					0x18
#define ABORT_HOME 						0x19
#define LIO_RD 							0x21
#define LIO_WR 							0x22
#define HOMING_MODE 					6
#define CSP_MODE    					8
#define CSV_MODE    					9
#define CST_MODE    					10
#define FREERUN     					0x00
#define DCSYNC      					0x01
#define STEP        					100
#define DRIVE       					0
#define IO          					1
#define HSP         					2
#define None        					0xF
void Motion(void);
int ECMSPIReadWrite(uint8_t *rdata, uint8_t *wdata, int rwlength);
void send_cmd_make(uint8_t MaxAxis);
void CMD00_GET_STATUS(uint16_t Channel);
void CMD00_ALL_GET_STATUS(void);
void CMD01_SET_STATE(uint8_t state);
void CMD02_SET_AXIS(uint8_t Group, uint8_t* pTopology);
void CMD03_SET_DC(uint32_t Time, uint32_t Offset);
void CMD06_DRIVE_MODE(uint16_t Channel, uint16_t Mode, uint16_t Type);
void CMD07_SDO_RD(uint16_t Channel, uint32_t Para);
void CMD08_SDO_WR(uint8_t Channel, uint32_t Para, int8_t Size, int32_t Value);
void CMD10_ALMCLR(uint16_t Channel);
void CMD11_SVON(uint16_t Channel);
void CMD12_SVOFF(uint16_t Channel);
void CMD13_IORD(uint16_t Channel);
void CMD14_IOWR(uint16_t Channel, uint32_t Output);
void CMD15_CSP(uint16_t Channel, int32_t Pulse);
void CMD16_CSV(uint16_t Channel, int32_t Velocity);
void CMD17_CST(uint16_t Channel, int32_t Torque);
void CMD18_GO_HOME(uint16_t Channel);
void CMD21_LIO_RD(void);
void CMD22_LIO_WR(uint16_t Data);
void SPI_exchange(void);
void SPI_exchange_polling(void);
void Motion_Traj_Gen(void);
void DriveGoHome(void);


extern uint8_t spi3_SendByte(uint8_t byte);
#endif
