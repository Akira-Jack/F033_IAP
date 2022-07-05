#ifndef _MAIN_H_
#define _MAIN_H_

#include "cms32f033x.h"
#include <string.h>

///*----------------------------------------------------------------------------
// **FLASH 区域
///*---------------------------------------------------------------------------*/
//#define	FLASH_CODE			(0x00)
//#define	FLASH_DATA			(0x01<< FLASH_MCTRL_MREG_Pos)
///*----------------------------------------------------------------------------
// **FLASH 操作
///*---------------------------------------------------------------------------*/
//#define	FLASH_WRITE			((0x2<< FLASH_MCTRL_MMODE_Pos) | FLASH_MCTRL_MSTART_Msk)
//#define	FLASH_READ			((0x0<< FLASH_MCTRL_MMODE_Pos) | FLASH_MCTRL_MSTART_Msk)
//#define	FLASH_ERASE			((0x3<< FLASH_MCTRL_MMODE_Pos) | FLASH_MCTRL_MSTART_Msk)


//UART0(TXD0=P25 RXD0=P26) 9600bps Fsys=24MHz
/*************主函数**************/
#define PAYLOAD_WIDTH    267
// #define     APP_BASE                0x800
#define APP_START_ADDRESS     0x0000/*xx替换上文*/
#define APP_END_ADDRESS       ((32 - 4) * 1024 - 1)/*xx添加。APP结束地址，当BOOT=4K时*/
#define CMD_HEAD_LEN	 6				//协议头，命令号，长度等的长度
#define HEAD0			 0x55			//协议头0
#define HEAD1			 0xaa			//协议头1

extern uint8_t rx_buf[PAYLOAD_WIDTH];

#define _nop2_()					_nop_(), _nop_()
#define _nop3_()					_nop_(), _nop2_()
#define _nop4_()					_nop_(), _nop3_()	
#define _nop5_()					_nop_(), _nop4_()	
#define _nop6_()					_nop_(), _nop5_()	

/*----------------------------------------------------------------------------
 **FMC 操作区域
-----------------------------------------------------------------------------*/
#define	FMC_APROM_START		(0x00UL)		
#define	FMC_APROM_END		(0xFFFFFFFFUL)				/*Max:64Kb*/

#define	FMC_DAT_START		(0x1C000000UL)
#define	FMC_DAT_END			(0x1C0003FFUL)				/*1Kb*/

#define FLASH_FEED_INFO_ADDR  	FMC_DAT_START + 4
#define FLASH_FUNC_ADDR  		FMC_DAT_START + SECTOR_SIZE

/*----------------------------------------------------------------------------
 **FMC 操作命令
-----------------------------------------------------------------------------*/
#define	FMC_CMD_READ		(0x1U)				
#define FMC_CMD_WRITE		(0x2U)				
#define FMC_CMD_CRC			(0xDU)				/*CRC校验 CRC16-CCITT*/
#define FMC_CMD_PAGE_ERASE		(0x3U)				/*页擦除*/
#define FMC_CMD_ALL_ERASE		(0x6U)				/*整体擦除*/

/*----------------------------------------------------------------------------
 **FMC 解锁
-----------------------------------------------------------------------------*/
#define	FMC_WRITE_KEY	(0x55AA6699)


#define CMD_UPDATE_APROM			0xA0
#define CMD_DATA_PKT				0xA1
#define CMD_LAST_PKT				0xA2
#define CMD_ACK_TRUE				0xA3
#define CMD_ACK_ERROR				0xA4
#define CMD_UPDATE_STOP				0xA5
#define CMD_CONNECT					0xA6
#define CMD_SET_APPINFO     		0xA7
#define CMD_GET_APPINFO     		0xA8
#define CMD_RUN_APROM				0xAB
#define CMD_RUN_LDROM				0xAC
#define CMD_GET_FWVER				0xAE
#define CMD_UPDATE_CONFIG			0xAF

#define CMD_GET_DEVICEID			0xB1

#define CMD_UPDATE_DATAFLASH 		0xC3
#define CMD_WRITE_CHECKSUM 	 		0xC9
#define CMD_GET_FLASHMODE 	 		0xCA

#define CMD_RESEND_PACKET       	0xFF

#define	V6M_AIRCR_VECTKEY_DATA		0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ		0x00000004UL

#define DISCONNECTED				0
#define CONNECTING					1
#define CONNECTED					2


/*****************************************************************************
 ** \brief	GPIO_CONFIG_IO_MODE
 **			设置IO口模式
 ** \param [in] port : GPIO0、GPIO1 、GPIO2、GPIO3、GPIO4
 **				PinNum: GPIO_PIN_0 ~ GPIO_PIN_7
 **				IOMode ：(1)GPIO_MODE_INPUT
 **						 (2)GPIO_MODE_OUTPUT
 **						 (3)GPIO_MODE_OPEN_DRAIN_WITHOUT_PULL_UP
 **						 (4)GPIO_MODE_INPUT_WITH_PULL_UP
 **						 (5)GPIO_MODE_INPUT_WITH_PULL_DOWN
 ** \return  none
 ** \note	 
*****************************************************************************/
#define   GPIO_CONFIG_IO_MODE(port,PinNum ,IOMode)  do{\
														port->PMS &= ~(GPIO_PMS_PMS0_Msk <<(PinNum*4));\
														port->PMS |= (IOMode <<(PinNum*4));\
													 }while(0)

													 
#define		MOVC_ShiftAddress   0x00			//IAP地址初始位置
#define		SECTOR_SIZE         512				//扇区大小
#define		SECTOR_AMOUNT       2				//扇区个数

//#define USED_BYTE_QTY_IN_ONE_SECTOR     128		//最大存储字节(中微实际512)
#define USED_BYTE_QTY_IN_ONE_SECTOR   256
//#define USED_BYTE_QTY_IN_ONE_SECTOR   512

//#define		FEED_ALL_NUM   0x29			//IAP地址初始位置
//#define		LOCK_KEY	   0x30			//锁标志地址

//#define FLASH_FEED_INFO_ADDR  	MOVC_ShiftAddress
//#define FLASH_FUNC_ADDR  		0x200


//void FMC_ErasPage(uint16_t pageaddress);
//uint8_t Checksum(uint8_t *buf, uint8_t len);//获取校验和;
//void WriteData(uint16_t addr_start, uint8_t length, uint8_t *wdata);  // Write wdata into flash


#endif
