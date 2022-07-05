#include "cms32f033x.h"
#include "main.h"
#include "user.h"

#define RED_LINK		1		//上电红灯闪烁
//#define NO_LINK			1		//上电无闪烁

#define PWR_LED				GPIO4->DO_f.P3			//网络指示灯
#define LINK_LED			GPIO3->DO_f.P6			//氛围灯
#define WIFI_PWR			GPIO4->DO_f.P0			//wifi电源控制

uint8_t rx_buf[PAYLOAD_WIDTH];
static volatile uint32_t U_IntID;
uint8_t F_transmit_0 = 0;
uint16_t P_Receive_Front = 0;

u8 upload_start[8] = {0x55, 0xaa, 0x03, 0x0a, 0x00, 0x01, 0x00, 0x0d};
u8 pkb_result[7] = {0x55, 0xaa, 0x03, 0x0b, 0x00, 0x00, 0x0d};

void send_byte(u8 cmd);
void send_buf(u8 *buf, u8 len);		//传输命令
void user_write_flash(uint32_t begin_addr, u8 *dat, u16 counter);
uint8_t BufferCmp(uint8_t *buf1,uint8_t *buf2,uint16_t bsize);
void FMC_ErasePage(uint32_t PageAddr);
void ReadData(uint32_t addr_start, uint16_t length, uint8_t *rdata, u8 model);  // Write wdata into flash
void WriteData(uint32_t addr_start, uint16_t length, uint8_t *wdata, u8 model);  // Write wdata into flash
uint8_t Checksum(uint8_t *buf, uint16_t len);//获取校验和
uint16_t FMC_CRC(uint32_t CRCStartAddr, uint32_t CRCStopAddr);

//跳转到主程序区执行(boot函数用)
void Jump(void)
{
	FMC->LOCK = FMC_WRITE_KEY;
	FMC->CON |= FMC_CON_ISPS_Msk;	
	FMC->LOCK = 0x00;
	SYS_ResetCPU();
}
void sys_init()
{
	SYS_DisableIOCFGProtect();			/*关闭IOCONFIG写保护*/
	SYS_DisableGPIO0Protect();			/*关闭GPIO0的相关寄存器写保护*/
	SYS_DisableGPIO1Protect();			/*关闭GPIO1的相关寄存器写保护*/
	SYS_DisableGPIO2Protect();			/*关闭GPIO2的相关寄存器写保护*/
	SYS_DisableGPIO3Protect();			/*关闭GPIO3的相关寄存器写保护*/
	SYS_DisableGPIO4Protect();			/*关闭GPIO4的相关寄存器写保护*/
	
	SYS_ConfigHSI(SYS_CLK_HSI_48M);		/*设置内部高速时钟为48Mhz*/
	SYS_EnableHSI();					/*开启高速时钟*/
	SYS_ConfigAHBClock(SYS_CLK_SEL_HSI,SYS_CLK_DIV_1);	/*设置AHB时钟为高速时钟的1分频*/
	SYS_ConfigAPBClock(AHB_CLK_DIV_1);					/*设置APB时钟为AHB时钟的1分频*/
}

	
uint16_t r_wait = 0;

int main(void)
{
/*参数初始化-------------------------------------------------------------*/
	uint8_t flashReadBuf[PAYLOAD_WIDTH];
	uint32_t i=0;

	uint8_t lcmd = 0,lrcsum = 0;
	uint32_t  start_address=0x0;//地址计数
	uint16_t  pkt_count=0;//包号计数
	uint8_t  flag_updata_error=0;
	uint16_t  pkt_number=0;//当前包号
	uint16_t  srclen = 0;
	uint16_t  us=0,ms=0;
	uint8_t *pSrc;
	u8  upload_flag = 0;
	u8  upload_state = 0;

	sys_init();
//	ResetReg();
	uart_init();
	
	pkt_count=0;
	flag_updata_error=0;
// 	start_address=APP_BASE;
    start_address = APP_START_ADDRESS;/*xx替换上文*/
    
	pkt_number=0;
	//total_len=0;
	
	//uint8_t isERASED=0;

	us=0;
	ms=0;
/*-----------------------------------------------------------------------*/
//	memset(rx_buf, 0, sizeof(rx_buf));
//	memset(flashReadBuf, 0, sizeof(flashReadBuf));


//	ReadData(FMC_DAT_START,1, &upload_state, 0);//read
//	if(upload_state > 2)
//	{
//		upload_state = 0;
////		user_write_flash(FMC_DAT_START, &upload_state, 1);
//	}

//	if(upload_state == 0)
//	{
//		Jump();/*xx替换上文*/
//	}
	
	#ifndef NO_LINK
	SYS_SET_IOCFG(IOP32CFG,SYS_IOCFG_P32_GPIO);			/*设置P36为GPIO模式*/
	GPIO_CONFIG_IO_MODE(GPIO2,GPIO_PIN_2,GPIO_MODE_OUTPUT);
	#ifdef RED_LINK
	SYS_SET_IOCFG(IOP36CFG,SYS_IOCFG_P36_GPIO);			/*设置P36为GPIO模式*/
	GPIO_CONFIG_IO_MODE(GPIO3,GPIO_PIN_6,GPIO_MODE_OUTPUT);
	#endif
	#endif

	SYS_SET_IOCFG(IOP43CFG,SYS_IOCFG_P43_GPIO);			/*设置P43为GPIO模式*/
	GPIO_CONFIG_IO_MODE(GPIO4,GPIO_PIN_3,GPIO_MODE_OUTPUT);
	PWR_LED = 0;
	
//	send_byte(0x11);
    while(1) 
	{
		WIFI_PWR = 1;
		#if 1
		us++;
		if(us>30551)
		{
			if(lcmd != 0x0B)
			{
				if(upload_state == 0)
				{
//					send_byte(0x12);
					#ifndef NO_LINK
					LINK_LED = 0;
					#ifdef RED_LINK
					PWR_LED = ~PWR_LED;
					#endif
					#endif
					us=0;
					ms++;
				}
			}
		}
		if(ms > 100)
		{
//			send_byte(0x13);
			Jump();/*xx替换上文*/
		}
		#endif
		
		r_wait++;
		if(r_wait >= 6000)
		{
			r_wait = 0;
			if(P_Receive_Front<36)
			{
				P_Receive_Front = 0;
			}		
		}
		
		if(rx_buf[0] != HEAD0)
		{
			P_Receive_Front = 0;
		}
		
		if(P_Receive_Front == CMD_HEAD_LEN && rx_buf[1] == HEAD1)
		{
			lcmd = rx_buf[3];								//命令号
			srclen = (rx_buf[4] << 8) + rx_buf[5] - 4;		//有效数据长度
			upload_flag = 1;
//			send_byte(0x14);
		}

		if(P_Receive_Front >= (srclen + 11) && upload_flag == 1)
		{
			pkt_number = (rx_buf[8] << 8) + rx_buf[9];		//数据包偏移量
			lrcsum = Checksum(rx_buf, srclen + 10);			//校验码
	
//			send_byte(0x15);
			if(lrcsum == (rx_buf[srclen + 10]))				//校验是否相等
			{
//				send_byte(0x16);
				#ifndef RED_LINK
				SYS_SET_IOCFG(IOP36CFG,SYS_IOCFG_P36_GPIO);			/*设置P36为GPIO模式*/
				GPIO_CONFIG_IO_MODE(GPIO3,GPIO_PIN_6,GPIO_MODE_OUTPUT);
				#endif
				
				LINK_LED = 1;
				
				if(lcmd == 0x0A)			//升级包大小通知
				{
					upload_state = 1;
					user_write_flash(FMC_DAT_START, &upload_state, 1);
					
					start_address = APP_START_ADDRESS;/*xx替换上文*/
					
					//erase
					for(i=APP_START_ADDRESS; i<=APP_END_ADDRESS;i=i+0x200)/*xx替换上文*/  
					{				
						FMC_ErasePage(i);
					}
					pkt_number = 0;
					pkt_count = 0;
					
					send_buf(upload_start, 8);
				}
				else if(lcmd == 0x0B)		//升级包传输
				{
					if((pkt_number == pkt_count) && (flag_updata_error == 0))//通过判断包号可以过滤重复包
					{
//						send_byte(0x19);
						pSrc = rx_buf + 10;
						
//						send_byte((start_address >> 8) & 0xff);
//						send_byte(start_address &0xff);

						WriteData(start_address,srclen,pSrc, 1);
						ReadData(start_address,srclen,flashReadBuf, 1);//read
						if(!BufferCmp(pSrc,flashReadBuf,srclen))//cmp
						{
							send_buf(pkb_result, 7);
							pkt_count+=srclen;

							PWR_LED = ~PWR_LED;

							start_address += srclen;
							if(pkt_number == APP_END_ADDRESS + 1)		//如果收到最后一包
							{
//								for(i=APP_START_ADDRESS; i<=APP_END_ADDRESS;i=i+0x200)/*xx替换上文*/  
//								{				
//									ReadData(i,0x200,flashReadBuf, 1);//read
//									for(ms = 0; ms < 0x200;ms++)
//									{
//										send_byte(flashReadBuf[ms]);
//									}
//								}
//								
////								upload_state = 0;
////								user_write_flash(FMC_DAT_START, &upload_state, 1);
//								us = 65535;
//								while(us--);
								Jump();/*xx替换上文*/
							}
						}
					}
				}
				else
				{
//					send_byte(0x22);
					Jump();/*xx替换上文*/
				}
				P_Receive_Front = 0;
				upload_flag = 0;
			}
			else 
			{
				P_Receive_Front = 0;
				upload_flag = 0;
			}
		}
	}
}

/*****************************************************************************
 ** \brief	FMC_CRC
 **			FMC CRC校验
 ** \param [in] CRCStartAddr:	  CRC校验起始地址
 **				CRCStopAddr:	  CRC校验结束地址
 ** \return  none
 ** \note	 只适用于APROM区
*****************************************************************************/
uint16_t FMC_CRC(uint32_t CRCStartAddr, uint32_t CRCStopAddr)
{
	uint16_t CrcResult;
	FMC->LOCK = FMC_WRITE_KEY;
	FMC->ADR = CRCStartAddr & 0xFFFFFFFC;
	FMC->CRCEA = CRCStopAddr & 0xFFFFFFFC;
	FMC->CRCIN = 0x00;
	FMC->CRCD = 0x00;
	FMC->CMD = FMC_CMD_CRC;
	while((FMC->CON & FMC_CON_BUSY_Msk)==FMC_CON_BUSY_Msk);
	CrcResult = FMC->CRCD;	
	FMC->LOCK = 0x00;
	return CrcResult;
}
void FMC_ErasePage(uint32_t PageAddr)
{
	FMC->LOCK = FMC_WRITE_KEY;
	FMC->ADR = PageAddr;
	while((FMC->CON & FMC_CON_BUSY_Msk)==FMC_CON_BUSY_Msk);
	FMC->CMD = FMC_CMD_PAGE_ERASE;
	while((FMC->CON & FMC_CON_BUSY_Msk)==FMC_CON_BUSY_Msk);
	FMC->LOCK = 0x00;	
}
uint32_t FMC_Read(uint32_t DataAddr)
{
	uint32_t FlashData;
	FMC->LOCK = FMC_WRITE_KEY;
	FMC->ADR = (DataAddr & 0xFFFFFFFC);
	FMC->CMD = FMC_CMD_READ;
	FlashData = FMC->DAT;
	FMC->LOCK = 0x00;
	return FlashData;
}
void FMC_Write(uint32_t DataAddr, uint32_t DataVlue)
{
	FMC->LOCK = FMC_WRITE_KEY;
	FMC->ADR = (DataAddr & 0xFFFFFFFC);
	FMC->DAT = DataVlue;
	while((FMC->CON & FMC_CON_BUSY_Msk)==FMC_CON_BUSY_Msk);
	FMC->CMD = FMC_CMD_WRITE;
	while((FMC->CON & FMC_CON_BUSY_Msk)==FMC_CON_BUSY_Msk);
	FMC->LOCK = 0x00;
}

uint8_t Checksum(uint8_t *buf, uint16_t len)//获取校验和
{
    uint16_t i=0;
    uint8_t c=0;

    do 
	{
        c += buf[i];
		i++;
    }while(i<len);

    return (c);
}


void WriteData(uint32_t addr_start, uint16_t length, uint8_t *wdata, u8 model)  // Write wdata into flash
{
	uint16_t i,j;
	uint32_t data = 0;
	
	for(i = 0; i < length / 4; i++)
	{
		data = (wdata[i * 4 + 3] << 24) | (wdata[i * 4 + 2] << 16) | (wdata[i * 4 + 1] << 8) | wdata[i * 4];
		FMC_Write((addr_start + (i*4)), data);
	}	
	data = 0;
	for(j = 0; j < length % 4; j++)
	{
		data = data + (wdata[i * 4 + (3-j)] << (24 - (j * 8)));
	}	
	if(length % 4 > 0)
	{
		FMC_Write((addr_start + (i*4)), data);
	}
}

void ReadData(uint32_t addr_start, uint16_t length, uint8_t *rdata, u8 model)  // Write wdata into flash
{
	uint16_t i,j;
	uint32_t data = 0;
	
	for(i = 0; i < length / 4; i++)
	{
		data = FMC_Read(addr_start + (i*4));
		*(rdata + (i * 4) + 3) 	= (data >> 24) & 0xFF;
		*(rdata + (i * 4) + 2) 	= (data >> 16) & 0xFF;
		*(rdata + (i * 4) + 1) 	= (data >> 8) & 0xFF;
		*(rdata + (i * 4)) 		= data & 0xFF;
	}
	data = FMC_Read(addr_start + (i*4));
	for(j = 0; j < length % 4; j++)
	{
		*(rdata + (i * 4) + (3-j)) = (data >> (24 - (j * 8))) & 0xFF;
	}		
}

uint8_t BufferCmp(uint8_t *buf1,uint8_t *buf2,uint16_t bsize)
{
	uint16_t i;
	for(i=0;i<bsize;i++)
	{
//		send_byte(i);
		send_byte(buf1[i]);
//		send_byte(buf2[i]);
		
		if(buf1[i] != buf2[i])
		{
			return 1;
		}
	}
	
	return 0;
}

void user_write_flash(uint32_t begin_addr, u8 *dat, u16 counter)
{
	u8 protect_buffer[USED_BYTE_QTY_IN_ONE_SECTOR];
	u16 i;
	uint32_t in_sector_begin_addr = 0;
//	u16 sector_addr = 0;

	memset(protect_buffer,0,sizeof(protect_buffer));
	
	in_sector_begin_addr = begin_addr & 0x01ff;	
	
	ReadData(FMC_DAT_START, USED_BYTE_QTY_IN_ONE_SECTOR, protect_buffer, 0);

	/* 将要写入的数据写入保护缓冲区的相应区域,其余部分保留 */
	for (i = 0; i < counter; i++) 
	{
		protect_buffer[in_sector_begin_addr++] = dat[i];
	}
	
	/* 擦除 要修改/写入 的扇区 */
	FMC_ErasePage(FMC_DAT_START);
	
	/* 将保护缓冲区的数据写入 Data Flash, EEPROM */
	WriteData(FMC_DAT_START, USED_BYTE_QTY_IN_ONE_SECTOR, protect_buffer, 0);		
}


void send_byte(u8 cmd)
{
	F_transmit_0 = 0;
	UART0->THR = cmd;
	while(!F_transmit_0);
	F_transmit_0 = 0;
}
void send_buf(u8 *buf, u8 len)		//传输命令
{
	u8 i;
	
	for (i=0; i<len; i++) 
	{
		send_byte(*(buf+i));
	}
}

void UART0_IRQHandler(void)
{
	U_IntID = (UART0->IIR & 0x0F);		//获取中断事件ID。此处不能打断点，会导致状态位丢失，读取后IIR寄存器值将会改变 
	U_IntID = U_IntID >>1;
	
	switch(U_IntID)								//处理事件
	{
		case 0x1:									//THR 寄存器为空 （发送后 THR的值会自动清除）		
			F_transmit_0 = 1;		
		  break;
					
		case 0x2:									//接收数据有效中断
			r_wait= 0;
			rx_buf[(P_Receive_Front++)] = UART0->RBR;
			break;		
	}	
}

