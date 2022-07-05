#include "cms32f033x.h"
#include "user.h"

void UART_ConfigRunMode(UART_T* uart,uint32_t Baudrate,uint32_t WordLength, 
						uint32_t Parity, uint32_t StopBit)
{
	uart->LCR &= ~(UART_LCR_WLS_Msk | UART_LCR_SBS_Msk|UART_LCR_PEN_Msk|UART_LCR_PSEL_Msk);
	uart->LCR |= WordLength | Parity | StopBit;
	uart->DLR = SystemAPBClock /16/Baudrate;
}

void uart_init(void)
{
	uint32_t  BuadRate =9600;
	/*
	(1)设置UARTx模式
	*/
	UART_ConfigRunMode(UART0, BuadRate, UART_WLS_8, UART_PARITY_NONE,UART_STOP_BIT_1);
	/*
	(2)开启UARTx时钟
	*/	
	SYS_EnablePeripheralClk(SYS_CLK_UART0_MSK);
	
	UART0->EFR &= ~(UART_EFR_AUTOIEN_Msk);
	UART0->IER |= UART_IER_RBRIE_Msk;
	UART0->IER |= UART_IER_THREIE_Msk;
	
	/*
	(3)开启UARTx输出
	*/	
	SYS_SET_IOCFG(IOP16CFG, SYS_IOCFG_P16_RXD0);	
	SYS_SET_IOCFG(IOP17CFG, SYS_IOCFG_P17_TXD0);

	NVIC_SetPriority(UART0_IRQn,3);	
	NVIC_EnableIRQ(UART0_IRQn); 
	
//	UART_ConfigRunMode(UART1, BuadRate, UART_WLS_8, UART_PARITY_NONE,UART_STOP_BIT_1);
//	/*
//	(2)开启UARTx时钟
//	*/	
//	SYS_EnablePeripheralClk(SYS_CLK_UART1_MSK);
//	/*
//	(3)开启UARTx输出
//	*/	
//	SYS_SET_IOCFG(IOP47CFG, SYS_IOCFG_P47_TXD1);

}

//_ARMABI int (putchar)(int ch)
//{
//	UART1->THR = ch;					//开始发送数据
//	while( !(((UART1->IIR & 0x0F) >> 1) & (1<<1)));
//	return 0;
//}
