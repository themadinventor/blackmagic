#include "gdb_if.h"
#include "platform.h"

#include <libopencm3/lm4f/uart.h>

#define UART_RX_BUF_SIZE 64
volatile static uint8_t uartRxBuf[UART_RX_BUF_SIZE];
volatile static uint8_t uartRxRead;
volatile static uint8_t uartRxWrite;
volatile static uint8_t uartRxUsed;

#define UART_TX_BUF_SIZE 64
volatile static uint8_t uartTxBuf[UART_TX_BUF_SIZE];
volatile static uint8_t uartTxRead;
volatile static uint8_t uartTxWrite;
volatile static uint8_t uartTxUsed;

static void uart_transmit(void)
{
	uart_disable_interrupts(UART0, UART_INT_TX);
	while( !uart_is_tx_fifo_full(UART0) && uartTxUsed > 0) {
		uartTxUsed--;
		uart_send(UART0, uartTxBuf[uartTxRead]);
		uartTxRead = (uartTxRead + 1) % UART_TX_BUF_SIZE;
	}
	uart_enable_interrupts(UART0, UART_INT_TX);
}

void uart0_isr(void)
{
	uint32_t clear_irq = 0;

	if( uart_is_interrupt_source(UART0, UART_INT_RX) || uart_is_interrupt_source(UART0, UART_INT_RT) ) {
		while( !uart_is_rx_fifo_empty(UART0) ) {
			uint8_t ch = uart_recv(UART0);
			if( uartRxUsed < UART_RX_BUF_SIZE) {
				uartRxBuf[uartRxWrite] = ch;
				uartRxWrite = (uartRxWrite + 1) % UART_RX_BUF_SIZE;
				uartRxUsed++;
			}
		}
		if( uart_is_interrupt_source(UART0, UART_INT_RX))
			clear_irq |= UART_INT_RX;
		if( uart_is_interrupt_source(UART0, UART_INT_RT))
			clear_irq |= UART_INT_RT;
	}

	if( uart_is_interrupt_source(UART0, UART_INT_TX) ) {
		uart_transmit();
		clear_irq |= UART_INT_TX;
	}

	uart_clear_interrupt_flag(UART0, clear_irq);
}

void
put_str(const char *s) {
	char *p = s;
	while( *p != '\0' ) {
		gdb_if_putchar(*p, 1);
		p++;
	}
}

int
gdb_if_init(void) 
{
	uartRxRead = uartRxWrite = uartRxUsed = 0;
	uartTxRead = uartTxWrite = uartTxUsed = 0; 

	return 0;
}

unsigned char
gdb_if_getchar(void)
{
	while(uartRxUsed == 0);
	unsigned char ch = uartRxBuf[uartRxRead];
	uartRxUsed--;
	uartRxRead = (uartRxRead + 1) % UART_RX_BUF_SIZE;
	return ch;
}

unsigned char
gdb_if_getchar_to(int timeout)
{
	timeout_counter = timeout/100;
	do {
		if( uartRxUsed > 0) {
			return gdb_if_getchar();
		}
	} while(timeout_counter);

	return -1;
}

void
gdb_if_putchar(unsigned char c, int flush)
{
	while(uartTxUsed == UART_TX_BUF_SIZE) {};
	
	uartTxBuf[uartTxWrite] = c;
	uartTxUsed++;
	uartTxWrite = (uartTxWrite+1) % UART_TX_BUF_SIZE;

	if( flush || uartTxUsed == UART_TX_BUF_SIZE ) {
		uart_transmit();
		while(uartTxUsed > 0 ) {}
	}
}
