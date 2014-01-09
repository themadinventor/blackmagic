#include "platform.h"
#include "gdb_if.h"

#include <libopencm3/lm4f/rcc.h>
#include <libopencm3/lm4f/nvic.h>
#include <libopencm3/lm4f/uart.h>
#include <libopencm3/cm3/systick.h>

#define SYSTICKHZ	10
#define SYSTICKMS	(1000 / SYSTICKHZ)

#define PLL_DIV_80MHZ	5

jmp_buf fatal_error_jmpbuf;
uint8_t running_status;
volatile uint32_t timeout_counter;

static void morse_update(void);

/* Morse code patterns and lengths */
static const struct {
	uint16_t code;
	uint8_t bits;
} morse_letter[] = {
	{        0b00011101,  8}, // 'A' .-
	{    0b000101010111, 12}, // 'B' -...
	{  0b00010111010111, 14}, // 'C' -.-.
	{      0b0001010111, 10}, // 'D' -..
	{            0b0001,  4}, // 'E' .
	{    0b000101110101, 12}, // 'F' ..-.
	{    0b000101110111, 12}, // 'G' --.
	{      0b0001010101, 10}, // 'H' ....
	{          0b000101,  6}, // 'I' ..
	{0b0001110111011101, 16}, // 'J' .---
	{    0b000111010111, 12}, // 'K' -.-
	{    0b000101011101, 12}, // 'L' .-..
	{      0b0001110111, 10}, // 'M' --
	{        0b00010111,  8}, // 'N' -.
	{  0b00011101110111, 14}, // 'O' ---
	{  0b00010111011101, 14}, // 'P' .--.
	{0b0001110101110111, 16}, // 'Q' --.-
	{      0b0001011101, 10}, // 'R' .-.
	{        0b00010101,  8}, // 'S' ...
	{          0b000111,  6}, // 'T' -
	{      0b0001110101, 10}, // 'U' ..-
	{    0b000111010101, 12}, // 'V' ...-
	{    0b000111011101, 12}, // 'W' .--
	{  0b00011101010111, 14}, // 'X' -..-
	{0b0001110111010111, 16}, // 'Y' -.--
	{  0b00010101110111, 14}, // 'Z' --..
};

const char *morse_msg;
static const char * volatile morse_ptr;
static char morse_repeat;

void morse(const char *msg, char repeat)
{
	morse_msg = morse_ptr = msg;
	morse_repeat = repeat;
	SET_ERROR_STATE(0);
}

static void morse_update(void)
{
	static uint16_t code;
	static uint8_t bits;

	if(!morse_ptr) return;

	if(!bits) {
		char c = *morse_ptr++;
		if(!c) {
			if(morse_repeat) {
				morse_ptr = morse_msg;
				c = *morse_ptr++;
			} else {
				morse_ptr = 0;
				return;
			}
		}
		if((c >= 'A') && (c <= 'Z')) {
			c -= 'A';
			code = morse_letter[c].code;
			bits = morse_letter[c].bits;
		} else {
			code = 0; bits = 4;
		}
	}
	SET_ERROR_STATE(code & 1);
	code >>= 1; bits--;
}


void sys_tick_handler(void)
{
	if(running_status) {
		gpio_toggle(LED_PORT, LED_IDLE_RUN);
	}

	if(timeout_counter)
		timeout_counter--;

	morse_update();
}


/*static void
uart_init(void)
{
	periph_clock_enable(RCC_GPIOA);
	periph_clock_enable(RCC_UART0);

	// Three cycles delay
	__asm__("nop"); __asm__("nop"); __asm__("nop");

	gpio_set_af(GPIOA_APB_BASE, 0x1, GPIO0 | GPIO1);
	gpio_mode_setup(GPIOA_APB_BASE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOA_APB_BASE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);

	uart_disable(UART0);

	uart_clock_from_sysclk(UART0);
	uart_set_baudrate(UART0, 115200);
	uart_set_databits(UART0, 8);
	uart_set_stopbits(UART0, 1);
	uart_set_parity(UART0, UART_PARITY_NONE);

	// Enable FIFO
	uart_enable_fifo(UART0);

	// Set FIFO interrupt trigger levels to 1/8 full for RX buffer and
	// 7/8 empty (1/8 full) for TX buffer
	uart_set_fifo_trigger_levels(UART0, UART_FIFO_RX_TRIG_1_8, UART_FIFO_TX_TRIG_7_8);

	uart_clear_interrupt_flag(UART0, UART_INT_RX | UART_INT_RT);
	uart_clear_interrupt_flag(UART0, UART_INT_TX);

	uart_enable_interrupts(UART0, UART_INT_RX| UART_INT_RT);
	uart_enable_interrupts(UART0, UART_INT_TX);
	uart_enable(UART0);
}*/


int
platform_init(void)
{
        int i;
        for(i=0; i<1000000; i++);

	rcc_sysclk_config(OSCSRC_MOSC, XTAL_16M, PLL_DIV_80MHZ);
	
	//uart_init();

	// Enable LED
	periph_clock_enable(RCC_GPIOF);
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_IDLE_RUN);
	gpio_set_output_config(LED_PORT, GPIO_OTYPE_PP, GPIO_DRIVE_2MA, LED_IDLE_RUN);
	//gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_ERROR);
	//gpio_set_output_config(LED_PORT, GPIO_OTYPE_PP, GPIO_DRIVE_2MA, LED_ERROR);

	// Enable all JTAG ports and set pins to output
	periph_clock_enable(RCC_GPIOA);
	periph_clock_enable(RCC_GPIOB);
	periph_clock_enable(RCC_GPIOE);

	gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TMS_PIN);
	gpio_mode_setup(TCK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TCK_PIN);
	gpio_mode_setup(TDI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TDI_PIN);
	gpio_mode_setup(TDO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TDO_PIN);
	gpio_mode_setup(TRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRST_PIN);
	gpio_mode_setup(SRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SRST_PIN);

	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);
	systick_set_reload(rcc_get_system_clock_frequency() / (SYSTICKHZ * 8));

	systick_interrupt_enable();
	systick_counter_enable();

	nvic_enable_irq(NVIC_SYSTICK_IRQ);
	nvic_enable_irq(NVIC_UART0_IRQ);

	usbuart_init();
	cdcacm_init();

	jtag_scan(NULL);

	return 0;
}

void platform_delay(uint32_t delay)
{
	timeout_counter = delay;
	while(timeout_counter);
}

const char *platform_target_voltage(void)
{
	return "not supported";
}

void assert_boot_pin(void)
{
	// TODO
}
