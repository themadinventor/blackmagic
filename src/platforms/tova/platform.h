#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <stdint.h>

#include <setjmp.h>
#include <alloca.h>

#include <libopencm3/lm4f/gpio.h>

#include "gdb_packet.h"

extern jmp_buf fatal_error_jmpbuf;
extern uint8_t running_status;
extern const char *morse_msg;
extern volatile uint32_t timeout_counter;

#define LED_PORT	GPIOF_APB_BASE
#define LED_IDLE_RUN	GPIO4
//#define LED_ERROR	GPIO3

#define TMS_PORT	GPIOB_APB_BASE
#define TMS_PIN		GPIO5

#define TCK_PORT	GPIOB_APB_BASE
#define TCK_PIN		GPIO0

#define TDI_PORT	GPIOB_APB_BASE
#define TDI_PIN		GPIO1

#define TDO_PORT	GPIOE_APB_BASE
#define TDO_PIN		GPIO4

#define SWDIO_PORT	TMS_PORT
#define SWDIO_PIN	TMS_PIN

#define SWCLK_PORT	TCK_PORT
#define SWCLK_PIN	TCK_PIN

#define TRST_PORT	GPIOA_APB_BASE
#define TRST_PIN	GPIO5

#define SRST_PORT	GPIOA_APB_BASE
#define SRST_PIN	GPIO6

#define TMS_SET_MODE()	{								\
	gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TMS_PIN);		\
	gpio_set_output_config(TMS_PORT, GPIO_OTYPE_PP, GPIO_DRIVE_2MA, TMS_PIN);	\
}

#define SWDIO_MODE_FLOAT() {								\
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SWDIO_PIN);	\
}

#define SWDIO_MODE_DRIVE() {									\
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SWDIO_PIN);		\
	gpio_set_output_config(SWDIO_PORT, GPIO_OTYPE_PP, GPIO_DRIVE_2MA, SWDIO_PIN);		\
}

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf

#define DEBUG(...)

#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, LED_IDLE_RUN, state);}
//#define SET_ERROR_STATE(state)	{} //{gpio_set_val(LED_PORT, LED_ERROR, state);}
#define SET_ERROR_STATE(state)	SET_IDLE_STATE(state)

#define PLATFORM_SET_FATAL_ERROR_RECOVERY()	{setjmp(fatal_error_jmpbuf);}
#define PLATFORM_FATAL_ERROR(error) {			\
	if( running_status ) gdb_putpacketz("X1D");	\
		else gdb_putpacketz("EFF");		\
	running_status = 0;				\
	target_list_free();				\
	morse("TARGET LOST.", 1);			\
	longjmp(fatal_error_jmpbuf, (error));		\
}

int platform_init(void);
void morse(const char *msg, char repeat);

inline static void gpio_set_val(uint32_t port, uint8_t pin, uint8_t val) {
	gpio_write(port, pin, val == 0 ? 0 : 0xff);
}

inline static uint8_t gpio_get(uint32_t port, uint8_t pin) {
	return !(gpio_read(port, pin) == 0);
}

void platform_delay(uint32_t delay);

#endif
