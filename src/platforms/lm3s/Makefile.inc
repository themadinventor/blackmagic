CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

STELLARIS = /home/pf/opt/stellaris-launchpad
INCLUDES = -I../libopencm3/include

CPU_FLAGS = -mcpu=cortex-m3 -mthumb
CFLAGS += $(INCLUDES) $(CPU_FLAGS) -DLM3S

LINKER_SCRIPT="platforms/lm3s/lm3s.ld"
LDFLAGS = -nostartfiles -lc $(CPU_FLAGS) -nodefaultlibs -T$(LINKER_SCRIPT) -Wl,--gc-sections \
	-L../libopencm3/lib -lopencm3_lm3s -lnosys -lm -lgcc
	
VPATH += platforms/lm3s

SRC +=	cdcacm.c	\
	usbuart.c

all: blackmagic.bin

blackmagic.bin: blackmagic
	$(OBJCOPY) -O binary $^ $@
