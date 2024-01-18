#TOOLCHAIN=~/toolchain/gcc-arm-none-eabi-4_9-2014q4/bin
#PREFIX=$(TOOLCHAIN)/arm-none-eabi-
PREFIX=arm-none-eabi-

ARCHFLAGS=-mthumb -mcpu=cortex-m0plus
COMMONFLAGS=-g3 -Og -Wall -Werror $(ARCHFLAGS) -DCPU_MKL46Z256VMP4

CFLAGS=-I./include -I./drivers -I./utilities -I./BOARD $(COMMONFLAGS) 
LDFLAGS=$(COMMONFLAGS) --specs=nano.specs -Wl,--gc-sections,-Map,$(TARGET).map,-Tlink.ld
LDLIBS=

CC=$(PREFIX)gcc
LD=$(PREFIX)gcc
OBJCOPY=$(PREFIX)objcopy
SIZE=$(PREFIX)size
RM=rm -f

TARGET=main

SRC=$(wildcard *.c drivers/*.c utilities/*.c include/*.c BOARD/*.c) 
OBJ=$(patsubst %.c, %.o, $(SRC))

elf: $(TARGET).elf

clean:
	$(RM) $(TARGET).srec $(TARGET).elf $(TARGET).bin $(TARGET).map $(OBJ)

$(TARGET).elf: $(OBJ)
	$(LD) $(LDFLAGS) $(OBJ) $(LDLIBS) -o $@

flash: elf
	openocd -f openocd.cfg -c "program $(TARGET).elf verify reset exit"