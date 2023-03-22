TARGET = main
BUILD_DIR = build

# Define the linker script location and chip architecture.
LD_SCRIPT = STM32F072.ld
MCU_SPEC  = cortex-m0

# TODO, replace your gcc toolchain for embedded system
TOOLCHAIN = /usr
CC = $(TOOLCHAIN)/bin/arm-none-eabi-gcc
AS = $(TOOLCHAIN)/bin/arm-none-eabi-as
LD = $(TOOLCHAIN)/bin/arm-none-eabi-ld
OC = $(TOOLCHAIN)/bin/arm-none-eabi-objcopy
OD = $(TOOLCHAIN)/bin/arm-none-eabi-objdump
OS = $(TOOLCHAIN)/bin/arm-none-eabi-size

# Assembly directives.
ASFLAGS = -c -O0 -mcpu=$(MCU_SPEC) -mthumb -Wall -fmessage-length=0

# C compilation directives
CFLAGS = -mcpu=$(MCU_SPEC) -mthumb -Wall -g -fmessage-length=0 --specs=nosys.specs

# Linker directives
LSCRIPT = ./ld/$(LD_SCRIPT)
LFLAGS += -mcpu=$(MCU_SPEC) -mthumb -Wall --specs=nosys.specs -nostdlib -lgcc -T$(LSCRIPT)

AS_SRC   = ./src/boot.s
C_SRC    =  $(wildcard ./src/*.c)
INCLUDE  =  -I./ -I./device_headers
OBJS  = $(AS_SRC:.s=.o)
OBJS += $(C_SRC:.c=.o)

.PHONY: all clean 

all: $(BUILD_DIR)/$(TARGET).bin
	mkdir -p $(BUILD_DIR)

%.o: %.s
	$(CC) -x assembler-with-cpp $(ASFLAGS) $< -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $(INCLUDE) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	mkdir -p $(@D)
	$(CC) $^ $(LFLAGS) -o $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	mkdir -p $(@D)
	$(OC) -S -O binary $< $@
	$(OS) $<

flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash write $(BUILD_DIR)/$(TARGET).bin 0x8000000 

test: src/Sensor/test/filter_test.c
	mkdir -p test
	gcc src/Sensor/test/filter_test.c -o test/filter_test.o
	test/filter_test.o

clean:
	rm -rf build
	rm -rf test
	rm -f $(OBJS)