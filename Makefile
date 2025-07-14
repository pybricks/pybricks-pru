CC = pru-gcc
OBJCOPY = pru-objcopy
BUILD_TARGET = pru_ledpwm

ASFLAGS += -g
CFLAGS += -g -Wall -Wno-main -Os
LDFLAGS += -g -nostdlib -mmcu=am1808.pru1.specs

.PHONY: all clean

all: $(BUILD_TARGET).bin

# Manual dependency tracking
main.o: am1808.h

$(BUILD_TARGET).bin: $(BUILD_TARGET).elf

$(BUILD_TARGET).elf: start.o main.o
	$(CC) $(LDFLAGS) -o $@ $+

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf *.o *.elf *.bin
