MCU ?= atmega8
PROGRAMMER ?= usbasp
PROGRAMMER_PARAMS ?= -B 10
BIN = ./bin

.PHONY: clean prepare
all: prepare build

build:
	avr-gcc -save-temps=obj -std=c99 -mmcu=$(MCU) -Wall -Os -o $(BIN)/main.elf main.c
	avr-objcopy -j .text -j .data -O ihex $(BIN)/main.elf $(BIN)/main.hex

flash:
		avrdude -e -D -p $(MCU) -c $(PROGRAMMER) -U flash:w:$(BIN)/main.hex $(PROGRAMMER_PARAMS)

test:
	avrdude -p $(MCU) -c $(PROGRAMMER) $(PROGRAMMER_PARAMS)

prepare:
	mkdir -p $(BIN)

clean:
	rm -f $(BIN)/*.{o,elf,hex,i,s}
fuses:
	avrdude -p $(MCU) -c $(PROGRAMMER) -U lfuse:w:0xff:m -U hfuse:w:0xd9:m $(PROGRAMMER_PARAMS)
