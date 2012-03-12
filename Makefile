PRG            = mpmboot
OBJ            = main.o
MCU_TARGET     = atmega32
OPTIMIZE       = -Os

#AVRDUDE_PROG   = -c avr910 -b 115200 -P /dev/ttyUSB2
AVRDUDE_PROG   = -c dragon_isp -P usb

ifeq ($(MCU_TARGET), atmega32)
# hfuse = 0x?A (2k bootloader)
BOOTLOADER_START=0x7800
AVRDUDE_MCU=m32
endif

DEFS           = -DBOOTLOADER_START=$(BOOTLOADER_START) -DOSCCAL_VALUE=0xA8 -DOSCCAL_CHECK=0 -DMPM_ADDRESS=0x11
LIBS           =

# Override is only needed by avr-lib build system.
override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map,--section-start=.text=$(BOOTLOADER_START)

CC             = avr-gcc
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
SIZE           = avr-size

all: $(PRG).elf lst text
	$(SIZE) -x -A $(PRG).elf

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.c $(MAKEFILE_LIST)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf *.o $(PRG).lst $(PRG).map $(PRG).elf $(PRG).hex $(PRG).bin

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

text: hex bin

hex:  $(PRG).hex
bin:  $(PRG).bin

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

install: text
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) -V -U flash:w:$(PRG).hex

fuses:
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) -U hfuse:w:0xda:m
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) -U lfuse:w:0xd4:m

osccal:
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) -U calibration:r:-:h

reset:
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU)
