CC	:= avr-gcc
LD	:= avr-ld
OBJCOPY	:= avr-objcopy
OBJDUMP	:= avr-objdump
SIZE	:= avr-size

TARGET = mpmboot
SOURCE = $(wildcard *.c)

# select MCU
MCU = atmega32

AVRDUDE_PROG := -c avr910 -b 115200 -P /dev/ttyUSB0
#AVRDUDE_PROG := -c dragon_isp -P usb

# ---------------------------------------------------------------------------

ifeq ($(MCU), atmega32)
# (8Mhz internal RC-Osz.)
AVRDUDE_MCU=m32
AVRDUDE_FUSES=lfuse:w:0xd4:m hfuse:w:0xda:m

BOOTLOADER_START=0x7800
endif

# ---------------------------------------------------------------------------

CFLAGS = -pipe -g -Os -mmcu=$(MCU) -Wall -fdata-sections -ffunction-sections
CFLAGS += -Wa,-adhlns=$(*F).lst -DBOOTLOADER_START=$(BOOTLOADER_START)
LDFLAGS = -Wl,-Map,$(@:.elf=.map),--cref,--relax,--gc-sections,--section-start=.text=$(BOOTLOADER_START)

CFLAGS += -DOSCCAL_VALUE=0xA8 -DOSCCAL_CHECK=0 -DMPM_ADDRESS=0x11

# ---------------------------------------------------------------------------

$(TARGET): $(TARGET).elf
	@$(SIZE) -B -x --mcu=$(MCU) $<

$(TARGET).elf: $(SOURCE:.c=.o)
	@echo " Linking file:  $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
	@$(OBJDUMP) -h -S $@ > $(@:.elf=.lss)
	@$(OBJCOPY) -j .text -j .data -O ihex $@ $(@:.elf=.hex)
	@$(OBJCOPY) -j .text -j .data -O binary $@ $(@:.elf=.bin)

%.o: %.c $(MAKEFILE_LIST)
	@echo " Building file: $<"
	@$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -rf $(SOURCE:.c=.o) $(SOURCE:.c=.lst) $(addprefix $(TARGET), .elf .map .lss .hex .bin)

install: $(TARGET).elf
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) -U flash:w:$(<:.elf=.hex)

fuses:
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) $(patsubst %,-U %, $(AVRDUDE_FUSES))

osccal:
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) -U calibration:r:-:h

reset:
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU)
