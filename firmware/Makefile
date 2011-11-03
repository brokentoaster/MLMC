# Name: Makefile
# Author: <insert your name here>
# Copyright: <insert your copyright message here>
# License: <insert your license reference here>

# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

DEVICE     = attiny261
CLOCK      = 16000000
#PROGRAMMER = -c avrisp2 -P /dev/cu.usbserial-ER051111
PROGRAMMER = -c avrisp2 -P /dev/ttyUSB1
OBJECTS    = main.o
FUSES      = -U hfuse:w:0xdf:m -U lfuse:w:0xe2:m

# ATtiny261 fuse bits (fuse bits for other devices are different!):
# Example for 8 MHz internal oscillator


# Fuse ext byte:
# 0xff = 1 1 1 1   1 1 1 0 <-- SELFPRGEN (set to 0 to enable self programming )  

# Fuse high byte:
# 0xdf = 1 1 0 1   1 1 1 1 <-- BODLEVEL0 (if 0: 4V, if 1: 2.7V)
#        ^ ^ ^ ^   ^ ^ ^------ BODLEVEL1 (if 0: 4V, if 1: 2.7V)
#        | | | |   | +-------- BODLEVEL2 (if 0: 4V, if 1: 2.7V)\
#        | | | |   +---------- EESAVE (set to 0 to preserve EEPROM over chip erase)
#        | | | +-------------- WDTON (if set to 0, watchdog is always on)
#        | | +---------------- SPIEN (if set to 1, serial programming is disabled)
#        | +------------------ DWIEN (if set to 1, serial programming is disabled)
#        +-------------------- RSTDISBL (if set to 0, RESET pin is disabled)

# Fuse low byte:
# 0xe2 = 1 1 1 0   0 0 1 0 
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (8M internal RC)
#        | |  +--------------- SUT 1..0 (crystal osc, BOD enabled)
#        | +------------------ CKOUT (if 0, brown-out detector is enabled)
#        +-------------------- CKDIV8 (if set to 0, clock divided to 8)


# Fuse low byte:
# 0xe1 = 1 1 1 0   0 0 0 1 
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (16M internal RC and PLL)
#        | |  +--------------- SUT 1..0 (crystal osc, BOD enabled)
#        | +------------------ CKOUT (if 0, brown-out detector is enabled)
#        +-------------------- CKDIV8 (if set to 0, clock divided to 8)



# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -Wa,-ahls=main.lst -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -g

# symbolic targets:
all:	main.hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID main.hex

clean:
	rm -f main.hex main.elf $(OBJECTS)

# file targets:
main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c
