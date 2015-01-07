COMPILE = avr-g++ -g -Wall -O2 -DF_CPU=16000000 -mmcu=atmega328p
OBJECTS = main.o 
AVRDUDE = avrdude -c arduino -p atmega328p -P /dev/tty.usb* -U flash:w:main.hex:i
PROG=main
all: main.hex

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -O ihex main.elf main.hex
	avr-size --format=avr --mcu=atmega328p main.elf

main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

.cpp.o:
	$(COMPILE) -c $< -o $@

flash:
	$(AVRDUDE)

.PHONY: clean
clean:
	rm main.hex main.elf $(OBJECTS)

read:
	readelf -a ${PROG}.elf