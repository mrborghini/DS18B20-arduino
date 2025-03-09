default:
	mkdir -p bin
	avr-gcc -Os -DF_CPU=16000000UL -std=c23 -mmcu=atmega328p -c -o bin/main.o src/main.c
	avr-gcc -mmcu=atmega328p -o bin/main.bin bin/main.o
	avr-objcopy -O ihex -R .eeprom bin/main.bin bin/main.hex
	avrdude -F -V -c arduino -p ATMEGA328P -P COM6 -b 57600 -U flash:w:bin/main.hex