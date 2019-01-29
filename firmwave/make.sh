#!/bin/bash

avr-gcc -mmcu="atmega328p" -D__AVR__ spi.c flash.c main.c -Wall -o main.elf -O2
avr-size -C --mcu=atmega328p main.elf
avr-objcopy -j .text -j .data -O ihex main.elf main.hex

