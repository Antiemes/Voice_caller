#!/bin/bash

SERDEV="/dev/ttyACM0"
echo "Press reset."
while [ -e $SERDEV ]
  do
    false
  done
while [ ! -w $SERDEV ]
  do
    false
  done
avrdude -p atmega32u4 -c avr109 -b 57600 -D -P /dev/ttyACM0 -U flash:w:main.hex:i

