sudo avrdude -c arduino -p m328p -P /dev/ttyUSB0 -Uflash:w:main.hex:i -B 3
#sudo avrdude -c avrispmkII -p t45 -P usb -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m 
