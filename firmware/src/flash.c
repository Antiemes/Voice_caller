//Flash type: S25FL127SABMFI101 -  Flash Memory, NOR, 16 MB, 108 MHz, SPI, SOIC, 8 Pins 
//Farnell order code: 2328002
#include "spi.h"
#include "flash.h"

uint8_t readSample()
{
  sampleCounter--;
  return spi_transmit_receive(0xff);
}

void erase_chip()
{
  deselect_chip ();
  select_chip ();
  spi_transmit_receive (0x06);	//enable write
  deselect_chip ();
  select_chip ();

  spi_transmit_receive (0xc7);	//chip erase
  deselect_chip ();
  select_chip ();

  spi_transmit_receive (0x05);	//initiate read status register

  while (spi_transmit_receive (0xff) == 3);

  deselect_chip ();
}

void initRead(uint32_t startAddr)
{
  deselect_chip();
  select_chip();
  spi_transmit_receive(0x03);	//initiate read from the address
  spi_transmit_receive((startAddr >> 16) & 0xff);	//address msB
  spi_transmit_receive((startAddr >>  8) & 0xff);
  spi_transmit_receive( startAddr        & 0xff);	//address lsB
  sampleCounter=0;
}

