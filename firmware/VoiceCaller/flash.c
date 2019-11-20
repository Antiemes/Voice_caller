//Flash type: S25FL127SABMFI101 -  Flash Memory, NOR, 16 MB, 108 MHz, SPI, SOIC, 8 Pins 
//Farnell order code: 2328002
//SST26VF064B-104I/SM
#include "spi.h"
#include "flash.h"

uint8_t readSample()
{
  sampleCounter--;
  return spi_transmit_receive(0xff);
}

void erase_block(uint32_t addr)
{
  deselect_chip();
  select_chip();
  spi_transmit_receive(0x06);	//enable write

  deselect_chip();
  select_chip();
  //spi_transmit_receive (0xd8);	//block erase (64k or 256k)
  spi_transmit_receive (0x20);	//block erase (4k)
  spi_transmit_receive((addr >> 16) & 0xff);	//address msB
  spi_transmit_receive((addr >>  8) & 0xff);
  spi_transmit_receive( addr        & 0xff);	//address lsB
  deselect_chip();
  select_chip();
}

void flash_unlock()
{
  deselect_chip();
  select_chip();
  spi_transmit_receive(0x06);	//enable write
  
  deselect_chip();
  select_chip();
  spi_transmit_receive (0x98);	//Global Block Protection Unlock
}

void program_start(uint32_t addr)
{
  deselect_chip();
  select_chip();
  spi_transmit_receive(0x06);	//enable write
  
  deselect_chip();
  select_chip();
  spi_transmit_receive (0x02);	//page program
  spi_transmit_receive((addr >> 16) & 0xff);	//address msB
  spi_transmit_receive((addr >>  8) & 0xff);
  spi_transmit_receive( addr        & 0xff);	//address lsB
}

void program_byte(uint8_t data)
{
  spi_transmit_receive(data);	//programming byte
}

uint8_t write_in_progress(void)
{
  uint8_t busy;
  deselect_chip();
  select_chip();
  spi_transmit_receive(0x05);	//read status register
  busy=spi_transmit_receive(0xff) & 1;
  deselect_chip();
  select_chip();
  return busy;
}

void program_end()
{
  deselect_chip();
  select_chip();
  //spi_transmit_receive(0x04);	//disable write
  //deselect_chip();
  //select_chip();
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

