#include "spi.h"

//initialize bitbanged SPI
void spi_init()
{
  deselect_chip ();
  SPI_DDR = (MOSI_PIN) | (SCK_PIN);
  SPI_CS_DDR = (CS_PIN);
  SPI_SCK_HIGH ();
}

//=====================================================================

uint8_t spi_transmit_receive (uint8_t c)
{
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    if (c & (1 << 7))
    {
      SPI_MOSI_HIGH ();
    }
    else
    {
      SPI_MOSI_LOW ();
    }
    SPI_SCK_LOW ();
    c <<= 1;
    if (read_miso ())
    {
      c |= 1;
    }
    SPI_SCK_HIGH ();
  }
  return c;
}


