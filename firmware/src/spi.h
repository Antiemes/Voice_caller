#ifndef __SPI_H_
#define __SPI_H_

#include <avr/io.h>

#define SPI_PORT	PORTB
#define SPI_DDR		DDRB
#define SPI_PIN		PINB

#define MOSI_PIN	(1<<PB2)
#define MISO_PIN 	(1<<PB3)
#define SCK_PIN 	(1<<PB1)

#define SPI_CS_PORT	PORTD
#define SPI_CS_DDR		DDRD
#define SPI_CS_PIN		PIND
#define CS_PIN		(1<<PD0)

#define read_miso()		(SPI_PIN & (MISO_PIN))
#define select_chip()	(SPI_CS_PORT &= ~(CS_PIN))
#define deselect_chip()	(SPI_CS_PORT |= (CS_PIN))
#define SPI_SCK_HIGH()	(SPI_PORT |= SCK_PIN)
#define SPI_SCK_LOW()	(SPI_PORT &= ~SCK_PIN)
#define SPI_MOSI_HIGH()	(SPI_PORT |= MOSI_PIN)
#define SPI_MOSI_LOW()	(SPI_PORT &= ~MOSI_PIN)

//initialize bitbanged SPI
void spi_init();

uint8_t spi_transmit_receive (uint8_t c);

#endif
