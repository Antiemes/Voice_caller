#ifndef __FLASH_H_
#define __FLASH_H_

volatile uint32_t sampleCounter;

uint8_t readSample(void );

void erase_chip(void);

void initRead(uint32_t startAddr);

void erase_block(uint32_t addr);

void program_start(uint32_t addr);

void program_byte(uint8_t data);

void program_end(void);

uint8_t write_in_progress(void);

#endif
