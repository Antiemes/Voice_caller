#ifndef __FLASH_H_
#define __FLASH_H_

void setSampleCounter(uint32_t c);

uint32_t getSampleCounter(void);

uint8_t readSample(void);

void erase_chip(void);

void initRead(uint32_t startAddr);

void erase_block(uint32_t addr);

void program_start(uint32_t addr);

void program_byte(uint8_t data);

void program_end(void);

uint8_t write_in_progress(void);

void flash_unlock(void);

#endif
