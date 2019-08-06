#ifndef __FLASH_H_
#define __FLASH_H_

volatile uint32_t sampleCounter;

uint8_t readSample();

void erase_chip();

void initRead(uint32_t startAddr);

#endif
