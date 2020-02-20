#ifndef CC1101_HAL_H_
#define CC1101_HAL_H_

#include <stdint.h>

// CC1101 radio functions
void CC1101_regWrite(uint8_t addr, uint8_t value);
void CC1101_reset(void);
void CC1101_init(void);

// Manipulate chip select (CS) pin of radio CC1101
void CC1101_clearCS(void);
void CC1101_setCS(void);

#endif