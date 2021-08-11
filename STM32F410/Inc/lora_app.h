#ifndef __LORA_APP_H
#define __LORA_APP_H

#include "main.h"
#include "lora_sx1276.h"
#include <stdio.h>

enum {
    LORA_INT_EMPTY = 0,
    LORA_INT_ACTIVE
};

uint8_t lora_start(lora_sx1276 *lora, SPI_HandleTypeDef *spi);

uint8_t lora_receivetest(lora_sx1276 *lora, uint8_t *buf, size_t buflen);


#endif
