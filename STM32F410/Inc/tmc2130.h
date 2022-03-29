
#ifndef __TMC2130_H
#define __TMC2130_H

#include "main.h"
#include <stdio.h>

#define TMC2130_OK 0
#define TMC2130_ERROR 1

// Asetustruktuuri stepperille. Luo jokaiselle stepperille oma tälläinen
typedef struct {
    SPI_HandleTypeDef   *spi;
    GPIO_TypeDef        *enable_port;
    uint16_t            enable_pin;
    GPIO_TypeDef        *direction_port;
    uint16_t            direction_pin;
    GPIO_TypeDef        *step_port;
    uint16_t            step_pin;
    GPIO_TypeDef        *nss_port;
    uint16_t            nss_pin;
    uint16_t            spi_timeout;
    uint8_t             gstat_val;
} tmc2130;


// prototyypit
uint8_t tmc2130_init(tmc2130 *tmc, SPI_HandleTypeDef *spi, 
    GPIO_TypeDef *enable_port,
    GPIO_TypeDef *nss_port,
    uint16_t enable_pin,
    uint16_t nss_pin
    );

void tmc2130_enable(tmc2130 *tmc);
void tmc2130_disable(tmc2130 *tmc);

#endif
