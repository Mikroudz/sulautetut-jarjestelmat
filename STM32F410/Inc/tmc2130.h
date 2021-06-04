
#ifndef __TMC2130_H
#define __TMC2130_H

#include "main.h"


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

} tmc2130;

#endif
