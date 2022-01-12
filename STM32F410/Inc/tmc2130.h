
#ifndef __TMC2130_H
#define __TMC2130_H

#include "main.h"
#include <stdio.h>

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
uint32_t read_REG_GCONF(tmc2130 *tmc);
uint8_t read_REG_GSTAT(tmc2130 *tmc);

//void stepper_set_dir(tmc2130 *tmc, StepDir dir);
uint32_t read_REG_DRVSTATUS(tmc2130 *tmc);
void write_IHOLD_RUN(tmc2130 *tmc, uint8_t ihold, uint8_t irun, uint8_t iholddelay);
uint32_t read_REG_CHOPCONF(tmc2130 *tmc);
uint32_t read_REG_PWMCONF(tmc2130 *tmc);

void write_CHOPCONF(tmc2130 *tmc);
void write_GCONF(tmc2130 *tmc);
void write_PWMCONF(tmc2130 *tmc);
void write_COOLCONF(tmc2130 *tmc);
void stepper_enable(tmc2130 *tmc);
void stepper_disable(tmc2130 *tmc);
void stepper_update(tmc2130 *tmc);
//void stepper_setspeed(tmc2130 *tmc, int speed);

#endif
