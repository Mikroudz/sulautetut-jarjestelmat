#include "tmc2130.h"

// TMC2130 registers 

//TMC2130 registers
#define WRITE_FLAG     (1<<7) //write flag
#define READ_FLAG      (0<<7) //read flag
#define REG_GCONF      0x00
#define REG_GSTAT      0x01
#define REG_IHOLD_IRUN 0x10
#define REG_CHOPCONF   0x6C
#define REG_COOLCONF   0x6D
#define REG_DCCTRL     0x6E
#define REG_DRVSTATUS  0x6F

// GCONF register configs
#define I_scale_analog      (1 << 0)

// Reads single register
static uint32_t read_register(tmc2130 *tmc, uint8_t address)
{
  //uint8_t value[5] = {0};

  // 7bit controls read/write mode
  CLEAR_BIT(address, READ_FLAG);

  uint8_t value[5] = {address, 0x0, 0x0, 0x0, 0x0};
  uint8_t dummy[5] = {address, 0x0, 0x0, 0x0, 0x0};

  // Cleear old SPI reply so we can read any register now
  HAL_GPIO_WritePin(tmc->nss_port, tmc->nss_pin, GPIO_PIN_RESET);

  // Read dummy
  HAL_SPI_Transmit(tmc->spi, (uint8_t *) dummy, 5, 1000);

  // End SPI transaction
  HAL_GPIO_WritePin(tmc->nss_port, tmc->nss_pin, GPIO_PIN_SET);

  // Start SPI transaction
  HAL_GPIO_WritePin(tmc->nss_port, tmc->nss_pin, GPIO_PIN_RESET);

  // Transmit reg address, then receive it value
  HAL_SPI_Receive(tmc->spi, (uint8_t *) value, 5, 1000);

  //HAL_StatusTypeDef res2 = HAL_SPI_TransmitReceive(tmc->spi, (uint8_t *) txdata, (uint8_t *) value, 5, tmc->spi_timeout);
  // End SPI transaction
  HAL_GPIO_WritePin(tmc->nss_port, tmc->nss_pin, GPIO_PIN_SET);

  uint32_t ret = (value[1] << 24) | (value[2] << 16) | (value[3] << 8) | value[4];

  tmc->gstat_val = value[0];
/*
  uint8_t i;
  printf("Read address: 0x%08x\n\r", address);
  printf("Values:\n\r");
  for (i = 0; i < 5; i++) {
    printf("%d\n\r", value[i]);
  }*/

  return ret;
}

// Writes single register
static void write_register(tmc2130 *tmc, uint8_t address, uint32_t value)
{
  // 7bit controls read/write mode
  SET_BIT(address, WRITE_FLAG);

  // Reg address + its new value
  //uint16_t payload = (value << 8) | address;

  uint8_t payload [5] = {0};
  
  HAL_SPI_Transmit(tmc->spi, (uint8_t *) payload, 5, tmc->spi_timeout);

  payload[0] = address;
  payload[1] = (uint8_t)(value >> 24);
  payload[2] = (uint8_t)(value >> 16);
  payload[3] = (uint8_t)(value >> 8);
  payload[4] = (uint8_t)value;

  // Start SPI transaction, send address + value
  HAL_GPIO_WritePin(tmc->nss_port, tmc->nss_pin, GPIO_PIN_RESET);
  uint32_t res = HAL_SPI_Transmit(tmc->spi, payload, 5, tmc->spi_timeout);
  // End SPI transaction
  HAL_GPIO_WritePin(tmc->nss_port, tmc->nss_pin, GPIO_PIN_SET);
  //printf("Data sent:\n");
  //for (int i = 0; i < 5; i++) {
  //  printf("%d\n\r", payload[i]);
  //}
  if (res != HAL_OK) {
    printf("SPI transmit failed: %d", res);
  }
}

uint8_t tmc2130_init(tmc2130 *tmc, SPI_HandleTypeDef *spi, 
    GPIO_TypeDef *enable_port,
    GPIO_TypeDef *direction_port,
    GPIO_TypeDef *step_port,
    GPIO_TypeDef *nss_port,
    uint16_t enable_pin,
    uint16_t direction_pin,
    uint16_t step_pin,
    uint16_t nss_pin
    ){
    assert_param(tmc && spi);

  tmc->spi = spi;
  tmc->enable_port = enable_port;
  tmc->direction_port = direction_port;
  tmc->step_port = step_port;
  tmc->enable_pin = enable_pin;
  tmc->direction_pin = direction_pin;
  tmc->step_pin = step_pin;
  tmc->nss_port = nss_port;
  tmc->nss_pin = nss_pin;
  tmc->spi_timeout = 1000;

  stepper_disable(tmc);
  stepper_set_dir(tmc, BACKWARD);

    //printf("REG_CHOPCONF: 0x%08x\n\r", read_REG_CHOPCONF(&stepper1));

  //printf("REG_CHOPCONF: 0x%08x\n\r", read_REG_CHOPCONF(&stepper1));
  write_GCONF(tmc);
  write_IHOLD_RUN(tmc, 12, 12, 0);
  write_CHOPCONF(tmc);

  //    printf("Read IHOLD_RUN: 0x%08x\n\r", read_IHOLD_RUN(&stepper1));
  //printf("Read GSTAT: 0x%x\n\r", read_REG_GSTAT(tmc));
  //    printf("Read GSTAT: 0x%08x\n\r", read_REG_GSTAT(&stepper1));
  printf("Read CHOPCONF: 0x%08x\n\r", read_REG_CHOPCONF(tmc));
  printf("Read GCONF: 0x%08x\n\r", read_REG_GCONF(tmc));
  printf("Read DRVSTATUS: 0x%08x\n\r", read_REG_DRVSTATUS(tmc));
  printf("Read GSTAT: 0x%x\n\r", tmc->gstat_val);

  return 0;
}

void stepper_set_dir(tmc2130 *tmc, StepDir dir){
  tmc->direction = dir;
  if(dir == FORWARD)
    HAL_GPIO_WritePin(tmc->direction_port, tmc->direction_pin, GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(tmc->direction_port, tmc->direction_pin, GPIO_PIN_SET);
}

void stepper_enable(tmc2130 *tmc){
  HAL_GPIO_WritePin(tmc->enable_port, tmc->enable_pin, GPIO_PIN_RESET);
}

void stepper_disable(tmc2130 *tmc){
  HAL_GPIO_WritePin(tmc->enable_port, tmc->enable_pin, GPIO_PIN_SET);
}

uint32_t read_REG_GCONF(tmc2130 *tmc){
  return read_register(tmc, REG_GCONF);// & 0xffff8000;
}

uint8_t read_REG_GSTAT(tmc2130 *tmc){
  return (uint8_t)read_register(tmc, REG_GSTAT) & 0xffffff00;
}

uint32_t read_REG_DRVSTATUS(tmc2130 *tmc){
  
  return read_register(tmc, REG_DRVSTATUS);
}

void write_CHOPCONF(tmc2130 *tmc){

  uint32_t val = 0x000100C3;//0x00008008; = 0x0001 << 24;// MRES microstep resolution 0 //0x000100C3;
  val |= 4 << 24; // fullstep
  val |= 1 << 28; // interpolation
  printf("Chopconf written: 0x%08x\n", val);
  write_register(tmc, REG_CHOPCONF, val);
}

uint32_t read_REG_CHOPCONF(tmc2130 *tmc){
  return read_register(tmc, REG_CHOPCONF);
}

void write_IHOLD_RUN(tmc2130 *tmc, uint8_t ihold, uint8_t irun, uint8_t iholddelay) {
  
  uint32_t reg_value = 0;
  reg_value |= (uint32_t) ihold;
  reg_value |= ((uint32_t)irun << 8);
  reg_value |= ((uint32_t)iholddelay << 16);
  printf("Ihold written: 0x%08x\n", reg_value);

  write_register(tmc, REG_IHOLD_IRUN, reg_value);
}

void write_GCONF(tmc2130 *tmc){
  uint32_t reg_value = 0x1;

  printf("GCONF written: 0x%08x\n", reg_value);

  write_register(tmc, REG_GCONF, reg_value);
}

void stepper_step(tmc2130 *tmc, unsigned int steps){
  for(int i = 0; i < steps; i++){
    HAL_GPIO_WritePin(tmc->step_port, tmc->step_pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(tmc->step_port, tmc->step_pin, GPIO_PIN_RESET);
  }
}