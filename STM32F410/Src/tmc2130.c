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
  uint8_t value[5] = {0};

  // 7bit controls read/write mode
  CLEAR_BIT(address, READ_FLAG);

  uint8_t full_read[5] = {address, 0x0, 0x0, 0x0, 0x0};

  // Start SPI transaction
  HAL_GPIO_WritePin(tmc->nss_port, tmc->nss_pin, GPIO_PIN_RESET);
  // Transmit reg address, then receive it value
  uint32_t res1 = HAL_SPI_Transmit(tmc->spi, (uint8_t *) full_read, 5, tmc->spi_timeout);
  uint32_t res2 = HAL_SPI_Receive(tmc->spi, (uint8_t *) value, 5, tmc->spi_timeout);
  // End SPI transaction
  HAL_GPIO_WritePin(tmc->nss_port, tmc->nss_pin, GPIO_PIN_SET);

  uint32_t ret = value[1];
  ret |= value[2];
  ret |= value[3];
  ret |= value[4];
  //*gstat_val = value[0];

  if (res1 != HAL_OK || res2 != HAL_OK) {
    printf("SPI transmit/receive failed (%d %d)", res1, res2);
  }

  return ret;
}

// Writes single register
static void write_register(tmc2130 *tmc, uint8_t address, uint8_t value)
{
  // 7bit controls read/write mode
  SET_BIT(address, WRITE_FLAG);

  // Reg address + its new value
  //uint16_t payload = (value << 8) | address;

  uint8_t payload [5] = {};

  payload[0] = address;
  payload[1] = value;

  // Start SPI transaction, send address + value
  HAL_GPIO_WritePin(tmc->nss_port, tmc->nss_pin, GPIO_PIN_RESET);
  uint32_t res = HAL_SPI_Transmit(tmc->spi, payload, 5, tmc->spi_timeout);
  // End SPI transaction
  HAL_GPIO_WritePin(tmc->nss_port, tmc->nss_pin, GPIO_PIN_SET);

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


    HAL_GPIO_WritePin(enable_port, enable_pin, GPIO_PIN_RESET);
    uint8_t reg_val = read_register(tmc, REG_GSTAT);

    printf("%x\n", reg_val);
    reg_val = read_register(tmc, REG_GCONF);
    printf("%x\n", reg_val);

    write_register(tmc, REG_GCONF, I_scale_analog);
    
    reg_val = read_register(tmc, REG_GCONF);
    printf("%x\n", reg_val);
    return 0;
}

void stepper_enable(tmc2130 *tmc){
  HAL_GPIO_WritePin(tmc->enable_port, tmc->enable_pin, GPIO_PIN_RESET);
}

void stepper_disable(tmc2130 *tmc){
  HAL_GPIO_WritePin(tmc->enable_port, tmc->enable_pin, GPIO_PIN_SET);
}

uint32_t read_REG_GCONF(tmc2130 *tmc){
  return read_register(tmc, REG_GCONF) & 0xffff8000;
}
uint32_t read_REG_GSTAT(tmc2130 *tmc){
  return read_register(tmc, REG_GSTAT) & 0x07;
}

void write_IHOLD_RUN(tmc2130 *tmc, uint8_t ihold, uint8_t irun, uint8_t iholddelay) {
  
  uint32_t reg_value = 0;
  reg_value |= (uint32_t) ihold;
  reg_value |= (((uint32_t) irun) << 5);
  reg_value |= (((uint32_t) iholddelay) << 10);

  write_register(tmc, REG_IHOLD_IRUN, reg_value);
}

uint8_t read_IHOLD_RUN(tmc2130 *tmc){
  return read_register(tmc, REG_IHOLD_IRUN);
}