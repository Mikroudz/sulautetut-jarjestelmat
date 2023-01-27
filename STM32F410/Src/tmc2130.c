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
#define REG_PWMCONF    0x70
#define REG_XDIRECT    0x2D


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

void tmc2130_enable(tmc2130 *tmc){
  HAL_GPIO_WritePin(tmc->enable_port, tmc->enable_pin, GPIO_PIN_RESET);
}

void tmc2130_disable(tmc2130 *tmc){
  HAL_GPIO_WritePin(tmc->enable_port, tmc->enable_pin, GPIO_PIN_SET);
}

static uint32_t read_REG_GCONF(tmc2130 *tmc){
  return read_register(tmc, REG_GCONF);// & 0xffff8000;
}

static uint8_t read_REG_GSTAT(tmc2130 *tmc){
  return (uint8_t)read_register(tmc, REG_GSTAT) & 0xffffff00;
}

static uint32_t read_REG_DRVSTATUS(tmc2130 *tmc){
  
  return read_register(tmc, REG_DRVSTATUS);
}

static uint32_t read_REG_PWMCONF(tmc2130 *tmc){
  return read_register(tmc, REG_PWMCONF);
}

static void write_CHOPCONF(tmc2130 *tmc){

  uint32_t val = 0x000100C3;//0x00008008; = 0x0001 << 24;// MRES microstep resolution 0 //0x000100C3;
  val |= 6 << 24; // fullstep
  val |= 1 << 28; // interpolation
  val |= 1 << 17; // Vsense = 1
  val |= 2 << 15; //TBL 1 = 24
  val |= 2 << 4; // HSTRT
  val |= 3 << 7; // HEND
  val |= 3; //Toff
  printf("Chopconf written: 0x%08x\n", val);
  write_register(tmc, REG_CHOPCONF, val);
}

static void write_PWMCONF(tmc2130 *tmc){

  uint32_t val = 0;//0x00008008; = 0x0001 << 24;// MRES microstep resolution 0 //0x000100C3;
  val |= 3 << 8; // PWM_GRAD
  val |= 1 << 18; // PWM automatic amplitude scaling
  val |= 1 << 16; // pwm_freq0


  val |= 255; // fullstep

  printf("PWMCONF written: 0x%08x\n", val);
  write_register(tmc, REG_PWMCONF, val);
}

static void write_IHOLD_RUN(tmc2130 *tmc, uint8_t ihold, uint8_t irun, uint8_t iholddelay) {
  
  uint32_t reg_value = 0;
  reg_value |= (uint32_t) ihold;
  reg_value |= ((uint32_t)irun << 8);
  reg_value |= ((uint32_t)iholddelay << 16);
  printf("Ihold written: 0x%08x\n", reg_value);

  write_register(tmc, REG_IHOLD_IRUN, reg_value);
}

static void write_COOLCONF(tmc2130 *tmc) {
  
  uint32_t reg_value = 0;
  printf("Coolconf written: 0x%08x\n", reg_value);

  write_register(tmc, REG_COOLCONF, reg_value);
}


static void write_GCONF(tmc2130 *tmc){
  uint32_t reg_value = 0;

  reg_value |= 1 << 2; // PWM enable
  reg_value |= 1; // AIN reference

  printf("GCONF written: 0x%08x\n", reg_value);

  write_register(tmc, REG_GCONF, reg_value);
}

static uint32_t read_REG_CHOPCONF(tmc2130 *tmc){
  return read_register(tmc, REG_CHOPCONF);
}


uint8_t tmc2130_init(tmc2130 *tmc, SPI_HandleTypeDef *spi, 
    GPIO_TypeDef *enable_port,
    GPIO_TypeDef *nss_port,
    uint16_t enable_pin,
    uint16_t nss_pin
    ){
  assert_param(tmc && spi);

  tmc->spi = spi;
  tmc->enable_port = enable_port;
  tmc->enable_pin = enable_pin;
  tmc->nss_port = nss_port;
  tmc->nss_pin = nss_pin;
  tmc->spi_timeout = 1000;

  tmc2130_disable(tmc);
  HAL_Delay(10);

    //printf("REG_CHOPCONF: 0x%08x\n\r", read_REG_CHOPCONF(&stepper1));

  //printf("REG_CHOPCONF: 0x%08x\n\r", read_REG_CHOPCONF(&stepper1));
  write_CHOPCONF(tmc);
  HAL_Delay(10);
  write_PWMCONF(tmc);
  HAL_Delay(10);
  write_GCONF(tmc);
  HAL_Delay(10);
  write_IHOLD_RUN(tmc, 2, 20, 4);  
  HAL_Delay(10);
  write_COOLCONF(tmc);
  //    printf("Read IHOLD_RUN: 0x%08x\n\r", read_IHOLD_RUN(&stepper1));
  //printf("Read GSTAT: 0x%x\n\r", read_REG_GSTAT(tmc));
  //    printf("Read GSTAT: 0x%08x\n\r", read_REG_GSTAT(&stepper1));
  printf("Read CHOPCONF: 0x%08x\n\r", read_REG_CHOPCONF(tmc));
  printf("Read GCONF: 0x%08x\n\r", read_REG_GCONF(tmc));
  printf("Read DRVSTATUS: 0x%08x\n\r", read_REG_DRVSTATUS(tmc));
  //printf("Read PWMCONF: 0x%08x\n\r", read_REG_PWMCONF(tmc));

  printf("Read GSTAT: 0x%x\n\r", tmc->gstat_val);

  return 0;
}
