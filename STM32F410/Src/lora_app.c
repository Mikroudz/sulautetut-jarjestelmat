#include "lora_app.h"

/*
uint8_t lora_start(lora_sx1276 *lora, SPI_HandleTypeDef *spi){
  
  // SX1276 compatible module connected to SPI1, NSS pin connected to GPIO with label LORA_NSS
  uint8_t res = lora_init(lora, spi, GPIOB, GPIO_PIN_9, LORA_BASE_FREQUENCY_EU);
  if (res != LORA_OK) {
    // Initialization failed
    printf("Lora ei init koodi: %d\n", res);
  }
  lora_mode_sleep(lora);
  lora_mode_sleep(lora);
  lora_set_preamble_length(lora, 10);
  lora_set_spreading_factor(lora, 7);
  lora_set_signal_bandwidth(lora, LORA_BANDWIDTH_125_KHZ);
  lora_set_tx_power(lora, 20);
  lora_set_crc(lora, 1);
  lora_set_coding_rate(lora, LORA_CODING_RATE_4_5);
  lora_mode_standby(lora);
  printf("lora init DONE\n");
  //printf("Lora config:\n");
  //lora_print_config(lora);

}

// unblocking lora receive mode
uint8_t lora_receivetest(lora_sx1276 *lora, uint8_t *buf, size_t buflen){
  
    lora_enable_interrupt_rx_done(lora);
    // Put LoRa modem into continuous receive mode
    lora_mode_receive_continuous(lora);
    // Wait for packet up to 10sec
    uint8_t res;
    uint8_t len = lora_receive_packet(lora, buf, buflen, &res);
    if (res != LORA_OK) {
      printf("Receive faile, code: %d\n", res);
      // Receive failed
    }
    buf[len] = 0;  // null terminate string to print it
    return len;
    
    
}
*/
uint8_t lora_create_measurement_message(RobotData_t *data, uint8_t *buf){
  // type
  buf[0] = 'C';
  // Temp
  buf[1] = 0xef;
  buf[2] = 0xaa;
  // distance 1
  buf[3] = (uint8_t)(data->distance_front >> 8);
  buf[4] = (uint8_t)(data->distance_front);
  // distance 2
  buf[5] = (uint8_t)(data->distance_rear >> 8);
  buf[6] = (uint8_t)(data->distance_rear);
  // light sensor
  buf[7] = (uint8_t)(data->illumination >> 8);
  buf[8] = (uint8_t)(data->illumination);
  // voltage
  buf[9] = (uint8_t)(data->voltage >> 8);
  buf[10] = (uint8_t)(data->voltage);
  buf[11] = data->robot_status;

  for(int i = 0; i < 12; i++){
    printf("%x", buf[i]);
  }
  printf("\n");
  return 12; // message lenght
}

static void float_to_uint8(float f, uint8_t *buf){
  union {
    float f;
    uint8_t u[4];
  } val;
  val.f = f;

  buf[0] = val.u[3];
  buf[1] = val.u[2];
  buf[2] = val.u[1];
  buf[3] = val.u[0];
}

static float uint8_to_float(uint8_t *buf){
  union {
    float f;
    uint8_t u[4];
  } val;
  val.u[0] = buf[3];
  val.u[1] = buf[2];
  val.u[2] = buf[1];
  val.u[3] = buf[0];

  return val.f;
}

uint8_t lora_create_pid_message(PID_TypeDef *pid, uint8_t *buf, char type){
  // type pid
  buf[0] = 'A';
  // which parameter. V for velocity, A for angle
  buf[1] = type;
  
  float_to_uint8(0.5, &buf[2]);
  float_to_uint8(pid->KI, &buf[6]);
  float_to_uint8(pid->KD, &buf[10]);
 
  for(int i = 0; i < 15; i++){
    printf("%x", buf[i]);
  }
  printf("\n");
  return 14; // message len
}

void lora_parse_pid(uint8_t *buf){
  char * c_buf[6];
  gcvt(uint8_to_float(buf), 3, c_buf);

  printf("float val get: %s\n", c_buf);

}

void lora_parse_message(uint8_t *buf){

  switch(buf[0]){
    case 'A': // pid parameter

      if (buf[1] == 'A'){ // angle
        lora_parse_pid(buf[3]);
      }else if (buf[1] == 'V'){ // velocity
        lora_parse_pid(buf[3]);
      }

    break;
    case 'B': // Direction command
    break;
    case 'C': // Measurements
    // Will not be implemented if we dont want to make a query system for measurements. Current plan is to send them every second
    break;
  }
}