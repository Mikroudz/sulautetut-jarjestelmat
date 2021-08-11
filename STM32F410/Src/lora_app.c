#include "lora_app.h"


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