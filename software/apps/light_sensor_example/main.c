// Light sensor app
//
// Reads from the OPT3004 light sensor on Buckler

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_twi_mngr.h"

#include "buckler.h"
#include "opt3004.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  

  // initialize OPT3004 driver
  const opt3004_config_t config = {
      .range_number = OPT3004_AUTORANGE,       // Sensor range. Use OPT3004_AUTORANGE to enable autoranging
      .conversion_time = 100,    // Conversion time of sensor. 100 or 800ms
      .latch_interrupt = 0,    // Latch interrupts until config register is read
      .interrupt_polarity= 0, // Interrupt polarity. Active high or low
      .fault_count=4,        // Number of faults before interrupt trigger
  };

  opt3004_init(&twi_mngr_instance);//, BUCKLER_LIGHT_INTERRUPT);
  opt3004_config(config);
  printf("OPT3004 initialized\n");

  // loop forever
  while (1) {
    float lux = opt3004_read_result();
    printf("Reading (lux): %f\n", lux);
    nrf_delay_ms(100);
  }
}

