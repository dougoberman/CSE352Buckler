
/*
 Code: Threads for Blinking LED
 Author: Umer Huzaifa
 Date: 03/27/2023
 Comments: On Using FreeRTOS for doing hardware I/O
           Task1. Periodically toggle two LEDs
           Task2. Check the input on a pin and toggle third LED 
*/



#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"

#include "buckler.h"
#include "FreeRTOS.h"


  // Configuring the RTOS here

void SIG_OUTPUT_COMPARE1A( void ) __attribute__ ( ( signal ) );

void SIG_OUTPUT_COMPARE1A( void )
{
    /* ISR C code for RTOS tick. */
    vPortYieldFromTick();
}


void led0_toggle() {

    nrf_gpio_pin_toggle(BUCKLER_LED0);
}

void led1_toggle() {

    nrf_gpio_pin_toggle(BUCKLER_LED1);
}

void led2_toggle() {

    nrf_gpio_pin_toggle(BUCKLER_LED2);
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Board initialized!\n");

  // Configuring the GPIO pins for LEDs as outputs
  nrf_gpio_pin_dir_set(BUCKLER_LED0, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(BUCKLER_LED1, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(BUCKLER_LED2, NRF_GPIO_PIN_DIR_OUTPUT);

  nrf_gpio_pin_dir_set(BUCKLER_BUTTON0, NRF_GPIO_PIN_DIR_INPUT);

  nrf_gpio_pin_set(BUCKLER_LED0);
  nrf_gpio_pin_set(BUCKLER_LED1);
  nrf_gpio_pin_set(BUCKLER_LED2);


  
  // loop forever
  while (1) {
    printf("Wait for the input to come in while scheduling LED tasks \n");
    
    nrf_delay_ms(10);
  }
}
