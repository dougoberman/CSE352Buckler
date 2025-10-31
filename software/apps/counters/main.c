

//** Code: Hard Timers (Lab 6)
//** Author: Umer Huzaifa
//** Comments: 
//** Implementing a timer and a counter. 
//** Counter is another timer that can be triggered as we like instead of the clock signals


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

#define count_threshold 10000000


void timer_init(){
  // all the register settings for setting up TIMER4 for 3 seconds timing operation
  
  // Additional settings for declaring TIMER4 as an interrupt
  

}

void counter_init(){

  // Configure TIMER3 as a counter
}

uint32_t read_timer(){
    
}

uint32_t read_counter(){
  // just like TIMER4, read the values of TIMER3
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Board initialized!\n");

  // You can use the NRF GPIO library to test your timers
   
  
  timer_init();  
  counter_init();

  nrf_delay_ms(3000);
  

  // loop forever
  while (1) {
    printf("Timer 4 readings are: %d \n", read_timer());

    printf("Counter (timer 3) readings are: %d \n", read_counter());

    // Place a check when counter needs to be incremented.
    // 
    nrf_delay_ms(1000);
  }
}

