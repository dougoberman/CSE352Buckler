// Template for Finite State Machine
// 

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

// Choose among the following header files as you like
// Their paths are included already in Board.mk, Includes.mk files
#include "app_timer.h"
#include "nrf_gpio.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "mpu9250.h"

// Customized header files -- need to be present inside the application folder
#include "gpio.h"
#include "states.h"
#include "buckler.h"

/* Application Body 
*******************************************************/

// define state machine variables

state current_state;

void print_state(){

  printf("CURRENT STATE: %d \n", current_state);
}

void initialize_hardware(){

  // a function to take care of beforehand hardware initializations (e.g. GPIOs etc.)

  gpio_config(BUCKLER_LED0, true);
  gpio_config(BUCKLER_LED1, true);
  gpio_config(BUCKLER_LED2, true);


  gpio_set(BUCKLER_LED0);
  gpio_set(BUCKLER_LED1);
  gpio_set(BUCKLER_LED2);

  gpio_config(BUCKLER_BUTTON0, false);

  // any additional robot related initialization can be placed here as well

}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize LEDs

  initialize_hardware();
  
  // initialize the state variable

  current_state = INIT; 
  
  // loop forever, running state machine
  while (1) {
    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(50);

    // useful for debugging purposes
    print_state();

    // iterate statechart
    switch(current_state){


      case INIT:
	 /* initially we are in state OFF */
        current_state = OFF;
        break;
      case ON:
	 /* statements to execute in state ON */	

        if (<condition for switching from ON to OFF>)
          {
		current_state = OFF;                        
          }
        break;
      case OFF:
	 /* statements to execute in state OFF */
        if (<condition for switching from OFF to ON>)
          {
		current_state = ON;                        
          }
        break;
      default:
        current_state = OFF;
    }
    
  }
}
