#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "buckler.h"

void SWI1_EGU1_IRQHandler(void) {
   NRF_EGU1->EVENTS_TRIGGERED[0]=0;
   // rest of the code follows here
}

// Step B: Write an IRQ Handler/ISR for the Interrupt
void GPIOTE_IRQHandler(void) {
   // Clear the Events register
   
}

int main(void) {
  

    
  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");
 
  // Initialize LED and BUTTON pins on the Buckler board
  //............


  // Steps for Setting up Interrupt
  // 1. configure the interrupt


  // 2. enable interrupt -- NRF_GPIOTE->...


  // 3. enable IRQ on NVIC  -- NVIC_EnableIRQ(...)


    while (1) {
        // Main application loop
    }
}

