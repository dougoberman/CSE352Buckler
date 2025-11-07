#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrf_twi_mngr.h"
#include "nrfx_gpiote.h"

#include "buckler.h"

// defining the instance of TWI


void scan_i2c_bus(void) {
    ret_code_t err_code;
    uint8_t address;
    uint8_t sample_data;
    
    printf("Scanning I2C addresses...\n");
    
    // Read from the addresses ranging from 0 to 127
    // Whichever sends NRF_SUCCESS is on the bus
    // Display the addresses on the bus
}

int main(void) {

    // Configure the TWI
    
    // Initialize TWI 

    // Enable TWi 
    
    
    scan_i2c_bus();

    while (1);
}
