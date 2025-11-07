// Analog accelerometer app
//
// Reads data from the ADXL327 analog accelerometer

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>  // needed for tilt angle calculation


#include "timestamp.h"

#define RAD2DEG 57.13 

// ADC channels
#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2


float timestamp=0.0;



// callback for SAADC events
void saadc_callback (nrfx_saadc_evt_t const * p_event) {
  // don't care about adc callbacks
}

// sample a particular analog channel in blocking mode
 nrf_saadc_value_t sample_value (uint8_t channel) {
   nrf_saadc_value_t val;
   ret_code_t error_code = nrfx_saadc_sample_convert(channel, &val);
   APP_ERROR_CHECK(error_code);
   return val;
 }


int main (void) {
//-------------------------------------------------------------//	 
//----------------- MODIFY THE FOLLOWING CODE -----------------//
  ret_code_t error_code = NRF_SUCCESS;

  // // initialize RTT ENDS_INIT();

  // initialize analog to digital converter
  // ...
 // ...
 error_code = nrfx_saadc_init(&saadc_config, saadc_callback);
  APP_ERROR_CHECK(error_code);

  // // initialize analog inputs
  // // configure with 0 as input pin for now
   // ...
   // ...
  // // specify input pin for X axis and initialize that ADC channel
   // ...
  // ...
  APP_ERROR_CHECK(error_code);

  // // specify input pin for Y axis and initialize that ADC channel
  // ...
  

  error_code = nrfx_saadc_channel_init(Y_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // // specify input pin for Z axis and initialize that ADC channel
 // ...
  error_code = nrfx_saadc_channel_init(Z_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // initialization complete
  printf("Buckler initialized!\n");

  
  
  ////////////////////

  // calibration of the voltage output from the ADC

  // .. copy your work from the accelerometer lab

  // variables for sampling raw values from the ADC
	
  // defining tilt variables
  
  
  // Initialize the SD Card for logging data in TESTFILE.csv
  init_SDCard();

  // loop forever
  while (1) {
   
    // sample analog inputs

   // convert the ADC output to voltage values
    

   // calculation of acceleration from voltage output
    
  
// display results
    float ax, ay, az;
     
    printf("a_x: %f\ta_y: %f\ta_z:%f\n", ax, ay, az);
   
    // code for logging data from the accelerometer    
      // if (!gpio_read(BUCKLER_BUTTON0)) {
         
        timestamp = get_timestamp();      
        simple_logger_log("%f,%f,%f,%f\n", timestamp, ax, ay, az);
      
        printf("%f - Wrote line to SD card\n", timestamp);
        // Signal that lines were written
        
        // gpio_clear(BUCKLER_LED0);      
        nrf_delay_ms(10);
      // }
      // else
        // gpio_set(BUCKLER_LED0);  
  }   
}


