#include "timestamp.h"

float init_time;
void timer_init(){

  NRF_TIMER4->PRESCALER   = 0x09;   // this prescaler gives the slowest clock for Timer4
  NRF_TIMER4->BITMODE     = 0x02;   // configures the timer to 32 (3)/24 (2) bits
  NRF_TIMER4->MODE        = 0x00;   // configure as a timer not counter
   
////////////////////////////////////////////////////// 
 
  NRF_TIMER4->TASKS_CLEAR = 0x01;   // timer4 is reset
  NRF_TIMER4->TASKS_START = 0x01;   // timer4 starts again 
  
}

float get_timestamp(){
  
  return (float) (read_timer() - init_time)*1/31250;
}

uint32_t read_timer(){
    NRF_TIMER4->TASKS_CAPTURE[1] = 1;

    // Should return the value of the internal counter for TIMER4
    return NRF_TIMER4->CC[1];

}

void init_SDCard(){
  ret_code_t error_code = NRF_SUCCESS;
  // SD Card Checking
  printf("Started SD card demo app...\n");
  timer_init();

  // Enable SoftDevice (used to get RTC running)
  nrf_sdh_enable_request();

  // Initialize GPIO driver
  if (!nrfx_gpiote_is_init()) {
    error_code = nrfx_gpiote_init();
  }
  APP_ERROR_CHECK(error_code);
  // Configure GPIOs
  nrf_gpio_cfg_output(BUCKLER_SD_ENABLE);
  nrf_gpio_cfg_output(BUCKLER_SD_CS);
  nrf_gpio_cfg_output(BUCKLER_SD_MOSI);
  nrf_gpio_cfg_output(BUCKLER_SD_SCLK);
  nrf_gpio_cfg_input(BUCKLER_SD_MISO, NRF_GPIO_PIN_NOPULL);

  nrf_gpio_pin_set(BUCKLER_SD_ENABLE);
  nrf_gpio_pin_set(BUCKLER_SD_CS);
  gpio_config(BUCKLER_BUTTON0, 0);
  // Initialize SD card
  const char filename[] = "testfile.csv";
  const char permissions[] = "w"; // w = write, a = append
  // Start file
  simple_logger_init(filename, permissions);
  // simple_logger_log_header("HEADER for file \'%s\', written on %s \n", filename, "DATE");
  simple_logger_log_header("Time, ax, ay, az \n");
  printf("Wrote header to SD card\n");
  
  // Activate BUTTON0 as the start record button
  gpio_config(BUCKLER_BUTTON0, 0);
  
  // Indicator LED0 for data recording.
  gpio_config(BUCKLER_LED0, 1);
  // initial time recorded
  init_time = read_timer();
}