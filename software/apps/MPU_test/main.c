// Analog accelerometer app
//
// Reads data from the ADXL327 analog accelerometer

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>  // needed for tilt angle calculation

#include "lsm9ds1.h" 
#include "nrf_twi_mngr.h"

#include "timestamp.h"
#include "display.h"
#include "mpu9250.h"

#define RAD2DEG 57.13 

// ADC channels
#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2


float timestamp=0.0;

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);


int main (void) {

  ret_code_t error_code = NRF_SUCCESS;

  // initialization complete
  printf("Buckler initialized!\n");
  
  /// Display related

  // // initialize display
  // nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  // nrf_drv_spi_config_t spi_config = {
  //   .sck_pin = BUCKLER_LCD_SCLK,
  //   .mosi_pin = BUCKLER_LCD_MOSI,
  //   .miso_pin = BUCKLER_LCD_MISO,
  //   .ss_pin = BUCKLER_LCD_CS,
  //   .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
  //   .orc = 0,
  //   .frequency = NRF_DRV_SPI_FREQ_4M,
  //   .mode = NRF_DRV_SPI_MODE_2,
  //   .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  // };
  // error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  // APP_ERROR_CHECK(error_code);
  // display_init(&spi_instance);
  // display_write("Hello, Human!", DISPLAY_LINE_0);
  // printf("Display initialized!\n");



  ////////////////////

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;

  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);

  ////////////////////
  // Initialize the IMU MPU-9250
  mpu9250_init(&twi_mngr_instance);  
  
  nrf_delay_ms(2000);

  // static uint8_t i = 0;
  // static char display_buf[16];
  

  // display_write("SD Card Ready\n", DISPLAY_LINE_0);
  
  printf("Display initialized!\n");
  
  
  // gyro has taken its start only now.
  error_code = mpu9250_start_gyro_integration();
  APP_ERROR_CHECK(error_code);  


   // measurement variables for storing the IMU values
  mpu9250_measurement_t ACC_MPU;
  mpu9250_measurement_t GYR_MPU;
  mpu9250_measurement_t MAG_MPU;
  // loop forever
  while (1) {


  // Reading Digital IMU

    ACC_MPU = mpu9250_read_accelerometer();
    
    GYR_MPU = mpu9250_read_gyro();

    MAG_MPU = mpu9250_read_magnetometer();
  
     
    // print results
    printf("                      X-Axis\t    Y-Axis\t    Z-Axis\n");
    printf("                  ----------\t----------\t----------\n");
    printf("Acceleration (g): %10.3f\t%10.3f\t%10.3f\n", ACC_MPU.x_axis, ACC_MPU.y_axis, ACC_MPU.z_axis);
    printf("Angle  (degrees): %10.3f\t%10.3f\t%10.3f\n", GYR_MPU.x_axis, GYR_MPU.y_axis, GYR_MPU.z_axis);
    printf("Magnetism   (uT): %10.3f\t%10.3f\t%10.3f\n", MAG_MPU.x_axis, MAG_MPU.y_axis, MAG_MPU.z_axis);
    printf("\n");
	                
  
    
    nrf_delay_ms(500);
      
  }   
}


