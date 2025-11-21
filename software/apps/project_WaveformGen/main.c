// PWM Frequency Measurement Lab
// nRF52832 Embedded Systems
//
// This lab teaches you to measure PWM frequency using TIMER, GPIOTE, and interrupts

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "software_interrupt.h"

#include "buckler.h"
#include "gpio.h"
#include "display.h"

// PWM Configuration
#define PWM_INSTANCE    NRF_PWM0
#define PWM_PIN1        23        // GPIO pin for PWM channel 1
#define PWM_PIN2        24        // GPIO pin for PWM channel 2
#define PWM_PIN3        25        // GPIO pin for PWM channel 3
#define PWM_INCREMENT   100

// PWM Parameters
uint16_t pwm = 8000;              // PWM compare value (duty cycle)
uint16_t prescalar = PWM_PRESCALER_PRESCALER_DIV_1;
uint32_t counterTop = 16000;      // PWM period value

// Measurement Variables
uint16_t lastCount = 0;           // Stores the frequency count

/**
 * TODO 1: Initialize TIMER4 for periodic interrupts
 * 
 * This timer will generate interrupts at a fixed interval (duration in seconds)
 * to trigger frequency measurements.
 * 
 * Steps:
 * 1. Calculate the number of timer ticks needed for the desired duration
 *    Formula: N = duration * (16MHz / 2^prescaler)
 * 2. Enable COMPARE[0] interrupt
 * 3. Enable and set priority for TIMER4 IRQ in NVIC
 * 4. Configure TIMER4:
 *    - Set prescaler (0-9, where 0 = no division)
 *    - Set bit mode (use 24-bit: value 2)
 *    - Set mode to Timer (value 0)
 *    - Set CC[0] register to N
 * 5. Start the timer
 * 
 */
void timer_init(uint8_t duration) {
    // TODO: Calculate prescaler and timer frequency
    const uint32_t prescaler = 0; // 0-9
    
    // TODO: Calculate timer frequency
    // fTimer = 16MHz / 2^prescaler
    uint32_t fTimer = 0; // FILL THIS IN
    
    // TODO: Calculate number of ticks for desired duration
    // N = duration * fTimer
    uint32_t N = 0; // FILL THIS IN
    
    // TODO: Enable COMPARE[0] interrupt
    // Hint: Set bit 16 in INTENSET register
    NRF_TIMER4->INTENSET = 0; // FILL THIS IN
    
    // TODO: Enable TIMER4 interrupt in NVIC
    // FILL THIS IN
    
    // TODO: Set NVIC priority for TIMER4
    // FILL THIS IN
    
    // TODO: Configure TIMER4 prescaler
    NRF_TIMER4->PRESCALER = 0; // FILL THIS IN
    
    // TODO: Set bit mode (24-bit = 2)
    NRF_TIMER4->BITMODE = 0; // FILL THIS IN
    
    // TODO: Set timer mode (Timer = 0)
    NRF_TIMER4->MODE = 0; // FILL THIS IN
    
    // TODO: Set compare value
    NRF_TIMER4->CC[0] = 0; // FILL THIS IN
    
    // TODO: Start the timer
    // FILL THIS IN
}

/**
 * Reset and restart TIMER4
 */
void reset_timer() {
    NRF_TIMER4->TASKS_STOP = 1;
    NRF_TIMER4->TASKS_CLEAR = 1;
    NRF_TIMER4->TASKS_START = 1;
}

/**
 * TODO 2: Initialize TIMER3 as a counter
 * 
 * This timer will count pulses from the PWM signal via GPIOTE events.
 * Each pulse will increment the counter.
 * 
 * Steps:
 * 1. Set bit mode (use 24-bit: value 2)
 * 2. Set mode to Counter (value 1)
 * 3. Clear the counter
 * 4. Start the counter
 * 
 * Hint: Counter mode increments when TASKS_COUNT is triggered
 */
void counter_init() {
    // TODO: Set bit mode (24-bit = 2)
    NRF_TIMER3->BITMODE = 0; // FILL THIS IN
    
    // TODO: Set counter mode (Counter = 1)
    NRF_TIMER3->MODE = 0; // FILL THIS IN
    
    // TODO: Clear the counter
    // FILL THIS IN
    
    // TODO: Start the counter
    // FILL THIS IN
}

/**
 * Read the current counter value
 * 
 * Returns: Current count value
 */
uint32_t read_counter() {
    NRF_TIMER3->TASKS_CAPTURE[1] = 1;
    return NRF_TIMER3->CC[1];
}

/**
 * Initialize PWM on pin 11
 */
void init_pwm(uint16_t pwm, uint32_t prescalar, uint32_t counterTop) {
    uint16_t pwm_seq[4] = ;

    NRF_PWM0->PSEL.OUT[0] = ;

    NRF_PWM0->ENABLE = ;
    NRF_PWM0->MODE = ;
    NRF_PWM0->PRESCALER = ;
    NRF_PWM0->COUNTERTOP = ;
    NRF_PWM0->LOOP = ;
    NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | 
                        (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);

    NRF_PWM0->SEQ[0].PTR = ((uint32_t)(pwm_seq) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM0->SEQ[0].CNT = ((sizeof(pwm_seq) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;

    NRF_PWM0->TASKS_SEQSTART[0] = 1;

    printf("PWM_DECODER_MODE_Pos: %ld\n", PWM_DECODER_MODE_Pos);
}

/**
 * TODO 3: GPIOTE Interrupt Handler
 * 
 * This handler responds to two events:
 * 1. EVENTS_IN[0]: Button 0 press - adjust PWM frequency
 * 2. EVENTS_IN[1]: PWM pulse on pin 13 - increment counter
 * 
 * For EVENTS_IN[0]:
 * - Clear the event flag
 * - Check switch position (pin 22) to determine increase/decrease
 * - Adjust counterTop by PWM_INCREMENT
 * - Reinitialize PWM with new parameters
 * 
 * For EVENTS_IN[1]:
 * - Clear the event flag
 * - Trigger TIMER3 to count (TASKS_COUNT)
 */
void GPIOTE_IRQHandler(void) {
    
    // TODO: Check if EVENTS_IN[0] occurred (button press)
    if (0) { // FILL THIS IN - Check NRF_GPIOTE->EVENTS_IN[0]
        
        // TODO: Clear EVENTS_IN[0] flag
        // FILL THIS IN
        
        // TODO: Check switch position (pin 22) to determine direction
        if (0) { // FILL THIS IN - Use gpio_read(22) == 0
            // TODO: Increase counterTop
            // FILL THIS IN
            
            // TODO: Reinitialize PWM with new parameters
            // FILL THIS IN
        } else {
            // TODO: Decrease counterTop
            // FILL THIS IN
            
            // TODO: Reinitialize PWM with new parameters
            // FILL THIS IN
        }
    }
    
    // TODO: Check if EVENTS_IN[1] occurred (PWM pulse detected)
    if (0) { // FILL THIS IN - Check NRF_GPIOTE->EVENTS_IN[1]
        
        // TODO: Clear EVENTS_IN[1] flag
        // FILL THIS IN
        
        // TODO: Increment TIMER3 counter
        // Hint: Trigger TASKS_COUNT
        // FILL THIS IN
    }
}

/**
 * TODO 4: TIMER4 Interrupt Handler
 * 
 * This handler is called periodically (every 'duration' seconds).
 * It reads the pulse count, stores it, and resets the measurement.
 * 
 * Steps:
 * 1. Clear EVENTS_COMPARE[0] flag
 * 2. Read the counter value and store in lastCount
 * 3. Reinitialize the counter (clear it)
 * 4. Reset the timer for next measurement period
 */
void TIMER4_IRQHandler(void) {
    
    // TODO: Clear EVENTS_COMPARE[0]
    // FILL THIS IN
    
    // TODO: Read counter and store in lastCount
    lastCount = 0; // FILL THIS IN - Use read_counter()
    
    // TODO: Reinitialize counter (clear for next measurement)
    // FILL THIS IN
    
    // TODO: Reset timer
    // FILL THIS IN
}

int main(void) {
    ret_code_t error_code = NRF_SUCCESS;

    // Initialize RTT library
    error_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(error_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    printf("Log initialized!\n");

    // Initialize SPI for display
    nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
    nrf_drv_spi_config_t spi_config = {
        .sck_pin = BUCKLER_LCD_SCLK,
        .mosi_pin = BUCKLER_LCD_MOSI,
        .miso_pin = BUCKLER_LCD_MISO,
        .ss_pin = BUCKLER_LCD_CS,
        .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
        .orc = 0,
        .frequency = NRF_DRV_SPI_FREQ_4M,
        .mode = NRF_DRV_SPI_MODE_2,
        .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
    };
    error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
    APP_ERROR_CHECK(error_code);

    /**
     * TODO 5: Configure GPIOTE for button and PWM input
     * 
     * GPIOTE CONFIG[0]: Button 0 (pin 28)
     * - Mode: Event (1)
     * - Pin: 28
     * - Polarity: Hi to Lo (2)
     * 
     * GPIOTE CONFIG[1]: PWM input (pin 13)
     * - Mode: Event (1)
     * - Pin: 13
     * - Polarity: Lo to Hi (1)
     */
    
    // TODO: Configure GPIOTE CONFIG[0] for button 0
    
        
    // TODO: Enable interrupt for GPIOTE channel 0    
    
    // TODO: Configure GPIOTE CONFIG[1] for PWM input (pin 13)
    
    
    // TODO: Enable interrupt for GPIOTE channel 1    
    
    // TODO: Enable GPIOTE interrupt in NVIC
    // FILL THIS IN
    
    // TODO: Set NVIC priority for GPIOTE
    // FILL THIS IN
    
    // Measurement duration (1 second)
    uint8_t duration = 1;
    
    // Initialize peripherals
    init_pwm(pwm, prescalar, counterTop);
    timer_init(duration);
    counter_init();
    
    // Configure pin 22 as input (for switch)    
    
    // Main loop
    while (1) {
        char buf[2][16];
        
        // TODO 6: Display frequency on LCD
        // Uncomment these lines after display is configured
        // snprintf(buf[0], sizeof(buf[0]), "targ: %.2f Hz", (float)((float)pwm / (float)counterTop) * 1000);
        // snprintf(buf[1], sizeof(buf[1]), "real: %d.00 Hz", lastCount);
        // display_write(buf[0], 0);
        // display_write(buf[1], 1);
        
        // Print frequency measurements
        printf("Target: %.2f Hz \n", (float)((float)pwm/(float)counterTop) * 1000);
        printf("Measured: %d.00 Hz \n", lastCount);
        
        __WFI(); // Wait for interrupt
    }
}

