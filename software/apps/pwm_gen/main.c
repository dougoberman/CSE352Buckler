#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_pwm.h"

#define PWM_INSTANCE    NRF_PWM0  // Use PWM0 instance
#define PWM_PIN1        23        // GPIO pin for PWM channel 1
#define PWM_PIN2        24        // GPIO pin for PWM channel 2
#define PWM_PIN3        25        // GPIO pin for PWM channel 3

#define PWM_FREQUENCY   1000      // PWM frequency in Hz
#define PWM_TOP_VALUE   --        // Find the corresponding TOP VALUE using the formula in slides

// Define duty cycles (values from 0 to PWM_TOP_VALUE)
uint16_t pwm_seq_values[] = {
    ...,  // Fraction of PWM_TOP_VALUE -- the value to go to COMP0 register
    ...,  // Fraction of PWM_TOP_VALUE -- the value to go to COMP1 register
    ...,  // Fraction of PWM_TOP_VALUE -- the value to go to COMP2 register
    0                            // Unused
};


// Function to initialize PWM
void pwm_init(void) {
    // Configure PWM pins, 

    // Configure PWM mode and PRESCALER

    // Configure LOAD, COUNTERTOP, LOOP, SEQ[0]

    // Enable PWM and start sequence
    
    
}

int main(void) {
    pwm_init();

    while (1) {
        // Infinite loop, PWM runs independently
    }
}
