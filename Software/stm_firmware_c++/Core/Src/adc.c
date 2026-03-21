#include "adc.h"

// TODO: Make sure SCLK and CLK are configured correctly 
// TODO: Setup CRC stuff 

// Setup/Class initialization 
    // Set reset pin low to stop ADC during setup 
    // Setup SPI bus (clock speed configured to 12MHz in Cube)
    // Configure PWM for the ADC external CLC (TIMER2-CH1)
    // After reset time, set the (active low) RST pin for the ADC to pull it out of reset and start it running
        // If the pin is active low why set it low for setup to stop ADC??

    // Setup the OSR and Gain for the fin (remember don't need all four fins now)

// Set Gain Method

// Set OSR Method 

// Disable ADC channels method

// Enable ADC channels method

// Read Register Method

// Read ADC Method

// Write register method 

// Assert chip select 

// unassert chip select 

// convert volts to temp

// convert adc to volts 


    
