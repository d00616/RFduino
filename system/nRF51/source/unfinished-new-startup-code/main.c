/*
 * This is an LED blinking example without SDK dependency
 * High drive is enabled. A maximum current of 15mA for all enabled ports are available (3*5mA .. 15*1mA)
 *
 * Red LED on PCA10000
 */

#include <stdint.h>
#include "nrf51.h"
#include "nrf51_bitfields.h"

#define LED_PIN 21

/**
 * Configure a pin as high drive output
 */
static __INLINE void configure_pin(uint32_t pin)
{
    NRF_GPIO->PIN_CNF[pin]  = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                            | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                            | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
}

// Set a Pin
static __INLINE void set_pin(uint32_t pin)
{
    NRF_GPIO->OUTSET = (1UL << pin);
}

// Clear a Pin
static __INLINE void clear_pin(uint32_t pin)
{
    NRF_GPIO->OUTCLR = (1UL << pin);
}


/**
 * Main function
 */
int main(void)
{
    // Configure Pin as HIGH drive Output
    configure_pin(LED_PIN);
    
    // endless loop
    for(char c=0;;c++)
    {
	if (c & 1) clear_pin(LED_PIN);
		else set_pin(LED_PIN);

	for (uint32_t delay1=0; delay1<65535; delay1++)
	{
		for (uint8_t delay2=0; delay2<10; delay2++)
		{
		}
	}
   }
}

