#include <stdint.h>
#include "libnRF51duino.h"
#include "nrf51.h"

/**
 * \brief Ultra low power delay for the amount of time (in milliseconds) specified as parameter.
 *        You can use the following macros: MILLISECONDS(x), SECONDS(x), MINUTES(x), HOURS(x), DAY(x), INFINITE
 *        ie: RFduino_ULPDelay( MINUTES(2) + SECONDS(30) )
 *
 *        RFduino_pinWake() can be used to cause HIGH / LOW edges on GPIO pins to exit ulp_delay.
 *        see documentation for RFduino_pinWake for more details.
 *
 * \param ms the number of milliseconds to switch to ultra low power (max is DAYS(7) unless INFINITE)
 */
//extern void RFduino_ULPDelay( uint64_t ms );

/**
 * \brief Configures the pin to wake the device from system off.
 *
 * \param ulPin The number of the digital pin you want to read (int)
 * \param dwWake Either DISABLED, HIGH or LOW
 */
//extern void RFduino_pinWake( uint32_t ulPin, uint32_t dwWake );

/**
 * \brief Test whether a pin caused us to wakeup and exit ulp_delay
 *
 * \param ulPin The number of the digital pin you want to read (int)
 *
 *        You must call RFduino_resetPinWoke() to reset this condition after processing
 */
//extern int RFduino_pinWoke( uint32_t ulPin );

/**
 * \brief Reset state of pin that caused us to wakeup and exit ulp_delay
 *
 * \param ulPin The number of the digital pin you want to reset
 */
//extern void RFduino_resetPinWake( uint32_t ulPin );

/**
 * \brief Configures the pin to wake the device from system off and execute a callback.
 *
 * \param ulPin The number of the digital pin you want to read (int)
 * \param dwWake Either DISABLED, HIGH or LOW
 * \param callback The callback function to execute when the pin is detected.
 *                 Here is an example pin callback function:
 *                 int my_pin_callback(uint32_t pin)
 *                 {
 *                   // do something
 *                   // return 0 to reset wake_detect, return 1 to exit RFduino_ULPDelay
 *                   return 0;
 *                 }
 *                 To call this function on the when pin 5 goes high:
 *                 RFduino_pinWake(5, HIGH, my_pin_callback);
 */
//extern void RFduino_pinWakeCallback( uint32_t ulPin, uint32_t dwWake, pin_callback_t callback ) ;

/**
 * \brief Reset the system
 */
//extern void RFduino_systemReset(void);

/**
 * \brief Turn the system off
 */
//extern void RFduino_systemOff(void);

/**
 * \brief Get a sample from the temperature sensor
 *
 * \param scale Either CELSIUS or FAHRENHEIT
 *
 * returns:
 *     degrees f (-198.00 to +260.00)
 *     degrees c (-128.00 to +127.00)
 */
//extern float RFduino_temperature(int scale);

void nrf51duino_ppi_channel_assign(uint8_t channel_num, const volatile void * event_endpoint, const volatile void * task_endpoint);
void nrf51duino_ppi_channel_unassign(uint8_t channel_num);

void nrf51duino_nvic_enableirq(uint8_t irq_num)
{
  NVIC_ClearPendingIRQ(irq_num);
  NVIC_EnableIRQ(irq_num);
}

inline void nrf51duino_nvic_disableirq(uint8_t irq_num)
{
  NVIC_DisableIRQ(irq_num);
}

//void rfduino_nvic_setpriority(uint8_t irq_num, uint32_t priority);

// 1 if the BLE radio is active (or about to become active), 0 if not
//extern volatile int RFduinoBLE_radioActive;
