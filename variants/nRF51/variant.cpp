/*
  Copyright (c) 2014 nRF51duino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


/*
 Copyright (c) 2013 OpenSourceRF.com.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"

#define LFCLK_FREQUENCY    (32768UL)

/*
 * UART objects
 */

RingBuffer rxBuffer, txBuffer;

UARTClass Serial( &rxBuffer, &txBuffer );

#ifdef __cplusplus
extern "C" {
#endif

bool override_uart_limit = false;

/*
 * Pins descriptions
 */
extern uint8_t PPI_Channels_Occupied[PINS_COUNT][2] = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
extern uint8_t GPIOTE_Channels_Occupied[PINS_COUNT] = {255, 255, 255, 255, 255, 255, 255};
extern uint8_t Timer1_Compare_Unit_Occupied_by_Pin[3] = {255, 255, 255}; // Contains Pin number which occupied appropriate CC register (0, 1 or 2) of Timer 1 for PWM purposes
extern uint8_t Timer2_Compare_Unit_Occupied_by_Pin[3] = {255, 255, 255}; // Contains Pin number which occupied appropriate CC register (0, 1 or 2) of Timer 2 for PWM purposes
extern uint8_t Pin_Occupied_for_PWM[PINS_COUNT] = {0, 0, 0, 0, 0, 0, 0}; // Determines if particular pin is occupied for PWM output

// declared in wiring.c
extern "C" void RTC1_Interrupt(void);

void rtc_config()
{
    NRF_RTC1->TASKS_STOP = 1;	// Stop RTC timer
    NRF_RTC1->TASKS_CLEAR = 1;	// Clear timer
    NRF_RTC1->PRESCALER = 0;	// No prescaling => 1 tick = 1/32768Hz = 30.517us
    NRF_RTC1->EVTENSET = (RTC_EVTENSET_OVRFLW_Set << RTC_EVTENSET_OVRFLW_Pos); // Enable OVRFLW Event
    NRF_RTC1->INTENSET = (RTC_INTENSET_OVRFLW_Set << RTC_INTENSET_OVRFLW_Pos); // Enable OVRFLW Interrupt
    attachInterrupt(RTC1_IRQn, RTC1_Interrupt);
    NRF_RTC1->TASKS_START = 1;	// Start RTC
}


void __libc_init_array(void);

void init( void )
{
  #ifdef __NRF51_HFCLOCK__
    // Initialize 16 MHz crystal oscillator. 470uA + 1.1mA@400us Startup current
    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    // Wait until oscillator startup is finished
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) { }
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  #else
    // RC oscillator 750uA + 400uA@2.5us Startup current
  #endif
  
  // Start 32768Hz Clock for RTC
  #ifndef __NRF51_LFCLOCK__
      // External oscillator 0.4uA + 1.3uA@300ms Startup current
      NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
  #else
      #ifdef __NRF51_HFCLOCK__
        // Syntetic source 15uA
        NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Synth << CLOCK_LFCLKSRC_SRC_Pos);
      #else
        // RC oscillator 0.5-1.1uA
        NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos);        
      #endif
  #endif
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_LFCLKSTART = 1;
    // Wait until oscillator startup is finished
  while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) { }
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;

  // Initialize C library
  __libc_init_array();
    
  // NRF51822 doesn't implement SysTick, so use the RTC for timing
  rtc_config();
}

uint32_t getDeviceIdLow()
{
  return NRF_FICR->DEVICEID[0];
}

uint32_t getDeviceIdHigh()
{
  return NRF_FICR->DEVICEID[1];
}

uint64_t getDeviceId()
{
  return ((uint64_t)getDeviceIdHigh() << 32) | getDeviceIdLow();
}

#ifdef __cplusplus
}
#endif
