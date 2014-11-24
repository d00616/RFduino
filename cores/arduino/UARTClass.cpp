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

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//#include "variant.h"
#include "nrf51.h"
#include "WInterrupts.h"

// Remove me
#include "nrf_gpio.h"

// Constructors

UARTClass::UARTClass( RingBuffer* pRx_buffer, RingBuffer* pTx_buffer )
{
  _rx_buffer = pRx_buffer ;
  _tx_buffer = pTx_buffer ;
  UART0_State = UART0_State_NotStarted;
}

// Public Methods

void UARTClass::begin( const uint32_t dwBaudRate )
{
  this->begin( dwBaudRate, UART0_RX_PIN, UART0_TX_PIN, UART0_RTS_PIN, UART0_CTS_PIN );
}

void UARTClass::begin( const uint32_t dwBaudRate, uint8_t rx_pin, uint8_t tx_pin )
{
  this->begin( dwBaudRate, rx_pin, tx_pin, 255, 255 );
}

void UARTClass::begin( const uint32_t dwBaudRate, uint8_t rx_pin, uint8_t tx_pin, uint8_t rts_pin, uint8_t cts_pin )
{
  // must STOP before restarting
  if (UART0_State != UART0_State_NotStarted)
    return;

  transmitting = false;

  // Configure TX Pin
  NRF_GPIO->PIN_CNF[tx_pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
              | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
              | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
              | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
              | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_UART0->PSELTXD = tx_pin;

  // Configure RX Pin
  NRF_GPIO->PIN_CNF[rx_pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
              | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
              | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
              | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
              | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
  NRF_UART0->PSELRXD = rx_pin;

  if ( (rts_pin<255) && (cts_pin<255))
  {
    // Configure RTS pin (output)
    NRF_GPIO->PIN_CNF[rts_pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
              | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
              | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
              | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
              | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
    NRF_UART0->PSELRTS = rts_pin;

    // Configure CTS pin (input)
    NRF_GPIO->PIN_CNF[cts_pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
              | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
              | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
              | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
              | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
    NRF_UART0->PSELCTS = cts_pin;
    
    // Enable Hardware Flow Control (HWFC)
    NRF_UART0->CONFIG  = (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
  }


  uint32_t dw;
  
  // Fixed baud rate needed?
  #ifndef UART0_FIXED_BAUD_RATE
    dwBaudRate=UART0_FIXED_BAUD_RATE;
    switch (dwBaudRate)
  #else
    switch (UART0_FIXED_BAUD_RATE)
  #endif
  {
    case 1200: dw = UART_BAUDRATE_BAUDRATE_Baud1200; break;
    case 2400: dw = UART_BAUDRATE_BAUDRATE_Baud2400; break;
    case 4800: dw = UART_BAUDRATE_BAUDRATE_Baud4800; break;
    case 9600: dw = UART_BAUDRATE_BAUDRATE_Baud9600; break;
    case 14400: dw = UART_BAUDRATE_BAUDRATE_Baud14400; break;
    case 19200: dw = UART_BAUDRATE_BAUDRATE_Baud19200; break;
    case 28800: dw = UART_BAUDRATE_BAUDRATE_Baud28800; break;
    case 31250: dw = UART_BAUDRATE_BAUDRATE_Baud31250; break;
    case 38400: dw = UART_BAUDRATE_BAUDRATE_Baud38400; break;
    case 57600: dw = UART_BAUDRATE_BAUDRATE_Baud57600; break;
    case 76800: dw = UART_BAUDRATE_BAUDRATE_Baud76800; break;
    case 115200: dw = UART_BAUDRATE_BAUDRATE_Baud115200; break;
    case 230400: dw = UART_BAUDRATE_BAUDRATE_Baud230400; break;
    case 250000: dw = UART_BAUDRATE_BAUDRATE_Baud250000; break;
    case 460800: dw = UART_BAUDRATE_BAUDRATE_Baud460800; break;
    case 921600: dw = UART_BAUDRATE_BAUDRATE_Baud921600; break;
    case 1000000: dw = UART_BAUDRATE_BAUDRATE_Baud1M; break;
  }

  NRF_UART0->BAUDRATE         = (dw << UART_BAUDRATE_BAUDRATE_Pos);
  NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);

  UART0_State = UART0_State_BeforeFirstTX;


  NRF_UART0->INTENSET        |= (UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos )
                              | (UART_INTENSET_TXDRDY_Enabled << UART_INTENSET_TXDRDY_Pos );

  attachInterrupt(UART0_IRQn, UART0_Interrupt );

  
  NRF_UART0->TASKS_STARTTX = 1;
  NRF_UART0->TASKS_STARTRX = 1;
  NRF_UART0->EVENTS_RXDRDY    = 0;
  NRF_UART0->EVENTS_TXDRDY    = 0;
}

void UARTClass::end( void )
{
  if (UART0_State == UART0_State_NotStarted)
    return;

  // Wait for any outstanding data to be sent
  this->flush();

  NRF_UART0->TASKS_STOPTX = 1;
  NRF_UART0->TASKS_STOPRX = 1;

  NRF_UART0->ENABLE       = (UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos);

  // Disable UART interrupt in NVIC
  detachInterrupt(UART0_IRQn);

  // Disconnect high drive tx pin
  int tx_pin = NRF_UART0->PSELTXD;
  NRF_GPIO->PIN_CNF[tx_pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
              | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
              | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
              | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
              | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);                    

  UART0_State = UART0_State_NotStarted;

  // clear any received data
  _rx_buffer->_iHead = _rx_buffer->_iTail ;

}

int UARTClass::available( void )
{
  return (uint32_t)(SERIAL_BUFFER_SIZE + _rx_buffer->_iHead - _rx_buffer->_iTail) % SERIAL_BUFFER_SIZE ;
}

int UARTClass::peek( void )
{
  if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
    return -1 ;

  return _rx_buffer->_aucBuffer[_rx_buffer->_iTail] ;
}

int UARTClass::read( void )
{
  // if the head isn't ahead of the tail, we don't have any characters
  if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
    return -1 ;

  uint8_t uc = _rx_buffer->_aucBuffer[_rx_buffer->_iTail] ;
  _rx_buffer->_iTail = (unsigned int)(_rx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE ;
  return uc ;
}

void UARTClass::flush( void )
{
  while (transmitting)
    ;
}

void UARTClass::tx( void )
{
  // if something in buffer?
  if ( _tx_buffer->_iHead == _tx_buffer->_iTail )
  {
    // set transmitting state top stop
    transmitting = false;
    return;
  }

  // set transmitting state to start
  transmitting = true;

  uint8_t uc = _tx_buffer->_aucBuffer[_tx_buffer->_iTail] ;
  _tx_buffer->_iTail = (unsigned int)(_tx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE ;

  // tx byte
  NRF_UART0->TXD = uc;

  // don't change start if not started
  if (UART0_State == UART0_State_BeforeFirstTX)
    UART0_State = UART0_State_AfterFirstTX;
}

size_t UARTClass::write( const uint8_t uc_data )
{
  // Wait if tx_buffer space is not available
  while ((_tx_buffer->_iHead + 1) % SERIAL_BUFFER_SIZE == _tx_buffer->_iTail)
    ;

  _tx_buffer->store_char(uc_data);

  if (! transmitting)
    tx();

  return 1;
}

// IRQ handler
void UART0_Interrupt()
{
  Serial.IrqHandler();
}

void UARTClass::IrqHandler( void )
{
  if (NRF_UART0->EVENTS_TXDRDY)
  {
    // TXReset
    NRF_UART0->EVENTS_TXDRDY = 0;
    tx();
  }

  // did we receive data
  if (NRF_UART0->EVENTS_RXDRDY)
  {
    // RXReset
    NRF_UART0->EVENTS_RXDRDY = 0;

    // Read Data
    uint8_t ch = NRF_UART0->RXD;

    // Check errors (UART0_RXErrorReset)
    if (NRF_UART0->ERRORSRC & UART_ERRORSRC_OVERRUN_Msk)
    {
      NRF_UART0->ERRORSRC = (UART_ERRORSRC_OVERRUN_Clear << UART_ERRORSRC_OVERRUN_Pos);
      return;
    }
    else if (NRF_UART0->ERRORSRC & UART_ERRORSRC_FRAMING_Msk)
    {
      NRF_UART0->ERRORSRC = (UART_ERRORSRC_FRAMING_Clear << UART_ERRORSRC_FRAMING_Pos);
      return;
    }

    // Store in buffer
    _rx_buffer->store_char(ch);

    if (serialEvent)
      serialEvent();
  }
}

int UARTClass::UART0_BaudRate()
{
  switch ((NRF_UART0->BAUDRATE & UART_BAUDRATE_BAUDRATE_Msk) >> UART_BAUDRATE_BAUDRATE_Pos)
  {
    case UART_BAUDRATE_BAUDRATE_Baud1200: return 1200;
    case UART_BAUDRATE_BAUDRATE_Baud2400: return 2400;
    case UART_BAUDRATE_BAUDRATE_Baud4800: return 4800;
    case UART_BAUDRATE_BAUDRATE_Baud9600: return 9600;
    case UART_BAUDRATE_BAUDRATE_Baud14400: return 14400;
    case UART_BAUDRATE_BAUDRATE_Baud19200: return 19200;
    case UART_BAUDRATE_BAUDRATE_Baud28800: return 28800;
    case UART_BAUDRATE_BAUDRATE_Baud31250: return 31250;
    case UART_BAUDRATE_BAUDRATE_Baud38400: return 38400;
    case UART_BAUDRATE_BAUDRATE_Baud57600: return 57600;
    case UART_BAUDRATE_BAUDRATE_Baud76800: return 76800;
    case UART_BAUDRATE_BAUDRATE_Baud115200: return 115200;
    case UART_BAUDRATE_BAUDRATE_Baud230400: return 230400;
    case UART_BAUDRATE_BAUDRATE_Baud250000: return 250000;
    case UART_BAUDRATE_BAUDRATE_Baud460800: return 460800;
    case UART_BAUDRATE_BAUDRATE_Baud921600: return 921600;
    case UART_BAUDRATE_BAUDRATE_Baud1M: return 1000000;
  }
  return 0;
}

