### Warning ###

This code is not working at the moment for plain nRF51422, nRF51822 and nRF51922 chips. It possible to
compile an arduino sketch but it looks like its not starting. Last time i flashed the above example my previous
code was started.

To use this Software you need access to original nRF51 SDK (Tested with version v6.0.0.43681).
Copy nRF51822 directory to hardware/arduino/RFduino/system/nRF51-SDK/nRF51822

To flash compiled code at the moment, call JLinkExe manually:
	/opt/arduino-1.5.7/hardware/tools/gcc-arm-none-eabi-4.8.3-2014q1/bin/arm-none-eabi-objcopy -O binary /tmp/build2977480947880635154.tmp/Blink.cpp.elf /tmp/build2977480947880635154.tmp/Blink.cpp.bin
	JLinkExe
		r
		loadbin /tmp/build2977480947880635154.tmp/Blink.cpp.bin 00000000
		r
		g
		exit

My example code for an PCA10000 board. LEDs stay off at the moment
<code>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"


void setup() {
  // put your setup code here, to run once:
      // Configure Pins as HIGH drive Output
    #ifdef LED_RGB_RED
     configure_pin(LED_RGB_RED);
    #endif

    #ifdef LED_RGB_GREEN
     configure_pin(LED_RGB_GREEN);
    #endif
   
    #ifdef LED_RGB_BLUE
     configure_pin(LED_RGB_BLUE);
    #endif
 
    nrf_gpio_pin_clear(LED_RGB_RED);
    nrf_gpio_pin_set(LED_RGB_BLUE);
    nrf_gpio_pin_clear(LED_RGB_GREEN);
}

void loop() {
  // put your main code here, to run repeatedly:

}

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
</code>

Startup code can be created unter linux with /opt/arduino/hardware/arduino/RFduino/system/nRF51/source/build.sh


### RFduino: Shrunk down an Arduino to the size of a finger-tip and made it Wireless!

![logo](https://raw.github.com/RFduino/RFduino/master/4up%20image.jpg)  

[Video](http://www.youtube.com/watch?v=arWBdGwCJcM)  

### Installation

* Get the hardware here: [http://RFduino.com](http://RFduino.com)

* Download Arduino 1.5 here: [http://arduino.cc](http://arduino.cc/en/Main/Software)  
  (on osx remember to open arduino first to make gatekeeper perform its magic)  

* Copy the RFduino directory from this repository in Arduino  
  (on Windows, C:\arduino-1.5.4\hardware\arduino)  
  (on OSX, /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino)  
  or "git clone https://github.com/RFduino/RFduino" in the directory indicated

* Install the FTDI drivers from here: [http://ftdichip.com](http://www.ftdichip.com/Drivers/VCP.htm)

Your ready to go!

Detailed instructions are available here: [Quick Start Guide](http://files.rfdigital.com/rfduino.quick.start.guide.pdf)

### Getting Started

* Attach the USB shield

* Select RFduino from the Tools/Board menu

* Select the port from the Tools/Port menu

* Select an example from the Files/Examples/RFduinoNonBLE or Files/Examples/RFduinoBLE directory

* Select Upload to compile, upload and execute the sketch

* Down the iPhone example apps from the iPhone App Store (search for "RFduino").

Detailed instructions for the Temperature app are available here: [Temperature App](http://files.rfdigital.com/rfduino.temperature.guide.pdf)  
Detailed instructions for the ColorWheel app are available here: [ColorWheel App](http://files.rfdigital.com/rfduino.rgb.colorwheel.guide.pdf)  

### Communicating with us

The best way to communiate with us is here: [RFduino Forum](http://forum.RFduino.com).

### A Lot More Coming!

We are working on documentation as fast as we can.  Until then, the best source for documentation is the sketch examples.

This project has been a huge success, and we have many exciting things planned that we want to share with the community.  At the same time, we are unbelievable busy, with a ton going on and lots of small items to clean up.  We are just a small team working on this project ... please bear with us!

We hope you enjoy creating stuff with your RFduino as much as we do!

The RFduino team.
