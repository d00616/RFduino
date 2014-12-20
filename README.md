### nRF51duino

This Software is completely unstable at the moment. It looks like there is a problem with
linker scripts or startup code. At my opinion the initialisation if bss and other segments
are not correctly. Code like printf, serial ring buffer are not working correctly.

At the moment i'm evaluate if RIOT OS is usable for me. If you are interested on using NRF51
with Arduino, so feel free to fork this project. I have added my unfinised startup code here.
(system/nRF51/source/unfinished-startup-code)

This code supports nRF51822 and nRF41422 chips from Nordic Semiconductor. At the moment no
Softdevice functionality is supported out of the box. A sketch cannot be compiled without
Nordic SDK. Compliling a sketch without SDK and a native Radio Library (nRF24 compatible)
is planned. Feel free to add Softdevice or native BLE support so BLE or ANT+ can be used
without Nordic nRF51 SDK.

Please keep in mind that using Nordic SDK is not open source friendly at the moment. All
example code and a lot of header files doesn't allow redistribution of source code.

To start you need an nRF51 development board with an JLink Adapter. You need to buy an offical
development kit. If you plan to use more than one board choose an Development Kit with an external
JLink adapter. If you want to use ANT+ protocol choose an nRF51422 or nRF51922 kit.

Whats not working at the moment
 - Flashing Software under Windows
 - Anything needs memory
 - A lot of untested functionality

Tested at the moment:
 - Startcode
 - RTC
 - Delay

I Need help to:
 - remove linker warnings
 - DigitalWrite
 - Serial interface (memory problems)
 - write libraries for integrated chip hardware (Random, AES, CRC, Radio, Timer, RTC0, ...)
 - port my flash script to windows
 - test my flash script on OSX
 - remove Nordic SDK dependencies
 ...

Not planned by me, but a good idea making this software better:
 - support SoftDevices without Nordic SDK (https://github.com/mrquincle/bluenet) or Porting a
   complete BLE stack http://code.google.com/p/btstack/
 - A Jlink indipended Bootloader

This is a fork of RFduino with broken compatibility because there are a lot of non open source
files in RFduino. I have broken command compatibility to get more indipended and make it easy
recreate core functionality. Thanks RFduino for your preliminary work.

WARNING: Please don't connect more then one board at the same thime when flashing.

### Installation

* Buy an Nordic NRF51 HDK, Download SDK and JLink Software

* Download Arduino 1.5 here: [http://arduino.cc](http://arduino.cc/en/Main/Software)  
  (on osx remember to open arduino first to make gatekeeper perform its magic)  

* Copy the nRF51duino directory from this repository in Arduino  
  (on Windows, C:\arduino-1.5.4\hardware\arduino)  
  (on OSX, /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino)  
  or "git clone https://github.com/d00616/nRF51duino" in the directory indicated

* Copy Nordic NRF51 SDK to hardware/arduino/nRF51duino/system/nRF51-SDK

* Install SEGGER JLINK Software

Your ready to go!

### Getting Started

* Select a nRF51 Board from the Tools/Board menu

* Select an example from the Files/Examples/RFduinoNonBLE directory

* Select Upload to compile, upload and execute the sketch

### Define you own board layout

nRF51duino comes with predifinions for an PCA10000 Board. You can create your
own "custom_board.h" file in your Sketch to override settings.
