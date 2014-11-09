/* Linker script to configure memory regions. */

SEARCH_DIR("hardware/arduino/RFduino/variants/nRF51/linker_scripts/gcc")
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00018000, LENGTH = 0x8000 
  RAM (rwx) :  ORIGIN = 0x20002800, LENGTH = 0x1800 
}


INCLUDE "gcc_nrf51_common.ld"
