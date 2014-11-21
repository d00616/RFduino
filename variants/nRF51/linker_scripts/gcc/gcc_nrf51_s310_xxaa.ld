/* Linker script to configure memory regions. */

SEARCH_DIR("hardware/arduino/nRF51duino/variants/nRF51/linker_scripts/gcc")
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00020000, LENGTH = 0x20000 
  RAM (rwx) :  ORIGIN = 0x20002400, LENGTH = 0x1C00 
}


INCLUDE "gcc_nrf51_common.ld"