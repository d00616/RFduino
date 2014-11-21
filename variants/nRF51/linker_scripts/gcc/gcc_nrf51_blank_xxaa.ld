/* Linker script to configure memory regions. */

SEARCH_DIR("hardware/arduino/nRF51duino/variants/nRF51/linker_scripts/gcc")
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x40000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x4000
}


INCLUDE "gcc_nrf51_common.ld"
