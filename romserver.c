/*
 * romserver.c
 *
 * Derived from StrangeCart codebase, a version of the ROM server
 * handler for the picocart.
 * Designed to run on core1 (i.e. the core not doing USB or other I/O)
 *
 *  Created on: 31 Dec 2022
 *      Author: erikpiehl
 */
#include <stdio.h>
#include "board.h"

#include "grom-hw.h"	// Hardware dependent stuff.

/**
 * @brief read the address bus as presented by the TI's cartridge port.
 * 
 * @return uint16_t 
 */

uint16_t __time_critical_func(read_address)() {
	sio_hw->gpio_oe_clr = PCn_DATAMASK;	// RP2040 databus pins as inputs
	sio_hw->gpio_set = (1 << PCnO_CSHIA) | (1 << PCnO_CSDAT) | (1 << PCnO_DDIR);	
	sio_hw->gpio_clr = (1 << PCnO_CSLOA);	
	__asm volatile ("nop");
	__asm volatile ("nop");
	__asm volatile ("nop");
    uint32_t a = (sio_hw->gpio_in & PCn_DATAMASK) >> PCn_D0;
    sio_hw->gpio_set = (1 << PCnO_CSLOA);
    sio_hw->gpio_clr = (1 << PCnO_CSHIA);
	__asm volatile ("nop");
	__asm volatile ("nop");
	__asm volatile ("nop");
    a |= (sio_hw->gpio_in & PCn_DATAMASK);
    return a;
}

uint32_t rom_bank = 0;
uint32_t max_bank = 0; 
uint32_t min_bank = 0xFFFF;
uint32_t bank_switches = 0;

/**
 * @brief the actual ROM server routine.
 *  Return data from the emulated "ROM".
 * Note that each read returns two bytes.
 * 
 * @return unsigned 
 */
unsigned __time_critical_func(rom_server)() {

  while( 1 ) {
    if(get_grom_cs() == 0) {
      grom_cs_low_process();
    }
    if(get_rom_cs())
      continue;

    uint16_t a = read_address() & 0x1FFF;
    if(get_RnW() == 0) {
      // Get the bank select information.
      unsigned mask = (active_rom_size >> 13) - 1;
      rom_bank = mask & ((a & 0xFE) >> 1);
      
      bank_switches++;
      if(rom_bank < min_bank) min_bank = rom_bank;
      if(rom_bank > max_bank) max_bank = rom_bank;

      while(get_rom_cs() == 0)
        ;
      return 0;
    }

    uint8_t *rom = active_rom_area + (rom_bank << 13);
    // uint8_t *rom = rominvaders_data;
    uint8_t d = rom[ a ];

    drive_bus( d );
    gpio_put(PCnO_DEBUG26, 1);

    // Wait for the cycle to end.
    uint32_t k;
    int count = 0;
    do {
      if(count++ == 16) {
        // Present the second byte to the TI-99/4A. 
        // Timing here derived from simple counter since we cannot 
        // both drive data out and read the address bus again with the curren picocart.
        d = rom[ a - 1 ];
        drive_bus( d );
        gpio_put(PCnO_DEBUG26, 0);
      }
      k = get_rom_cs();
    } while (!k);
    deactive_data_buffer();
    gpio_put(PCnO_DEBUG26, 0); 

  }
  return 0;
}
