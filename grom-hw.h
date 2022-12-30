
/*
 * groms-hw.h
 *
 * GROM hardware dependent definitions. The idea is to create versions 
 * of this file for the several platforms I have created:
 *  - StrangeCart
 *  - TI-GROMmy
 *  - TI-SuperGROMmy
 * 	- TI-picocart
 * 
 * This version for picocart
 *
 *  Created on: 30 Dec 2022
 *      Author: erikpiehl
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
// #include "main.h"
#include "board.h"


#ifndef DEBUG
// Currently not defining RUN_FROM_FLASH crashes when writing to GROM (in release build).
#define RUN_FROM_FLASH
#endif


#ifndef RUN_FROM_FLASH
__attribute__ ((section(".data")))
#endif

inline static void drive_bus(uint8_t data) {
	// Deactivate address bus buffers
	sio_hw->gpio_set = (1 << PCnO_CSHIA) | (1 << PCnO_CSLOA);
	// Activate data bus buffer, and make direction towards TI.
	sio_hw->gpio_clr = (1 << PCnO_DDIR) | (1 << PCnO_CSDAT); // Buffer direction: towards TI. Enable buffer.
	sio_hw->gpio_oe_set = PCn_DATAMASK;	// RP2040 databus out.
	unsigned d = data << 8;
	// Set the databus bits high or low based on the data.
	sio_hw->gpio_set = d;
	sio_hw->gpio_clr = PCn_DATAMASK & ~d;
	gpio_put(LED_PIN, 1);
}

/**
 * @brief turn data direction in, good for reading and not disturbing the bus.
 * 	All databus pins are in PORTB.
 * 	Bits 0..7.
 * 	GPIO MODER register has 2 bits per bit. For bit 0 they are bits 1 and 0.
 * 	Encoding (x means below the pin from 0..7):
 * 	MODEx 00 = input
 * 	      01 = output
 * 	      10 = analog functions
 * 	      11 = input/output
 */
#ifndef RUN_FROM_FLASH
__attribute__ ((section(".data")))
#endif

inline static void data_dir_in() {
	gpio_put(LED_PIN, 0);
	sio_hw->gpio_oe_clr = PCn_DATAMASK;	// RP2040 databus pins as inputs
	sio_hw->gpio_set = (1 << PCnO_CSHIA) | (1 << PCnO_CSLOA) | (1 << PCnO_DDIR);	// Deactivate address bus buffers, turn data bus as input.
	sio_hw->gpio_clr = (1 << PCnO_CSDAT);	// Enable external data bus buffer chip, now driving towards RP2040 pins.
}

/**
 * @brief return data from the bus
 * 
 * @return uint8_t 
 */
inline static uint8_t read_data() {
	data_dir_in();
	__asm volatile ("nop");
	__asm volatile ("nop");
	__asm volatile ("nop");
	uint32_t a = (sio_hw->gpio_in & PCn_DATAMASK) >> PCn_D0;
	return (uint8_t)a;
}

/**
 * @brief Get the status of the MO pin
 * 
 * @return unsigned 0/1
 */
inline static unsigned get_mo() {
	gpio_put(LED_PIN, 0);
	// Set buffers so that only the low address bus buffer is enabled.
	sio_hw->gpio_oe_clr = PCn_DATAMASK;	// RP2040 databus pins as inputs
	sio_hw->gpio_set = (1 << PCnO_CSHIA) | (1 << PCnO_CSDAT);	// Deactivate databus and addr HI drivers
	sio_hw->gpio_clr = (1 << PCnO_CSLOA);	// Enable external addr LO buffer chip.
	__asm volatile ("nop");
	__asm volatile ("nop");
	__asm volatile ("nop");	
	uint32_t a = (sio_hw->gpio_in & (1 << (PCn_D0+1)));
	return a >> (PCn_D0+1);
}

/**
 * @brief Get the RnW pin status
 * 
 * @return unsigned 0/1
 */
inline static unsigned get_RnW() {
	// return 1 & (!gpio_get(PCnI_DBIN));	// return DBIN, inverted
	return gpio_get(PCnI_WE);
}

/**
 * @brief Get the GROM cs pin state
 * 
 * @return unsigned 0/1
 */
inline static unsigned get_cs() {
	return (sio_hw->gpio_in & (1 << PCnI_GS)) >> PCnI_GS;
}

/**
 * @brief same as above but does not return 0/1 but 0 and non-zero
 */
inline static unsigned get_cs_Y() {
	return get_cs();	// lazy
}

/**
 * @brief Set gready pin state low. this is an open collector pin,
 * so actively driving it low. 
 * On the picocart this controls the output enable of 74LVC1G125 chip.
 */
inline static void set_gready_low() {
	sio_hw->gpio_clr = 1 << PCnO_GRDY;
}

/**
 * @brief Set the gready pin high. It's an open collector pin,
 * so setting it high means just don't drive it low anymore.
 * On the picocart this controls the output enable of 74LVC1G125 chip.
 * Thus setting it high enables pull up resistor on the motherboard
 * to do its stuff.
 */
inline static void set_gready_high() {
	sio_hw->gpio_set = 1 << PCnO_GRDY;
}

inline static void grom_sample_delay() {
	__asm volatile ("nop");
	__asm volatile ("nop");
	__asm volatile ("nop");
	__asm volatile ("nop");
	__asm volatile ("nop");
	__asm volatile ("nop");
	__asm volatile ("nop");
	__asm volatile ("nop");
	__asm volatile ("nop");
}

#define READ_DIR_MODE ((get_mo() << 1) | get_RnW())

extern const unsigned char grom994a_data[];
extern const unsigned char grominvaders_data[];

