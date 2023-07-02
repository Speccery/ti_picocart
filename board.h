
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

#ifndef _BOARD_H_
#define _BOARD_H_

#define BOARD_VER11 // Version 1.1 of the board

// GPIO bidirection databus is at bits 8..15
#define PCn_D0      8
#define PCn_D7      15
#define PCn_DATAMASK 0x0000FF00
// GPIO input bit numbers
#define PCnI_ROMCS  19
#define PCnI_WE     18
#define PCnI_GS     17
#define PCnI_DBIN   16
// GPIO control outputs
#if defined(BOARD_VER11)
#define SPI_SCK     2
#define SPI_MOSI    3
#define SPI_MISO    4
#define PCnO_PSRAM_CS 5
#define PCn_DQ2     6
#define PCn_DQ3     7
#define PCnO_SD_CS  27
#define PCnO_SEL0   20
#define PCnO_SEL1   21
#define PCnO_LEDS   26
#else
// Revision 1.0 board.
#define PCnO_CSLOA  6       // Drive low address to databus (active low)
#define PCnO_CSHIA  7       // Drive high address to databus (active low)
#define PCnO_CSDAT  21      // Activate databus buffer (careful!)
#define PCnO_DDIR   20      // Set direction of databus buffer (1=read,0=write)
#define PCnO_DEBUG26 26     // Labeled ADC0 on the picocart.
#define PCnO_DEBUG27 27
#endif

#define PCnO_GRDY   22      // Drive GREADY low when zero, tristate when 1
#define PCnI_BA0    28      // Labeled ADC2. Connected to address line 0.
#define LED_PIN     25

void init_grom_server();
unsigned grom_cs_low_process();
unsigned rom_server();

uint16_t read_address();

#endif

