
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
#define PCnO_CSLOA  6       // Drive low address to databus (active low)
#define PCnO_CSHIA  7       // Drive high address to databus (active low)
#define PCnO_CSDAT  21      // Activate databus buffer (careful!)
#define PCnO_DDIR   20      // Set direction of databus buffer (1=read,0=write)
#define PCnO_GRDY   22      // Drive GREADY low when zero, tristate when 1

#define LED_PIN     25

void init_grom_server();
unsigned grom_cs_low_process();

#endif

