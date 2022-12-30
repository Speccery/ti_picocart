/*
 * gromserver.c
 *
 * Originally from StrangeCart codebase, a portable version of this stuff.
 * Used in GROMmy boards and picocart as well.
 *
 *  Created on: 3 Sep 2022
 *      Author: erikpiehl
 */
#include <stdio.h>
#include "board.h"

#include "grom-hw.h"	// Hardware dependent stuff.

//------------------------------------------------
//--- StrangeCart GROM support -------------------
//------------------------------------------------

// TI-GROMmy2 performance:
// Write 6.96us Read 4.28us
// Write 6.00us Read 5.00us	optimized code, not sure why read time increased...
// Write 5.00us Read 3.66us replaced HAL_GPIO_ReadPin with macros
//
// I realized I was measuring this wrong, I should have measured from CS/MO/RnW low to GREADY starting
// to raise.
//
// With current code in a GROM read cycle GREADY starts to raise
//	2.50us from CS/MO/RnW 001 when grom_sample_delay is present
//  2.38us from CS/MO/RnW 001 when there is no delay.
// Thus the delta is 120 nanoseconds. The delays is 8 instructions (NOPs) which would mean
// 120ns/8=15ns cycle time. When running at 64MHz, the cycle time is 15.6ns so the MCU is
// really running at ~64MHz.
// It is interesting to note that GREADY rises so slowly that it takes 1.16us before the cycle
// ends. Of course the circuit sampling GREADY detects it at a certain level, and then it needs
// at least one 3MHz TMS9900 CPU clock pulse before the state changes. Perhaps two.
//
// A little further optimization and running the code from RAM:
// 2.6us from CS/MO/RnW 000 i.e. write cycle when there is no delay.
// 1.8uS from CS/MO/RnW 001 i.e. write cycle when there is no delay


#define GROM_TRACE_BUFFER 0
#define GROM_ADDRESS_READ_DRIVE	0

unsigned gromOffset = 0;
unsigned gromAddr = 0;
uint32_t gromAddrLatch = 0;
const uint8_t *gromRegion = grom994a_data;
uint32_t gromReadAddr = 0;
int total_reads = 0;
int total_addr_reads = 0;

#if GROM_TRACE_BUFFER
struct tracebuf_t {
	enum event_type_t { read_a, write_a } event_type;
	uint8_t abyte;
	uint16_t gromAddr;
	uint16_t reads;
	uint16_t a_reads;
};

int trace_index = 0;
struct tracebuf_t tracebuf[256];

void dump_trace_buf() {
	int rounds = 256;
	int start = trace_index;
	if(trace_index < 256) {
		start = 0;
		rounds = trace_index;
	}
	PRINTF("TRACEBUF %d EVENTS\r\n", trace_index);
	for(int i=0; i<rounds; i++) {
		struct tracebuf_t *p = &tracebuf[0xFF & (start+i)];
		PRINTF("%3d %c %02X %04X R=%d A_R=%d\r\n", i, p->event_type == read_a ? 'R' : 'W', p->abyte, p->gromAddr,
				p->reads, p->a_reads);
	}
}
#endif

uint8_t page_table[96];
#if defined(STM32G070xx)
uint8_t gram[24*1024];	// Our RAM area.
#else
uint8_t gram[6144];	// Our RAM area.
#endif
uint8_t *ram_alloc_ptr;

/**
 * @brief Initalize the gromserver process. Basically the paging table.
 * We have 24K of GROM space, divided into 256 byte pages here.
 * Thus a total of 4*24 = 96 pages. We have an array of 96 entries.
 * Each entry points initially to a page in ROM. If a page is written to,
 * that page is copied to RAM buffer, and the page entry initialize to point into it.
 * To save RAM, each page table entry is just a byte. The high byte determines if the page is
 * in ROM (0) or RAM (1).
 */
void init_grom_server() {
	for(int i=0; i<sizeof(page_table)/sizeof(page_table[0]); i++)
		page_table[i] = i;
	ram_alloc_ptr = gram;
}

#ifndef RUN_FROM_FLASH
__attribute__ ((section(".data")))
#endif
void copy_page(uint32_t *dst, uint32_t *src) {
	for(int i=256/sizeof(*dst); i>0 ; i--) {
		*dst++ = *src++;
	}
}

#ifndef RUN_FROM_FLASH
__attribute__ ((section(".data")))
#endif
unsigned grom_cs_low_process() {
	unsigned addr = 0xFFFFF;

	if(get_cs()) {
		return addr;	// Early out
	}
	// When we enter here, GREADY is already low and thus CPU will stop.
	grom_sample_delay();

	uint32_t	m = READ_DIR_MODE;
	switch(m) {
	case 0:	// write data	- write if flags permit. Fall through.
		break; // BUGBUG don't write
	case 1:	// read data
			// Read data from GROM or write to GRAM.
			if (gromRegion) {
				unsigned page_index = (gromOffset | (gromAddr & 0x1F00)) >> 8;
				uint8_t page = page_table[page_index];
				uint8_t *grom_page_ptr = 0;
				if(gromOffset < 0x6000) {
					grom_page_ptr = (page & 0x80) ? gram : (uint8_t *)grom994a_data;
				} else {
					grom_page_ptr = gromRegion;	
					page = page_index & 0x1F;
				}
				grom_page_ptr += (page & 0x7F) << 8;
				
				if(m == 1) {
					// Ordinary GROM read. Read from the current page.
					printf("GROM read %04X %04X %02X\n", gromOffset, gromAddr, grom_page_ptr[ gromAddr & 0xFF ]);
					drive_bus( grom_page_ptr[ gromAddr & 0xFF ] );
					total_reads++;
				} else {
					printf("GROM WRITE? %04X\n", read_data());
					// GROM write.
					// See if we need to allocate another page of RAM.
					if(!(page & 0x80)) {
						// Page still in ROM. Allocate RAM, and copy there.
						if(ram_alloc_ptr >= gram + sizeof(gram)) {
							// PRINTF("GRAM FULL! %d\r\n", page);		-- FIXME
							while(1)
								;	// STOP
						}
						uint8_t ram_page = 0x80 | ((ram_alloc_ptr - gram) >> 8);
						// PRINTF("Copying page %02X to RAM %02X\r\n", page, ram_page); -- FIXME
						// memcpy(ram_alloc_ptr, grom_page_ptr, 256);
						copy_page((uint32_t *)ram_alloc_ptr, (uint32_t *)grom_page_ptr);
						page = page_table[page_index] = ram_page;	// Update our variables.
						ram_alloc_ptr += 256;
					}
					// This chip is writable. So do the write.
					grom_page_ptr = gram + ((page &  0x7F) << 8);
					grom_page_ptr[gromAddr & 0xFF] = read_data();
				}
			}
			// gromAddr is incremented regardless if this address was for our GROM or not.
			gromAddr++;
			gromAddr &= 0x1FFF;
			// The returned read address is one past the next address
			gromReadAddr = gromOffset | ((1+gromAddr) & 0x1FFF);
			break;
		case 2: // write address counter
			gromAddrLatch <<= 8;
			gromAddrLatch |= read_data();
			gromAddrLatch &= 0xFFFF;
			addr = gromAddrLatch;
#if GROM_TRACE_BUFFER
			tracebuf[0xFF & trace_index].event_type = write_a;
			tracebuf[0xFF & trace_index].abyte = gromAddrLatch & 0xFF;
			tracebuf[0xFF & trace_index].gromAddr = gromAddrLatch;
			tracebuf[0xFF & trace_index].reads = total_reads;
			tracebuf[0xFF & trace_index].a_reads = total_addr_reads;
			trace_index++;
#endif
			gromReadAddr = (gromAddrLatch & 0xE000) | ((gromAddrLatch + 1) & 0x1FFF);

			// The code below used to be in read/write case, now moved here.
			gromAddr = gromAddrLatch & 0x1FFF;
			gromOffset = (gromAddrLatch & 0xE000);	// Keep the top 3 bits
			if(gromOffset < 0x6000) {
				// Override system GROM. This is done with a full 24K GROM image,
				// enabling extensions. Thus we assume 8K GROMs here.
				gromRegion = 0;	// picocart, let's not work on the system GROM area yet.
				// gromRegion = grom994a_data + (0x6000 & gromOffset);
			} else if(gromOffset < 0x8000) {
				gromRegion = grominvaders_data;
			} else {
				gromRegion = 0;	// TI-GROMmy only works with the bottom 24K
			}
			// printf("GROM Address write %04X region %04X\n", gromAddrLatch, gromRegion);
			break;
		case 3: {
			// read address counter
			uint8_t aw = gromReadAddr >> 8;
#if GROM_TRACE_BUFFER
			uint8_t r = read_data(); // Check what the chip is driving.
			if(r != aw) {
				PRINTF("Read address mismatch r=%02X aw=%02X %04X readAddr=%04X %d addr reads %d\r\n",
						r, aw, gromAddr, gromReadAddr, total_reads, total_addr_reads
						);
				dump_trace_buf();
				while(1)
					;	// stop here
			}
			tracebuf[0xFF & trace_index].event_type = read_a;
			tracebuf[0xFF & trace_index].abyte = r;
			tracebuf[0xFF & trace_index].gromAddr = gromReadAddr;
			tracebuf[0xFF & trace_index].reads = total_reads;
			tracebuf[0xFF & trace_index].a_reads = total_addr_reads;
			trace_index++;
			total_addr_reads++;
#endif				
			total_addr_reads++;
			// now drive our stuff
			#if GROM_ADDRESS_READ_DRIVE
			drive_bus(aw);	// Return high address first
			#endif
			// swap bytes to read
			gromReadAddr <<= 8;
			break;
		}  // case 3
	} // switch

	uint32_t state = save_and_disable_interrupts();
  set_gready_high();	// Let GROM_GREADY go high, we are done here.

	// Wait for the cycle to end.
	uint32_t k;
	do {
		k = get_cs();
	} while (!k);

	// Now the bus cycle has ended. Tri-state our databus and drive GROM_GREADY low for next cycle.
	data_dir_in();		// Data bus as input asap.
	set_gready_low();
	restore_interrupts(state);

	return addr;
}
