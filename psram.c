// psram.c
// bit banged SPI (and QPI) for PSRAM access
// Erik Piehl (C) 2023-07-03

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "board.h"
#include "psram.h"
                                        // SPI MODE      : QPI MODE
#define PSRAM_READ              0x03    // 0 wait cycles : N/A
#define PSRAM_FAST_READ         0x0B    // 8 wait cycles : 4 cycles
#define PSRAM_FAST_READ_QUAD    0xEB    // 6 wait cycles : 6 cycles
#define PSRAM_WRITE             0x02    // 0
#define PSRAM_WRITE_QUAD        0x38
#define PSRAM_ENTER_QUAD        0x35    // Only in SPI mode
#define PSRAM_EXIT_QUAD         0xF5    // Only in QPI mode
#define PSRAM_READ_ID           0x9F

// There is a discontiuity in the mask. pin 5 is PSRAM_CS in the middle. damn.
// This is how the bits are laid out:
// PSRAM    PICO GPIO
// SIO0     MOSI 3
// SIO1     MISO 4
// SIO2          6
// SIO3          7
#if !defined(QPI_REVISION)
# define QPI_MASK ((1 << SPI_MOSI) | (1 << SPI_MISO) | (1 << PCn_DQ2) | (1 << PCn_DQ3))
# define CMD_TO_BITS_HI(x) (x & 0xC0) | ((x & 0x30) >> 1)
# define CMD_TO_BITS_LO(x) ((x & 0x0C) << 4) | ((x & 0x03) << 3)
uint8_t lookup_hi[256];
uint8_t lookup_lo[256];
uint8_t lookup_lo_write[16];
# define BITS_TO_HI(x) ((b & 0xC0) | ((b & 0x18) << 1))
# define BITS_TO_LO(x) (((lo & 0xC0) >> 4) | ((lo & 0x18) >> 3))
#else
# define QPI_MASK ((1 << SPI_MOSI_DQ0) | (1 << SPI_MISO_DQ1) | (1 << PCn_DQ2) | (1 << PCn_DQ3))
# define CMD_TO_BITS_HI(x) ((x & 0xF0) << 4)
# define CMD_TO_BITS_LO(x) ((x & 0x0F) << 8) 
# define BITS_TO_HI(x) ((x >> 4) & 0xF0)
# define BITS_TO_LO(x) ((x >> 8) & 0x0F)
#endif

inline void config_pins_shared_spi(void) {
#if defined(QPI_REVISION)
    // Setup the shared databus pins properly for SPI mode.
    sio_hw->gpio_set = (1 << PCn_DQ2) | (1 << PCn_DQ3) | (1 << PCnO_SEL0) | (1 << PCnO_SEL1);   // Deactive all buffers.
    sio_hw->gpio_oe_set = (1 << SPI_MOSI_DQ0) | (1 << PCn_DQ2) | (1 << PCn_DQ3);
    sio_hw->gpio_oe_clr = (1 << SPI_MISO_DQ1);
#endif   
}

inline void restore_pins_shared_spi(void) {
#if defined(QPI_REVISION)
    sio_hw->gpio_oe_clr = QPI_MASK; // Turn everything we touched into inputs.
#endif
}


void psram_init(void) {
#if !defined(QPI_REVISION)    
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK) | (1 << PCn_DQ2) | (1 << PCn_DQ3);
    // Init the lookup tables to convert a read byte to properly masked and shifted nibble (for reads).
    // We read a byte from gpio and the table converts it to the respective nibble.
    for(int i=0; i<256; i++) {
        lookup_hi[i] = (i & 0xC0) | ((i & 0x18) << 1);
        lookup_lo[i] = ((i & 0xC0) >> 4) | ((i & 0x18) >> 3);        
    }
    for(int i=0; i<16; i++)
        lookup_lo_write[i] = CMD_TO_BITS_LO(i);
#else 
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
#endif        
}

void psram_read_spi(uint8_t *dest, uint32_t addr, uint32_t len) {
    config_pins_shared_spi();
    sio_hw->gpio_clr = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
    // Send read command and address
    addr = (PSRAM_READ << 24) | (addr & 0xFFFFFF);
    for(unsigned u=32; u;  u--) {
        if(addr & (1 << 31))
            sio_hw->gpio_set = (1 << SPI_MOSI_DQ0);
        else    
            sio_hw->gpio_clr = (1 << SPI_MOSI_DQ0);
        __asm volatile ("nop");
        sio_hw->gpio_set = 1 << SPI_SCK;
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        sio_hw->gpio_clr = 1 << SPI_SCK;
        __asm volatile ("nop");
        addr <<= 1;
    }
    // Read data
    for(;len;len--) {
        uint8_t b=0;
        for(unsigned u=8; u; u--) {
            b = (b << 1) | ((sio_hw->gpio_in >> SPI_MISO_DQ1) & 1);
            sio_hw->gpio_set = 1 << SPI_SCK;
            __asm volatile ("nop");
            __asm volatile ("nop");
            __asm volatile ("nop");
            sio_hw->gpio_clr = 1 << SPI_SCK;
            __asm volatile ("nop");
            __asm volatile ("nop");
            __asm volatile ("nop");
        }
        *dest++ = b;
    }
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
    restore_pins_shared_spi();
}

void psram_write_spi(uint32_t addr, uint32_t len, const uint8_t *src) {
    config_pins_shared_spi();     
    sio_hw->gpio_clr = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
    // Send read command and address
    addr = (PSRAM_WRITE << 24) | (addr & 0xFFFFFF);
    for(unsigned u=32; u;  u--) {
        if(addr & (1 << 31))
            sio_hw->gpio_set = (1 << SPI_MOSI_DQ0);
        else    
            sio_hw->gpio_clr = (1 << SPI_MOSI_DQ0);
        __asm volatile ("nop");
        sio_hw->gpio_set = 1 << SPI_SCK;
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        sio_hw->gpio_clr = 1 << SPI_SCK;
        addr <<= 1;
    }
    // write data
    for(;len;len--) {
        uint8_t b = *src++;
        for(unsigned u=8; u; u--) {
            if(b & 0x80)
                sio_hw->gpio_set = (1 << SPI_MOSI_DQ0);
            else
                sio_hw->gpio_clr = (1 << SPI_MOSI_DQ0);
            sio_hw->gpio_set = 1 << SPI_SCK;
            __asm volatile ("nop");
            __asm volatile ("nop");
            b <<= 1;
            sio_hw->gpio_clr = 1 << SPI_SCK;
        }
    }
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
    restore_pins_shared_spi();
}

void psram_read_id(uint8_t *dest2) {
    config_pins_shared_spi();
    sio_hw->gpio_clr = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
    // Send read id command and dummy address of zero
    uint32_t addr = PSRAM_READ_ID << 24;
    for(unsigned u=32; u;  u--) {
        if(addr & (1 << 31))
            sio_hw->gpio_set = (1 << SPI_MOSI_DQ0);
        else    
            sio_hw->gpio_clr = (1 << SPI_MOSI_DQ0);
        __asm volatile ("nop");
        sio_hw->gpio_set = 1 << SPI_SCK;
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        sio_hw->gpio_clr = 1 << SPI_SCK;
        addr <<= 1;    
    }
    // Read data
    for(unsigned len=2;len;len--) {
        uint8_t b=0;
        for(unsigned u=8; u; u--) {
            b = (b << 1) | ((sio_hw->gpio_in >> SPI_MISO_DQ1) & 1);
            sio_hw->gpio_set = 1 << SPI_SCK;
            __asm volatile ("nop");
            __asm volatile ("nop");
            __asm volatile ("nop");
            sio_hw->gpio_clr = 1 << SPI_SCK;
        }
        *dest2++ = b;
    }
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
    restore_pins_shared_spi();
}

void psram_enter_qpi(void) {
    config_pins_shared_spi();
    sio_hw->gpio_clr = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
    // Send read command and address
    uint8_t b = PSRAM_ENTER_QUAD;
    for(unsigned u=8; u; u--) {
        if(b & 0x80)
            sio_hw->gpio_set = (1 << SPI_MOSI_DQ0);
        else    
            sio_hw->gpio_clr = (1 << SPI_MOSI_DQ0);
        __asm volatile ("nop");
        sio_hw->gpio_set = 1 << SPI_SCK;
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        sio_hw->gpio_clr = 1 << SPI_SCK;
        b <<= 1;    
    }
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
    restore_pins_shared_spi();
}



/**
 * @brief only works in QPI mode.
*/
void psram_exit_qpi(void) {
  sio_hw->gpio_oe_set = QPI_MASK;
  sio_hw->gpio_clr = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK) | QPI_MASK;
  sio_hw->gpio_set = CMD_TO_BITS_HI( PSRAM_EXIT_QUAD );
  sio_hw->gpio_set = 1 << SPI_SCK;
  sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
  sio_hw->gpio_set = CMD_TO_BITS_LO( PSRAM_EXIT_QUAD );
  sio_hw->gpio_set = 1 << SPI_SCK;
  sio_hw->gpio_set = 1 << PCnO_PSRAM_CS;
#if defined(QPI_REVISION) 
  restore_pins_shared_spi();
#else 
  // Configure MISO as input  
  sio_hw->gpio_oe_clr = 1 << SPI_MISO;
#endif  
}

/**
 * @brief read data in QPI mode.
 * This code is the earlier version of the above. Easier to read.
*/
void __time_critical_func(psram_read_qpi2)(uint8_t *dest, uint32_t addr, uint32_t len) {
    sio_hw->gpio_clr = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_oe_set = QPI_MASK;
    // Send read command and address
    addr = (PSRAM_FAST_READ << 24) | (addr & 0xFFFFFF);
    for(unsigned u=8; u;  u--) {
      uint32_t d = addr >> 24;
      sio_hw->gpio_set = CMD_TO_BITS_HI(d);
      sio_hw->gpio_set = 1 << SPI_SCK;
      sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
      addr <<= 4;
    }
    // Change direction of data to inputs.
    sio_hw->gpio_oe_clr = QPI_MASK; // Now 4 bit input bus.
    // Issue four clocks. 
    sio_hw->gpio_set = 1 << SPI_SCK;
    sio_hw->gpio_clr = 1 << SPI_SCK;
    sio_hw->gpio_set = 1 << SPI_SCK;
    sio_hw->gpio_clr = 1 << SPI_SCK;
    sio_hw->gpio_set = 1 << SPI_SCK;
    sio_hw->gpio_clr = 1 << SPI_SCK;
    sio_hw->gpio_set = 1 << SPI_SCK;
    sio_hw->gpio_clr = 1 << SPI_SCK;
    // Ok time to read our stuff.
    for(;len;len--) {
      uint32_t b = sio_hw->gpio_in;
      sio_hw->gpio_set = 1 << SPI_SCK;
      sio_hw->gpio_clr = 1 << SPI_SCK;
      __asm volatile("nop");
      b = BITS_TO_HI(b);
      uint32_t lo = sio_hw->gpio_in;
      b |= BITS_TO_LO(lo);
      sio_hw->gpio_set = 1 << SPI_SCK;
      sio_hw->gpio_clr = 1 << SPI_SCK;
      *dest++ = b;
    }
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
#if defined(QPI_REVISION) 
    restore_pins_shared_spi();
#endif
}

/**
 * @brief read data in QPI mode.
*/
void __time_critical_func(psram_read_qpi)(uint8_t *dest, uint32_t addr, uint32_t len) {
    sio_hw->gpio_clr = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_oe_set = QPI_MASK;
    // Send read command and address
#if 0
    addr = (PSRAM_FAST_READ << 24) | (addr & 0xFFFFFF);
    for(unsigned u=8; u;  u--) {
      uint32_t d = addr >> 24;
      sio_hw->gpio_set = CMD_TO_BITS_HI(d);
      sio_hw->gpio_set = (1 << SPI_SCK);
      sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
      addr <<= 4;
    }
#else    
    // Issue command.
    sio_hw->gpio_set = CMD_TO_BITS_HI(PSRAM_FAST_READ);
    sio_hw->gpio_set =  (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_set = CMD_TO_BITS_LO(PSRAM_FAST_READ);
    sio_hw->gpio_set =  (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    // issue address
    sio_hw->gpio_set = CMD_TO_BITS_HI(addr >> 16)  | (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_set = CMD_TO_BITS_HI(addr >> 12)  | (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_set = CMD_TO_BITS_HI(addr >> 8)  | (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_set = CMD_TO_BITS_HI(addr >> 4)  | (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_set = CMD_TO_BITS_HI(addr     )  | (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_set = CMD_TO_BITS_HI(addr << 4)  | (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
  #endif
    //  Change direction of data to inputs.
    sio_hw->gpio_oe_clr = QPI_MASK; // Now 4 bit input bus.
    // Issue four clocks.
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
#if defined(QPI_REVISION)    
    // Ok time to read our stuff.
    for(;len;len--) {
      uint32_t hi = sio_hw->gpio_in;
      sio_hw->gpio_set = 1 << SPI_SCK;
      sio_hw->gpio_clr = 1 << SPI_SCK;
      __asm volatile ("nop");
      hi = (hi >> 4) & 0xF0;
      uint32_t lo = sio_hw->gpio_in;
      hi |= (lo >> 8) & 0x0F;
      sio_hw->gpio_set = 1 << SPI_SCK;
      sio_hw->gpio_clr = 1 << SPI_SCK;
      *dest++ = hi;
    }
#else
    // Ok time to read our stuff.
    for(;len;len--) {
      uint8_t hi = sio_hw->gpio_in;
      sio_hw->gpio_set = 1 << SPI_SCK;
      sio_hw->gpio_clr = 1 << SPI_SCK;
      __asm volatile ("nop");
      hi = lookup_hi[hi];
      uint8_t lo = sio_hw->gpio_in;
      hi |= lookup_lo[lo];
      sio_hw->gpio_set = 1 << SPI_SCK;
      sio_hw->gpio_clr = 1 << SPI_SCK;
      *dest++ = hi;
    }
#endif    
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
#if defined(QPI_REVISION)    
    restore_pins_shared_spi();
#endif    
}


#define QPI_MASK3 0xF00
#define CMD_TO_BITS_HI3(x) ((x & 0xF0) << 4)
#define CMD_TO_BITS_LO3(x) ((x & 0x0F) << 8) 
/**
 * @brief read data in QPI mode.
 * Version 3. See how this would work if the PSRAM chip was connected to the shared data bus.
*/
void __time_critical_func(psram_read_qpi3)(uint8_t *dest, uint32_t addr, uint32_t len) {
    sio_hw->gpio_clr = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_oe_set = QPI_MASK;
    // Send read command and address
#if 0
    addr = (PSRAM_FAST_READ << 24) | (addr & 0xFFFFFF);
    for(unsigned u=8; u;  u--) {
      uint32_t d = addr >> 24;
      sio_hw->gpio_set = CMD_TO_BITS_HI(d);
      sio_hw->gpio_set = (1 << SPI_SCK);
      sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
      addr <<= 4;
    }
#else    
    // Issue command.
    sio_hw->gpio_set = CMD_TO_BITS_HI3(PSRAM_FAST_READ);
    sio_hw->gpio_set =  (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_set = CMD_TO_BITS_LO3(PSRAM_FAST_READ);
    sio_hw->gpio_set =  (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    // issue address
    sio_hw->gpio_set = CMD_TO_BITS_HI3(addr >> 16)  | (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_set = CMD_TO_BITS_HI3(addr >> 12)  | (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_set = CMD_TO_BITS_HI3(addr >> 8)  | (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_set = CMD_TO_BITS_HI3(addr >> 4)  | (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_set = CMD_TO_BITS_HI3(addr     )  | (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
    sio_hw->gpio_set = CMD_TO_BITS_HI3(addr << 4)  | (1 << SPI_SCK);
    sio_hw->gpio_clr = (1 << SPI_SCK) | QPI_MASK;
  #endif
    //  Change direction of data to inputs.
    sio_hw->gpio_oe_clr = QPI_MASK; // Now 4 bit input bus.
    // Issue four clocks.
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    sio_hw->gpio_togl = 1 << SPI_SCK; 
    // Ok time to read our stuff.
    for(;len;len--) {
      uint32_t b = sio_hw->gpio_in;
      sio_hw->gpio_set = 1 << SPI_SCK;
      sio_hw->gpio_clr = 1 << SPI_SCK;
      __asm volatile("nop");
      b = 0xF0 & (b >> 4);
      uint32_t lo = sio_hw->gpio_in;
      b |= 0x0F & (lo >> 8);
      sio_hw->gpio_set = 1 << SPI_SCK;
      sio_hw->gpio_clr = 1 << SPI_SCK;
      *dest++ = b;
    }
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
}


