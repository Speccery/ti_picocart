// psram.c
// bit banged SPI (and QPI) for PSRAM access
//

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

void psram_init(void) {
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK) | (1 << PCn_DQ2) | (1 << PCn_DQ3);
}

void psram_read_spi(uint8_t *dest, uint32_t addr, uint32_t len) {
    sio_hw->gpio_clr = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
    // Send read command and address
    addr = (PSRAM_READ << 24) | (addr & 0xFFFFFF);
    for(unsigned u=32; u;  u--) {
        if(addr & (1 << 31))
            sio_hw->gpio_set = (1 << SPI_MOSI);
        else    
            sio_hw->gpio_clr = (1 << SPI_MOSI);
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
            b = (b << 1) | ((sio_hw->gpio_in >> SPI_MISO) & 1);
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
}

void psram_write_spi(uint32_t addr, uint32_t len, const uint8_t *src) {
    sio_hw->gpio_clr = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
    // Send read command and address
    addr = (PSRAM_WRITE << 24) | (addr & 0xFFFFFF);
    for(unsigned u=32; u;  u--) {
        if(addr & (1 << 31))
            sio_hw->gpio_set = (1 << SPI_MOSI);
        else    
            sio_hw->gpio_clr = (1 << SPI_MOSI);
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
                sio_hw->gpio_set = (1 << SPI_MOSI);
            else
                sio_hw->gpio_clr = (1 << SPI_MOSI);
            sio_hw->gpio_set = 1 << SPI_SCK;
            __asm volatile ("nop");
            __asm volatile ("nop");
            b <<= 1;
            sio_hw->gpio_clr = 1 << SPI_SCK;
        }
    }
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
}

void psram_read_id(uint8_t *dest2) {
    sio_hw->gpio_clr = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
    // Send read id command and dummy address of zero
    uint32_t addr = PSRAM_READ_ID << 24;
    for(unsigned u=32; u;  u--) {
        if(addr & (1 << 31))
            sio_hw->gpio_set = (1 << SPI_MOSI);
        else    
            sio_hw->gpio_clr = (1 << SPI_MOSI);
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
            b = (b << 1) | ((sio_hw->gpio_in >> SPI_MISO) & 1);
            sio_hw->gpio_set = 1 << SPI_SCK;
            __asm volatile ("nop");
            __asm volatile ("nop");
            __asm volatile ("nop");
            sio_hw->gpio_clr = 1 << SPI_SCK;
        }
        *dest2++ = b;
    }
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
}

void psram_enter_qpi(void) {
    sio_hw->gpio_clr = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
    // Send read command and address
    uint8_t b = PSRAM_ENTER_QUAD;
    for(unsigned u=8; u; u--) {
        if(b & 0x80)
            sio_hw->gpio_set = (1 << SPI_MOSI);
        else    
            sio_hw->gpio_clr = (1 << SPI_MOSI);
        __asm volatile ("nop");
        sio_hw->gpio_set = 1 << SPI_SCK;
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
        sio_hw->gpio_clr = 1 << SPI_SCK;
        b <<= 1;    
    }
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
}

// There is a discontiuity in the mask. pin 5 is PSRAM_CS in the middle. damn.
// This is how the bits are laid out:
// PSRAM    PICO GPIO
// SIO0     MOSI 3
// SIO1     MISO 4
// SIO2          6
// SIO3          7
#define QPI_MASK ((1 << SPI_MOSI) | (1 << SPI_MISO) | (1 << PCn_DQ2) | (1 << PCn_DQ3))
#define CMD_TO_BITS_HI(x) (x & 0xC0) | ((x & 0x30) >> 1)
#define CMD_TO_BITS_LO(x) ((x & 0x0C) << 4) | ((x & 0x03) << 3)

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
  // Configure MISO as input
  sio_hw->gpio_oe_clr = 1 << SPI_MISO;
}

/**
 * @brief read data in QPI mode.
*/
void __time_critical_func(psram_read_qpi)(uint8_t *dest, uint32_t addr, uint32_t len) {
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
    // Issue a four clocks. Change direction of data to inputs.
    sio_hw->gpio_oe_clr = QPI_MASK; // Now 4 bit input bus.
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
      b = (b & 0xC0) | ((b & 0x18) << 1);
      uint32_t lo = sio_hw->gpio_in;
      b |= ((lo & 0xC0) >> 4) | ((lo & 0x18) >> 3);
      sio_hw->gpio_set = 1 << SPI_SCK;
      sio_hw->gpio_clr = 1 << SPI_SCK;
      *dest++ = b;
    }
    sio_hw->gpio_set = (1 << PCnO_PSRAM_CS) | (1 << SPI_SCK);
}
