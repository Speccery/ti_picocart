/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "board.h"
#include "ws2812.pio.h"

#include "grom-hw.h"    // temporary during debug

#define PLL_SYS_KHZ (133 * 1000)

#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#define PICO_DEFAULT_LED_PIN 25
#endif

uint8_t  rom_area[32*1024];
unsigned active_rom_size = 8192;
uint8_t  *active_rom_area = rom_area;

void picocart_gpio_init() {
    for(int i=0; i<8; i++)
        gpio_init(PCn_D0+i);
    gpio_init(PCnI_ROMCS);  
    gpio_init(PCnI_WE   );  
    gpio_init(PCnI_GS   );  
    gpio_init(PCnI_DBIN );  
    gpio_init(PCnO_GRDY);
#if defined(BOARD_VER11)
    gpio_init(PCnO_PSRAM_CS);
    gpio_init(PCn_DQ2);
    gpio_init(PCn_DQ3);
    gpio_init(PCnO_SD_CS);
    gpio_init(PCnO_SEL0);
    gpio_init(PCnO_SEL1);
    gpio_init(PCnO_LEDS);
    // Config everything to inactive state
    gpio_put(PCnO_PSRAM_CS, 1);
    gpio_put(PCn_DQ2, 1);
    gpio_put(PCn_DQ3, 1);
    gpio_put(PCnO_SD_CS, 1);
    gpio_put(PCnO_SEL0, 1);
    gpio_put(PCnO_SEL1, 1);
    gpio_put(PCnO_LEDS, 1);
    // Config the above for outputs
    gpio_set_dir(PCnO_PSRAM_CS, GPIO_OUT);
    gpio_set_dir(PCn_DQ2, GPIO_OUT);
    gpio_set_dir(PCn_DQ3, GPIO_OUT);
    gpio_set_dir(PCnO_SD_CS, GPIO_OUT);
    gpio_set_dir(PCnO_SEL0, GPIO_OUT);
    gpio_set_dir(PCnO_SEL1, GPIO_OUT);
    gpio_set_dir(PCnO_LEDS, GPIO_OUT);
#else    
    gpio_init(PCnO_CSLOA);  
    gpio_init(PCnO_CSHIA);  
    gpio_init(PCnO_CSDAT);  
    gpio_init(PCnO_DDIR );  
    // Initially all buffers inactive
    gpio_put(PCnO_CSLOA, 1);
    gpio_put(PCnO_CSHIA, 1);
    gpio_put(PCnO_CSDAT, 1);
    gpio_put(PCnO_DDIR, 1);
    // Set direction to outputs
    gpio_set_dir(PCnO_CSLOA, GPIO_OUT);
    gpio_set_dir(PCnO_CSHIA, GPIO_OUT);
    gpio_set_dir(PCnO_CSDAT, GPIO_OUT);
    gpio_set_dir(PCnO_DDIR,  GPIO_OUT);
    // Configure our debug bits.
    for(int i=PCnO_DEBUG26; i<=PCnO_DEBUG27; i++) {
        gpio_init(i);
        gpio_put(i, 0);
        gpio_set_dir(i, GPIO_OUT);
        gpio_put(i, 0);
    }
#endif
    gpio_put(PCnO_GRDY, 1);
    gpio_set_dir(PCnO_GRDY, GPIO_OUT);
    // Set direction to inputs
    gpio_set_dir(PCnI_ROMCS, GPIO_IN);  
    gpio_set_dir(PCnI_WE, GPIO_IN   );  
    gpio_set_dir(PCnI_GS, GPIO_IN   );  
    gpio_set_dir(PCnI_DBIN, GPIO_IN );  
    // Turn the databus in, so we're not colliding with drivers.
    sio_hw->gpio_oe_clr = PCn_DATAMASK;
    // Configure a special pin to read A0.
    gpio_init(PCnI_BA0);
    gpio_set_dir(PCnI_BA0, GPIO_IN);
}



void core1_busserver() {
    // Start with GREADY low, i.e. we will stop upon GROM access
    gpio_put(PCnO_GRDY, 0);    
    uint32_t state = save_and_disable_interrupts(); //  Disable interrupts for this core.

    while(1) {
        rom_server();
    }
}

// Neopixel stuff
#define NEOC_PIN_START PCnO_LEDS
#define NEOC_LANE_START 0

static unsigned int WS2812_sm = 0; /* state machine index. \todo should find a free SM */

uint32_t NEO_GetPixel32bitForPIO(int lane, int pos) {
  uint32_t val;
  uint8_t r, g, b, w;
  if(pos == 0) {
    r = 0;
    g = 25;
    b = 0;
    w = 0;
  } else {
    r = 25; g=0; b=0; w=0;
  }
  // NEO_GetPixelWRGB(lane, pos, &w, &r, &g, &b);
  val = ((uint32_t)(g)<<24) | ((uint32_t)(r)<<16) | ((uint32_t)(b)<<8) | (uint32_t)(w);

  return val;
}

void WS2812_Init(void) {
  PIO pio = pio0; /* the PIO used */
  WS2812_sm = 0; /* state machine used */
  uint offset = pio_add_program(pio, &ws2812_program);
  ws2812_program_init(pio, WS2812_sm, offset, NEOC_PIN_START, 800000, 0); /* initialize it for 800 kHz */
}

int WS2812_Transfer(uint32_t address, size_t nofBytes) {
    PIO pio = pio0; /* the PIO used */
    WS2812_sm = 0; /* state machine used */
    int NEOC_NOF_LEDS_IN_LANE = 2;
    for(int i=0; i<NEOC_NOF_LEDS_IN_LANE; i++) { /* without DMA: writing one after each other */
      pio_sm_put_blocking(pio, WS2812_sm, NEO_GetPixel32bitForPIO(NEOC_LANE_START, i));
    }
  return 0; /* ok */
}


// debug variables.
extern uint32_t rom_bank;
extern uint32_t max_bank;
extern uint32_t min_bank;
extern uint32_t bank_switches;

int main() {

    // Set the system frequency to 133 MHz. vco_calc.py from the SDK tells us
    // this is exactly attainable at the PLL from a 12 MHz crystal: FBDIV =
    // 133 (so VCO of 1596 MHz), PD1 = 6, PD2 = 2. This function will set the
    // system PLL to 133 MHz and set the clk_sys divisor to 1.
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // The previous line automatically detached clk_peri from clk_sys, and
    // attached it to pll_usb, so that clk_peri won't be disturbed by future
    // changes to system clock or system PLL. If we need higher clk_peri
    // frequencies, we can attach clk_peri directly back to system PLL (no
    // divider available) and then use the clk_sys divider to scale clk_sys
    // independently of clk_peri.
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );

    picocart_gpio_init();


    // The serial clock won't vary from this point onward, so we can configure
    // the UART etc.
    stdio_init_all();
    init_grom_server();

    // Try to get neo pixel enabled
    WS2812_Init();
    WS2812_Transfer(0,0);

    multicore_launch_core1(core1_busserver);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);
    sleep_ms(100);
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);

    sleep_ms(2000);
    puts("Picocart has booted.\n");
    // memcpy(active_rom_area, rom_mspacman_data, rom_mspacman_size);
    // active_rom_size = rom_mspacman_size;
    active_rom_area = rom_area;
    if(ACTIVE_ROM_SIZE <= sizeof(rom_area))
        memcpy(active_rom_area, ACTIVE_ROM, ACTIVE_ROM_SIZE);
    else
        active_rom_area = (uint8_t *)ACTIVE_ROM;   // Can't copy to ROM to SRAM, try to run for external flash.
    active_rom_size = ACTIVE_ROM_SIZE;
    // memcpy(active_rom_area, rom_defender_data, rom_defender_size);
    // active_rom_size = rom_defender_size;
    puts("Cart data copied to RAM.\n");
    printf("ROM_SIZE %d GROM_SIZE %d\n", ACTIVE_ROM_SIZE, ACTIVE_GROM_SIZE);

    while(0) {
        // Read data.
#if defined(BOARD_VER11)
        sio_hw ->gpio_set = (1 << PCnO_SEL1);
        sio_hw ->gpio_clr = (1 << PCnO_SEL0);
#else        
        gpio_put(PCnO_DDIR, 1); 
        gpio_put(PCnO_CSDAT, 0);
#endif
        uint32_t in = sio_hw->gpio_in;
        uint32_t oe = sio_hw->gpio_hi_oe;
        // Read data.
        uint8_t dat = read_data();
        // Read address
        uint32_t a = read_address();

        printf("in=%08X oe=%08X addr=%04X dat=%02X\n", in, oe, a, dat);

        sleep_ms(50);
    }

    while( 1 ) {
        sleep_ms(1000);
        printf("rom_bank=%d max=%d min=%d switches=%d\n",
            rom_bank, max_bank, min_bank, bank_switches
        );
    }

/*
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        puts("LED off\n");
        sleep_ms(500);
    }
    */
}
