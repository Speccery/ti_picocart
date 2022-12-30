/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "board.h"

#include "grom-hw.h"    // temporary during debug

#define PLL_SYS_KHZ (133 * 1000)

#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#define PICO_DEFAULT_LED_PIN 25
#endif


void picocart_gpio_init() {
    for(int i=0; i<8; i++)
        gpio_init(PCn_D0+i);
    gpio_init(PCnI_ROMCS);  
    gpio_init(PCnI_WE   );  
    gpio_init(PCnI_GS   );  
    gpio_init(PCnI_DBIN );  
    gpio_init(PCnO_CSLOA);  
    gpio_init(PCnO_CSHIA);  
    gpio_init(PCnO_CSDAT);  
    gpio_init(PCnO_DDIR );  
    gpio_init(PCnO_GRDY);
    // Initially all buffers inactive
    gpio_put(PCnO_CSLOA, 1);
    gpio_put(PCnO_CSHIA, 1);
    gpio_put(PCnO_CSDAT, 1);
    gpio_put(PCnO_DDIR, 1);
    gpio_put(PCnO_GRDY, 1);
    // Set direction to outputs
    gpio_set_dir(PCnO_CSLOA, GPIO_OUT);
    gpio_set_dir(PCnO_CSHIA, GPIO_OUT);
    gpio_set_dir(PCnO_CSDAT, GPIO_OUT);
    gpio_set_dir(PCnO_DDIR,  GPIO_OUT);
    gpio_set_dir(PCnO_GRDY, GPIO_OUT);
    // Turn the databus in, so we're not colliding with drivers.
    sio_hw->gpio_oe_clr = PCn_DATAMASK;
}

uint16_t read_address() {
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

    // Start with GREADY low, i.e. we will stop upon GROM access
    gpio_put(PCnO_GRDY, 0);


    while(0) {
        // Read data.
        gpio_put(PCnO_DDIR, 1); 
        gpio_put(PCnO_CSDAT, 0);

        uint32_t in = sio_hw->gpio_in;
        uint32_t oe = sio_hw->gpio_hi_oe;
        // Read data.
        uint8_t dat = read_data();
        // Read address
        uint32_t a = read_address();

        printf("in=%08X oe=%08X addr=%04X dat=%02X\n", in, oe, a, dat);

        sleep_ms(50);
    }

    while(1) {
        grom_cs_low_process();
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
