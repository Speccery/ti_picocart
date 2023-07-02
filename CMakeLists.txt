cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(picocart_ti994a C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#set(PICO_BOARD pico_w)  # needed for Pico-W, otherwise #include "pico/cyw43_arch.h" won't be found
set(PICO_BOARD pico)  # set the 'Pico' as board

pico_sdk_init()

add_executable(${CMAKE_PROJECT_NAME})

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)
message(${PROJECT_NAME})
# message("LIST_DIR is " ${CMAKE_CURRENT_LIST_DIR})
# message("BINARY_DIR is " ${CMAKE_CURRENT_BINARY_DIR})
# file(RENAME ${CMAKE_CURRENT_BINARY_DIR}/ws2812.pio.h ws2812.pio.h)  # copy file from the build folder to the src folder

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
	picocart.c
	gromserver.c
	romserver.c
	invadersgrom.c
	invadersrom.c
	minimemg.c
	sysgrom.c
	parsecgrom.c
	parsecrom.c
	mspacman_rom.c
	defender_rom.c
	extendedbasic_rom.c
	extendedbasic_grom.c
	dontmess_rom.c
	)



target_link_libraries(${CMAKE_PROJECT_NAME} 
	pico_multicore
	pico_stdlib
	hardware_pio
	)

pico_enable_stdio_usb(${CMAKE_PROJECT_NAME}  1)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME}  1)

pico_add_extra_outputs(${CMAKE_PROJECT_NAME} )
