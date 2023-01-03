cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(myproject_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(picocart
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
	)

target_link_libraries(picocart 
	pico_multicore
	pico_stdlib)

pico_enable_stdio_usb(picocart 1)
pico_enable_stdio_uart(picocart 1)

pico_add_extra_outputs(picocart)
