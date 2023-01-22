# ti_picocart
TI-99/4A Cartridge based on the Raspberry Pi Pico

## Features
The code is loosely based on my earlier StrangeCart and TI-GROMmy projects (which both continue on their own right).
Feature set is simple:
- ROM support
- GROM support
- Banked ROM cartridge support
No multicart support yet.

## Building
Building is done the same way as with any Rasbperry Pi Pico project:

```
mkdir build
cd build
cmake ..
make -j4
```

## ROM images
Currently the ROM images need to be manually converted to C source files with my rom2c utlity, and the source code reference in grow-hw.h needs to be configured appropriately. Yes it's not neat yet.

There are macros which make this a little easier, look at grom-hw.h:
```
#define ACTIVE_GROM 		grom_extendedbasic_data
#define ACTIVE_GROM_SIZE grom_extendedbasic_size
#define ACTIVE_ROM			rom_extendedbasic_data
#define ACTIVE_ROM_SIZE	rom_extendedbasic_size

In addition the CMakeLists.txt needs to include the ROM image files converted to C.
