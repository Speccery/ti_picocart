# ti_picocart
TI-99/4A Cartridge based on the Raspberry Pi Pico. This is still on the prototype phase, although it does work.

![picocart picture](pictures/IMG_7161%20Large.jpeg)

## Features
The code is loosely based on my earlier StrangeCart and TI-GROMmy projects (which both continue on their own right).
Feature set is simple:
- ROM support
- GROM support
- Banked ROM cartridge support
No multicart support yet.

## Building the firmware
I have used macos as my development system. Should work the same way with Linux.

Remember to set the PICO_SDK_PATH environment variable: 
```
export PICO_SDK_PATH=/path/to/your/installation
```

Building is done the same way as with any Rasbperry Pi Pico project:

```
mkdir build
cd build
cmake ..
make -j4
```


## ROM images
Currently the ROM images need to be manually converted to C source files with my rom2c utlity, and the source code reference in grow-hw.h needs to be configured appropriately. Yes it's not neat yet. I'm not including in the repository ROM or GROM images from TI.

There are macros which make this a little easier, look at grom-hw.h:
```
#define ACTIVE_GROM grom_extendedbasic_data
#define ACTIVE_GROM_SIZE grom_extendedbasic_size
#define ACTIVE_ROM rom_extendedbasic_data
#define ACTIVE_ROM_SIZE	rom_extendedbasic_size
```

In addition the CMakeLists.txt needs to include the ROM image files converted to C. Since the repository here doesn't include third party ROM contents, the project will not compile straight off the gate. Below is a section of the CMakeLists.txt with comments after hash sign. You only need to include ROM and GROM content for one cartridge, since the current software version does not support changing the configured cartridge on the fly. During linking only the active ROM and GROM modules will be included in the final build.
```
add_executable(picocart
        picocart.c        # main() function
        gromserver.c      # implements GROM chip emulation
        romserver.c       # implements ROM chip emulation
        
        # What follows are ROM and GROM contents.
        
        invadersgrom.c    # Invaders GROM file 
        invadersrom.c     # Invaders ROM
        
        minimemg.c        # Minimemory GROM (untested)
        
        sysgrom.c         # System GROM (untested)
        
        parsecgrom.c      # Parsec GROM
        parsecrom.c       # Parsec ROM
        
        mspacman_rom.c    # Ms. Pacman ROM
        
        defender_rom.c    # Defender ROM
        
        extendedbasic_rom.c # Extended BASIC ROM
        extendedbasic_grom.c  # Extended BASIC GROM
        
        dontmess_rom.c    # Don't mess with Texas demo ROM
        )
```

## Hardware architecture
The board is pretty simple, mainly consisting of the Pico board and 74LVC245 buffer chips.

## Hardware mod
The current prototype board requires two patch wires for the firmware to be able to support ROM cartridges properly:

![Patch wires](pictures/IMG_7199%20Large.jpeg)

