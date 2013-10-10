#This is the Makefile for Lab 1 Blinky for ELEC424 Fall 2013
# Authors: Steven Arroyo and Lin Zhong, Rice University


# use the arm compiler 
CC = arm-none-eabi-gcc

# Define paths for all included files.  This assumes STM standard library was placed unmodified 
# in a folder called lib/ as outlined in the lab description.
LIB = lib
STM_LIB = $(LIB)/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries
STM_STD_PERIF = $(STM_LIB)/STM32F10x_StdPeriph_Driver
STM_DEVICE_SUPPORT = $(STM_LIB)/CMSIS/CM3/DeviceSupport/ST/STM32F10x
STM_CORE_SUPPORT = $(STM_LIB)/CMSIS/CM3/CoreSupport
STM_STARTUP = $(STM_LIB)/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7

# Define the compiler flags
CFLAGS = -O0 -g3 -mcpu=cortex-m3 -mthumb -I$(STM_STD_PERIF)/inc -I$(STM_STARTUP) -I$(STM_CORE_SUPPORT) -I$(STM_DEVICE_SUPPORT) -DSTM32F10X_MD -include stm32f10x_conf.h -Wl,--gc-sections -T stm32_flash.ld

# build all relevant files and create .elf
all:
	$(CC) $(CFLAGS) $(STM_STARTUP)/startup_stm32f10x_md.s blinky.c $(STM_STD_PERIF)/src/stm32f10x_gpio.c $(STM_STD_PERIF)/src/stm32f10x_rcc.c -o blinky.elf

clean:
	rm -rf *o blinky

# program elf into crazyflie flash memory with busblaster
flash:
	openocd -d0 -f interface/busblaster.cfg -f target/stm32f1x.cfg -c init -c targets -c "reset halt" \
                 -c "flash write_image erase blinky.elf" -c "verify_image blinky.elf" -c "reset run" -c shutdown

# Start openocd, so we can use it with gdb 
openocd:
	openocd -d0 -f interface/busblaster.cfg -f target/stm32f1x.cfg -c init -c targets

# Example output when V=1 is passed (make V=1)
# arm-none-eabi-gcc -O0 -g3 -mcpu=cortex-m3 -mthumb -Ilib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/inc -Ilib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7 -Ilib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/CMSIS/CM3/CoreSupport -Ilib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x -DSTM32F10X_MD -include stm32f10x_conf.h -Wl,--gc-sections -T stm32_flash.ld lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md.s blinky.c lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c lib/STM32F10x_StdPeriph_Lib_V3.5.0/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c -o blinky.elf 
