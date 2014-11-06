#!/bin/sh
make -j4 ARCH=arm CROSS_COMPILE=/media/data/x-tools/gcc-arm-none-eabi-4_7-2013q3/bin/arm-none-eabi- uImage LOADADDR=0xD0200000 UIMAGE_ENTRYADDR=0xD0200001
make ARCH=arm CROSS_COMPILE=/media/data/x-tools/gcc-arm-none-eabi-4_7-2013q3/bin/arm-none-eabi- stm32f429-disco.dtb
