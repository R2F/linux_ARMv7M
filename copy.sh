#!/bin/sh
mount /dev/sdb1 /media/sdcard
cp arch/arm/boot/uImage arch/arm/boot/dts/stm32f429-disco.dtb /media/sdcard
sync
umount /media/sdcard
echo "Done!"
