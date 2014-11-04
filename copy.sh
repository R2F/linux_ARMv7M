#!/bin/sh
mount /dev/sdb1 /media/sdcard
cp arch/arm/boot/uImage /media/sdcard
sync
umount /media/sdcard
echo "Done!"
