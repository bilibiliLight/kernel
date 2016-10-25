#!/bin/sh

set -e 
make CROSS_COMPILE=arm-linux-
arm-linux-objcopy -O binary vmlinux vmlinux.bin
mkimage -A arm -O linux -T kernel -a 0x7fc0 -e 0x8000 -d vmlinux.bin vmlinux.ub
cp vmlinux.ub /tftpboot/
