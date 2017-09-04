#!/bin/bash - 
#===============================================================================
#
#          FILE: gen_vmlinux.sh
# 
#         USAGE: ./gen_vmlinux.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: Dr. Fritz Mehner (fgm), mehner.fritz@fh-swf.de
#  ORGANIZATION: FH Südwestfalen, Iserlohn, Germany
#       CREATED: 04/13/2016 14:14
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error
mkfs.ubifs -r ../rootfs  -m 2048 -e 124KiB -c 1762 -F  -o ubirootfs.pkg
ubinize -o rootfs.img -m 2048 -p 128KiB -s 2048   ./ubirootfs.cfg
mkfs.ubifs -r data  -m 2048 -e 124KiB -c 412 -F  -o ubidata.pkg
ubinize -o data.img -m 2048 -p 128KiB -s 2048 -e 126976 ./ubidata.cfg
