#! /bin/sh

# debugging message
#echo "MDEV=$1 : ACTION=$2 : SUBSYSTEM=$SUBSYSTEM : DEVPATH=$DEVPATH : DEVNAME=$DEVNAME" >> /dev/console

if [ "$1" == "" ]; then
	echo "parameter is none" > /tmp/error.txt
	exit 1
fi

MNT=$1
#if [ $(echo $1 | grep mmcblk) ]; then
#	if [ $(echo $1 | grep p[25]) ]; then
#		MNT=sdcard2
#	else
#		MNT=sdcard
#	fi
#elif [ $(echo $1 | grep sd) ]; then
#	if [ $(echo $1 | grep p[25]) ]; then
#		MNT=nandcard2
#	else
#		MNT=nandcard
#	fi
#fi

# there is no ACTION, it is for initial population
if [ "$2" = "X" ]; then
	mounted=`mount | grep $1 | wc -l`
	if [ $mounted -ge 1 ]; then
		# mounted, assume the ACTION is remove
		#ACT=Xremove
		# only set add for initial population
		ACT=Xadd
	else
		# not mounted, assume the ACTION is add
		ACT=Xadd
	fi
else
	ACT=$2
fi

if [ "$ACT" = "Xremove" ]; then
	# umount the device
	echo "$ACT /mnt/$1" >> /tmp/mdev.log
	if ! umount -l "/mnt/$1"; then
		exit 1
	else
		rm -f "/mnt/$MNT"
        # FIX 
		rm -f "/mnt/${MNT}100"
        # FIX END
		echo "[Umount FS]: /dev/$1 -X-> /mnt/$MNT" > /dev/console
	fi

	if ! rmdir "/mnt/$1"; then
		exit 1
	fi
else
	# mount the device
	mounted=`mount | grep $1 | wc -l`
	#echo "par=$1,mounted=$mounted,MNT=$MNT" > /dev/console
	if [ $mounted -ge 1 ]; then
		#echo "device $1 is already mounted" > /dev/console
		exit 0
	fi

	if ! mkdir -p "/mnt/$1"; then
		exit 1
	fi

	if [ $(echo $1 | grep mtd) ]; then
		if mount -t jffs2 "/dev/$1" "/mnt/$1"; then
			echo "[Mount JFFS2]: /dev/$1 --> /mnt/$MNT" > /dev/console
			echo "$ACT /mnt/$1" >> /tmp/mdev.log
		elif mount -t yaffs2 "/dev/$1" "/mnt/$1"; then
			echo "[Mount YAFFS2]: /dev/$1 --> /mnt/$MNT" > /dev/console
			echo "$ACT /mnt/$1" >> /tmp/mdev.log
		elif mount -t ubifs "/dev/$1" "/mnt/$1"; then
			echo "[Mount UBIFS]: /dev/$1 --> /mnt/$MNT" > /dev/console
			echo "$ACT /mnt/$1" >> /tmp/mdev.log
		else
			# failed to mount, clean up mountpoint
			if ! rmdir "/mnt/$1"; then
				exit 1
			fi
		fi
	else
		# try vfat only
		if mount -t vfat -o noatime,shortname=mixed,utf8 "/dev/$1" "/mnt/$1"; then
			# ln -s /mnt/$1 /mnt/${MNT}0
            # FIX 若最后的字符不是数字,是数字则不进行下面的操作
            ret_str=`echo $1|grep -o '.$' |grep '^[0-9.]'`
            if [ $? -eq 1 ];then
                ln -s /mnt/$1 /mnt/${MNT}100
            fi
            # FIX END
			echo "[Mount VFAT]: /dev/$1 --> /mnt/$MNT" > /dev/console
			echo "$ACT /mnt/$1" >> /tmp/mdev.log
		else
			# failed to mount, clean up mountpoint
			if ! rmdir "/mnt/$1"; then
				exit 1
			fi
			exit 1
		fi
	fi
fi

# cd /mnt/mtdblock2
# ./scale_app.arm.elf


if [ "$1" == "mtdblock2" ]; then

	UpgradeFile="/mnt/mtdblock2/scale_app.arm.elf.out"
	CurrentFile="/mnt/mtdblock2/scale_app.arm.elf"
	BackFile="/mnt/mtdblock2/scale_app.arm.elf.bak"
	if [ ! -f "$UpgradeFile" ]; then
		echo "No Need Upgread!!"
	else
		if [ ! -f "$CurrentFile" ];then
			echo "No Need Upgread!!"
		else
			cp $CurrentFile $BackFile
			cp $UpgradeFile $CurrentFile
		fi
	fi


        if [ -f "/mnt/mtdblock2/auto.sh" ]; then
                echo "Run auto.sh !!!" > /dev/console
		chmod +x /mnt/mtdblock2/auto.sh
                /mnt/mtdblock2/auto.sh
        elif [ -f "/mnt/mtdblock2/scale_app.arm.elf" ]; then
                echo "Start app !!!" > /dev/console
                chmod +x /mnt/mtdblock2/scale_app.arm.elf
		chmod +x /mnt/mtdblock2/wifi/hostapd
		chmod +x /mnt/mtdblock2/wifi/wpa_supplicant
                /mnt/mtdblock2/scale_app.arm.elf &
        else
                echo "No valid app , please burn it in !!!" > /dev/console
        fi

	sleep 1

	if [ ! -f "$UpgradeFile" ]; then
		echo "No $UpgradeFile"
	else
		while true
		do
			process=`ps w|grep scale_app.arm.elf | grep -v grep`;
			if [ "$process" == "" ]; then
				echo "no process"
				cp $BackFile $CurrentFile
				rm -rf $BackFile
				chmod +x /mnt/mtdblock2/scale_app.arm.elf
				/mnt/mtdblock2/scale_app.arm.elf &
				break
			else
				echo "Upgread success@@!"
				rm -rf $UpgradeFile
				rm -rf $BackFile
				break
			fi

		done
	fi
    if [ ! -f "/mnt/mtdblock2/scale_app.arm.elf" ];then
        /sbin/scale_app.arm.elf &
    fi

fi



# first time
if [ `echo "$1" | grep 'sd[a-z]'` ]; then
	if [ -f "/mnt/$1/init/auto.sh" ] && [ ! -f "/mnt/mtdblock2/auto.sh" ]; then
		cp /mnt/$1/init/auto.sh /mnt/mtdblock2
	fi

	if [ -d "/mnt/$1/init/wifi" ] && [ ! -f "/mnt/mtdblock2/wifi" ]; then
		cp -rf /mnt/$1/init/wifi /mnt/mtdblock2
	fi

	if [ -f "/mnt/$1/init/scale_app.arm.elf" ] && [ ! -f "/mnt/mtdblock2/scale_app.arm.elf" ]; then
		cp /mnt/$1/init/scale_app.arm.elf /mnt/mtdblock2
		reboot
	fi
fi

