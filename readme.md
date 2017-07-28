# 使用说明

## 准备工作(必读)
* 使用项目的配置文件
`cp -a ./config_for_company7 ./.config` : light

## 编译
* light项目
`make  ARCH=arm CROSS_COMPILE=arm-linux-`

### 编译参数说明
* 暂无

## 不同项目的编译宏
* light
`暂无`

## 不同项目的内核配置文件
* light
`config_for_company7`

## 文件说明
.
├── linux-3.10.x                       #内核源码。  
└── rootfs                             #根文件系统（需要内核编译之前更新svn， 在内核目录执行`rm usr/initramfs_data.cpio;make ARCH=arm CROSS_COMPILE=arm-linux-`可重新将rootfs打包进内核）。  

## rootfs
  由于console设备文件是root权限，固首次`rootfs`更新需要将系统的`console`文件`copy`进下载的`rootfs`中 即是执行`sudo cp -a /dev/console rootfs/dev/`即可。

## 总结
* rootfs中含有设备文件以及软连接，这些特性在`windows`中`FAT/NTFS`系列文件系统中不受支持,请将这些文件放在`ext`系列文件系统中(即是`linux`中).
* rootfs中含有root用户的设备节点`/dev/console`.

## 注意

