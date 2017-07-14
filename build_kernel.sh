#!/bin/bash

export ARCH=arm
<<<<<<< HEAD
export CROSS_COMPILE=$(pwd)/../PLATFORM/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin/arm-eabi-

mkdir output

make -C $(pwd) O=output VARIANT_DEFCONFIG=apq8084_sec_trlte_tmo_defconfig apq8084_sec_defconfig SELINUX_DEFCONFIG=selinux_defconfig
make -C $(pwd) O=output

cp output/arch/arm/boot/Image $(pwd)/arch/arm/boot/zImage
=======
export PATH=$(pwd)/../PLATFORM/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin:$PATH

mkdir output

make -C $(pwd) O=output CROSS_COMPILE=arm-eabi- VARIANT_DEFCONFIG=apq8084_sec_lentislte_skt_defconfig apq8084_sec_defconfig SELINUX_DEFCONFIG=selinux_defconfig SELINUX_LOG_DEFCONFIG=selinux_log_defconfig TIMA_DEFCONFIG=tima_defconfig DMVERITY_DEFCONFIG=dmverity_defconfig
make -j64 -C $(pwd) O=output CROSS_COMPILE=arm-eabi-

cp output/arch/arm/boot/zImage $(pwd)/arch/arm/boot/zImage
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
