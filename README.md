
Исходный код ядра от Cellon, предоставленный Highscreen(Спасибо им за это!).

How to build:

1.

export ARCH=arm

export SUBARCH=arm

2.

make Exynos4Brothers_newman_defconfig

or

make Exynos4Brothers_highscreen_defconfig

3.

Build it. I suggest using GCC 4.4.3

make -j5 ARCH=arm CROSS_COMPILE=/<...>/toolchains/arm-linux-androideabi-4.4.3/prebuilt/linux-x86_64/bin/arm-linux-androideabi-

4.

Flash zImage to your phone:

fastboot -i 0x283b flash kernel '/<...>/arch/arm/boot/zImage'

fastboot -i 0x283b reboot

5.

Profit!

Exynos4Brothers Open Source Project pages:

http://4pda.ru/forum/index.php?showtopic=473990

http://4pda.ru/forum/index.php?showtopic=478538

Good luck!
