Building instructions:

Checkout branch topic/features with
    git checkout topic/features

There is already a .config file present in this branch.

Build the kernel by issuing
    make ARCH=arm CROSS_COMPILE=/path/to/cross/compiler/arm-none-eabi- uImage

Copy arch/arm/boot/uImage to /boot/uImage.e60q22 on your ereader rootfs.
