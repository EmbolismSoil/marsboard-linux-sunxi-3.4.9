obj-m:=sunxi-onboard-codec.o
KDIR:=/home/seminelee/github/marsboard-linux-sunxi-3.4.9
PWD:=$(shell pwd)

ccflags-y := -std=gnu99 -Wno-declaration-after-statement

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C $(KDIR) M=$(PWD) modules
