KERNEL_LOCATION=/build/linux
ARMMAKE=make ARCH=arm SUBARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

obj-m := bjr.o

bjr.ko: bjr.c
	$(ARMMAKE) -C $(KERNEL_LOCATION) M=$(PWD) modules

clean:
	rm -f *.ko *.o *.mod.c *.symvers *.order
