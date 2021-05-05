KERNEL_LOCATION=/build/linux
ARMMAKE=make ARCH=arm SUBARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

obj-m := eda-irq.o

eda-irq.ko: eda-irq.c
	$(ARMMAKE) -C $(KERNEL_LOCATION) M=$(PWD) modules

clean:
	rm -f *.ko *.o *.mod.c *.symvers *.order
