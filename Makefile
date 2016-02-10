# makefile for charmod

obj-m += dma_kernel.o

default:
	$(MAKE) -C /usr/src/linux M=$(PWD) modules
clean:
	rm *.ko
	rm *.o
	rm *.mod.c

