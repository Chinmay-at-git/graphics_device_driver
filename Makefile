# makefile for charmod

obj-m += lab3_kernelcode2.o

default:
	$(MAKE) -C /usr/src/linux M=$(PWD) modules
clean:
	rm *.ko
	rm *.o
	rm *.mod.c

