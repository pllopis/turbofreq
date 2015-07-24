
ifneq ($(KERNELRELEASE),)

obj-m  := turbofreq.o 

else

KERNELDIR := /lib/modules/`uname -r`/build 
#CC := 
#ARCH :=

all::
	$(MAKE) -C $(KERNELDIR) M=$(PWD) # CC=$(CC)  ARCH=$(ARCH) 

clean::
	rm -f *.o  *.ko *.mod.c  modules.order  Module.symvers
	rm -rf .[a-z]*
	rm -f *~

endif
