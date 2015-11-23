KERNELDIR ?= /lib/modules/$(shell uname -r)/build

obj-m	:= tileusb.o

PWD       := $(shell pwd)

all: cleanish
	$(MAKE) -C $(KERNELDIR) M=$(PWD) 

cleanish:
	rm -rf *.o *~ core .depend .*.cmd *.mod.c .tmp_versions

clean: cleanish
	rm -rf *.ko

