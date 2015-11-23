KERNELDIR ?= /lib/modules/$(shell uname -r)/build

obj-m	:= tilegxpci.o

tilegxpci-objs := \
  tilegxpci_host.o \
  barmem.o \
  boards.o \
  packet_queue.o \
  raw_dma.o \
  gxpci_host_nic.o \
  tilegxpci_shared_code.o

obj-m	+= tilegxpci_nic.o

tilegxpci_nic-objs += \
  tilegxpci_host_nic.o \
  barmem.o \
  boards.o \
  packet_queue.o \
  raw_dma.o \
  gxpci_host_nic.o \
  tilegxpci_shared_code.o

obj-m   += tilegxpci_p2p.o

tilegxpci_p2p-objs += \
  tilegxpci_host_p2p.o \
  barmem.o \
  boards.o \
  packet_queue.o \
  raw_dma.o \
  gxpci_host_nic.o \
  tilegxpci_shared_code.o

obj-m	+= tilegxpci_vf.o

tilegxpci_vf-objs += \
  tilegxpci_host_vf.o \
  gxpci_host_nic_vf.o \
  packet_queue_vf.o

obj-m   += tilegxpci_vf_p2p.o

tilegxpci_vf_p2p-objs += \
  tilegxpci_host_vf_p2p.o \
  gxpci_host_nic_vf_p2p.o \
  packet_queue_vf.o

obj-m	+= tilegxpci_sdnic.o

tilegxpci_sdnic-objs += \
  tilegxpci_host_sdnic.o \
  barmem.o \
  boards.o \
  packet_queue.o \
  raw_dma.o \
  gxpci_host_nic.o \
  gxpci_host_netdev_ctrl.o \
  tilegxpci_shared_code.o

PWD       := $(shell pwd)

all: tilegxpci tilegxpci_nic tilegxpci_p2p tilegxpci_vf tilegxpci_vf_p2p tilegxpci_sdnic

tilegxpci: cleanish
	cp tilegxpci_host.c tilegxpci_host_nic.c
	cp tilegxpci_host.c tilegxpci_host_p2p.c
	cp tilegxpci_host_vf.c tilegxpci_host_vf_p2p.c
	cp gxpci_host_nic_vf.c gxpci_host_nic_vf_p2p.c
	cp tilegxpci_host.c tilegxpci_host_sdnic.c
	$(MAKE) -C $(KERNELDIR) M=$(PWD) \
	  EXTRA_CFLAGS+=-DTILEPCI_HOST
	cp tilegxpci.ko tilegxpci.ko.tmp

tilegxpci_nic: cleanish_nic
	$(MAKE) -C $(KERNELDIR) M=$(PWD) \
	  EXTRA_CFLAGS+="-DTILEPCI_HOST -DGXPCI_NETLIB_VNIC"
	cp tilegxpci_nic.ko tilegxpci_nic.ko.tmp

tilegxpci_p2p: cleanish_p2p
	$(MAKE) -C $(KERNELDIR) M=$(PWD) \
	  EXTRA_CFLAGS+="-DTILEPCI_HOST -DGXPCI_HOST_NIC_P2P"
	cp tilegxpci_p2p.ko tilegxpci_p2p.ko.tmp

tilegxpci_vf: cleanish_vf
	$(MAKE) -C $(KERNELDIR) M=$(PWD) \
	  EXTRA_CFLAGS+=-DTILEPCI_HOST
	cp tilegxpci_vf.ko tilegxpci_vf.ko.tmp

tilegxpci_vf_p2p: cleanish_vf_p2p
	$(MAKE) -C $(KERNELDIR) M=$(PWD) \
	  EXTRA_CFLAGS+="-DTILEPCI_HOST -DGXPCI_HOST_NIC_P2P"
	cp tilegxpci_vf_p2p.ko tilegxpci_vf_p2p.ko.tmp

tilegxpci_sdnic: cleanish_sdnic
	$(MAKE) -C $(KERNELDIR) M=$(PWD) \
	  EXTRA_CFLAGS+="-DTILEPCI_HOST -DGXPCI_HOST_SDNIC"
	mv tilegxpci.ko.tmp tilegxpci.ko
	mv tilegxpci_nic.ko.tmp tilegxpci_nic.ko
	mv tilegxpci_p2p.ko.tmp tilegxpci_p2p.ko
	mv tilegxpci_vf.ko.tmp tilegxpci_vf.ko
	mv tilegxpci_vf_p2p.ko.tmp tilegxpci_vf_p2p.ko

cleanish:
	rm -rf *.o *~ core .depend .*.cmd *.mod.c .tmp_versions

cleanish_nic:
	rm -rf *.o *~ core .depend .*.cmd *.mod.c .tmp_versions

cleanish_p2p:
	rm -rf *.o *~ core .depend .*.cmd *.mod.c .tmp_versions

cleanish_vf:
	rm -rf *.o *~ core .depend .*.cmd *.mod.c .tmp_versions

cleanish_vf_p2p:
	rm -rf *.o *~ core .depend .*.cmd *.mod.c .tmp_versions

cleanish_sdnic:
	rm -rf *.o *~ core .depend .*.cmd *.mod.c .tmp_versions

clean: cleanish
	rm -rf *.ko
