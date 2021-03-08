ifneq ($(KERNELRELEASE),)

USProDrv-objs :=	  \
	sources/dma/mw-dma-mem.o					\
	sources/dma/mw-dma-user.o					\
	sources/dma/mw-dma-kernel.o					\
	sources/dma/mw-dma-phy.o					\
	sources/ospi/ospi-linux.o					\
	sources/ospi/linux-file.o  					\
	sources/avstream/v4l2.o				        \
	sources/avstream/v4l2-sg-buf.o				\
	sources/avstream/mw-event-dev.o				\
	sources/avstream/mw-event-ioctl.o			\
	sources/avstream/alsa.o				        \
	sources/avstream/capture.o						\
	sources/avstream/xi-driver.o					\
	sources/avstream/xi-misc.o					\
	sources/avstream/mw-stream.o 					\
	sources/avstream/ad7842_control.o			\
	sources/avstream/ad7611_control.o			\
	sources/avstream/wm8731_control.o			\
	sources/avstream/det_vga_timings.o 		\
	sources/audio-resample/resample.o 		
	

OS_ARCH := $(shell uname -m | sed -e 's/i.86/i386/')
#ifneq ($(OS_ARCH), i386)
#	USProDrv-objs += USProDrvLib_64.a
#else
#	USProDrv-objs += USProDrvLib_32.a
#endif


obj-m = USProDrv.o

EXTRA_CFLAGS += -I$(src) -I$(src)/public -I$(src)/sources -Wno-missing-braces -Wno-unused-variable -Wno-declaration-after-statement -Wno-unused-label -Wno-unused-function -Wno-unused-value -g
# strip debug symbols
EXTRA_LDFLAGS += -S

else

KERNELDIR ?= /lib/modules/$(shell uname -r)/build

MODULE_INSTALLDIR = /lib/modules/`uname -r`
#KERNELDIR?=/opt/navsim/linux-3.0.35.public

#KERNELDIR ?= /usr/src/linux-headers-4.10.0-40-generic
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
	$(MAKE)  -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
endif

install: modules_install

modules_install:
	#/bin/mkdir -p $(MODULE_INSTALLDIR)
	/bin/cp ./USProDrv.ko $(MODULE_INSTALLDIR)
	/bin/chmod 644 $(MODULE_INSTALLDIR)/USProDrv.ko
	/sbin/depmod -ae
	@echo
	@echo '**************************************************************************'
	@echo '* Driver installed successfully. Type "make load" to load it.            *'
	@echo '**************************************************************************'
	@echo

load:
	/sbin/modprobe USProDrv

unload:
	/sbin/modprobe -r USProDrv
