ifeq ($(DEVICE),pre)
	MARCH_TUNE := -Os -march=armv7-a -mcpu=cortex-a8 -mtune=cortex-a8
else
ifeq ($(DEVICE),pixi)
	MARCH_TUNE := -Os -march=armv6k -mtune=arm1136jf-s
endif
endif

XVM = sub-projects/allocators/xvmalloc-kmod
EXTRA_CFLAGS	:=	-DCONFIG_BLK_DEV_RAMZSWAP_STATS	\
			-I$(PWD)/$(XVM)			\
			-Wall ${MARCH_TUNE}

obj-m	+=	ramzswap.o

modules:
	${MAKE} -C $(KERNEL_BUILD_PATH) M=$(PWD)/$(XVM) modules
	${MAKE} -C $(KERNEL_BUILD_PATH) M=$(PWD) modules

modules_install:
	${MAKE} -C $(KERNEL_BUILD_PATH) M=$(PWD)/$(XVM) modules_install
	${MAKE} -C $(KERNEL_BUILD_PATH) M=$(PWD) modules_install

clean:
	${MAKE} -C $(KERNEL_BUILD_PATH) M=$(PWD) clean
	${MAKE} -C $(KERNEL_BUILD_PATH) M=$(PWD)/$(XVM) clean
