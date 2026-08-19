# SPDX-License-Identifier: GPL-2.0
# Out-of-tree ISP V4L2 driver — SDK cross-build, DKMS, and on-board native build.
#
# DKMS invokes: make KERNELRELEASE=<ver> ... modules
# Do NOT gate top-level targets on KERNELRELEASE (that breaks DKMS). Use MAKELEVEL.

ccflags-y += -I$(src)/platform
ccflags-y += -I$(src)/common
ccflags-y += -I$(src)/common/isp_hw_if
ccflags-y += -I$(src)/sensor
ccflags-y += -I$(src)/isp
ccflags-y += -I$(src)/platform/logger
ccflags-y += -I$(src)/cixvihw
ccflags-y += -DARMCB_CAM_KO
ccflags-y += -DARMCB_CAM_AHB

ifneq ($(wildcard $(srctree)/drivers/misc),)
ccflags-y += -I$(srctree)/drivers/misc
endif

obj-m += armcb_isp_v4l2.o
armcb_isp_v4l2-objs := sensor/armcb_sensor.o \
	common/armcb_camera_io_drv.o \
	common/system_dma.o \
	common/armcb_v4l_sd.o \
	platform/armcb_platform.o \
	isp/armcb_vb2.o \
	isp/armcb_v4l2_stream.o \
	isp/armcb_isp_driver.o \
	isp/armcb_v4l2_config.o \
	isp/armcb_v4l2_core.o \
	isp/armcb_isp_execstart.o \
	sensor/actuator/armcb_actuator.o \
	common/isp_hw_if/isp_hw_ops.o \
	common/isp_hw_if/isp_hw_utils.o \
	platform/logger/system_logger.o \
	cixvihw/cix_vi_hw.o \
	armcb_isp_entry.o

# Top-level make only (DKMS / developer). Kbuild re-invokes this file with MAKELEVEL>0.
ifeq ($(MAKELEVEL),0)

ARCH ?= arm64

PATH_ROOT ?= $(abspath $(CURDIR)/../../../..)

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
ifeq ($(wildcard $(KERNEL_SRC)/Makefile),)
KERNEL_SRC := $(PATH_ROOT)/linux
endif

# aarch64 board / DKMS: native gcc. x86 SDK host: cross unless CROSS_COMPILE is set.
ifeq ($(origin CROSS_COMPILE),undefined)
ifneq ($(shell uname -m),aarch64)
CROSS_COMPILE := aarch64-none-linux-gnu-
else
CROSS_COMPILE :=
endif
endif

.PHONY: build modules clean

# DKMS (via dkms.conf) and on-board: native kbuild, no cross prefix.
modules:
	$(MAKE) -C "$(KERNEL_SRC)" ARCH=$(ARCH) M="$(CURDIR)" modules

# SDK build-scripts: cross-compile against PATH_ROOT/linux.
build:
	$(MAKE) -C "$(KERNEL_SRC)" ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M="$(CURDIR)" modules

clean:
	-$(MAKE) -C "$(KERNEL_SRC)" M="$(CURDIR)" clean
	find . \( -name '*.o' -o -name '*.ko' -o -name '*.mod.c' -o -name '*.mod' \
		-o -name '*.symvers' -o -name '*.order' -o -name '*.a' \
		-o -name '*.cmd' -o -name '*.o.d' \) 2>/dev/null | xargs -r rm -f

endif
