KDIR ?= /lib/modules/`uname -r`/build

KERNELRELEASE ?= $(shell cat $(KDIR)/include/config/kernel.release 2> /dev/null)

ifneq ($(wildcard /boot/System.map-$(KERNELRELEASE)),)
SYSMAP := /boot/System.map-$(KERNELRELEASE)
else
SYSMAP := $(KDIR)/System.map
endif

all: modules

modules: generated/pvdrm_imported.h
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	$(RM) generated/*
	$(MAKE) -C $(KDIR) M=$$PWD clean

install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

help:
	$(MAKE) -C $(KDIR) M=$$PWD help

.PHONY : all modules clean install help

# Importing hidden kernel symbols.

generated/pvdrm_imported.h: generated/System.map-$(KERNELRELEASE) common/pvdrm_imported.h.in
	@tools/import-kernel-symbols.py \
		"common/pvdrm_imported.h.in" \
		"generated/System.map-$(KERNELRELEASE)" > generated/pvdrm_imported.h

generated/System.map-$(KERNELRELEASE): $(SYSMAP)
	@tools/copy-system-map \
		"$(SYSMAP)" \
		"generated/System.map-$(KERNELRELEASE)"
