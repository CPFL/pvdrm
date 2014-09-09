# This Makefile is loosely based on the drm.git/linux-core/Makefile and
# the preliminary work by Christopher James Halse Rogers.
# Edited by Pekka Paalanen <pq@iki.fi>
# Edited by Albert Pool <albertpool@solcon.nl>
# Edited by Martin Peres <martin.peres@free.fr>

# By default, the build is done against the running linux kernel source.
# To build against a different kernel source tree, set LINUXDIR:
#
#    make LINUXDIR=/path/to/kernel/source

# By default, it is assumed the Nouveau kernel tree is found in $PWD/../.
# To use a different path to the Nouveau kernel tree, set NOUVEAUROOTDIR:
#
#    make NOUVEAUROOTDIR=/path/to/nouveau/linux-2.6

# 'make headers' will install the headers by default to
# $NOUVEAUROOTDIR/usr/include/drm. To use a different location,
# set HEADERINSDIR:
#
#    make HEADERINSDIR=/path/to/install/headers

# configuration

NOUVEAUROOTDIR=/root/dev/linux-3.6.5/

ifndef LINUXDIR
RUNNING_REL := $(shell uname -r)

LINUXDIR := $(shell if [ -e /lib/modules/$(RUNNING_REL)/build ]; then \
                echo /lib/modules/$(RUNNING_REL)/build; \
                else echo /lib/modules/$(RUNNING_REL)/source; fi)
endif

NOUVEAUGITREMOTE ?= origin
NOUVEAUROOTDIR ?= $(CURDIR)/..
override NOUVEAUROOTDIR := $(abspath $(NOUVEAUROOTDIR))
HEADERINSDIR ?= $(NOUVEAUROOTDIR)/usr/include/uapi/drm
override HEADERINSDIR := $(abspath $(HEADERINSDIR))

TESTFILE := drivers/gpu/drm/nouveau/nouveau_drv.h
ifeq ($(wildcard $(NOUVEAUROOTDIR)/$(TESTFILE)),)
	$(error $(NOUVEAUROOTDIR) does not look like the right kernel tree, \
		please set NOUVEAUROOTDIR)
endif

GIT_REVISION := $(shell GIT_DIR=$(NOUVEAUROOTDIR)/.git \
	git describe --always --abbrev=17 2> /dev/null)

DRMINC := $(NOUVEAUROOTDIR)/include/drm
DRMDIR := $(NOUVEAUROOTDIR)/drivers/gpu/drm
DRMCFG := \
	CONFIG_DRM=m \
	CONFIG_DRM_KMS_HELPER=m \
	CONFIG_DRM_TTM=m \
	CONFIG_DRM_NOUVEAU=m \
	CONFIG_DRM_NOUVEAU_KMS=m \
	CONFIG_DRM_NOUVEAU_BACKLIGHT=n \
	CONFIG_DRM_NOUVEAU_DEBUG=n \
	CONFIG_DRM_I2C_CH7006=m \
	CONFIG_DRM_I2C_SIL164=m \
	CONFIG_DRM_AST=n \
	CONFIG_DRM_CIRRUS_QEMU=n \
	CONFIG_DRM_TDFX=n \
	CONFIG_DRM_R128=n \
	CONFIG_DRM_RADEON=n \
	CONFIG_DRM_MGA=n \
	CONFIG_DRM_MGAG200=n \
	CONFIG_DRM_I810=n \
	CONFIG_DRM_I830=n \
	CONFIG_DRM_I915=n \
	CONFIG_DRM_GMA500=n \
	CONFIG_DRM_SIS=n \
	CONFIG_DRM_SAVAGE=n \
	CONFIG_DRM_UDL=n \
	CONFIG_DRM_VIA=n \
	CONFIG_DRM_VMWGFX=n \
	CONFIG_DRM_PVDRM=m \
	CONFIG_DRM_PVDRM_BACK=m

EXTRA_CFLAGS :=
MYEXTRA_CFLAGS :=
MYEXTRA_CFLAGS += -DCONFIG_DRM_NOUVEAU_BACKLIGHT
MYEXTRA_CFLAGS += -DCONFIG_DRM_NOUVEAU_DEBUG

ifneq ($(GIT_REVISION),)
	MYEXTRA_CFLAGS += '-DGIT_REVISION=\"$(GIT_REVISION)\"'
endif

MYPARMS := -C $(LINUXDIR) KCPPFLAGS="-I$(DRMINC)" SUBDIRS="$(DRMDIR)" $(DRMCFG)

MYEXTRA_CFLAGS += $(EXTRA_CFLAGS)
ifneq ($(MYEXTRA_CFLAGS),)
MYPARMS += EXTRA_CFLAGS="$(MYEXTRA_CFLAGS)"
endif

.PHONY: all modules install clean archive headers FORCE

all: modules
FORCE:

modules:
	$(MAKE) $(MYPARMS) modules

install:
	$(MAKE) $(MYPARMS) modules_install

clean:
	$(MAKE) $(MYPARMS) clean

headers:
	mkdir -p $(HEADERINSDIR)
	cd $(LINUXDIR) && \
	perl scripts/headers_install.pl $(DRMINC) $(HEADERINSDIR) dummy \
	$(unifdef-y)

archive: newttm-devel-compat.tar.gz newttm-devel.tar.gz master.tar.gz

newttm-devel-compat.tar.gz: EXTRAS=nouveau README-nouveau

%.tar: FORCE
	GIT_DIR=$(NOUVEAUROOTDIR)/.git \
	git archive --format=tar --prefix=$*/ $(NOUVEAUGITREMOTE)/$* \
	drivers/gpu/drm include/drm $(EXTRAS) > $@

%.tar.gz: %.tar
	gzip -9 -c $< > $@

# The commit id of the archive can be read with
# $ zcat foo.tar.gz | git get-tar-commit-id
