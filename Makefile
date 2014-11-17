KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

help:
	$(MAKE) -C $(KDIR) M=$$PWD help
