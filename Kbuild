EXTRA_CFLAGS := \
    -Iinclude/drm \
    -I$(src)

obj-m := \
    pvdrm-back.o \
    pvdrm-front.o

pvdrm-front-y := \
	frontend/pvdrm_nouveau_abi16.o \
	frontend/pvdrm_cache.o \
	frontend/pvdrm_channel.o \
	frontend/pvdrm_drm.o \
	frontend/pvdrm_gem.o \
	frontend/pvdrm_host_table.o \
	frontend/pvdrm_irq.o \
	frontend/pvdrm_pushbuf.o \
	frontend/pvdrm_slot.o \
	frontend/pvdrm_ttm.o \
	frontend/pvdrm_vblank.o \
	frontend/drm_xenbus.o

pvdrm-back-y := \
	backend/pvdrm_back_drv.o \
	backend/pvdrm_back_file.o \
	backend/pvdrm_back_vma.o
