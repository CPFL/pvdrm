ccflags-y := -Iinclude/drm
pvdrm-y  := \
	pvdrm_nouveau_abi16.o \
	pvdrm_cache.o \
	pvdrm_channel.o \
	pvdrm_drm.o \
	pvdrm_gem.o \
	pvdrm_host_table.o \
	pvdrm_irq.o \
	pvdrm_pushbuf.o \
	pvdrm_slot.o \
	pvdrm_ttm.o \
	pvdrm_vblank.o

obj-$(CONFIG_DRM_PVDRM) += pvdrm.o
