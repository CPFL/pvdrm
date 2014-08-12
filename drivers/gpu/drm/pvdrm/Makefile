ccflags-y := -Iinclude/drm
pvdrm-y  := \
	pvdrm_nouveau_abi16.o \
	pvdrm_drm.o \
	pvdrm_gem.o \
	pvdrm_irq.o \
	pvdrm_load.o \
	pvdrm_slot.o \
	pvdrm_ttm.o \
	pvdrm_vblank.o

obj-$(CONFIG_DRM_PVDRM) += pvdrm.o
