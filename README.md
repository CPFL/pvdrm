# PVDRM: Para-Virtualization leveraging DRM as a PV boundary.

PVDRM (Para-Virtualized DRM) is the para-virtualization approach for GPGPU computing.
It takes a conventional Xen splitted driver model;
it provides front-end(`pvdrm-front`) and back-end drivers (`pvdrm-back`).

In particular, PVDRM uses DRM as a PV boundary.
DRM is a common abstraction layer for GPU drivers
(e.g. i915 for Intel Intgrated GPUs, radeon for AMD GPUs and nouveau for NVIDIA GPUs).
By leveraging DRM APIs (including GEM object APIs), PVDRM enables multiplexing and isolation among VMs without compromising performance. And it gives portability over linux kernel versions.

Currently, it only supports nouveau driver for NVIDIA GPUs.
We tested PVDRM on 3.6.5 and 3.17.2 Linux kernels.
