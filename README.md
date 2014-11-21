# PVDRM
PVDRM: Para-Virtualization leveraging DRM as a PV boundary.

[![Build Status](https://magnum.travis-ci.com/CPFL/pvdrm.svg?token=NfefyoR2J5KLwyG5o6Lh&branch=master)](https://magnum.travis-ci.com/CPFL/pvdrm)

PVDRM (Para-Virtualized DRM) is the para-virtualization approach for GPGPU computing.
It takes a conventional Xen splitted driver model;
it provides front-end(`pvdrm-front`) and back-end drivers (`pvdrm-back`).

In particular, PVDRM uses DRM as a PV boundary.
DRM is a common abstraction layer for GPU drivers
(e.g. i915 for Intel Intgrated GPUs, radeon for AMD GPUs and nouveau for NVIDIA GPUs).
By leveraging DRM APIs (including GEM object APIs), PVDRM enables multiplexing and isolation among VMs without compromising performance. And it gives portability over linux kernel versions.

Currently, it only supports nouveau driver for NVIDIA GPUs.
We tested PVDRM on 3.6.5 and 3.17.2 Linux kernels.

## Build Instruction

To build the PVDRM kernel modules, simply hit the `make` command.
Linux kernel headers need to be installed to build kernel modules.
```sh
make
```

And install it by
```sh
make install
```

Now, `pvdrm-front` and `pvdrm-back` drivers are installed.

## Using PVDRM

On the domain 0 side, you need to run `nouveau` driver for NVIDIA GPUs.
And after that, `modprobe pvdrm-back` will load `pvdrm-back` back-end driver in the domain 0.

After launching the domain U VM, `modprobe pvdrm-front` will load `pvdrm-front` front-end driver.
And executing `tools/vdrm <device-path> <domain-U>` will connect vdrm devices to the specified domain U.
For example, `tools/vdrm /dev/dri/card0 1` will create the virtual devices on Xen bus and it will be probed by `pvdrm-back` and `pvdrm-front`.

## Options

### back-end driver

`pvdrm-back` supports `sequential` option. When it is specified, `pvdrm-back` will execute DRM API requests sequentially

### front-end driver

`pvdrm-front` supports `cache` option. When it is specified, `pvdrm-front` will pool allocated GEM object in the front-end driver.
It will reduce hypercall frequency and improve the performance.
