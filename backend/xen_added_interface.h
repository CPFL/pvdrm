/*
  Copyright (C) 2014 Yusuke Suzuki <yusuke.suzuki@sslab.ics.keio.ac.jp>

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef PVDRM_BACK_XEN_ADDED_INTERFACE_H_
#define PVDRM_BACK_XEN_ADDED_INTERFACE_H_

/*
 * FIXME: calling domctl is not well-formed. However, without Xen's modification,
 * there's no approprite interface for this.
 * In the future, we'll add this to Xen's new API, or construct backend inside
 * qemu's user space application.
 */

//#define XEN_DOMCTL_INTERFACE_VERSION 0x00000008
#define XEN_DOMCTL_INTERFACE_VERSION 0x0000000a

#define uint64_aligned_t uint64_t __attribute__((aligned(8)))

#define __HYPERVISOR_domctl               36

/* Bind machine I/O address range -> HVM address range. */
/* XEN_DOMCTL_memory_mapping */
#define DPCI_ADD_MAPPING         1
#define DPCI_REMOVE_MAPPING      0
struct xen_domctl_memory_mapping {
    uint64_aligned_t first_gfn; /* first page (hvm guest phys page) in range */
    uint64_aligned_t first_mfn; /* first page (machine page) in range */
    uint64_aligned_t nr_mfns;   /* number of pages in range (>0) */
    uint32_t add_mapping;       /* add or remove mapping */
    uint32_t padding;           /* padding for 64-bit aligned structure */
};
DEFINE_GUEST_HANDLE_STRUCT(xen_domctl_memory_mapping);

struct xen_domctl_iomem_permission {
    uint64_aligned_t first_mfn;/* first page (physical page number) in range */
    uint64_aligned_t nr_mfns;  /* number of pages in range (>0) */
    uint8_t  allow_access;     /* allow (!0) or deny (0) access to range? */
};
DEFINE_GUEST_HANDLE_STRUCT(xen_domctl_iomem_permission);


struct xen_domctl {
    uint32_t cmd;
#define XEN_DOMCTL_iomem_permission              20
#define XEN_DOMCTL_memory_mapping                39
    uint32_t interface_version; /* XEN_DOMCTL_INTERFACE_VERSION */
    domid_t  domain;
    union {
        struct xen_domctl_iomem_permission  iomem_permission;
        struct xen_domctl_memory_mapping    memory_mapping;
        uint8_t                             pad[128];
    } u;
};
DEFINE_GUEST_HANDLE_STRUCT(xen_domctl);

#endif  /* PVDRM_BACK_XEN_ADDED_INTERFACE_H_ */
/* vim: set sw=8 ts=8 et tw=80 : */
