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
#ifndef PVDRM_IDR_H_
#define PVDRM_IDR_H_

#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,15,0)
static inline int pvdrm_idr_alloc(struct idr* idr, spinlock_t* lock, void* ptr, int above)
{
	unsigned long flags;
	int ret = 0;
	idr_preload(GFP_KERNEL);
	spin_lock_irqsave(lock, flags);
	ret = idr_alloc(idr, ptr, above, 0, GFP_NOWAIT);
	spin_unlock_irqrestore(lock, flags);
	idr_preload_end();
	return ret;
}
#else
static inline int pvdrm_idr_alloc(struct idr* idr, spinlock_t* lock, void* ptr, int above)
{
	unsigned long flags;
	int ret = 0;
	int handle = 0;
	if (idr_pre_get(idr, GFP_KERNEL) == 0) {
		return -ENOMEM;
	}
again:
	spin_lock_irqsave(lock, flags);
	ret = idr_get_new_above(idr, ptr, above, (int *)&handle);
	spin_unlock_irqrestore(lock, flags);
	if (ret == -EAGAIN) {
		goto again;
	}

	if (ret < 0) {
		return ret;
	}

	return handle;
}
#endif

#endif  /* PVDRM_IDR_H_ */
