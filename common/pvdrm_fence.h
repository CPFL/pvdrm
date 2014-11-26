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
#ifndef PVDRM_FENCE_H_
#define PVDRM_FENCE_H_

#include <linux/atomic.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/types.h>

#include "pvdrm_wait.h"
#include "pvdrm_limits.h"

struct pvdrm_device;

enum {
	PVDRM_FENCE_DONE = UINT32_MAX
};

struct pvdrm_fence {
	atomic_t seq;
};

static inline void pvdrm_fence_emit(struct pvdrm_fence* fence, uint32_t seq)
{
	wmb();
	atomic_set(&fence->seq, seq);
}

static inline uint32_t pvdrm_fence_read(struct pvdrm_fence* fence)
{
	return atomic_read(&fence->seq);
}

static inline int pvdrm_fence_wait(struct pvdrm_fence* fence, uint32_t expected, bool interruptible)
{
	int ret = 0;
	int once = PVDRM_POLL_COUNT;

	wmb();
	while (pvdrm_fence_read(fence) != expected) {
		if (once-- < 0) {
			ktime_t time;
			__set_current_state(interruptible ? TASK_INTERRUPTIBLE : TASK_UNINTERRUPTIBLE);

			time = ktime_set(0, 200);  /* This value derived from Paradice [ASPLOS '14]. */
			schedule_hrtimeout(&time, HRTIMER_MODE_REL);
			if (interruptible && signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}
			once = PVDRM_POLL_COUNT;
		}
	}

	__set_current_state(TASK_RUNNING);
	return ret;
}

#endif  /* PVDRM_FENCE_H_ */
