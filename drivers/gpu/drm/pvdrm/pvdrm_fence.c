/*
 * Copyright (C) 2007 Ben Skeggs.
 * Copyright (C) 2014 Yusuke Suzuki <utatane.tea@gmail.com>
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER(S) AND/OR ITS SUPPLIERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <linux/atomic.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/time.h>

#include "pvdrm_fence.h"


void pvdrm_fence_init(struct pvdrm_fence* fence)
{
	pvdrm_fence_emit(fence, 0);
}

int pvdrm_fence_wait(struct pvdrm_fence* fence, bool interruptible)
{
	unsigned long sleep_time = NSEC_PER_MSEC / 1000;
	int ret = 0;

	while (!pvdrm_fence_done(fence)) {
		ktime_t time;
		__set_current_state(interruptible ? TASK_INTERRUPTIBLE : TASK_UNINTERRUPTIBLE);

		time = ktime_set(0, sleep_time);
		schedule_hrtimeout(&time, HRTIMER_MODE_REL);
		sleep_time *= 2;
		if (sleep_time > NSEC_PER_MSEC) {
			sleep_time = NSEC_PER_MSEC;
		}

		if (interruptible && signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}
	}

	__set_current_state(TASK_RUNNING);
	return ret;
}

uint32_t pvdrm_fence_read(struct pvdrm_fence* fence)
{
	return atomic_read(&fence->seq);
}

bool pvdrm_fence_done(struct pvdrm_fence* fence)
{
	return pvdrm_fence_read(fence) != 0;
}

/* vim: set sw=8 ts=8 et tw=80 : */
