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

#ifndef PVDRM_WAIT_H_
#define PVDRM_WAIT_H_

#define PVDRM_POLL_COUNT 100000
#define PVDRM_WAIT(condition, interruptible) \
	({\
		int __PVDRM_WAIT_RET__ = 0; \
		int __PVDRM_POLL_COUNTER__ = PVDRM_POLL_COUNT; \
		while (!(condition)) { \
			if (__PVDRM_POLL_COUNTER__-- < 0) { \
				/* Sleep. */ \
				ktime_t time; \
				__set_current_state((interruptible) ? TASK_INTERRUPTIBLE : TASK_UNINTERRUPTIBLE); \
				time = ktime_set(0, 20);  /* This value derived from Paradice [ASPLOS '14]. */ \
				schedule_hrtimeout(&time, HRTIMER_MODE_REL); \
				if ((interruptible) && signal_pending(current)) { \
					__PVDRM_WAIT_RET__ = -ERESTARTSYS; \
					break; \
				} \
				__PVDRM_POLL_COUNTER__ = PVDRM_POLL_COUNT; \
			} \
		} \
		__set_current_state(TASK_RUNNING); \
		__PVDRM_WAIT_RET__;\
	})

#endif  /* PVDRM_WAIT_H_ */
