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
#ifndef PVDRM_BENCH_H_
#define PVDRM_BENCH_H_
#include <linux/time.h>
#include "pvdrm_log.h"

struct pvdrm_bench {
        struct timespec elapsed;
        bool opened;
};

static inline void pvdrm_bench_open(struct pvdrm_bench* bench)
{
        bench->opened = true;
        bench->elapsed = CURRENT_TIME;
}

static inline void pvdrm_bench_close(struct pvdrm_bench* bench, bool dump)
{
        const struct timespec finish = CURRENT_TIME;
        bench->elapsed = timespec_sub(finish, bench->elapsed);
        bench->opened = false;
        if (dump) {
                const double ms = bench->elapsed.tv_sec * 1000 + (bench->elapsed.tv_nsec / 1000.0);
                PVDRM_INFO("elapsed time: %0.6fms", ms);
        }
}

#define PVDRM_BENCH(bench) \
        for (pvdrm_bench_open(bench); (bench)->opened; pvdrm_bench_close(bench, true))

#endif  /* PVDRM_BENCH_H_ */
/* vim: set sw=8 ts=8 et tw=80 : */
