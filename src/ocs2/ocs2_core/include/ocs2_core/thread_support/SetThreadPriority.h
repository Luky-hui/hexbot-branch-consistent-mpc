/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <algorithm>
#include <cstring>
#include <sched.h>
#include <pthread.h>
#include <iostream>
#include <thread>

namespace ocs2 {

/**
 * Sets the priority of the input thread.
 *
 * @param priority: The priority of the thread from 0 (lowest) to 99 (highest)
 * @param thread: A reference to the tread.
 */
inline void setThreadPriority(int priority, pthread_t thread) {
  if (priority == 0) {
    return;
  }

  const int minPriority = sched_get_priority_min(SCHED_FIFO);
  const int maxPriority = sched_get_priority_max(SCHED_FIFO);
  const int boundedPriority = std::min(std::max(priority, minPriority), maxPriority);
  if (boundedPriority != priority) {
    std::cerr << "WARNING: Requested thread priority " << priority << " is outside SCHED_FIFO bounds ["
              << minPriority << ", " << maxPriority << "], clamping to " << boundedPriority << "." << std::endl;
  }

  sched_param sched{};
  sched.sched_priority = boundedPriority;

  const int rc = pthread_setschedparam(thread, SCHED_FIFO, &sched);
  if (rc != 0) {
    std::cerr << "WARNING: Failed to set thread priority to " << boundedPriority
              << " with SCHED_FIFO (pthread_setschedparam rc=" << rc << ": " << std::strerror(rc)
              << "). Check realtime permissions (e.g., RLIMIT_RTPRIO and PAM limits)." << std::endl;
  }
}

/**
 * Sets the priority of the input thread.
 *
 * @param priority: The priority of the thread from 0 (lowest) to 99 (highest)
 * @param thread: A reference to the tread.
 */
inline void setThreadPriority(int priority, std::thread& thread) {
  setThreadPriority(priority, thread.native_handle());
}

/**
 * Sets the priority of the thread this function is called from.
 *
 * @param priority: The priority of the thread from 0 (lowest) to 99 (highest)
 */
inline void setThisThreadPriority(int priority) {
  setThreadPriority(priority, pthread_self());
}

}  // namespace ocs2
