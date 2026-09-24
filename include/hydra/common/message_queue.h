/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once
#include <chrono>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>

/**
 * @file message_queue.h
 * @brief Thread-safe FIFO queue used to pass messages between pipeline modules.
 *        A mutex guards every operation and a condition variable lets consumers
 *        wait for data; an optional capacity bounds the queue.
 */

namespace hydra {

/// @brief Mutex-protected FIFO queue with optional capacity and waiting.
/// @tparam T Element type stored in the queue.
template <typename T>
struct MessageQueue {
  using Ptr = std::shared_ptr<MessageQueue<T>>;
  std::list<T> queue;                   ///< Underlying element storage.
  mutable std::mutex mutex;             ///< Guards all access to the queue.
  mutable std::condition_variable cv;   ///< Signals waiters on push.
  size_t max_size;                      ///< Capacity limit; 0 means unbounded.

  /// @brief Construct a queue with a maximum size.
  /// @param max_size Capacity limit; 0 leaves the queue unbounded.
  explicit MessageQueue(size_t max_size) : max_size(max_size) {}

  /// @brief Construct an unbounded queue.
  MessageQueue() : MessageQueue(0) {}

  /// @brief Whether the queue currently holds no elements.
  bool empty() const {
    std::unique_lock<std::mutex> lock(mutex);
    return queue.empty();
  }

  /// @brief Access the front element without removing it.
  const T& front() const {
    std::unique_lock<std::mutex> lock(mutex);
    return queue.front();
  }

  /// @brief Access the back element without removing it.
  const T& back() const {
    std::unique_lock<std::mutex> lock(mutex);
    return queue.back();
  }

  /// @brief Wait up to a timeout for the queue to become non-empty.
  /// @param wait_time_us Maximum time to wait, in microseconds.
  /// @return True if the queue has data within the timeout.
  bool poll(int wait_time_us = 1000) const {
    std::chrono::microseconds wait_duration(wait_time_us);
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, wait_duration, [&] { return !queue.empty(); });
  }

  /// @brief Wait up to a timeout for the queue to become empty.
  /// @param wait_time_us Maximum time to wait, in microseconds.
  /// @return True if the queue is empty within the timeout.
  bool block(int wait_time_us = 1000) const {
    std::chrono::microseconds wait_duration(wait_time_us);
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, wait_duration, [&] { return queue.empty(); });
  }

  /// @brief Append an element, respecting the capacity limit.
  /// @param input Element to move into the queue.
  /// @return True if added, false if the queue was at capacity.
  bool push(T input) {
    bool added = false;
    {
      std::unique_lock<std::mutex> lock(mutex);
      if (!max_size || queue.size() < max_size) {
        queue.push_back(std::move(input));
        added = true;
      }
    }

    cv.notify_all();

    return added;
  }

  /// @brief Remove and return the front element.
  /// @return The element that was at the front of the queue.
  T pop() {
    std::unique_lock<std::mutex> lock(mutex);
    auto value = std::move(queue.front());
    queue.pop_front();
    return value;
  }

  /// @brief Current number of elements in the queue.
  size_t size() const {
    std::unique_lock<std::mutex> lock(mutex);
    return queue.size();
  }

  /// @brief Remove all elements from the queue.
  void clear() {
    std::unique_lock<std::mutex> lock(mutex);
    queue.clear();
  }
};

}  // namespace hydra
