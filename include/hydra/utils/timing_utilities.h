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
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "hydra/utils/log_utilities.h"

/**
 * @file timing_utilities.h
 * @brief Wall-clock timing of named code sections. A singleton recorder accumulates
 *        per-timer measurements and reports statistics or writes them to log files;
 *        ScopedTimer measures the lifetime of a scope against a named timer in the
 *        recorder.
 */

namespace hydra {

namespace timing {

/// @brief Summary statistics for one named timer, in seconds.
struct ElapsedStatistics {
  double last_s;            ///< Most recent measurement.
  double mean_s;            ///< Mean across all measurements.
  double min_s;             ///< Minimum measurement.
  double max_s;             ///< Maximum measurement.
  double stddev_s;          ///< Standard deviation of measurements.
  size_t num_measurements;  ///< Number of recorded measurements.
};

/// @brief Stream timer statistics in a human-readable form.
std::ostream& operator<<(std::ostream& out, const ElapsedStatistics& stats);

/// @brief Singleton that accumulates elapsed-time measurements for named timers.
/// Measurements can be reported as statistics or written to per-timer and aggregate log
/// files, optionally appended incrementally as they are recorded.
class ElapsedTimeRecorder {
 public:
  /// @brief Access the global recorder instance, creating it on first use.
  static ElapsedTimeRecorder& instance() {
    if (!instance_) {
      instance_.reset(new ElapsedTimeRecorder());
    }
    return *instance_;
  }

  /// @brief Mark the start of a measurement for a timer.
  /// @param timer_name Timer to start.
  /// @param timestamp Data timestamp associated with this measurement.
  void start(const std::string& timer_name, const uint64_t& timestamp);

  /// @brief Mark the end of the running measurement for a timer and record the elapsed time.
  void stop(const std::string& timer_name);

  /// @brief Record a measurement directly without a start/stop pair.
  /// @param timer_name Timer to record under.
  /// @param timestamp Data timestamp for the measurement.
  /// @param elapsed Elapsed duration to record.
  void record(const std::string& timer_name,
              const uint64_t timestamp,
              const std::chrono::nanoseconds elapsed);

  /// @brief Discard all recorded measurements and timer state.
  void reset();

  /// @brief Most recent elapsed time for a timer in seconds, if any.
  std::optional<double> getLastElapsed(const std::string& timer_name) const;

  /// @brief Compute summary statistics for a timer.
  ElapsedStatistics getStats(const std::string& timer_name) const;

  /// @brief Write the raw measurements of one timer to its log file.
  void logElapsed(const std::string& name, const LogSetup& log_config) const;

  /// @brief Write the raw measurements of every timer to their log files.
  void logAllElapsed(const LogSetup& log_config) const;

  /// @brief Build a printable table of statistics across all timers.
  std::string getPrintableStats() const;

  /// @brief Write aggregate statistics for all timers to the given file.
  void logStats(const std::string& stat_filepath) const;

  /// @brief Enable appending measurements to log files as they are recorded.
  void setupIncrementalLogging(const LogSetup::Ptr& log_config);

  bool timing_disabled;          ///< When true, start/stop record nothing.
  bool disable_output;           ///< When true, suppress per-measurement console output.
  bool log_to_same_folder = false;  ///< When true, write all raw timer files into one folder.

 private:
  using TimeList = std::list<std::chrono::nanoseconds>;
  using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
  using TimeMap = std::map<std::string, TimePoint>;
  using TimeStamps = std::list<uint64_t>;
  using TimeStamp = std::map<std::string, uint64_t>;

  ElapsedTimeRecorder();

  static std::unique_ptr<ElapsedTimeRecorder> instance_;  ///< The singleton instance.

  TimeMap starts_;            ///< Start time of each currently running timer.
  TimeStamp start_stamps_;    ///< Data timestamp captured at each timer's start.
  std::map<std::string, TimeList> elapsed_;   ///< Recorded durations per timer.
  std::map<std::string, TimeStamps> stamps_;  ///< Data timestamps per recorded measurement.
  mutable std::mutex mutex_;   ///< Guards concurrent access to recorder state.

  bool log_incrementally_;     ///< Whether measurements are appended to files as recorded.
  LogSetup::Ptr log_setup_;    ///< Log setup used for incremental logging.
  std::map<std::string, std::shared_ptr<std::ofstream>> files_;  ///< Open per-timer output streams.
};

/// @brief Times the lifetime of a scope against a named timer in the recorder.
/// Construction starts the measurement and destruction stops it. Construction can be
/// configured to log on stop and to suppress timing below a verbosity level.
class ScopedTimer {
 public:
  /// @brief Start timing a scope under the given timer name.
  /// @param name Timer name to record under.
  /// @param timestamp Data timestamp for the measurement.
  explicit ScopedTimer(const std::string& name, uint64_t timestamp);

  /// @brief Start timing a scope with logging and verbosity control.
  /// @param name Timer name to record under.
  /// @param timestamp Data timestamp for the measurement.
  /// @param verbose If true, log the elapsed time when the timer stops.
  /// @param verbosity Verbosity level associated with this timer's logging.
  /// @param elapsed_only If true, log only the elapsed time rather than full statistics.
  /// @param verbosity_disables If true, skip timing entirely unless the verbosity level is active.
  ScopedTimer(const std::string& name,
              uint64_t timestamp,
              bool verbose,
              int verbosity,
              bool elapsed_only = true,
              bool verbosity_disables = false);

  /// @brief Stop the timer if still running.
  ~ScopedTimer();

  /// @brief Begin the measurement.
  void start();
  /// @brief End the measurement, recording and optionally logging the elapsed time.
  void stop();
  /// @brief Stop the current timer and restart under a new name with the same timestamp.
  void reset(const std::string& name);
  /// @brief Stop the current timer and restart under a new name and timestamp.
  void reset(const std::string& name, uint64_t timestamp);

 private:
  std::string name_;         ///< Timer name currently being measured.
  uint64_t timestamp_;       ///< Data timestamp for the measurement.
  bool verbose_;             ///< Whether to log on stop.
  int verbosity_;            ///< Verbosity level for logging.
  bool elapsed_only_;        ///< Log only elapsed time rather than statistics.
  bool verbosity_disables_;  ///< Skip timing unless the verbosity level is active.
  bool is_running_ = false;  ///< Whether a measurement is in progress.
};

}  // namespace timing
}  // namespace hydra
