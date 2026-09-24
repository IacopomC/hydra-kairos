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

#include <functional>
#include <list>
#include <memory>
#include <set>
#include <string>

/**
 * @file log_utilities.h
 * @brief Configuration and setup for the output log directory. LogSetup creates the
 *        directory tree, resolves per-namespace and per-timer file paths, and runs
 *        registered callbacks at destruction so subsystems can flush their logs.
 */

namespace hydra {

/// @brief Settings controlling where and how logs and timing data are written.
struct LogConfig {
  std::string log_dir = "";  ///< Root output directory; empty disables logging.
  bool log_timing_incrementally = false;  ///< Append timing data as it is recorded rather than only at the end.
  std::string timing_stats_name = "timing_stats.csv";  ///< Filename for aggregate timing statistics.
  std::string timing_suffix = "_timing_raw.csv";  ///< Suffix for per-timer raw measurement files.

  /// @brief If true, write all raw timer files into one directory with '/' in timer
  /// names replaced by '_'. If false, create nested directories for '/' separators.
  bool log_raw_timers_to_single_dir = false;

  /// @brief Build a config that only sets the output directory.
  /// @param output_path Root log directory.
  static LogConfig fromString(const std::string& output_path) {
    LogConfig config;
    config.log_dir = output_path;
    return config;
  }
};

/// @brief Register LogConfig fields with the config system.
void declare_config(LogConfig& config);

/// @brief Owns the log output directory and resolves log file paths.
/// On destruction it invokes any registered exit callbacks so subsystems can write out
/// their final logs.
class LogSetup {
 public:
  using Ptr = std::shared_ptr<LogSetup>;  ///< Shared pointer alias.

  /// @brief Construct from a full config.
  explicit LogSetup(const LogConfig& config);

  /// @brief Construct from an output path, leaving other settings at their defaults.
  explicit LogSetup(const std::string& output_path);

  /// @brief Run registered exit callbacks if logging is valid.
  ~LogSetup();

  /// @brief Perform any deferred initialization of the log setup.
  void init();

  /// @brief Get the root log directory, or an empty string if logging is disabled.
  std::string getLogDir() const;

  /// @brief Get (and create) a subdirectory of the log directory for a namespace.
  /// @param log_namespace Subdirectory name.
  std::string getLogDir(const std::string& log_namespace) const;

  /// @brief Get the path of the aggregate timing statistics file.
  std::string getTimerFilepath() const;

  /// @brief Get the raw measurement file path for a named timer.
  /// @param timer_name Timer whose file path to build.
  std::string getTimerFilepath(const std::string& timer_name) const;

  /// @brief Report whether the log directory was set up successfully.
  bool valid() const;

  /// @brief Access the underlying configuration.
  const LogConfig& config() const;

  /// @brief Register a callback to run when this object is destroyed.
  /// @param func Callback invoked with this setup at destruction.
  void registerExitCallback(const std::function<void(const LogSetup&)>& func);

 private:
  bool valid_;                              ///< True once the log directory exists.
  LogConfig config_;                        ///< Active configuration.
  mutable std::set<std::string> namespaces_;  ///< Namespaces for which subdirectories were created.

  std::list<std::function<void(const LogSetup&)>> callbacks_;  ///< Callbacks run at destruction.
};

}  // namespace hydra
