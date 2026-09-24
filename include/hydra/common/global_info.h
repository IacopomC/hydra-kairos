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
#include <config_utilities/virtual_config.h>

#include <array>
#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include "hydra/common/label_remapper.h"
#include "hydra/common/label_space_config.h"
#include "hydra/common/robot_prefix_config.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/input/sensor.h"
#include "hydra/utils/log_utilities.h"

#include "hydra/active_window/volumetric_window.h"

/**
 * @file global_info.h
 * @brief Pipeline-wide configuration and the process-global singleton that holds
 *        it. Defines the coordinate-frame and pipeline config structs, and a
 *        GlobalInfo accessor that resolves the robot prefix, logging, label
 *        space, color map, sensors, and the active spatial window for all
 *        modules.
 */

namespace hydra {

class SemanticColorMap;

/// @brief Names of the coordinate frames used by the pipeline.
struct FrameConfig {
  std::string robot = "base_link";  ///< Frame attached to the robot body.
  std::string odom = "odom";        ///< Odometry reference frame.
  std::string map = "map";          ///< Global map frame.
};

/// @brief Top-level configuration shared across the whole pipeline.
/// Holds feature toggles, defaults inherited by modules, layer-to-prefix
/// mappings, logging, frames, the active map window, and the label space.
struct PipelineConfig {
  bool enable_reconstruction = true;  ///< Enable volumetric reconstruction.
  bool enable_lcd = false;            ///< Enable loop-closure detection.
  bool enable_places = true;          ///< Enable the places layer.
  bool timing_disabled = false;       ///< Disable timing instrumentation.
  bool disable_timer_output = true;   ///< Suppress periodic timer printouts.
  bool enable_pgmo_logging = true;    ///< Enable pose-graph optimization logging.

  int default_verbosity = 1;   ///< Default module verbosity; modules may override.
  int default_num_threads = -1;  ///< Default thread count; -1 uses all available threads.

  bool store_visualization_details = false;  ///< Store extra detail for spatio-temporal visualization.
  /// @brief Maps each scene-graph layer to the symbol prefix used for its nodes.
  std::map<spark_dsg::LayerId, char> layer_id_map{
      {spark_dsg::DsgLayers::OBJECTS, 'o'},
      {spark_dsg::DsgLayers::PLACES, 'p'},
      {spark_dsg::DsgLayers::MESH_PLACES, 'q'},
      {spark_dsg::DsgLayers::ARCHETYPES, 'A'},
      {spark_dsg::DsgLayers::ROOMS, 'r'},
      {spark_dsg::DsgLayers::BUILDINGS, 'b'},
  };
  LogConfig logs;          ///< Logging configuration.
  FrameConfig frames;      ///< Coordinate-frame names.
  config::VirtualConfig<VolumetricWindow> map_window{SpatialWindowChecker::Config()};  ///< Active spatial window selection.
  LabelSpaceConfig label_space;  ///< Semantic label space configuration.
  std::map<uint32_t, std::string> label_names;  ///< Optional human-readable label names by id.
};

/// @brief Register the frame config fields with the config system.
void declare_config(FrameConfig& conf);
/// @brief Register the pipeline config fields with the config system.
void declare_config(PipelineConfig& conf);

/// @brief Process-global access point for pipeline configuration and derived state.
/// Implemented as a singleton; once frozen the configuration is immutable and
/// modules read shared resources (prefix, logging, labels, sensors) from it.
class GlobalInfo {
 public:
  /// @brief Access the current global instance.
  static GlobalInfo& instance();

  /// @brief Initialize the global instance from a config.
  /// @param config Pipeline configuration to install.
  /// @param robot_id Identifier used to derive the robot prefix.
  /// @param freeze If true, freeze the configuration so it can no longer change.
  /// @return Reference to the initialized instance.
  static GlobalInfo& init(const PipelineConfig& config,
                          int robot_id = 0,
                          bool freeze = true);

  /// @brief Tear down the global instance.
  static void exit();

  /// @brief Invalidate any existing instance; primarily intended for testing.
  static void reset();

  /// @brief Whether the configuration has been frozen and is now immutable.
  inline bool frozen() const { return frozen_; }

  /// @brief Request or clear a forced pipeline shutdown.
  /// @param force_shutdown True to signal shutdown, false to clear it.
  void setForceShutdown(bool force_shutdown);

  /// @brief Whether a forced shutdown has been requested.
  bool force_shutdown() const;

  /// @brief Access the installed pipeline configuration.
  const PipelineConfig& getConfig() const;

  /// @brief Access the coordinate-frame names.
  const FrameConfig& getFrames() const;

  /// @brief Access the robot prefix derived from the robot id.
  const RobotPrefixConfig& getRobotPrefix() const;

  /// @brief Access the shared logging setup.
  const LogSetup::Ptr& getLogs() const;

  /// @brief Access the map of label id to human-readable name.
  const std::map<uint32_t, std::string>& getLabelToNameMap() const;

  /// @brief Access the semantic label space configuration.
  const LabelSpaceConfig& getLabelSpaceConfig() const;

  /// @brief Total number of labels in the configured label space.
  size_t getTotalLabels() const;

  /// @brief Access the label remapper built from the label space config.
  const LabelRemapper& getLabelRemapper() const;

  /// @brief Access the semantic color map, or nullptr if none is configured.
  const SemanticColorMap* getSemanticColorMap() const;

  /// @brief Create a shared scene-graph info object using the configured layer map.
  SharedDsgInfo::Ptr createSharedDsg() const;

  /// @brief Register a sensor under its name.
  /// @param sensor Sensor to register.
  /// @param allow_override If false, registering a duplicate name fails.
  /// @return True if the sensor was stored.
  bool setSensor(const Sensor::Ptr& sensor, bool allow_override = true);

  /// @brief Look up a previously registered sensor by name.
  /// @param name Name the sensor was registered under.
  /// @return The sensor, or nullptr if no sensor matches the name.
  Sensor::ConstPtr getSensor(const std::string& name) const;

  /// @brief List the names of all registered sensors.
  std::vector<std::string> getAvailableSensors() const;

  /// @brief Construct the active spatial window from the configured selection.
  std::unique_ptr<VolumetricWindow> createVolumetricWindow() const;

 private:
  GlobalInfo();

  void configureTimers();

  void initFromConfig(const PipelineConfig& config, int robot_id);

  void checkFrozen() const;

 private:
  static std::unique_ptr<GlobalInfo> instance_;  ///< The singleton instance.
  bool frozen_ = false;                          ///< Whether the config is locked.
  PipelineConfig config_;                        ///< Installed pipeline configuration.
  std::atomic<bool> force_shutdown_;             ///< Forced-shutdown request flag.

  RobotPrefixConfig robot_prefix_;               ///< Prefix derived from the robot id.
  LogSetup::Ptr logs_;                           ///< Shared logging setup.
  LabelRemapper label_remapper_;                 ///< Label remapper from the label space.
  std::shared_ptr<SemanticColorMap> label_colormap_;  ///< Optional semantic color map.

  std::map<std::string, std::shared_ptr<const Sensor>> sensors_;  ///< Registered sensors by name.
};

/// @brief Stream a human-readable summary of the global configuration.
std::ostream& operator<<(std::ostream& out, const GlobalInfo& config);

}  // namespace hydra
