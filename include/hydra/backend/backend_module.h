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
#include <kimera_pgmo/kimera_pgmo_interface.h>
#include <spark_dsg/scene_graph_logger.h>

#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "hydra/backend/backend_input.h"
#include "hydra/backend/merge_tracker.h"
#include "hydra/backend/pgmo_configs.h"
#include "hydra/common/module.h"
#include "hydra/common/output_sink.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/common/shared_module_state.h"
#include "hydra/utils/log_utilities.h"

/**
 * @file backend_module.h
 * @brief Backend optimization module. Consumes front-end inputs, maintains the
 *        pose-graph-and-mesh factor graph, runs optimization on loop closures,
 *        and applies an ordered chain of per-layer update functors to keep the
 *        global scene graph consistent with the optimized trajectory and mesh.
 */

namespace hydra {

/// @brief Record of a single loop-closure factor added to the graph.
struct LoopClosureLog {
  gtsam::Symbol src;        ///< Source key of the factor edge.
  gtsam::Symbol dest;       ///< Destination key of the factor edge.
  gtsam::Pose3 src_T_dest;  ///< Relative pose from the source frame to the destination frame.
  bool dsg;                 ///< Whether the closure came from the scene graph rather than the trajectory.
  int64_t level;            ///< Hierarchy level the closure was detected at.
};

/// @brief Running counters describing the backend optimization state.
struct BackendModuleStatus {
  size_t total_loop_closures;  ///< Loop closures accumulated so far.
  size_t new_loop_closures;    ///< Loop closures added in the latest step.
  size_t total_factors;        ///< Total factors in the graph.
  size_t total_values;         ///< Total optimized values in the graph.
  size_t new_factors;          ///< Factors added in the latest step.
  size_t new_graph_factors;    ///< Scene-graph factors added in the latest step.
  size_t trajectory_len;       ///< Number of poses in the trajectory.
  size_t num_merges_undone;    ///< Merges reverted in the latest step.

  /// @brief Resets all counters to zero.
  void reset();
};

/// @brief Optimization module that maintains the global scene graph.
class BackendModule : public kimera_pgmo::KimeraPgmoInterface, public Module {
 public:
  using Ptr = std::shared_ptr<BackendModule>;
  using Sink = OutputSink<uint64_t,
                          const DynamicSceneGraph&,
                          const kimera_pgmo::DeformationGraph&>;
  using NodeToRobotMap = std::unordered_map<NodeId, size_t>;

  struct Config {
    HydraPgmoConfig pgmo;  ///< Optimizer config including the scene-graph factor covariances.
    bool add_places_to_deformation_graph = true;  ///< Attach the places layer to the deformation graph via a spanning tree.
    bool optimize_on_lc = true;             ///< Run optimization on every detected loop closure.
    bool enable_node_merging = true;        ///< Allow merging multiple nodes into one.
    bool enable_exhaustive_merging = false;  ///< Repeat merge detection until no further merges are found.
    double max_external_loop_closure_time_difference = 1.0;  ///< Max time gap (s) between an external loop closure and the nearest agent pose; 0 disables the check.
    /// @brief Update functors applied in the configured order after optimization.
    config::OrderedMap<std::string, config::VirtualConfig<UpdateFunctor, true>>
        update_functors;
    std::vector<Sink::Factory> sinks;  ///< Sinks that receive the latest optimized scene graph and state.
  } const config;

  /// @brief Constructs the module from its configuration and shared state.
  /// @param config Module configuration.
  /// @param dsg Shared scene-graph the module reads from and writes to.
  /// @param state Shared state exchanged with the rest of the pipeline.
  /// @param logs Optional logging setup.
  BackendModule(const Config& config,
                const SharedDsgInfo::Ptr& dsg,
                const SharedModuleState::Ptr& state,
                const LogSetup::Ptr& logs = nullptr);

  virtual ~BackendModule();

  BackendModule(const BackendModule& other) = delete;

  BackendModule& operator=(const BackendModule& other) = delete;

  /// @brief Starts the background spin thread.
  void start() override;

  /// @brief Stops the module and joins the spin thread.
  void stop() override;

  /// @brief Writes the module's logs and saved state to disk.
  void save(const LogSetup& log_setup) override;

  /// @brief Returns a human-readable description of the module state.
  std::string printInfo() const override;

  /// @brief Runs the processing loop until shutdown is requested.
  void spin();

  /// @brief Processes one input from the queue.
  /// @param force_optimize Force optimization even without a new loop closure.
  /// @return True if an input was processed.
  bool step(bool force_optimize = false);

  /// @brief Requests that the backend scene graph be rebuilt on the next step.
  void triggerBackendDsgReset() { reset_backend_dsg_ = true; }

  /// @brief Loads a previously saved mesh and deformation graph.
  /// @param mesh_path Path to the saved mesh.
  /// @param dgrf_path Path to the saved deformation graph.
  /// @param force_loopclosures Whether to treat the loaded factors as loop closures.
  void loadState(const std::filesystem::path& mesh_path,
                 const std::filesystem::path& dgrf_path,
                 bool force_loopclosures = true);

  /// @brief Registers a sink that receives each optimized scene graph.
  void addSink(const Sink::Ptr& sink);

  /// @brief Registers a sink that fires inside callUpdateFunctions before the
  /// update functors run. Use this to write external state into the graph that
  /// the functors should consume in the same iteration.
  void addPreUpdateSink(const Sink::Ptr& sink);

 protected:
  /// @brief Processes one input and optionally optimizes; returns true on progress.
  virtual bool spinOnce(bool force_update = true);

  /// @brief Pushes the configured robust-solver parameters into the optimizer.
  void setSolverParams();

  /// @brief Adds a loop-closure factor between two keys.
  /// @param src Source key.
  /// @param dest Destination key.
  /// @param src_T_dest Relative pose from source to destination.
  /// @param variance Measurement variance for the factor.
  void addLoopClosure(const gtsam::Key& src,
                      const gtsam::Key& dest,
                      const gtsam::Pose3& src_T_dest,
                      double variance);

  /// @brief Folds the input's deformation graph and mesh delta into the factor graph.
  virtual void updateFactorGraph(const BackendInput& input);

  /// @brief Drains queued loop-closure detections into the factor graph.
  /// @return True if any loop closure was added.
  virtual bool updateFromLcdQueue();

  /// @brief Applies externally supplied loop closures to the factor graph.
  virtual void updateExternalLoopClosures();

  /// @brief Copies the input mesh delta into the working mesh.
  virtual void copyMeshDelta(const BackendInput& input);

  /// @brief Refreshes the private scene graph from the latest optimized state.
  /// @return True if the graph was updated.
  virtual bool updatePrivateDsg(size_t timestamp_ns, bool force_update = true);

  /// @brief Finds the agent node closest in time to a timestamp.
  /// @param timestamp_ns Query timestamp (ns).
  /// @param robot_id Optional robot to restrict the search to.
  /// @param max_diff_s Max allowed time difference (s); 0 disables the bound.
  /// @return Closest agent node id, or empty if none qualifies.
  virtual std::optional<NodeId> findClosestAgentId(
      uint64_t timestamp_ns,
      std::optional<int> robot_id = std::nullopt,
      double max_diff_s = 0.0) const;

  /// @brief Adds agent pose measurements to the factor graph.
  virtual void updateAgentNodeMeasurements(const pose_graph_tools::PoseGraph& meas);

  /// @brief Runs the pose-graph-and-mesh optimization.
  virtual void optimize(size_t timestamp_ns);

  /// @brief Deforms the scene-graph mesh using the optimized deformation graph.
  virtual void updateDsgMesh(size_t timestamp_ns, bool force_mesh_update = false);

  /// @brief Rebuilds the backend scene graph from scratch.
  virtual void resetBackendDsg(size_t timestamp_ns);

  /// @brief Runs the ordered chain of update functors over the scene graph.
  /// @param timestamp_ns Current timestamp (ns).
  /// @param places_values Optimized place positions, if any.
  /// @param pgmo_values Optimized mesh and agent poses, if any.
  /// @param new_loop_closure Whether a loop closure triggered this update.
  /// @param given_merges Externally supplied merges to apply.
  /// @param node_to_robot Optional mapping from node to owning robot.
  virtual void callUpdateFunctions(size_t timestamp_ns,
                                   const gtsam::Values& places_values = gtsam::Values(),
                                   const gtsam::Values& pgmo_values = gtsam::Values(),
                                   bool new_loop_closure = false,
                                   const UpdateInfo::LayerMerges& given_merges = {},
                                   const NodeToRobotMap* node_to_robot = nullptr);

  /// @brief Records newly applied merges so later references can be remapped.
  void updateMergedNodes(const std::map<NodeId, NodeId>& new_merges);

  /// @brief Logs the current status counters.
  void logStatus(bool init = false) const;

  /// @brief Logs the loop closures contained in an incremental pose graph.
  void logIncrementalLoopClosures(const pose_graph_tools::PoseGraph& graph);

 protected:
  /// @brief Signals the spin thread to stop and joins it.
  void stopImpl();

  std::unique_ptr<std::thread> spin_thread_;     ///< Background processing thread.
  std::atomic<bool> should_shutdown_{false};     ///< Signals the spin thread to stop.
  bool force_optimize_ = false;                  ///< Forces optimization on the next step.
  bool have_loopclosures_ = false;               ///< Whether any loop closure has been seen.
  bool have_new_loopclosures_ = false;           ///< Whether a loop closure arrived since the last optimization.
  bool have_new_mesh_ = false;                   ///< Whether a new mesh delta is pending.
  size_t prev_num_archived_vertices_ = 0;        ///< Archived vertex count before the latest mesh update.
  size_t num_archived_vertices_ = 0;             ///< Current count of vertices archived out of the active window.
  bool reset_backend_dsg_ = false;               ///< Pending request to rebuild the backend scene graph.
  uint64_t last_sequence_number_ = 0;            ///< Sequence number of the last processed input.

  SharedDsgInfo::Ptr private_dsg_;               ///< Backend-owned scene graph with merges applied.
  DynamicSceneGraph::Ptr unmerged_graph_;        ///< Scene graph before merges, used to recompute attributes.
  SharedModuleState::Ptr state_;                 ///< Shared state exchanged with the rest of the pipeline.

  kimera_pgmo::Path trajectory_;                 ///< Optimized agent trajectory.
  std::vector<size_t> timestamps_;               ///< Timestamps of the trajectory poses.
  std::queue<size_t> unconnected_nodes_;         ///< Trajectory nodes awaiting connection in the factor graph.
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_vertices_;  ///< Pre-deformation mesh vertices.
  std::vector<uint64_t> vertex_stamps_;          ///< Timestamp per mesh vertex.

  MergeTracker merge_tracker;                    ///< Applies and tracks the history of node merges.
  std::vector<UpdateFunctor::Ptr> update_functors_;  ///< Per-layer update functors run in order.

  LogSetup::Ptr logs_;                           ///< Logging setup, if enabled.
  BackendModuleStatus status_;                   ///< Running optimization counters.
  SceneGraphLogger backend_graph_logger_;        ///< Logs scene-graph statistics over time.
  std::list<LoopClosureLog> loop_closures_;      ///< History of loop closures added to the graph.

  Sink::List pre_update_sinks_;                  ///< Sinks fired before the update functors run.
  Sink::List sinks_;                             ///< Sinks fired with each optimized scene graph.

  std::mutex mutex_;                             ///< Guards module data during manipulation.
};

/// @brief Declares the configuration fields for BackendModule.
void declare_config(BackendModule::Config& conf);

}  // namespace hydra
