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
#include <kimera_pgmo/hashing.h>
#include <pose_graph_tools/bow_query.h>
#include <spark_dsg/scene_graph_logger.h>

#include <memory>
#include <mutex>
#include <thread>

#include "hydra/active_window/active_window_output.h"
#include "hydra/backend/backend_input.h"
#include "hydra/common/message_queue.h"
#include "hydra/common/module.h"
#include "hydra/common/output_sink.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/common/shared_module_state.h"
#include "hydra/frontend/freespace_places_interface.h"
#include "hydra/frontend/graph_connector.h"
#include "hydra/frontend/mesh_segmenter.h"
#include "hydra/frontend/surface_places_interface.h"
#include "hydra/frontend/view_database.h"
#include "hydra/loop_closure/lcd_input.h"
#include "hydra/odometry/pose_graph_from_odom.h"
#include "hydra/utils/log_utilities.h"

namespace kimera_pgmo {
class DeltaCompression;
class MeshCompression;
}  // namespace kimera_pgmo

/**
 * @file graph_builder.h
 * @brief Frontend module that turns active-window output into the scene graph.
 *        It compresses the mesh, segments objects, extracts surface and
 *        free-space places and frontiers, builds the deformation and pose
 *        graphs, and forwards results to the backend and loop-closure stages.
 */

namespace hydra {

class FrontierExtractor;
class NearestNodeFinder;
struct VolumetricWindow;

/// @brief Runs the frontend stage of the pipeline. On each active-window packet
/// it updates the mesh, objects, places (surface and free-space), frontiers,
/// deformation graph, and pose graph in the shared scene graph, then merges the
/// result into the backend and loop-closure graphs and notifies its sinks.
class GraphBuilder : public Module {
 public:
  using Ptr = std::shared_ptr<GraphBuilder>;
  using InputQueue = MessageQueue<ActiveWindowOutput::Ptr>;  ///< Queue of incoming active-window packets.
  using InputCallback = std::function<void(const ActiveWindowOutput&)>;  ///< Per-packet callback type.
  using Sink = OutputSink<uint64_t, const DynamicSceneGraph&, const BackendInput&>;  ///< Per-step output observer.

  /// @brief Settings selecting which frontend stages run and how.
  struct Config {
    bool lcd_use_bow_vectors = true;  ///< Forward bag-of-words vectors to loop closure.

    /// @brief Resolution and horizon settings for the deformation graph.
    struct DeformationConfig {
      double mesh_resolution = 0.1;     ///< Mesh vertex spacing for compression (m).
      double d_graph_resolution = 1.5;  ///< Deformation-graph node spacing (m).
      double time_horizon = 10.0;       ///< Deformation-graph time horizon (s).
    } pgmo;

    GraphUpdater::Config graph_updater{{{"objects", {'O', std::nullopt}}}};  ///< Mapping for applying graph updates.
    GraphConnector::Config graph_connector;  ///< Interlayer edge connector settings.
    bool enable_mesh_objects = true;         ///< Segment objects from the mesh.
    MeshSegmenter::Config object_config;     ///< Object segmenter settings.
    config::VirtualConfig<PoseGraphTracker> pose_graph_tracker{
        PoseGraphFromOdom::Config()};        ///< Source of pose-graph updates.
    bool enable_place_mesh_mapping = false;  ///< Link place nodes to mesh vertices.
    config::VirtualConfig<SurfacePlacesInterface> surface_places;    ///< Surface place extractor (optional).
    config::VirtualConfig<FreespacePlacesInterface> freespace_places; ///< Free-space place extractor (optional).
    bool use_frontiers = false;              ///< Extract exploration frontiers.
    config::VirtualConfig<FrontierExtractor> frontier_places;  ///< Frontier extractor (optional).
    ViewDatabase::Config view_database;      ///< View database settings for feature assignment.
    std::vector<Sink::Factory> sinks;        ///< Factories for per-step output observers.
    bool no_packet_collation = false;        ///< Process each input packet separately instead of merging queued packets.
    bool overwrite_mesh_timestamps = false;  ///< Overwrite mesh vertex timestamps from the tracking layer.
  } const config;

  /// @brief Construct the frontend module.
  /// @param config Frontend settings.
  /// @param dsg Shared scene-graph state to populate.
  /// @param state Shared state holding the backend and loop-closure graphs.
  /// @param logs Optional logging setup.
  GraphBuilder(const Config& config,
               const SharedDsgInfo::Ptr& dsg,
               const SharedModuleState::Ptr& state,
               const LogSetup::Ptr& logs = nullptr);

  virtual ~GraphBuilder();

  /// @brief Launch the spin thread.
  void start() override;

  /// @brief Signal shutdown and join the spin thread.
  void stop() override;

  /// @brief Persist frontend outputs and sub-module state.
  void save(const LogSetup& log_setup) override;

  /// @brief Return a human-readable summary of the module's status.
  std::string printInfo() const override;

  /// @brief Run the input-processing loop until shutdown.
  virtual void spin();

  /// @brief Process a single queued input packet, if any.
  /// @return True if a packet was processed.
  bool spinOnce();

  /// @brief Access the input queue.
  inline InputQueue::Ptr queue() const { return queue_; }

  /// @brief Register a per-step output observer.
  void addSink(const Sink::Ptr& sink);

 protected:
  /// @brief Register a callback invoked early in each update.
  void addInputCallback(InputCallback callback);

  /// @brief Register a callback invoked after the mesh is updated.
  void addPostMeshCallback(InputCallback callback);

  /// @brief Run spinOnce for a packet on a detached thread.
  void dispatchSpin(ActiveWindowOutput::Ptr msg);

  /// @brief Process one packet: update the graph, then merge to backend and LCD.
  void spinOnce(const ActiveWindowOutput::Ptr& msg);

  /// @brief Apply graph updates and run input callbacks and the connector.
  virtual void updateImpl(const ActiveWindowOutput::Ptr& msg);

 protected:
  /// @brief Compress and archive the mesh from the active-window output.
  void updateMesh(const ActiveWindowOutput& msg);

  /// @brief Segment objects from the mesh and update object nodes.
  void updateObjects(const ActiveWindowOutput& msg);

  /// @brief Detect exploration frontiers and update frontier nodes.
  void updateFrontiers(const ActiveWindowOutput& msg);

  /// @brief Extract free-space places and update place nodes.
  void updatePlaces(const ActiveWindowOutput& msg);

  /// @brief Extract 2D surface places and update place nodes.
  void updatePlaces2d(const ActiveWindowOutput& msg);

  /// @brief Update the deformation graph from the latest mesh.
  void updateDeformationGraph(const ActiveWindowOutput& msg);

  /// @brief Apply pose-graph updates from the tracker.
  void updatePoseGraph(const ActiveWindowOutput& msg);

 protected:
  /// @brief Attach cached bag-of-words vectors to agent nodes.
  void assignBowVectors(const DynamicSceneGraphLayer& agents);

  /// @brief Archive 2D places that are no longer active.
  void archivePlaces2d(const NodeIdSet active_places);

  /// @brief Run all per-packet processing stages in order.
  void processNextInput(const ActiveWindowOutput& msg);

  /// @brief Connect place nodes to nearby mesh vertices.
  void updatePlaceMeshMapping(const ActiveWindowOutput& input);

 protected:
  uint64_t sequence_number_;                       ///< Monotonic counter of processed packets.
  mutable std::mutex gvd_mutex_;                   ///< Guards free-space place extraction.
  std::atomic<bool> should_shutdown_{false};       ///< Set to request the spin thread to stop.
  std::unique_ptr<std::thread> spin_thread_;       ///< Background input-processing thread.
  InputQueue::Ptr queue_;                          ///< Queue of incoming active-window packets.
  std::atomic<bool> spin_finished_;                ///< Tracks completion of an in-flight spin.

  LcdInput::Ptr lcd_input_;                         ///< Loop-closure input assembled this step.
  BackendInput::Ptr backend_input_;                 ///< Backend input assembled this step.

  SharedDsgInfo::Ptr dsg_;                          ///< Shared scene graph populated by the frontend.
  SharedModuleState::Ptr state_;                    ///< Shared state with backend and LCD graphs.
  kimera_pgmo::MeshDelta::Ptr last_mesh_update_;    ///< Most recent mesh delta forwarded to the backend.
  StampUpdate::Ptr last_mesh_stamp_update_;         ///< Most recent mesh timestamp update.

  kimera_pgmo::Graph deformation_graph_;            ///< Deformation graph maintained over the mesh.
  std::unique_ptr<kimera_pgmo::DeltaCompression> mesh_compression_;        ///< Incremental mesh compressor.
  std::unique_ptr<kimera_pgmo::MeshCompression> deformation_compression_;  ///< Deformation-graph mesh compressor.
  kimera_pgmo::HashedIndexMapping deformation_remapping_;                  ///< Index remapping for the deformation graph.
  std::shared_ptr<kimera_pgmo::HashedIndexMapping> mesh_remapping_;        ///< Index remapping for the compressed mesh.

  GraphUpdater graph_updater_;                      ///< Applies incoming graph updates.
  GraphConnector graph_connector_;                  ///< Maintains interlayer parent/child edges.

  std::unique_ptr<VolumetricWindow> map_window_;    ///< Active-window bounds.
  std::unique_ptr<MeshSegmenter> segmenter_;        ///< Object segmenter.
  std::unique_ptr<PoseGraphTracker> tracker_;       ///< Source of pose-graph updates.
  std::unique_ptr<SurfacePlacesInterface> surface_places_;     ///< Surface place extractor (optional).
  std::unique_ptr<FreespacePlacesInterface> freespace_places_; ///< Free-space place extractor (optional).
  std::unique_ptr<FrontierExtractor> frontier_places_;         ///< Frontier extractor (optional).
  ViewDatabase view_database_;                      ///< Buffers views and assigns place features.

  SceneGraphLogger frontend_graph_logger_;          ///< Logs the graph each step when enabled.
  LogSetup::Ptr logs_;                              ///< Logging setup, if any.
  MessageQueue<PoseGraphPacket> pose_graph_updates_;  ///< Pending pose-graph packets.

  NodeIdSet previous_active_places_;                ///< Active free-space places from the prior step.
  NodeIdSet previous_active_places_2d_;             ///< Active surface places from the prior step.
  std::map<NodeId, size_t> agent_key_map_;          ///< Maps agent nodes to their bag-of-words keys.
  std::list<pose_graph_tools::BowQuery::ConstPtr> cached_bow_messages_;  ///< Bag-of-words messages awaiting assignment.

  Sink::List sinks_;                                ///< Per-step output observers.

  std::mutex mutex_;                                ///< Guards all frontend data during a spin.

 private:
  /// @brief Internal shutdown helper.
  void stopImpl();

  std::vector<std::function<void(ActiveWindowOutput::Ptr)>> input_callbacks_;       ///< Registered input callbacks.
  std::vector<std::function<void(const ActiveWindowOutput&)>> post_mesh_callbacks_; ///< Registered post-mesh callbacks.
};

/// @brief Declare the config fields for the frontend module.
void declare_config(GraphBuilder::Config& config);

}  // namespace hydra
