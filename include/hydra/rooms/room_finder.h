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
#include <fstream>
#include <unordered_set>

#include "hydra/common/dsg_types.h"
#include "hydra/rooms/graph_clustering.h"
#include "hydra/rooms/room_finder_config.h"

/**
 * @file room_finder.h
 * @brief Detects rooms from a places layer. The finder computes a filtration of the
 *        places, selects a dilation threshold to extract candidate components, clusters
 *        them into rooms, and builds a room layer. It also adds the edges linking rooms
 *        to their member places and neighboring rooms to one another.
 */

namespace hydra {

/// @brief Bookkeeping for one serialized place graph written to the graph log.
struct GraphInfo {
  size_t offset;     ///< Byte offset of the entry within the graph log file.
  size_t size;       ///< Serialized size of the entry in bytes.
  size_t num_nodes;  ///< Node count of the logged graph.
  size_t num_edges;  ///< Edge count of the logged graph.
};

/// @brief Groups places into rooms and constructs the corresponding room layer and edges.
class RoomFinder {
 public:
  using ClusterMap = std::map<NodeId, std::vector<NodeId>>;  ///< Member place IDs per room node.

  /// @brief Construct with the given configuration.
  /// @param config Parameters controlling clustering and threshold selection.
  explicit RoomFinder(const RoomFinderConfig& config);

  virtual ~RoomFinder();

  /// @brief Detect rooms and return a freshly built room layer.
  /// @param places Places layer to cluster.
  /// @return The new room layer, or nullptr when no rooms are found.
  SceneGraphLayer::Ptr findRooms(const SceneGraphLayer& places);

  /// @brief Insert parent edges from each detected room to its member places.
  /// @param graph Scene graph holding the places and room layers.
  void addRoomPlaceEdges(DynamicSceneGraph& graph) const;

  /// @brief Insert parent edges from rooms to archetype nodes, voting by member place
  /// membership and falling back to the nearest room by centroid when no votes exist.
  /// @param graph Scene graph holding the rooms, archetypes, and places layers.
  void addRoomArchetypeEdges(DynamicSceneGraph& graph) const;

  /// @brief Open the filtration log file at the given path when logging is enabled.
  /// @param log_path Base path; a ".json" suffix is appended for the filtration log.
  void enableLogging(const std::string& log_path);

  /// @brief Collect the member places of each room from the last detection result.
  /// @param places Places layer used to enumerate candidate members.
  /// @param assignments Output map from room node ID to its member place IDs.
  void fillClusterMap(const SceneGraphLayer& places, ClusterMap& assignments) const;

 protected:
  /// @brief Build the filtration, choose a dilation threshold, and extract the candidate
  /// components large enough to seed clustering.
  /// @param places Places layer to analyze.
  /// @return Candidate seed components, empty if none qualify.
  InitialClusters getBestComponents(const SceneGraphLayer& places) const;

  /// @brief Turn the last clustering result into a room layer with positioned room nodes
  /// and inter-room edges.
  /// @param places Places layer used to position rooms and connect neighbors.
  /// @return The constructed room layer.
  SceneGraphLayer::Ptr makeRoomLayer(const SceneGraphLayer& places);

  RoomFinderConfig config_;                  ///< Detection parameters.
  ClusterResults last_results_;              ///< Clustering result from the most recent call.
  std::map<size_t, NodeId> cluster_room_map_;  ///< Maps each cluster label to its room node ID.
  mutable bool logged_once_ = false;         ///< True once a filtration entry has been written.
  std::unique_ptr<std::ofstream> log_file_;  ///< Open filtration log stream, if logging.
  std::unique_ptr<std::ofstream> graph_log_file_;  ///< Open place-graph log stream, if logging.
  mutable std::vector<GraphInfo> graph_entries_;   ///< Index of serialized place graphs.
  mutable size_t graph_offset_ = 0;          ///< Running byte offset into the graph log.
};

}  // namespace hydra
