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
#include <pose_graph_tools/pose_graph.h>

#include <Eigen/Geometry>
#include <list>
#include <memory>

/**
 * @file pose_graph_tracker.h
 * @brief Interface for turning a stream of body poses into pose-graph
 *        increments, plus the packet of increments and priors it produces.
 */

namespace hydra {

/// @brief Pose-graph increments for one update, consumed by the backend.
struct PoseGraphPacket {
  using Ptr = std::shared_ptr<PoseGraphPacket>;
  uint64_t timestamp_ns;  ///< Timestamp of this update (ns).
  std::vector<pose_graph_tools::PoseGraph> pose_graphs;  ///< New pose-graph segments (nodes and odometry edges).
  pose_graph_tools::PoseGraph::ConstPtr external_priors;  ///< Optional priors from an external optimizer.

  /// @brief Merge another packet into this one (append segments, take its timestamp and priors).
  void updateFrom(const PoseGraphPacket& other) {
    timestamp_ns = other.timestamp_ns;
    pose_graphs.insert(pose_graphs.end(), other.pose_graphs.begin(), other.pose_graphs.end());
    // TODO(nathan) this is technically bad, but we'll get to it
    external_priors = other.external_priors;
  }
};

/// @brief Base class that maps incoming body poses to PoseGraphPackets.
struct PoseGraphTracker {
  using Ptr = std::unique_ptr<PoseGraphTracker>;
  virtual ~PoseGraphTracker() = default;

  /// @brief Ingest a body pose at a timestamp and return any resulting pose-graph increment.
  /// @param timestamp_ns Pose timestamp (ns).
  /// @param world_T_body Body pose in the world frame.
  /// @return Packet of new pose-graph segments (possibly empty if the pose was dropped).
  virtual PoseGraphPacket update(uint64_t timestamp_ns,
                                 const Eigen::Isometry3d& world_T_body) = 0;
};

}  // namespace hydra
