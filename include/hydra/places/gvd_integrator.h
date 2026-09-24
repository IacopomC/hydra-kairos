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

#include <spatial_hash/neighbor_utils.h>

#include <utility>

#include "hydra/places/graph_extractor_interface.h"
#include "hydra/places/gvd_integrator_config.h"
#include "hydra/places/gvd_parent_tracker.h"
#include "hydra/places/gvd_utilities.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/places/update_statistics.h"
#include "hydra/reconstruction/voxel_types.h"
#include "hydra/utils/bucket_queue.h"

/**
 * @file gvd_integrator.h
 * @brief Incremental integrator that turns a TSDF into a Euclidean signed
 *        distance field and a generalized Voronoi diagram. Propagates distance
 *        changes through raise/lower wavefronts, tracks each voxel's
 *        nearest-surface parent, marks Voronoi cells, and feeds the result to a
 *        place-graph extractor.
 *
 * Based on the ESDF/GVD method in https://arxiv.org/abs/1611.03631
 * @see https://arxiv.org/abs/1611.03631
 */

namespace hydra::places {

/// @brief Entry in the ESDF propagation queue pairing an index with its voxel.
struct OpenQueueEntry {
  GlobalIndex index;  ///< Voxel index.
  GvdVoxel* voxel;    ///< Pointer to the voxel being processed.
};

/// @brief Builds and incrementally maintains the ESDF and GVD from a TSDF.
class GvdIntegrator {
 public:
  /// @brief Construct over a GVD layer with a place-graph extractor.
  GvdIntegrator(const GvdIntegratorConfig& config,
                const GvdLayer::Ptr& gvd_layer,
                const GraphExtractorInterface::Ptr& graph_extractor);

  virtual ~GvdIntegrator() = default;

  /// @brief Pull surface information from the TSDF and seed the propagation queue.
  /// @param clear_updated_flag Whether to clear each processed TSDF block's updated flag.
  /// @param use_all_blocks Whether to process all blocks rather than only updated ones.
  void updateFromTsdf(uint64_t timestamp_ns,
                      const TsdfLayer& tsdf,
                      bool clear_updated_flag,
                      bool use_all_blocks = false);

  /// @brief Run the wavefront propagation and extract the place graph.
  void updateGvd(uint64_t timestamp_ns);

  /// @brief Drop the named blocks from the GVD, cleaning up dependent state.
  void archiveBlocks(const BlockIndices& blocks);

  /// @brief Assign a fixed-band parent to a voxel from its neighbors.
  /// @return Whether a parent was successfully assigned.
  static bool setFixedParent(const GvdLayer& layer,
                             const GlobalIndices& neighbor_indices,
                             const GlobalIndex& voxel_index,
                             GvdVoxel& voxel);

 protected:
  /// @brief Update one voxel's GVD membership against a neighbor.
  void updateGvdVoxel(const GlobalIndex& voxel_index, GvdVoxel& voxel, GvdVoxel& other);

  /// @brief Clear a voxel's GVD membership and parent bookkeeping.
  void clearGvdVoxel(const GlobalIndex& index, GvdVoxel& voxel);

  /// @brief Test a voxel pair for the Voronoi condition and update membership.
  void updateVoronoiQueue(GvdVoxel& curr_voxel,
                          const GlobalIndex& curr_pos,
                          GvdVoxel& neighbor_voxel,
                          const GlobalIndex& neighbor_pos);

  /// @brief Mark surface voxels in a block as GVD seeds.
  void propagateSurface(const BlockIndex& block_index, const TsdfLayer& tsdf);

  /// @brief Fold a single TSDF block into the GVD layer.
  void processTsdfBlock(const TsdfBlock& block, const BlockIndex& index);

  /// @brief Initialize a GVD voxel from a TSDF voxel that was previously unobserved.
  void updateUnobservedVoxel(const TsdfVoxel& tsdf_voxel,
                             const GlobalIndex& index,
                             GvdVoxel& gvd_voxel);

  /// @brief Refresh an already-observed GVD voxel from its TSDF voxel.
  void updateObservedVoxel(const TsdfVoxel& tsdf_voxel,
                           const GlobalIndex& index,
                           GvdVoxel& gvd_voxel);

  /// @brief Push a voxel onto the open propagation queue.
  void pushToQueue(const GlobalIndex& index, GvdVoxel& voxel);

  /// @brief Propagate a distance increase outward from a voxel (raise wavefront).
  void raiseVoxel(const GlobalIndex& index, GvdVoxel& voxel);

  /// @brief Propagate a distance decrease outward from a voxel (lower wavefront).
  void lowerVoxel(const GlobalIndex& index, GvdVoxel& voxel);

  /// @brief Drain the open queue, applying raise and lower updates to convergence.
  void processOpenQueue();

  /// @brief Return whether a TSDF voxel lies within the trusted fixed band.
  bool isTsdfFixed(const TsdfVoxel& voxel);

  /// @brief Compute the world position of a voxel's nearest-surface parent.
  Point getParentPosition(const GlobalIndex& index, const GvdVoxel& voxel) const;

  /// @brief Estimate the TSDF gradient at a voxel, if it can be computed.
  std::optional<Point> computeGradient(const TsdfLayer& tsdf,
                                       const GlobalIndex& index) const;

 protected:
  UpdateStatistics update_stats_;  ///< Counters for the most recent update.

  const double default_distance_;  ///< Distance assigned to voxels with no known surface.
  GvdIntegratorConfig config_;     ///< Integrator configuration.
  GvdLayer::Ptr gvd_layer_;        ///< GVD layer being maintained.

  GraphExtractorInterface::Ptr graph_extractor_;          ///< Consumer that builds the place graph.
  GvdParentTracker parent_tracker_;                       ///< Tracks Voronoi voxel parents and vertices.
  const spatial_hash::NeighborSearch neighbor_search_;    ///< Neighbor-offset helper for propagation.

  BucketQueue<OpenQueueEntry> open_;  ///< Distance-bucketed queue driving the wavefronts.

  float voxel_size_;                    ///< Edge length of a voxel (m).
  float min_integration_distance_m_;    ///< Smallest distance at which TSDF data is integrated (m).
};

}  // namespace hydra::places
