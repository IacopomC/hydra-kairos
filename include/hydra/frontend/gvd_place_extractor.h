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

#include <memory>

#include "hydra/common/output_sink.h"
#include "hydra/frontend/freespace_places_interface.h"
#include "hydra/places/graph_extractor_config.h"
#include "hydra/places/gvd_integrator_config.h"
#include "hydra/places/gvd_voxel.h"
#include "hydra/reconstruction/tsdf_interpolators.h"
#include "hydra/utils/log_utilities.h"

/**
 * @file gvd_place_extractor.h
 * @brief Free-space place extractor built on a generalized Voronoi diagram. It
 *        integrates the TSDF into a GVD, extracts a sparse places graph from
 *        the medial axis, and writes the resulting place nodes and edges.
 */

namespace hydra {

namespace places {
class GvdIntegrator;          ///< Forward declaration; integrates the TSDF into a GVD.
class GraphExtractorInterface; ///< Forward declaration; extracts a places graph from the GVD.
}  // namespace places

struct VolumetricWindow;

/// @brief Extracts free-space places from a generalized Voronoi diagram (GVD).
/// The TSDF is integrated into a GVD layer, a graph extractor pulls a sparse
/// places graph from it, and the result is filtered and merged into the scene
/// graph. Supports querying whether arbitrary positions lie in free space.
class GvdPlaceExtractor : public FreespacePlacesInterface {
 public:
  using PositionMatrix = Eigen::Matrix<double, 3, Eigen::Dynamic>;  ///< 3xN matrix of world positions.
  using Sink = OutputSink<uint64_t,
                          const Eigen::Isometry3f&,
                          const places::GvdLayer&,
                          const places::GraphExtractorInterface*>;  ///< Observer hook for extraction results.

  /// @brief Settings for GVD integration, graph extraction, and filtering.
  struct Config {
    places::GvdIntegratorConfig gvd;  ///< GVD integration parameters.
    config::VirtualConfig<places::GraphExtractorInterface> graph;  ///< Places-graph extractor to use.
    config::VirtualConfig<TsdfInterpolator> tsdf_interpolator;     ///< Optional TSDF downsampler.
    size_t min_component_size = 3;    ///< Smallest connected place component kept.
    bool filter_places = true;        ///< Drop isolated place nodes during extraction.
    bool filter_ground = false;       ///< Drop place nodes near the ground plane.
    double robot_height = 0.0;        ///< Body height used by ground filtering (m).
    double node_tolerance = 1.0;      ///< Position tolerance for matching existing nodes (m).
    double edge_tolerance = 1.0;      ///< Tolerance for matching existing edges (m).
    bool add_freespace_edges = false; ///< Add extra edges across nearby free space.
    places::FreespaceEdgeConfig freespace_config;  ///< Parameters for free-space edge insertion.
    std::vector<Sink::Factory> sinks;  ///< Factories for extraction-result observers.
  } const config;

  /// @brief Construct and validate the extractor, creating its sub-modules.
  explicit GvdPlaceExtractor(const Config& config);

  virtual ~GvdPlaceExtractor();

  /// @brief Save the extracted places graph for offline inspection.
  void save(const LogSetup& logs) const override;

  /// @brief Return the set of place nodes currently considered active.
  NodeIdSet getActiveNodes() const override;

  /// @brief Test whether each input position lies in observed free space.
  /// @param positions 3xN matrix of world positions to query.
  /// @param freespace_distance_m Minimum clearance required to count as free.
  /// @return One flag per column; empty when there is no input.
  std::vector<bool> inFreespace(const PositionMatrix& positions,
                                double freespace_distance_m) const override;

  /// @brief Integrate the latest TSDF into the GVD and extract the places graph.
  void detect(const ActiveWindowOutput& msg) override;

  /// @brief Merge the extracted places into the scene graph at the given time.
  void updateGraph(uint64_t timestamp_ns, DynamicSceneGraph& graph) override;

  /// @brief Remove place nodes whose component is smaller than the configured minimum.
  /// @param graph Scene graph updated in place.
  /// @param active_neighborhood Active node set pruned alongside the graph.
  void filterIsolated(DynamicSceneGraph& graph, NodeIdSet& active_neighborhood);

  /// @brief Remove place nodes that fall near the ground plane.
  void filterGround(DynamicSceneGraph& graph);

 protected:
  mutable std::mutex gvd_mutex_;                       ///< Guards the GVD layer during update and queries.
  places::GvdLayer::Ptr gvd_;                          ///< GVD layer integrated from the TSDF.
  std::shared_ptr<places::GraphExtractorInterface> graph_extractor_;  ///< Pulls a places graph from the GVD.
  std::unique_ptr<places::GvdIntegrator> gvd_integrator_;  ///< Integrates the TSDF into the GVD.
  std::unique_ptr<TsdfInterpolator> tsdf_interpolator_;    ///< Optional TSDF downsampler.
  std::unique_ptr<VolumetricWindow> map_window_;       ///< Window bounds applied during integration.
  NodeIdSet active_nodes_;                             ///< Place nodes currently active.
  Eigen::Vector3d latest_pos_;                         ///< Most recent body position in the world frame.
  Sink::List sinks_;                                   ///< Extraction-result observers.

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<FreespacePlacesInterface,
                                     GvdPlaceExtractor,
                                     Config>("gvd");  ///< Self-registration under the name "gvd".
};

/// @brief Declare the config fields for the GVD place extractor.
void declare_config(GvdPlaceExtractor::Config& config);

}  // namespace hydra
