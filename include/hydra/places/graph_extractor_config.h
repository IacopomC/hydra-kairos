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
#include <cstddef>
#include <cstdint>
#include <string>

/**
 * @file graph_extractor_config.h
 * @brief Configuration structures for extracting a sparse place graph from the
 *        GVD, including edge-proposal parameters and the settings specific to
 *        the compression-based and floodfill-based extractors.
 */

namespace hydra::places {

/// @brief Parameters for proposing edges between nodes with overlapping free space.
struct OverlapEdgeConfig {
  size_t num_neighbors_to_check = 4;  ///< Number of nearest neighbors to test for free-space overlap.
  double min_clearance_m = 0.4;       ///< Minimum obstacle clearance required at the overlap (m).
};

/// @brief Parameters for projecting edges that bridge disconnected components.
struct FreespaceEdgeConfig {
  double max_length_m = 2.0;          ///< Longest edge to attempt to project (m).
  size_t num_nodes_to_check = 5;      ///< Nodes per disconnected component to use as edge candidates.
  size_t num_neighbors_to_find = 1;   ///< Nearest neighbors to find in another component.
  double min_clearance_m = 0.5;       ///< Minimum obstacle clearance required along the edge (m).
};

/// @brief Common extractor settings shared by all graph-extraction strategies.
struct GraphExtractorConfig {
  bool add_overlap_edges = true;       ///< Whether to add edges between nodes with overlapping free space.
  OverlapEdgeConfig overlap_edges;     ///< Parameters for overlap-based edges.
  bool add_freespace_edges = true;     ///< Whether to add edges bridging disconnected components.
  FreespaceEdgeConfig freespace_edges; ///< Parameters for free-space bridging edges.
  double node_merge_distance_m = 0.2;  ///< Maximum node separation to consider for merging (m).
};

/// @brief Settings for the extractor that compresses the GVD to a target resolution.
struct CompressionExtractorConfig : public GraphExtractorConfig {
  double compression_distance_m = 0.5;  ///< Target average spacing of the sparse graph (m).
  bool validate_graph = false;          ///< Whether to run consistency checks on the produced graph.
  double min_node_distance_m = 0.3;     ///< Minimum clearance for a GVD node to be compressed (m).
  double min_edge_distance_m = 0.4;     ///< Minimum clearance for an edge to be kept (m).
  // node_merge_distance_m is inherited from GraphExtractorConfig (do NOT redeclare:
  // a derived copy shadows the base member that declare_config binds, so the YAML
  // value silently never reaches mergeNearbyNodes()).
  bool merge_nearby_nodes = true;       ///< Whether to merge nodes closer than node_merge_distance_m.
  bool add_heuristic_edges = false;     ///< Whether to add heuristic edges beyond the GVD-derived ones.
  std::string merge_policy = "basis_points";  ///< Name of the merge policy used to pick node representatives.
};

/// @brief Settings for the extractor that floodfills the GVD into nodes and edges.
struct FloodfillExtractorConfig : public GraphExtractorConfig {
  uint8_t min_extra_basis = 2;          ///< Minimum basis-point count for a voxel to be extracted.
  uint8_t min_vertex_basis = 3;         ///< Basis-point count at which a voxel is automatically a vertex.
  bool merge_new_nodes = true;          ///< Whether to merge nearby nodes during initial edge extraction.
  bool edge_splitting_merge_nodes = true;  ///< Whether to merge nearby nodes while splitting edges.
  size_t max_edge_split_iterations = 5; ///< Maximum edge-splitting passes; 0 disables splitting.
  int64_t max_edge_deviation = 4;       ///< Maximum squared voxel deviation of an edge from its supporting voxels.
};

/// @brief Bind OverlapEdgeConfig fields for serialization.
void declare_config(OverlapEdgeConfig& conf);
/// @brief Bind GraphExtractorConfig fields for serialization.
void declare_config(GraphExtractorConfig& conf);
/// @brief Bind CompressionExtractorConfig fields for serialization.
void declare_config(CompressionExtractorConfig& conf);
/// @brief Bind FloodfillExtractorConfig fields for serialization.
void declare_config(FloodfillExtractorConfig& conf);

}  // namespace hydra::places
