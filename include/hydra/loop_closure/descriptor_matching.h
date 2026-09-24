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
/**
 * @file descriptor_matching.h
 * @brief Scoring of descriptor pairs and search routines that rank candidate
 *        matches against a query descriptor. Scores are similarities where a
 *        larger value means a closer match.
 */
#pragma once
#include "hydra/loop_closure/scene_graph_descriptors.h"

namespace hydra::lcd {

/// @brief Selects the similarity used to compare two descriptors.
enum class DescriptorScoreType {
  COSINE,  ///< Cosine similarity between descriptor value vectors.
  L1       ///< Bag-of-words style L1 similarity.
};

/// @brief Thresholds and limits applied when searching for descriptor matches.
struct DescriptorMatchConfig {
  float min_score = 1.0;              ///< Minimum similarity for a candidate to be kept as a valid match.
  float min_registration_score = 1.0;///< Minimum similarity for a match to proceed to registration.
  double min_time_separation_s = 0.0; ///< Required time gap for same-robot matches to suppress near-in-time hits.
  size_t max_registration_matches = 5;///< Cap on the number of matches forwarded to registration.
  double min_score_ratio = 0.7;       ///< Keep matches scoring at least this fraction of the best score.
  double min_match_separation_m = 5.0;///< Minimum spatial spacing between retained match roots (m).
  DescriptorScoreType type = DescriptorScoreType::L1;  ///< Similarity used for scoring.
};

/// @brief Outcome of searching one layer: the ranked candidate roots and the
/// query and matched node sets needed to set up registration.
struct LayerSearchResults {
  std::vector<float> score;                  ///< Similarity of each retained match, ordered with match_root.
  std::set<NodeId> valid_matches;            ///< Roots that passed scoring and separation checks.
  std::set<NodeId> query_nodes;              ///< Nodes forming the query subgraph.
  std::vector<std::set<NodeId>> match_nodes; ///< Nodes of each matched subgraph, aligned with match_root.
  NodeId query_root;                         ///< Root node of the query.
  std::vector<NodeId> match_root;            ///< Retained match roots, best first.
};

using DescriptorCache = std::map<NodeId, Descriptor::Ptr>;       ///< Descriptors keyed by their root node.
using DescriptorCacheMap = std::map<NodeId, DescriptorCache>;    ///< Per-root caches of leaf descriptors.

/// @brief Accumulate a similarity over matched descriptor entries.
/// @param lhs First descriptor.
/// @param rhs Second descriptor.
/// @param distance_func Per-entry contribution combining the two values.
/// @return The accumulated similarity.
float computeDistance(const Descriptor& lhs,
                      const Descriptor& rhs,
                      const std::function<float(float, float)>& distance_func);

/// @brief Compute the cosine similarity between two descriptors.
/// @param lhs First descriptor.
/// @param rhs Second descriptor.
/// @return Cosine similarity; 1.0 when both descriptors have zero norm.
inline float computeCosineDistance(const Descriptor& lhs, const Descriptor& rhs) {
  float lhs_scale = lhs.normalized ? 1.0f : lhs.values.norm();
  float rhs_scale = rhs.normalized ? 1.0f : rhs.values.norm();

  if (lhs_scale == 0.0f && rhs_scale == 0.0f) {
    return 1.0f;
  }

  float scale = lhs_scale * rhs_scale;
  // TODO: we might want a looser check than this
  if (scale == 0.0f) {
    scale = 1.0f;  // force all zero descriptors to have 0 norm (instead of nan)
  }

  return computeDistance(
      lhs, rhs, [&scale](float lhs, float rhs) { return (lhs * rhs) / scale; });
}

/// @brief Compute the bag-of-words style L1 similarity between two descriptors.
/// @param lhs First descriptor.
/// @param rhs Second descriptor.
/// @return Similarity in [0, 2] where larger values indicate closer descriptors.
inline float computeL1Distance(const Descriptor& lhs, const Descriptor& rhs) {
  float lhs_scale = lhs.normalized ? 1.0f : lhs.values.lpNorm<1>();
  float rhs_scale = rhs.normalized ? 1.0f : rhs.values.lpNorm<1>();

  if (rhs_scale == 0.0f and lhs_scale == 0.0f) {
    return 0.0f;
  }

  lhs_scale = lhs_scale == 0.0f ? 1.0f : lhs_scale;
  rhs_scale = rhs_scale == 0.0f ? 1.0f : rhs_scale;

  const float l1_diff = computeDistance(lhs, rhs, [&](float lhs, float rhs) {
    const float lhs_val = lhs / lhs_scale;
    const float rhs_val = rhs / rhs_scale;
    return std::abs(lhs_val - rhs_val) - std::abs(lhs_val) - std::abs(rhs_val);
  });
  return 2.0f + l1_diff;
}

/// @brief Score two descriptors using the requested similarity type.
/// @param lhs First descriptor.
/// @param rhs Second descriptor.
/// @param type Similarity to apply.
/// @return The similarity score.
float computeDescriptorScore(const Descriptor& lhs,
                             const Descriptor& rhs,
                             DescriptorScoreType type);

/// @brief Search a cache of root descriptors for matches to a query.
/// @param descriptor Query descriptor.
/// @param match_config Scoring thresholds and separation limits.
/// @param valid_matches Candidate roots to consider.
/// @param descriptors Cache of root descriptors keyed by node.
/// @param root_leaf_map Mapping from each root to the leaf nodes under it.
/// @param query_id Root node of the query.
/// @return The ranked matches and the node sets needed for registration.
LayerSearchResults searchDescriptors(
    const Descriptor& descriptor,
    const DescriptorMatchConfig& match_config,
    const std::set<NodeId>& valid_matches,
    const DescriptorCache& descriptors,
    const std::map<NodeId, std::set<NodeId>>& root_leaf_map,
    NodeId query_id);

/// @brief Search per-root leaf descriptor caches for matches to a query.
/// @param descriptor Query descriptor.
/// @param match_config Scoring thresholds and separation limits.
/// @param valid_matches Candidate roots to consider.
/// @param leaf_cache_map Per-root caches of leaf descriptors.
/// @param query_id Root node of the query.
/// @return The ranked matches and the node sets needed for registration.
LayerSearchResults searchLeafDescriptors(const Descriptor& descriptor,
                                         const DescriptorMatchConfig& match_config,
                                         const std::set<NodeId>& valid_matches,
                                         const DescriptorCacheMap& leaf_cache_map,
                                         NodeId query_id);

}  // namespace hydra::lcd
