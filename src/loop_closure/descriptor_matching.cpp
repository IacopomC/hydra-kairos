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
#include "hydra/loop_closure/descriptor_matching.h"

#include <glog/logging.h>

namespace hydra::lcd {

using Dsg = DynamicSceneGraph;

float computeDistanceHist(const Descriptor& lhs,
                          const Descriptor& rhs,
                          const std::function<float(float, float)>& distance_func) {
  CHECK_EQ(lhs.values.rows(), rhs.values.rows());

  float score = 0.0;
  for (int r = 0; r < lhs.values.rows(); ++r) {
    // see https://ieeexplore.ieee.org/document/1641018 for derivation,
    // but this needs to match the BoW based distance
    if (lhs.values(r, 0) == 0.0 || rhs.values(r, 0) == 0.0) {
      continue;
    }

    score += distance_func(lhs.values(r, 0), rhs.values(r, 0));
  }

  return score;
}

float computeDistanceBow(const Descriptor& lhs,
                         const Descriptor& rhs,
                         const std::function<float(float, float)>& distance_func) {
  CHECK_EQ(lhs.values.rows(), lhs.words.rows());
  CHECK_EQ(rhs.values.rows(), rhs.words.rows());
  // Both descriptors store their word ids sorted ascending, so a merge-style
  // walk over the two sorted arrays accumulates the per-word contribution only
  // where the same word appears in both; non-shared words are skipped.
  float score = 0.0;
  int r1 = 0;
  int r2 = 0;
  while (r1 < lhs.values.rows() && r2 < rhs.values.rows()) {
    const uint32_t word1 = lhs.words(r1, 0);
    const uint32_t word2 = rhs.words(r2, 0);

    if (word1 == word2) {
      score += distance_func(lhs.values(r1, 0), rhs.values(r2, 0));
      ++r1;
      ++r2;
    } else if (word1 < word2) {
      ++r1;
    } else {
      ++r2;
    }
  }

  return score;
}

float computeDistance(const Descriptor& lhs,
                      const Descriptor& rhs,
                      const std::function<float(float, float)>& distance_func) {
  // Dense histogram descriptors carry no word ids; sparse BoW descriptors do.
  // Pick the matching accumulation strategy accordingly.
  if (!lhs.words.size() && !rhs.words.size()) {
    return computeDistanceHist(lhs, rhs, distance_func);
  } else {
    return computeDistanceBow(lhs, rhs, distance_func);
  }
}

float computeDescriptorScore(const Descriptor& lhs,
                             const Descriptor& rhs,
                             DescriptorScoreType type) {
  // Returns a similarity in [0, 1] (higher is more similar), not a distance.
  // Each branch remaps the underlying distance metric into that range.
  switch (type) {
    case DescriptorScoreType::COSINE:
      // map [-1, 1] to [0, 1]
      return 0.5f * computeCosineDistance(lhs, rhs) + 0.5f;
    case DescriptorScoreType::L1:
    default:
      // map [2, 0] to [0, 1]
      return 1.0f - 0.5f * computeL1Distance(lhs, rhs);
  }
}

LayerSearchResults searchDescriptors(
    const Descriptor& descriptor,
    const DescriptorMatchConfig& match_config,
    const std::set<NodeId>& valid_matches,
    const DescriptorCache& descriptors,
    const std::map<NodeId, std::set<NodeId>>& root_leaf_map,
    NodeId query_id) {
  float best_score = 0.0f;
  std::vector<std::pair<NodeId, float>> new_valid_match_scores;
  std::set<NodeId> new_valid_matches;

  size_t num_same_parent = 0;
  size_t num_inside_horizon = 0;
  size_t num_low_score = 0;
  size_t num_null = 0;
  size_t num_default_match = 0;
  size_t num_shared_nodes = 0;

  VLOG(10) << "--------------------------------------------------";

  // Score the query descriptor against every candidate root, dropping
  // candidates that fail a sequence of cheap rejection tests before the
  // (more expensive) descriptor comparison.
  for (const auto& valid_id : valid_matches) {
    // Reject: the query agent is itself a child of this candidate root, so a
    // match would be trivially against its own neighborhood.
    if (root_leaf_map.at(valid_id).count(query_id)) {
      ++num_same_parent;
      continue;
    }

    const auto& other_ptr = descriptors.at(valid_id);
    if (!other_ptr) {
      num_null++;
      continue;
    }

    const bool same_robot = NodeSymbol(query_id).category() ==
                            NodeSymbol(*root_leaf_map.at(valid_id).begin()).category();
    if (!same_robot) {
      const auto child = NodeSymbol(*root_leaf_map.at(valid_id).begin());
      LOG(WARNING) << "Found different robot: query " << NodeSymbol(query_id).getLabel()
                   << ", putative: " << child.getLabel();
    }

    // Reject: candidate is too recent in time on the same robot's trajectory
    // (within the temporal horizon), so it isn't a genuine revisit.
    const Descriptor& other_descriptor = *other_ptr;
    std::chrono::duration<double> diff_s =
        descriptor.timestamp - other_descriptor.timestamp;
    if (same_robot && diff_s.count() < match_config.min_time_separation_s) {
      VLOG(10) << "diff: " << diff_s.count()
               << " (threshold: " << match_config.min_time_separation_s << ")";
      ++num_inside_horizon;
      continue;
    }

    // Either descriptor is a placeholder (e.g. empty subgraph): accept as a
    // default match with a sentinel score so registration can still try it.
    if (descriptor.is_null || other_descriptor.is_null) {
      ++num_default_match;
      new_valid_matches.insert(valid_id);
      new_valid_match_scores.push_back({valid_id, -1.0});
      continue;
    }

    // Reject: the two subgraphs overlap in actual nodes, so this is the same
    // physical region rather than a distinct revisit.
    size_t curr_shared = 0;
    for (const auto& id : descriptor.nodes) {
      if (other_descriptor.nodes.count(id)) {
        ++curr_shared;
      }
    }

    if (curr_shared > 0) {
      ++num_shared_nodes;
      continue;
    }

    // Surviving candidates get scored; track the best score seen for the
    // ratio test applied during selection below.
    const float curr_score =
        computeDescriptorScore(descriptor, other_descriptor, match_config.type);
    if (curr_score > best_score) {
      best_score = curr_score;
    }

    if (curr_score > match_config.min_score) {
      new_valid_matches.insert(valid_id);
      new_valid_match_scores.push_back({valid_id, curr_score});
    } else {
      ++num_low_score;
    }
  }

  // TODO: add layer id in again or handle stats better
  VLOG(1) << "matching "
          << " -> shared: " << num_same_parent << ", null: " << num_null
          << ", horizon: " << num_inside_horizon << ", low: " << num_low_score
          << ", default: " << num_default_match << ", shared: " << num_shared_nodes
          << ", valid: " << new_valid_match_scores.size();

  // Rank candidates best-first, then pick the top registration candidates:
  // keep only those close to the best score (ratio test), spatially separated
  // from already-chosen matches, and above the registration threshold.
  std::sort(new_valid_match_scores.begin(),
            new_valid_match_scores.end(),
            [](const std::pair<NodeId, float>& a, const std::pair<NodeId, float>& b) {
              return a.second > b.second;
            });
  std::vector<std::set<NodeId>> match_nodes;
  std::vector<NodeId> matches;
  std::vector<float> match_scores;
  for (const auto& id_score : new_valid_match_scores) {
    // Sorted descending, so once we fall below the registration threshold no
    // later candidate can qualify either.
    if (id_score.second < match_config.min_registration_score) {
      break;
    }

    if (id_score.second > best_score * match_config.min_score_ratio) {
      // Skip candidates whose root is too close to one already selected, so
      // the registration set spans distinct locations.
      bool spatialy_distinct = true;
      for (const auto& m : matches) {
        if ((descriptors.at(id_score.first)->root_position -
             descriptors.at(m)->root_position)
                .norm() < match_config.min_match_separation_m) {
          spatialy_distinct = false;
          break;
        }
      }

      if (!spatialy_distinct) {
        continue;
      }

      match_nodes.push_back(descriptors.at(id_score.first)->nodes);
      matches.push_back(id_score.first);
      match_scores.push_back(id_score.second);
    }
    if (matches.size() == match_config.max_registration_matches) {
      break;
    }
  }

  // No registration candidate cleared the bar: still report the best score so
  // callers can see how close this layer came (matches/match_nodes stay empty).
  if (match_scores.empty()) {
    match_scores.push_back(best_score);
  }

  return {match_scores,
          new_valid_matches,
          descriptor.nodes,
          match_nodes,
          descriptor.root_node,
          matches};
}

LayerSearchResults searchLeafDescriptors(const Descriptor& descriptor,
                                         const DescriptorMatchConfig& match_config,
                                         const std::set<NodeId>& valid_matches,
                                         const DescriptorCacheMap& leaf_cache_map,
                                         NodeId query_id) {
  float best_score = 0.0f;
  std::vector<std::pair<std::pair<NodeId, NodeId>, float>> new_valid_match_scores;
  std::set<NodeId> new_valid_matches;
  // Diagnostics: counters mirror the ones searchDescriptors prints, so we
  // can see why agent-layer matching produces 0 valid matches in practice.
  size_t leaves_seen = 0;
  size_t skipped_self = 0;
  size_t skipped_horizon = 0;
  size_t skipped_low = 0;
  size_t empty_words = 0;
  // Agent-layer search: the valid roots come from the layer above, and each
  // root owns a cache of leaf (agent) descriptors. Compare the query agent
  // against every leaf under every surviving root.
  for (const auto& valid_id : valid_matches) {
    const DescriptorCache& leaf_cache = leaf_cache_map.at(valid_id);

    for (const auto& id_desc_pair : leaf_cache) {
      ++leaves_seen;
      bool same_robot = true;
      if (NodeSymbol(id_desc_pair.first).category() !=
          NodeSymbol(query_id).category()) {
        same_robot = false;
      }
      if (id_desc_pair.first == query_id) {
        ++skipped_self;
        continue;  // disallow self matches even if they probably can't happen
      }

      const Descriptor& other_descriptor = *id_desc_pair.second;
      std::chrono::duration<double> diff_s =
          descriptor.timestamp - other_descriptor.timestamp;
      if (same_robot && diff_s.count() < match_config.min_time_separation_s) {
        ++skipped_horizon;
        continue;
      }

      // Track whether the BoW vectors are actually populated; an empty
      // descriptor.words on either side makes computeDistanceBow return 0
      // -> score = 1 - 0.5*2 = 0, falsely below threshold.
      if (descriptor.words.size() == 0 || other_descriptor.words.size() == 0) {
        ++empty_words;
      }

      const float curr_score =
          computeDescriptorScore(descriptor, other_descriptor, match_config.type);

      if (curr_score > best_score) {
        best_score = curr_score;
      }

      if (curr_score > match_config.min_score) {
        new_valid_matches.insert(id_desc_pair.first);
        new_valid_match_scores.push_back(
            {{id_desc_pair.first, other_descriptor.root_node}, curr_score});
      } else {
        ++skipped_low;
      }
    }
  }
  LOG(INFO) << "[Hydra LCD] searchLeafDescriptors: roots=" << valid_matches.size()
            << " leaves_seen=" << leaves_seen << " skipped_self=" << skipped_self
            << " skipped_horizon=" << skipped_horizon
            << " empty_words=" << empty_words
            << " skipped_low=" << skipped_low << " valid=" << new_valid_matches.size()
            << " best_score=" << best_score
            << " min_score=" << match_config.min_score
            << " (query=" << NodeSymbol(query_id).getLabel()
            << " query_words=" << descriptor.words.size() << ")";

  std::sort(new_valid_match_scores.begin(),
            new_valid_match_scores.end(),
            [](const std::pair<std::pair<NodeId, NodeId>, float>& a,
               const std::pair<std::pair<NodeId, NodeId>, float>& b) {
              return a.second > b.second;
            });
  std::vector<std::set<NodeId>> match_nodes;
  std::vector<NodeId> matches;
  std::vector<float> match_scores;

  // Select registration candidates best-first. The top candidate is always
  // kept; the rest must clear the ratio test against the best score. Here a
  // match stores the matched agent leaf in match_nodes and its place root in
  // matches (the inverse layout from searchDescriptors).
  size_t match_index = 0;
  for (const auto& id_id_score : new_valid_match_scores) {
    bool passes_ratio = match_index == 0 ||
                        id_id_score.second >= best_score * match_config.min_score_ratio;
    ++match_index;

    if (!passes_ratio) {
      continue;
    }

    match_nodes.push_back({id_id_score.first.first});
    matches.push_back(id_id_score.first.second);
    match_scores.push_back(id_id_score.second);

    if (matches.size() == match_config.max_registration_matches) {
      break;
    }
  }

  return {match_scores,
          new_valid_matches,
          descriptor.nodes,
          match_nodes,
          descriptor.root_node,
          matches};
}

}  // namespace hydra::lcd
