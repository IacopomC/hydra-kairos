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

/**
 * @file association_strategies.h
 * @brief Concrete association strategies that propose merge candidates for a
 *        node within a scene-graph layer. Each strategy returns the subset of
 *        the layer that should be tested against a given node, ranging from
 *        every other node to spatial nearest-neighbor lookups, optionally
 *        restricted to nodes sharing the same semantic label.
 */
#include "hydra/backend/merge_proposer.h"
#include "hydra/utils/nearest_neighbor_utilities.h"

namespace hydra::association {

/// @brief Considers every other node in the layer as a merge candidate.
struct Pairwise : AssociationStrategy {
  struct Config {};

  Pairwise(const Config, const SceneGraphLayer&) {}
  virtual ~Pairwise() = default;

  /// @brief Returns all nodes in the layer as candidates for the given node.
  /// @param layer Layer holding the candidate nodes.
  /// @param node Node whose merge candidates are requested.
  /// @return View over the candidate nodes.
  LayerView candidates(const SceneGraphLayer& layer,
                       const SceneGraphNode& node) const override;
};

/// @brief Declares the configuration fields for Pairwise.
void declare_config(Pairwise::Config& config);

/// @brief Considers every other node sharing the node's semantic label.
struct SemanticPairwise : AssociationStrategy {
  struct Config {};

  SemanticPairwise(const Config, const SceneGraphLayer&) {}
  virtual ~SemanticPairwise() = default;

  /// @brief Returns nodes with a matching semantic label as candidates.
  /// @param layer Layer holding the candidate nodes.
  /// @param node Node whose merge candidates are requested.
  /// @return View over the candidate nodes.
  LayerView candidates(const SceneGraphLayer& layer,
                       const SceneGraphNode& node) const override;
};

/// @brief Declares the configuration fields for SemanticPairwise.
void declare_config(SemanticPairwise::Config& config);

/// @brief Proposes the spatially nearest nodes as merge candidates.
struct NearestNode : AssociationStrategy {
  struct Config {
    size_t num_merges_to_consider = 1;  ///< Number of nearest candidates to return per node.
  } const config;

  /// @brief Builds a nearest-neighbor index over the layer.
  /// @param config Strategy configuration.
  /// @param layer Layer indexed for nearest-neighbor queries.
  NearestNode(const Config& config, const SceneGraphLayer& layer);

  virtual ~NearestNode();

  /// @brief Returns the nearest nodes to the given node as candidates.
  /// @param layer Layer holding the candidate nodes.
  /// @param node Node whose merge candidates are requested.
  /// @return View over the nearest candidate nodes.
  LayerView candidates(const SceneGraphLayer& layer,
                       const SceneGraphNode& node) const override;

  NearestNodeFinder::Ptr node_finder;  ///< Spatial index used to answer queries.
};

/// @brief Declares the configuration fields for NearestNode.
void declare_config(NearestNode::Config& config);

/// @brief Proposes the spatially nearest nodes that share a semantic label.
struct SemanticNearestNode : AssociationStrategy {
  struct Config {
    size_t num_merges_to_consider = 1;  ///< Number of nearest candidates to return per node.
  } const config;

  /// @brief Builds a per-label nearest-neighbor index over the layer.
  /// @param config Strategy configuration.
  /// @param layer Layer indexed for nearest-neighbor queries.
  SemanticNearestNode(const Config& config, const SceneGraphLayer& layer);

  virtual ~SemanticNearestNode();

  /// @brief Returns the nearest same-label nodes as candidates.
  /// @param layer Layer holding the candidate nodes.
  /// @param node Node whose merge candidates are requested.
  /// @return View over the nearest same-label candidate nodes.
  LayerView candidates(const SceneGraphLayer& layer,
                       const SceneGraphNode& node) const override;

  SemanticNodeFinders node_finders;  ///< Per-label spatial indices used to answer queries.
};

/// @brief Declares the configuration fields for SemanticNearestNode.
void declare_config(SemanticNearestNode::Config& config);

}  // namespace hydra::association
