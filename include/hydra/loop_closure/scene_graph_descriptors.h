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
 * @file scene_graph_descriptors.h
 * @brief Descriptor representation and the factories that build descriptors
 *        from a node's surrounding subgraph. A descriptor summarizes the
 *        neighborhood of an agent so it can be matched against other
 *        neighborhoods to find revisits.
 */
#pragma once
#include "hydra/common/dsg_types.h"
#include "hydra/loop_closure/subgraph_extraction.h"

namespace hydra::lcd {

/// @brief Fixed-length summary of the subgraph around one node, used for matching.
struct Descriptor {
  using Ptr = std::unique_ptr<Descriptor>;                  ///< Owning handle to a descriptor.
  Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> words;         ///< Word indices for sparse representations.
  Eigen::VectorXf values;                                   ///< Descriptor values, aligned with words when sparse.
  bool normalized = false;                                  ///< True when values already carry unit scale.
  bool is_null = false;                                     ///< True when the source subgraph was too small to describe.
  std::set<NodeId> nodes;                                   ///< Nodes that contributed to the descriptor.
  NodeId root_node;                                         ///< Node at the center of the described subgraph.
  Eigen::Vector3d root_position;                            ///< World position of the root node.
  std::chrono::nanoseconds timestamp;                       ///< Time associated with the root node.
};

/// @brief Interface for objects that build a descriptor from a node and its subgraph.
struct DescriptorFactory {
  using Ptr = std::unique_ptr<DescriptorFactory>;  ///< Owning handle to a factory.

  virtual ~DescriptorFactory() = default;

  /// @brief Build a descriptor for the neighborhood of the given node.
  /// @param dsg Scene graph supplying node attributes and connectivity.
  /// @param agent_node Node whose neighborhood is summarized.
  /// @return The constructed descriptor, possibly flagged null when there is too little support.
  virtual Descriptor::Ptr construct(const DynamicSceneGraph& dsg,
                                    const SceneGraphNode& agent_node) const = 0;
};

/// @brief Builds a descriptor directly from an agent node's stored features.
struct AgentDescriptorFactory : DescriptorFactory {
  AgentDescriptorFactory() = default;

  virtual ~AgentDescriptorFactory() = default;

  /// @brief Construct an agent-layer descriptor from the node's attributes.
  /// @param graph Scene graph supplying the agent attributes.
  /// @param agent_node Agent node to summarize.
  /// @return The agent descriptor.
  Descriptor::Ptr construct(const DynamicSceneGraph& graph,
                            const SceneGraphNode& agent_node) const override;
};

/// @brief Builds a descriptor from the semantic labels of nearby objects.
struct ObjectDescriptorFactory : DescriptorFactory {
  /// @brief Construct the factory.
  /// @param config Subgraph extraction settings for gathering nearby objects.
  /// @param num_classes Number of semantic classes spanning the label histogram.
  ObjectDescriptorFactory(const SubgraphConfig& config, size_t num_classes);

  /// @brief Construct a descriptor from the labels of objects around a node.
  /// @param graph Scene graph supplying object attributes and connectivity.
  /// @param agent_node Node at the center of the object neighborhood.
  /// @return The object descriptor.
  Descriptor::Ptr construct(const DynamicSceneGraph& graph,
                            const SceneGraphNode& agent_node) const override;

  const SubgraphConfig config;  ///< Subgraph extraction settings.
  const size_t num_classes;     ///< Number of semantic classes the descriptor spans.
};

/// @brief Maps a scalar value onto one of a fixed set of bins.
template <typename T>
struct HistogramConfig {
  /// @brief Construct an explicit binning over [min, max].
  /// @param min Lower edge of the range.
  /// @param max Upper edge of the range.
  /// @param bins Number of bins spanning the range.
  HistogramConfig(T min, T max, size_t bins) : min(min), max(max), bins(bins) {}

  /// @brief Construct a degenerate single-bin histogram.
  HistogramConfig() : HistogramConfig(0, 0, 1) {}

  T min;        ///< Lower edge of the binned range.
  T max;        ///< Upper edge of the binned range.
  size_t bins;  ///< Number of bins.

  /// @brief Map a value to its bin index, clamping to the end bins out of range.
  /// @param value Value to bin.
  /// @return Index in [0, bins - 1].
  size_t getBin(T value) const {
    if (value <= min) {
      return 0;
    }

    if (value >= max) {
      return bins - 1;
    }

    const T step = (max - min) / bins;
    return std::floor((value - min) / step);
  }
};

/// @brief Builds a descriptor by histogramming a geometric property of nearby places.
struct PlaceDescriptorFactory : DescriptorFactory {
  /// @brief Construct the factory.
  /// @param config Subgraph extraction settings for gathering nearby places.
  /// @param histogram Binning applied to the per-place property.
  PlaceDescriptorFactory(const SubgraphConfig& config,
                         const HistogramConfig<double>& histogram);

  /// @brief Construct a descriptor from the places around a node.
  /// @param graph Scene graph supplying place attributes and connectivity.
  /// @param agent_node Node at the center of the place neighborhood.
  /// @return The place descriptor.
  Descriptor::Ptr construct(const DynamicSceneGraph& graph,
                            const SceneGraphNode& agent_node) const override;

  const SubgraphConfig config;             ///< Subgraph extraction settings.
  const HistogramConfig<double> histogram; ///< Binning applied to the place property.
};

}  // namespace hydra::lcd
