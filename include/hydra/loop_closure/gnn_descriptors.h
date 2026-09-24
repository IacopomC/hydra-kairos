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
 * @file gnn_descriptors.h
 * @brief Descriptor factories that encode a node's neighborhood with a learned
 *        graph neural network. The object and place variants build a feature
 *        tensor from the surrounding subgraph and run it through a model to
 *        produce the descriptor values.
 */
#pragma once
#include "hydra/gnn/gnn_interface.h"
#include "hydra/loop_closure/scene_graph_descriptors.h"

namespace hydra::lcd {

/// @brief Builds object-layer descriptors by running a graph network over the
/// objects around a node, with each object encoded by its label embedding.
struct ObjectGnnDescriptor : DescriptorFactory {
  using LabelEmbeddings = std::map<uint8_t, Eigen::VectorXf>;  ///< Per-label feature vectors keyed by semantic label.

  /// @brief Construct the factory and load its model.
  /// @param model_path Path to the serialized graph network.
  /// @param config Subgraph extraction settings for gathering nearby objects.
  /// @param max_edge_distance_m Maximum spacing for an edge between two nodes in the input graph.
  /// @param label_embeddings Feature vector for each semantic label.
  /// @param use_pos_in_feature Append node position to each node feature when true.
  ObjectGnnDescriptor(const std::string& model_path,
                      const SubgraphConfig& config,
                      double max_edge_distance_m,
                      const LabelEmbeddings& label_embeddings,
                      bool use_pos_in_feature = true);

  /// @brief Assemble the model input tensors from a set of nodes.
  /// @param graph Scene graph supplying node attributes and positions.
  /// @param nodes Nodes that make up the subgraph.
  /// @return Named tensors of node features and edges for the model.
  gnn::TensorMap makeInput(const DynamicSceneGraph& graph,
                           const std::set<NodeId>& nodes) const;

  /// @brief Build an object descriptor for the neighborhood of a node.
  /// @param graph Scene graph supplying the object neighborhood.
  /// @param agent_node Node at the center of the neighborhood.
  /// @return The model-produced descriptor.
  Descriptor::Ptr construct(const DynamicSceneGraph& graph,
                            const DynamicSceneGraphNode& agent_node) const override;

 protected:
  const SubgraphConfig config_;                  ///< Subgraph extraction settings.
  std::unique_ptr<gnn::GnnInterface> model_;     ///< Loaded graph network.

  double max_edge_distance_m_;                   ///< Maximum spacing for an input-graph edge.
  size_t label_embedding_size_;                  ///< Dimension of each label embedding.
  std::map<uint8_t, Eigen::VectorXf> label_embeddings_;  ///< Feature vector per semantic label.
  const bool use_pos_in_feature_;                ///< Whether node position is appended to features.
};

/// @brief Builds place-layer descriptors by running a graph network over the
/// places around a node.
struct PlaceGnnDescriptor : DescriptorFactory {
  /// @brief Construct the factory and load its model.
  /// @param model_path Path to the serialized graph network.
  /// @param config Subgraph extraction settings for gathering nearby places.
  /// @param use_pos_in_feature Append node position to each node feature when true.
  PlaceGnnDescriptor(const std::string& model_path,
                     const SubgraphConfig& config,
                     bool use_pos_in_feature = true);

  /// @brief Assemble the model input tensors from a set of nodes.
  /// @param graph Scene graph supplying node attributes and positions.
  /// @param nodes Nodes that make up the subgraph.
  /// @return Named tensors of node features and edges for the model.
  gnn::TensorMap makeInput(const DynamicSceneGraph& graph,
                           const std::set<NodeId>& nodes) const;

  /// @brief Build a place descriptor for the neighborhood of a node.
  /// @param graph Scene graph supplying the place neighborhood.
  /// @param agent_node Node at the center of the neighborhood.
  /// @return The model-produced descriptor.
  Descriptor::Ptr construct(const DynamicSceneGraph& graph,
                            const DynamicSceneGraphNode& agent_node) const override;

 protected:
  const SubgraphConfig config_;               ///< Subgraph extraction settings.
  const bool use_pos_in_feature_;             ///< Whether node position is appended to features.
  std::unique_ptr<gnn::GnnInterface> model_;  ///< Loaded graph network.
};

/// @brief Load semantic-label embeddings from a file.
/// @param filename Path to the embeddings file.
/// @return The embedding vector for each label.
ObjectGnnDescriptor::LabelEmbeddings loadLabelEmbeddings(const std::string& filename);

}  // namespace hydra::lcd
