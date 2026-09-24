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
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "hydra/gnn/tensor.h"

/**
 * @file gnn_interface.h
 * @brief Loads a graph-neural-net model and runs inference on it. Wraps a
 *        session of the ONNX Runtime inference library behind a private
 *        implementation and exposes call operators that map named input tensors
 *        to named output tensors, with optional resolution of dynamic output
 *        axes.
 */

namespace hydra::gnn {

/// @brief Maps a field name to its tensor value.
using TensorMap = std::map<std::string, Tensor>;
/// @brief Maps an output field name to the dynamic-axis indices it resolves.
using DynamicIndexMap = std::map<std::string, std::vector<size_t>>;

/// @brief Forward declaration of the hidden runtime-backed implementation.
struct GnnInterfaceImpl;

/// @brief Runs inference on a loaded graph-neural-net model.
class GnnInterface {
 public:
  /// @brief Load a model from disk with no dynamic output axes.
  /// @param model_path Path to the model file.
  explicit GnnInterface(const std::string& model_path);

  /// @brief Load a model and declare which output axes are dynamic.
  /// @param model_path Path to the model file.
  /// @param output_map Per-output mapping from dynamic axes to size indices.
  GnnInterface(const std::string& model_path, const DynamicIndexMap& output_map);

  ~GnnInterface();

  /// @brief Run inference from node features and edge indices.
  ///
  /// Assumes the model was exported with inputs named "x" then "edge_index" and
  /// a single output named "output".
  /// @param x Node features.
  /// @param edge_index Edge indices.
  /// @return The model's output tensor.
  Tensor operator()(const Tensor& x, const Tensor& edge_index) const;

  /// @brief Run inference with arbitrary named inputs.
  /// @param input Map from input field name to tensor value.
  /// @return Map from output field name to result tensor.
  TensorMap operator()(const TensorMap& input) const;

  /// @brief Run inference with arbitrary named inputs and dynamic output sizes.
  /// @param input Map from input field name to tensor value.
  /// @param output_sizes Candidate sizes that dynamic output axes draw from.
  /// @return Map from output field name to result tensor.
  TensorMap operator()(const TensorMap& input,
                       const std::vector<int64_t>& output_sizes) const;

 protected:
  std::unique_ptr<GnnInterfaceImpl> impl_;  ///< Hidden runtime-backed state.

  std::string model_path_;  ///< Path the model was loaded from.

  /// @brief Write the available runtime providers and the model's input and
  ///        output fields.
  /// @param out Stream to write to.
  /// @param gnn Interface to describe.
  /// @return The same stream, to allow chaining.
  friend std::ostream& operator<<(std::ostream& out, const GnnInterface& gnn);
};

}  // namespace::hydra
