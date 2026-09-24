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
#include <onnxruntime_cxx_api.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "hydra/gnn/tensor.h"

/**
 * @file ort_utilities.h
 * @brief Helpers bridging the project tensor type and the ONNX Runtime
 *        inference library. Describes a model input or output field (its name,
 *        shape, and element type), validates and converts tensors into runtime
 *        values, and queries a session for its declared inputs and outputs.
 */

namespace hydra::gnn {

/// @brief Describes a single named input or output of an inference model.
struct FieldInfo {
  std::string name;                ///< Field name as declared by the model.
  std::vector<int64_t> dims;       ///< Declared shape; -1 marks a dynamic axis.
  ONNXTensorElementDataType type;  ///< Runtime element type of the field.
  std::string type_name;           ///< Human-readable name of the element type.

  /// @brief Whether the tensor's element type matches this field's type.
  /// @param tensor Tensor to check.
  /// @return True when the element types correspond.
  bool tensorMatchesType(const Tensor& tensor) const;

  /// @brief Check that the tensor's shape (allowing dynamic axes) matches this
  ///        field, throwing std::invalid_argument on a mismatch.
  /// @param tensor Tensor to validate.
  void validateTensor(const Tensor& tensor) const;

  /// @brief Wrap a tensor as a runtime value that aliases its buffer.
  /// @param mem_info Runtime memory descriptor for the buffer location.
  /// @param input Tensor whose data backs the value.
  /// @param validate When true, validate the tensor against this field first.
  /// @return Runtime value referring to the tensor's memory.
  Ort::Value makeOrtValue(const OrtMemoryInfo* mem_info,
                          const Tensor& input,
                          bool validate = true) const;

  /// @brief Allocate a tensor matching this field's static shape and type.
  /// @return Tensor sized to the declared dimensions.
  Tensor getTensor() const;

  /// @brief Allocate a tensor, resolving each dynamic axis from supplied sizes.
  /// @param dims_to_read For each dynamic axis in order, the index into
  ///        output_dims that provides its size.
  /// @param output_dims Candidate sizes that dynamic axes draw from.
  /// @return Tensor with all axes resolved to concrete sizes.
  Tensor getDynamicTensor(const std::vector<size_t>& dims_to_read,
                          const std::vector<int64_t>& output_dims) const;
};

/// @brief Read the input fields declared by an inference session.
/// @param session Session to query.
/// @param allocator Allocator used for field names.
/// @return One entry per input, empty when session or allocator is null.
std::vector<FieldInfo> getSessionInputs(const Ort::Session* session,
                                        OrtAllocator* allocator);

/// @brief Read the output fields declared by an inference session.
/// @param session Session to query.
/// @param allocator Allocator used for field names.
/// @return One entry per output, empty when session or allocator is null.
std::vector<FieldInfo> getSessionOutputs(const Ort::Session* session,
                                         OrtAllocator* allocator);

/// @brief Human-readable name for a runtime element type.
/// @param type Element type to name.
/// @return Type name, or "invalid" for an unrecognized value.
std::string getElementTypeString(ONNXTensorElementDataType type);

/// @brief Write a field's name, type, and shape (dynamic axes shown as N).
/// @param out Stream to write to.
/// @param info Field to describe.
/// @return The same stream, to allow chaining.
std::ostream& operator<<(std::ostream& out, const FieldInfo& info);

}  // namespace::hydra
