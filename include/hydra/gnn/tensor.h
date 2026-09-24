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
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

/**
 * @file tensor.h
 * @brief Dense, multi-dimensional numeric buffer used to feed and receive data
 *        from graph-neural-net inference. Stores element type, shape, and a
 *        reference-counted byte buffer, and offers typed access plus a 2D Eigen
 *        view over the underlying memory.
 */

namespace hydra::gnn {

/// @brief Forward declaration so the stream operator can be declared first.
class Tensor;

/// @brief Write a human-readable type and shape summary of the tensor.
/// @param out Stream to write to.
/// @param tensor Tensor to describe.
/// @return The same stream, to allow chaining.
std::ostream& operator<<(std::ostream& out, const Tensor& tensor);

/// @brief Owns a typed, shaped block of contiguous memory for inference I/O.
class Tensor {
 public:
  /// @brief Eigen row-major matrix view over a tensor's elements of type T.
  template <typename T>
  using MapTemplate =
      Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

  /// @brief Supported element types.
  ///
  /// This is a subset of the element types the ONNX Runtime inference library
  /// handles; types such as float16 or complex64 are omitted because they have
  /// no trivial Eigen representation.
  enum class Type {
    FLOAT32,  ///< 32-bit IEEE floating point.
    FLOAT64,  ///< 64-bit IEEE floating point.
    INT8,     ///< 8-bit signed integer.
    INT16,    ///< 16-bit signed integer.
    INT32,    ///< 32-bit signed integer.
    INT64,    ///< 64-bit signed integer.
    UINT8,    ///< 8-bit unsigned integer.
    UINT16,   ///< 16-bit unsigned integer.
    UINT32,   ///< 32-bit unsigned integer.
    UINT64,   ///< 64-bit unsigned integer.
  };

  /// @brief Test whether the stored element type matches the C++ type T.
  ///
  /// The primary template returns false; an explicit specialization is provided
  /// for each supported scalar type.
  /// @param type Element type to compare against.
  /// @return True when T corresponds to the given element type.
  template <typename T>
  static inline bool matchesType([[maybe_unused]] Type type,
                                 [[maybe_unused]] T* dummy = nullptr) {
    return false;
  }

  /// @brief Construct an empty FLOAT32 tensor with no dimensions.
  Tensor();

  /// @brief Construct an empty tensor of the given type with no dimensions.
  /// @param type Element type.
  explicit Tensor(Type type);

  /// @brief Construct a FLOAT32 tensor with the given two-dimensional shape.
  /// @param rows Number of rows.
  /// @param cols Number of columns.
  Tensor(int64_t rows, int64_t cols);

  /// @brief Construct a typed tensor with the given two-dimensional shape.
  /// @param rows Number of rows.
  /// @param cols Number of columns.
  /// @param type Element type.
  Tensor(int64_t rows, int64_t cols, Type type);

  /// @brief Construct a typed tensor with an arbitrary shape.
  /// @param dims Size of each dimension.
  /// @param type Element type.
  Tensor(const std::vector<int64_t>& dims, Type type);

  ~Tensor() = default;

  /// @brief True when the tensor holds at least one element.
  operator bool() const;

  /// @brief Element type of the tensor.
  Type type() const;

  /// @brief Total number of elements (product of all dimensions).
  size_t size() const;

  /// @brief Size of the backing buffer in bytes.
  size_t num_bytes() const;

  /// @brief Size of each dimension.
  const std::vector<int64_t>& dims() const;

  /// @brief Number of dimensions.
  size_t num_dims() const;

  /// @brief First dimension, or 0 when the tensor has no dimensions.
  int64_t rows() const;

  /// @brief Second dimension; 1 for a one-dimensional tensor and 0 when empty.
  int64_t cols() const;

  /// @brief Read-only typed pointer to the underlying element data.
  /// @param validate When true, throw if T does not match the stored type.
  /// @return Pointer to the first element, reinterpreted as T.
  template <typename T>
  const T* data(bool validate = true) const {
    if (validate && !matchesType<T>(type_)) {
      throw std::invalid_argument("invalid data conversion for tensor");
    }

    return reinterpret_cast<const T*>(memory_->data());
  }

  /// @brief Mutable typed pointer to the underlying element data.
  /// @param validate When true, throw if T does not match the stored type.
  /// @return Pointer to the first element, reinterpreted as T.
  template <typename T>
  T* data(bool validate = true) {
    return const_cast<T*>(const_cast<const Tensor*>(this)->data<T>(validate));
  }

  /// @brief Map the data onto a 2D row-major Eigen matrix sized rows() by cols().
  /// @param validate When true, throw if T mismatches or the tensor has more
  ///        than two dimensions.
  /// @return Eigen map over the existing buffer (no copy).
  template <typename T>
  MapTemplate<T> map(bool validate = true) {
    if (validate && dims_.size() > 2) {
      throw std::invalid_argument("cannot map data to 2D matrix");
    }

    return MapTemplate<T>(data<T>(validate), rows(), cols());
  }

 private:
  Type type_;                                ///< Element type of the stored data.
  std::shared_ptr<std::vector<char>> memory_;  ///< Reference-counted byte buffer.
  std::vector<int64_t> dims_;                ///< Size of each dimension.
  size_t size_;                              ///< Total number of elements.
};

/// @brief Matches float against FLOAT32.
template <>
inline bool Tensor::matchesType(Tensor::Type type, float*) {
  return type == Tensor::Type::FLOAT32;
}

/// @brief Matches double against FLOAT64.
template <>
inline bool Tensor::matchesType(Tensor::Type type, double*) {
  return type == Tensor::Type::FLOAT64;
}

/// @brief Matches uint8_t against UINT8.
template <>
inline bool Tensor::matchesType(Tensor::Type type, uint8_t*) {
  return type == Tensor::Type::UINT8;
}

/// @brief Matches uint16_t against UINT16.
template <>
inline bool Tensor::matchesType(Tensor::Type type, uint16_t*) {
  return type == Tensor::Type::UINT16;
}

/// @brief Matches uint32_t against UINT32.
template <>
inline bool Tensor::matchesType(Tensor::Type type, uint32_t*) {
  return type == Tensor::Type::UINT32;
}

/// @brief Matches uint64_t against UINT64.
template <>
inline bool Tensor::matchesType(Tensor::Type type, uint64_t*) {
  return type == Tensor::Type::UINT64;
}

/// @brief Matches int8_t against INT8.
template <>
inline bool Tensor::matchesType(Tensor::Type type, int8_t*) {
  return type == Tensor::Type::INT8;
}

/// @brief Matches int16_t against INT16.
template <>
inline bool Tensor::matchesType(Tensor::Type type, int16_t*) {
  return type == Tensor::Type::INT16;
}

/// @brief Matches int32_t against INT32.
template <>
inline bool Tensor::matchesType(Tensor::Type type, int32_t*) {
  return type == Tensor::Type::INT32;
}

/// @brief Matches int64_t against INT64.
template <>
inline bool Tensor::matchesType(Tensor::Type type, int64_t*) {
  return type == Tensor::Type::INT64;
}

}  // namespace::hydra
