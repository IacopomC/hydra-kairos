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
#include <spark_dsg/scene_graph_types.h>

#include <Eigen/Geometry>
#include <map>
#include <string>

/**
 * @file config_utilities.h
 * @brief Conversion traits used when (de)serializing config values. Layer-keyed
 *        maps are exchanged through their human-readable layer names, and
 *        quaternions through a string-keyed map of components.
 */

namespace hydra {

/// @brief Converts a map keyed by layer id to and from one keyed by layer name.
/// Serializes through string layer names so configs stay human-readable.
/// @tparam T Mapped value type.
template <typename T>
struct LayerMapConversion {
  using TargetMap = std::map<std::string, T>;  ///< Serialized form keyed by layer name.
  using SourceMap = std::map<spark_dsg::LayerId, T>;  ///< In-memory form keyed by layer id.

  /// @brief Convert the layer-id map into a layer-name map for serialization.
  /// @param other Source map keyed by layer id.
  /// @return Map keyed by layer name.
  static TargetMap toIntermediate(const SourceMap& other, std::string&) {
    TargetMap to_return;
    for (const auto& kv_pair : other) {
      to_return[spark_dsg::DsgLayers::LayerIdToString(kv_pair.first)] = kv_pair.second;
    }

    return to_return;
  }

  /// @brief Rebuild the layer-id map from a serialized layer-name map.
  /// @param other Source map keyed by layer name.
  /// @param value Output map keyed by layer id; cleared before populating.
  static void fromIntermediate(const TargetMap& other, SourceMap& value, std::string&) {
    value.clear();
    for (const auto& kv_pair : other) {
      value[spark_dsg::DsgLayers::StringToLayerId(kv_pair.first)] = kv_pair.second;
    }
  }
};

/// @brief Converts a quaternion to and from a string-keyed map of its components.
struct QuaternionConverter {
  using DoubleMap = std::map<std::string, double>;  ///< Component map keyed by name.

  /// @brief Convert a quaternion into a map of named components for serialization.
  /// @param other Quaternion to serialize.
  /// @param error Set with a description if conversion fails.
  /// @return Map of quaternion components keyed by name.
  static DoubleMap toIntermediate(const Eigen::Quaterniond& other, std::string& error);

  /// @brief Rebuild a quaternion from a map of named components.
  /// @param other Component map keyed by name.
  /// @param value Output quaternion.
  /// @param error Set with a description if conversion fails.
  static void fromIntermediate(const DoubleMap& other,
                               Eigen::Quaterniond& value,
                               std::string& error);
};

}  // namespace hydra
