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
#include <spark_dsg/node_symbol.h>

#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

/**
 * @file display_utilities.h
 * @brief Helpers for turning numeric and node-id data into compact human-readable
 *        strings for logging and debugging.
 */

namespace hydra {

/// @brief Format a byte count as a human-readable size string.
/// @param bytes Size in bytes.
/// @return The size scaled to a convenient unit.
std::string getHumanReadableMemoryString(size_t bytes);

/// @brief Format a dynamic vector as a string, abbreviating long vectors.
/// Vectors with at most max_size entries are printed in full; longer vectors show the
/// first and last num_to_show entries with an ellipsis between them.
/// @tparam Scalar Element type of the vector.
/// @param vector Vector to format.
/// @param precision Number of digits printed per element.
/// @param max_size Length above which the vector is abbreviated.
/// @param num_to_show Entries shown from each end when abbreviating.
/// @return The formatted string.
template <typename Scalar>
std::string showVector(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& vector,
                       int precision = 3,
                       int max_size = 8,
                       int num_to_show = 3) {
  std::stringstream ss;
  if (vector.rows() <= max_size) {
    const Eigen::IOFormat format(
        precision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
    ss << vector.format(format);
    return ss.str();
  }

  const Eigen::IOFormat format(
      precision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");

  ss << "[" << vector.head(num_to_show).format(format) << ", ..., "
     << vector.tail(num_to_show).format(format) << "]";
  return ss.str();
}

/// @brief Format a node id as its symbolic label.
/// @param id Node id to format.
/// @return The label string for the node symbol.
inline std::string printNodeId(spark_dsg::NodeId id) {
  return spark_dsg::NodeSymbol(id).getLabel();
}

}  // namespace hydra
