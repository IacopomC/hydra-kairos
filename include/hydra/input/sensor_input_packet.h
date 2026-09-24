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
 * @file sensor_input_packet.h
 * @brief The raw sensor measurement carried by an input frame. Defines the
 *        SensorInputPacket base plus the image (RGB-D) and point-cloud variants,
 *        each of which knows how to populate an InputData via fillInputData().
 */

#include "hydra/input/input_data.h"

namespace hydra {

/// @brief Base class for a sensor's raw measurement at one timestamp. Subclasses
/// implement fillInputDataImpl() to convert themselves into processed InputData.
struct SensorInputPacket {
  using Ptr = std::shared_ptr<SensorInputPacket>;

  explicit SensorInputPacket(uint64_t stamp, const std::string& sensor_name)
      : timestamp_ns(stamp), sensor_name(sensor_name) {}

  virtual ~SensorInputPacket() = default;

  /// @brief Populate @p msg from this packet (returns false on failure).
  bool fillInputData(InputData& msg) const;

  const uint64_t timestamp_ns;     ///< Acquisition time of the measurement (ns).
  const std::string sensor_name;   ///< Name of the originating sensor.
  std::string sensor_frame;        ///< Coordinate frame the measurement is in.
  FeatureVector input_feature;     ///< Learned feature for the frame (e.g. CLIP for a camera).

 protected:
  virtual bool fillInputDataImpl(InputData& msg) const = 0;
};

/// @brief An RGB-D image measurement: color, depth, and label images per pixel.
struct ImageInputPacket : public SensorInputPacket {
  explicit ImageInputPacket(uint64_t stamp, const std::string& sensor_name);

  cv::Mat color;                   ///< Color image (one color per pixel).
  cv::Mat depth;                   ///< Depth image (one depth per pixel).
  cv::Mat labels;                  ///< Semantic label image (one label per pixel).
  FeatureMap<int> label_features;  ///< Features associated with each label.
  bool color_is_bgr = false;       ///< Whether the color image is in BGR order.

 protected:
  bool fillInputDataImpl(InputData& msg) const override;
};

/// @brief A point-cloud measurement: per-point positions, colors, and labels.
struct CloudInputPacket : public SensorInputPacket {
  explicit CloudInputPacket(uint64_t stamp, const std::string& sensor_name);

  bool in_world_frame = false;  ///< Whether points are already in the world frame.
  cv::Mat points;               ///< Per-point 3D positions.
  cv::Mat colors;               ///< Per-point colors.
  cv::Mat labels;               ///< Per-point semantic labels.

 protected:
  bool fillInputDataImpl(InputData& msg) const override;
};

}  // namespace hydra
