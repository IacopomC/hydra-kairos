#pragma once

/**
 * @file input_conversion.h
 * @brief Helpers that turn a raw InputPacket into processed InputData and
 *        normalize the image data: type/format normalization, label/depth/color
 *        conversion, and placing the vertex map in the requested frame.
 */

#include <memory>

#include "hydra/input/input_data.h"

namespace hydra {

struct InputPacket;

namespace conversions {

/// @brief Convert an input packet into InputData if possible.
/// @param input_packet The input packet to convert.
/// @param vertices_in_world_frame If true, convert the vertex image to the world
/// frame; otherwise leave it in the sensor frame.
/// @return The input data on success, nullptr otherwise.
std::unique_ptr<InputData> parseInputPacket(const InputPacket& input_packet,
                                            const bool vertices_in_world_frame = false);

/// @brief Ensure all images are of the expected type/format.
bool normalizeData(InputData& data, bool normalize_labels = true);

/// @brief Normalize the depth image to the expected type.
bool normalizeDepth(InputData& data);

/// @brief Derive a label image from a color image.
bool colorToLabels(cv::Mat& label_image, const cv::Mat& colors);

// TODO(nathan) check if the conversions are directly used...
/// @brief Convert the label image to the expected format.
bool convertLabels(InputData& data);

/// @brief Convert the depth image to the expected format.
bool convertDepth(InputData& data);

/// @brief Convert the color image to the expected format.
bool convertColor(InputData& data);

/// @brief Ensure the vertex map is in the requested frame.
/// @param data The input data to convert.
/// @param in_world_frame If true, convert to the world frame; otherwise sensor frame.
void convertVertexMap(InputData& data, bool in_world_frame);

}  // namespace conversions
}  // namespace hydra
