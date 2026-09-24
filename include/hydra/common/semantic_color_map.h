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

#include <spark_dsg/color.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

/**
 * @file semantic_color_map.h
 * @brief Bidirectional mapping between semantic labels and colors. Resolves a
 *        label from a pixel color and a color from a label, with a designated
 *        color for unknown labels, and can be built randomly or from a file.
 */

namespace hydra {

/// @brief Two-way lookup between semantic labels and their display colors.
/// Tracks colors and labels seen without a mapping so that unknown inputs
/// resolve to a configured fallback color.
class SemanticColorMap {
 public:
  using Ptr = std::unique_ptr<SemanticColorMap>;
  using ColorSet = std::unordered_set<spark_dsg::Color, spark_dsg::Color::Hash>;
  using LabelToColorMap = std::unordered_map<uint32_t, spark_dsg::Color>;
  using ColorToLabelMap =
      std::unordered_map<spark_dsg::Color, uint32_t, spark_dsg::Color::Hash>;

  /// @brief Construct an empty, invalid color map.
  SemanticColorMap();

  /// @brief Construct from an explicit color-to-label table.
  /// @param map Mapping from color to label id.
  /// @param unknown_color Color returned for labels with no entry.
  explicit SemanticColorMap(const ColorToLabelMap& map,
                            const spark_dsg::Color& unknown_color = {});

  /// @brief Resolve the label for a color.
  /// @param color Color to look up.
  /// @return The label, or empty if the color is not in the map.
  std::optional<uint32_t> getLabelFromColor(const spark_dsg::Color& color) const;

  /// @brief Resolve the color for a label.
  /// @param label Label id to look up.
  /// @return The mapped color, or the unknown color if absent.
  spark_dsg::Color getColorFromLabel(const uint32_t& label) const;

  /// @brief Number of labels in the map.
  size_t getNumLabels() const;

  /// @brief Whether the map holds any entries.
  bool isValid() const;

  /// @brief Boolean conversion that is true when the map is valid.
  inline operator bool() const { return isValid(); }

  /// @brief Build a human-readable summary of the map.
  std::string toString() const;

 public:
  /// @brief Build a map assigning a random color to each label.
  /// @param num_labels Number of labels to generate colors for.
  /// @param unknown Color used for unknown labels.
  /// @return The generated color map.
  static SemanticColorMap::Ptr randomColors(size_t num_labels,
                                            const spark_dsg::Color& unknown = {});

  /// @brief Build a map from a delimited text file of colors and labels.
  /// @param filename Path to the file to parse.
  /// @param unknown Color used for unknown labels.
  /// @param delimiter Field separator in the file.
  /// @param skip_first_line Whether to skip a header row.
  /// @return The parsed color map.
  static SemanticColorMap::Ptr fromCsv(const std::string& filename,
                                       const spark_dsg::Color& unknown = {},
                                       char delimiter = ',',
                                       bool skip_first_line = true);

 private:
  uint32_t max_label_;                ///< Highest label id stored.
  ColorToLabelMap color_to_label_;    ///< Lookup from color to label.
  LabelToColorMap label_to_color_;    ///< Lookup from label to color.
  spark_dsg::Color unknown_color_;    ///< Color returned for unknown labels.

  mutable ColorSet unknown_colors_;   ///< Colors queried without a mapping.
  mutable std::unordered_set<uint32_t> unknown_labels_;  ///< Labels queried without a mapping.
};

/// @brief Stream a human-readable summary of the color map.
std::ostream& operator<<(std::ostream& out, const SemanticColorMap& cmap);

}  // namespace hydra
