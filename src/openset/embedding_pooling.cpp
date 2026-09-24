#include "hydra/openset/embedding_pooling.h"

#include <config_utilities/config.h>

namespace hydra {

void declare_config(MeanPooling::Config&) { config::name("MeanPooling::Config"); }

FeatureVector MeanPooling::pool(const FeatureGroup& features,
                                const WeightVector&) const {
  if (features.cols() == 0) {
    return FeatureVector();
  }

  return features.rowwise().mean();
}

void declare_config(WeightedMeanPooling::Config&) {
  config::name("WeightedMeanPooling::Config");
}

FeatureVector WeightedMeanPooling::pool(const FeatureGroup& features,
                                        const WeightVector& weights) const {
  if (features.cols() == 0 || weights.rows() != features.cols()) {
    return FeatureVector();
  }

  // weighted sum of the feature columns, then average over the column count
  // (normalized by the number of features rather than by the sum of weights)
  const FeatureVector weighted = features * weights;
  return weighted / features.cols();
}

void declare_config(MaxPooling::Config&) { config::name("MaxPooling::Config"); }

FeatureVector MaxPooling::pool(const FeatureGroup& features,
                               const WeightVector& weights) const {
  if (features.cols() == 0 || weights.rows() != features.cols()) {
    return FeatureVector();
  }

  // select the single feature column carrying the largest weight; the locals
  // are named min_* but maxCoeff writes the index of the maximum weight
  int min_row = 0;
  int min_col = 0;
  weights.maxCoeff(&min_row, &min_col);
  return features.col(min_row);
}

}  // namespace hydra
