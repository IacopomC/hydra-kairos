/* -----------------------------------------------------------------------------
 * Offline BoW producer for the Hydra Python pipeline. Pairs cv::ORB feature
 * extraction with a DBoW3 vocabulary (DBoW2 .yml.gz files load fine — DBoW3
 * is a superset format-wise) and pushes the resulting BowQuery onto
 * PipelineQueues::bow_queue so the frontend's assignBowVectors can stamp
 * dbow_ids/dbow_values onto agent attributes.
 * -------------------------------------------------------------------------- */
#pragma once
#include <pybind11/pybind11.h>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <string>

#include "DBoW3.h"

namespace hydra::python {

class BowProducer {
 public:
  BowProducer(const std::string& vocab_path,
              int n_features = 1000,
              bool color_is_rgb = true);

  // Run ORB on `rgb`, transform to a BowVector via the loaded vocabulary,
  // and push a BowQuery(robot_id, pose_id, ...) onto bow_queue. Throws if
  // bow_queue is not allocated (frontend.lcd_use_bow_vectors must be true).
  void compute_and_push(const pybind11::buffer& rgb,
                        uint32_t robot_id,
                        uint32_t pose_id);

 private:
  cv::Ptr<cv::ORB> orb_;
  DBoW3::Vocabulary voc_;
  bool color_is_rgb_;
};

namespace python_bow {
void addBindings(pybind11::module_& m);
}  // namespace python_bow

}  // namespace hydra::python
