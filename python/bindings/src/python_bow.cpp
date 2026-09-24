/* -----------------------------------------------------------------------------
 * Offline BoW producer implementation. See python_bow.h.
 * -------------------------------------------------------------------------- */
#include "hydra/bindings/python_bow.h"

#include <glog/logging.h>
#include <hydra/common/pipeline_queues.h>
#include <opencv2/imgproc.hpp>
#include <pose_graph_tools/bow_query.h>
#include <pybind11/stl.h>

#include <memory>
#include <stdexcept>

#include "hydra/bindings/python_image.h"

namespace hydra::python {

BowProducer::BowProducer(const std::string& vocab_path,
                         int n_features,
                         bool color_is_rgb)
    : orb_(cv::ORB::create(n_features)), color_is_rgb_(color_is_rgb) {
  voc_.load(vocab_path);
  if (voc_.empty()) {
    throw std::runtime_error(
        "Failed to load DBoW3 vocabulary from '" + vocab_path +
        "'. Run scripts/fetch_orb_vocab.sh to download ORBvoc.yml.gz.");
  }
  LOG(INFO) << "[BowProducer] loaded DBoW3 vocab from " << vocab_path
            << " (size=" << voc_.size() << ", k=" << voc_.getBranchingFactor()
            << ", L=" << voc_.getDepthLevels() << ")";
}

void BowProducer::compute_and_push(const pybind11::buffer& rgb,
                                   uint32_t robot_id,
                                   uint32_t pose_id) {
  PythonImage py_img(rgb);
  cv::Mat color = getColorImage(py_img);
  if (color.empty()) {
    LOG(WARNING) << "[BowProducer] empty color image for pose_id=" << pose_id;
    return;
  }

  cv::Mat gray;
  const auto code = color_is_rgb_ ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY;
  cv::cvtColor(color, gray, code);

  std::vector<cv::KeyPoint> kpts;
  cv::Mat desc;
  orb_->detectAndCompute(gray, cv::noArray(), kpts, desc);

  DBoW3::BowVector bow;
  if (!desc.empty()) {
    voc_.transform(desc, bow);
  }

  auto& queue = PipelineQueues::instance().bow_queue;
  if (!queue) {
    throw std::runtime_error(
        "bow_queue not allocated; frontend.lcd_use_bow_vectors must be true "
        "and the pipeline must be constructed before BowProducer.push");
  }

  auto msg = std::make_shared<pose_graph_tools::BowQuery>();
  msg->robot_id = robot_id;
  msg->pose_id = pose_id;
  msg->bow_vector.word_ids.reserve(bow.size());
  msg->bow_vector.word_values.reserve(bow.size());
  for (const auto& entry : bow) {
    msg->bow_vector.word_ids.push_back(static_cast<uint32_t>(entry.first));
    msg->bow_vector.word_values.push_back(static_cast<float>(entry.second));
  }
  // shared_ptr<T> -> shared_ptr<const T> implicit conversion matches
  // BowQueue's value type (pipeline_queues.h:55).
  queue->push(msg);
}

namespace python_bow {

namespace py = pybind11;
using namespace py::literals;

void addBindings(py::module_& m) {
  py::class_<BowProducer>(m, "BowProducer")
      .def(py::init<const std::string&, int, bool>(),
           "vocab_path"_a,
           "n_features"_a = 1000,
           "color_is_rgb"_a = true)
      .def("compute_and_push",
           &BowProducer::compute_and_push,
           "rgb"_a,
           "robot_id"_a,
           "pose_id"_a);
}

}  // namespace python_bow
}  // namespace hydra::python
