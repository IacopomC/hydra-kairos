#include "hydra/bindings/glog_utilities.h"

#include <glog/logging.h>
#include <pybind11/stl/filesystem.h>

#include <cstdlib>
#include <iostream>

namespace hydra::python {

std::unique_ptr<GlogSingleton> GlogSingleton::instance_;

GlogSingleton& GlogSingleton::instance() {
  if (!instance_) {
    instance_.reset(new GlogSingleton());
  }
  return *instance_;
}

void GlogSingleton::setLogLevel(int log_level, int verbosity, bool override_settings) {
  if (configured_ && !override_settings) {
    return;
  }

  FLAGS_minloglevel = log_level;
  FLAGS_v = verbosity;
  configured_ = true;
}

void GlogSingleton::setLogDirectory(const std::string& dirname) {
  std::cerr << "[Hydra Python] Logging to: " << dirname << std::endl;
  FLAGS_log_dir = dirname;
  FLAGS_logtostderr = false;
  FLAGS_alsologtostderr = false;
  FLAGS_stderrthreshold = 1;
  FLAGS_minloglevel = 0;
  FLAGS_v = 5;
}

GlogSingleton::GlogSingleton() {
  FLAGS_logtostderr = true;
  // _hydra_bindings and _kairos_bindings each compile their own GlogSingleton
  // (static), but share a single libglog. Without this guard the second module
  // to construct its singleton calls InitGoogleLogging() on the already-
  // initialized shared glog and aborts ("You called InitGoogleLogging() twice").
  // glog 0.4 (focal) does not expose google::IsGoogleLoggingInitialized() in a
  // public header, so coordinate across modules with a process-global env flag.
  if (!std::getenv("HYDRA_PY_GLOG_INITIALIZED")) {
    google::InitGoogleLogging("hydra_python");
    google::InstallFailureSignalHandler();
    ::setenv("HYDRA_PY_GLOG_INITIALIZED", "1", 1);
  }
}

namespace glog_utilities {

using namespace pybind11::literals;

void addBindings(pybind11::module_& m) {
  m.def(
      "set_glog_level",
      [](int glog_level, int verbosity, bool override_settings) {
        GlogSingleton::instance().setLogLevel(glog_level, verbosity, override_settings);
      },
      "log_level"_a,
      "verbosity"_a = 0,
      "override_settings"_a = true);
  m.def(
      "set_glog_dir",
      [](const std::string& dirname) {
        GlogSingleton::instance().setLogDirectory(dirname);
      },
      "dirname"_a);
  m.def(
      "set_glog_dir",
      [](const std::filesystem::path& dirname) {
        GlogSingleton::instance().setLogDirectory(dirname);
      },
      "dirname"_a);
}

}  // namespace glog_utilities

}  // namespace hydra::python
