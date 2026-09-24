cmake_minimum_required(VERSION 3.14)
project(hydra_python)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

include(FetchContent)
FetchContent_Declare(
  pybind11 GIT_REPOSITORY https://github.com/pybind/pybind11.git GIT_TAG v2.13.6
)
FetchContent_MakeAvailable(pybind11)

# DBoW3 powers the offline BoW producer. Header+lib build, OpenCV-only deps.
# Pulled here (and not in the top-level hydra CMake) because it's only used
# by the python bindings.
FetchContent_Declare(
  dbow3
  GIT_REPOSITORY https://github.com/rmsalinas/DBow3.git
  GIT_TAG master
)
# Skip DBoW3's bundled demo/utility executables.
set(BUILD_UTILS OFF CACHE BOOL "" FORCE)
# DBoW3's CMakeLists declares cmake_minimum_required(VERSION 2.6), which
# CMake 3.27+ refuses. CMAKE_POLICY_VERSION_MINIMUM tells CMake to apply
# the policies of 3.5 even when the included project asked for 2.6.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
FetchContent_MakeAvailable(dbow3)
if(TARGET DBoW3)
  target_compile_options(
    DBoW3 PRIVATE -w  # silence third-party warnings
  )
  set_target_properties(DBoW3 PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# ORB lives in opencv_features2d, which the core hydra library doesn't link.
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d)

if(PROJECT_IS_TOP_LEVEL)
  find_package(hydra REQUIRED)
endif()

pybind11_add_module(
  _hydra_bindings
  bindings/src/bindings.cpp
  bindings/src/color_parser.cpp
  bindings/src/depth_parser.cpp
  bindings/src/glog_utilities.cpp
  bindings/src/label_parser.cpp
  bindings/src/python_batch.cpp
  bindings/src/python_bow.cpp
  bindings/src/python_image.cpp
  bindings/src/python_pipeline.cpp
  bindings/src/python_reconstruction.cpp
  bindings/src/python_sensor_input.cpp
  bindings/src/python_sensors.cpp
)
target_link_libraries(
  _hydra_bindings
  PRIVATE hydra::hydra DBoW3 ${OpenCV_LIBRARIES}
)
target_include_directories(
  _hydra_bindings
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/bindings/include
          ${dbow3_SOURCE_DIR}/src  # DBoW3.h umbrella header
)
