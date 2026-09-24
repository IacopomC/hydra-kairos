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
#include <config_utilities/virtual_config.h>

#include <list>
#include <memory>
#include <string>

/**
 * @file output_sink.h
 * @brief Generic observer interface for fanning out module output to consumers.
 *        A sink receives a fixed argument signature; sinks can wrap free
 *        functions or member functions, be built from config, and be invoked as
 *        a group.
 */

namespace hydra {

/// @brief Abstract consumer invoked with a fixed argument signature.
/// Provides factory helpers to wrap callbacks or methods, instantiate sinks
/// from config, and call or describe a whole list of sinks at once.
/// @tparam Args Argument types each sink is called with.
template <typename... Args>
struct OutputSink {
  using Sink = OutputSink<Args...>;
  using Ptr = std::shared_ptr<Sink>;
  using Factory = config::VirtualConfig<Sink, true>;  ///< Config that constructs a sink.
  using List = std::list<Ptr>;

  virtual ~OutputSink() = default;
  /// @brief Deliver one set of arguments to this sink.
  /// @param args Values forwarded to the consumer.
  virtual void call(Args... args) const = 0;
  /// @brief Return a human-readable description of the sink.
  virtual std::string printInfo() const { return "n/a"; }

  /// @brief Build a sink that forwards to a free callback.
  /// @param callback Function invoked when the sink is called.
  /// @return The wrapping sink.
  static Ptr fromCallback(const std::function<void(Args...)>& callback);

  /// @brief Build a sink that forwards to a const member function.
  /// @tparam T Type owning the method.
  /// @param callback Const member function to invoke.
  /// @param instance Object the method is called on.
  /// @return The wrapping sink.
  template <typename T>
  static Ptr fromMethod(void (T::*callback)(Args...) const, const T* instance);

  /// @brief Build a sink that forwards to a non-const member function.
  /// @tparam T Type owning the method.
  /// @param callback Member function to invoke.
  /// @param instance Object the method is called on.
  /// @return The wrapping sink.
  template <typename T>
  static Ptr fromMethod(void (T::*callback)(Args...), T* instance);

  /// @brief Construct sinks from a list of factory configs, skipping empty ones.
  /// @param configs Factory configs describing the sinks to create.
  /// @return The list of instantiated sinks.
  static List instantiate(const std::vector<Factory>& configs) {
    List sinks;
    for (const auto& config : configs) {
      if (config) {
        sinks.push_back(config.create());
      }
    }
    return sinks;
  }

  /// @brief Invoke every sink in a list with the same arguments.
  /// @param sinks Sinks to call.
  /// @param args Values forwarded to each sink.
  static void callAll(const List& sinks, Args... args) {
    for (const auto& sink : sinks) {
      if (sink) {
        sink->call(args...);
      }
    }
  }

  /// @brief Build a combined description of every sink in a list.
  /// @param sinks Sinks to describe.
  /// @return Newline-separated descriptions, one per sink.
  static std::string printSinks(const List& sinks) {
    std::stringstream ss;

    size_t sink_idx = 0;
    auto iter = sinks.begin();
    while (iter != sinks.end()) {
      const auto& sink = *iter;
      ss << "Sink " << sink_idx << ": " << (sink ? "\n" + sink->printInfo() : "n/a");
      ++iter;
      ++sink_idx;
      if (iter != sinks.end()) {
        ss << "\n";
      }
    }

    return ss.str();
  }
};

/// @brief Sink that forwards calls to a stored free callback.
/// @tparam Args Argument types the sink is called with.
template <typename... Args>
struct FunctionSink : OutputSink<Args...> {
  /// @brief Wrap a callback.
  /// @param f Function invoked on each call.
  FunctionSink(const std::function<void(Args...)>& f) : func(f) {}
  virtual ~FunctionSink() = default;

  void call(Args... args) const override { func(args...); }

  std::function<void(Args...)> func;  ///< Wrapped callback.
};

template <typename... Args>
typename OutputSink<Args...>::Ptr OutputSink<Args...>::fromCallback(
    const std::function<void(Args...)>& callback) {
  return std::make_shared<FunctionSink<Args...>>(callback);
}

/// @brief Sink that forwards calls to a const member function of an instance.
/// @tparam T Type owning the method.
/// @tparam Args Argument types the sink is called with.
template <typename T, typename... Args>
struct MethodSink : OutputSink<Args...> {
  /// @brief Bind a const member function to an instance.
  /// @param callback Const member function to invoke.
  /// @param instance Object the method is called on.
  MethodSink(void (T::*callback)(Args...) const, const T* instance)
      : callback(callback), instance(instance) {}

  void call(Args... args) const override { (instance->*callback)(args...); }

  void (T::*callback)(Args...) const;  ///< Bound const member function.
  const T* instance;                   ///< Object the method is called on.
};

template <typename... Args>
template <typename T>
typename OutputSink<Args...>::Ptr OutputSink<Args...>::fromMethod(
    void (T::*callback)(Args...) const, const T* instance) {
  return std::make_shared<MethodSink<T, Args...>>(callback, instance);
}

/// @brief Sink that forwards calls to a non-const member function of an instance.
/// @tparam T Type owning the method.
/// @tparam Args Argument types the sink is called with.
template <typename T, typename... Args>
struct NonConstMethodSink : OutputSink<Args...> {
  /// @brief Bind a non-const member function to an instance.
  /// @param callback Member function to invoke.
  /// @param instance Object the method is called on.
  NonConstMethodSink(void (T::*callback)(Args...), T* instance)
      : callback(callback), instance(instance) {}

  void call(Args... args) const override { (instance->*callback)(args...); }

  void (T::*callback)(Args...);  ///< Bound member function.
  T* instance;                   ///< Object the method is called on.
};

template <typename... Args>
template <typename T>
typename OutputSink<Args...>::Ptr OutputSink<Args...>::fromMethod(
    void (T::*callback)(Args...), T* instance) {
  return std::make_shared<NonConstMethodSink<T, Args...>>(callback, instance);
}

}  // namespace hydra
