// SPDX-License-Identifier: BSD-2-Clause
// ROS2 port helper: ros1-NodeHandle-like parameter interfaces on rclcpp
#ifndef PARAM_HELPER_HPP
#define PARAM_HELPER_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace hdl_graph_slam {

// free function version: declare-with-default if missing, then read
template<typename T>
T param_or(rclcpp::Node* node, const std::string& name, const T& default_value) {
  if(!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

// minimal ros1-NodeHandle-like wrapper used by ported library classes
// (holds a raw pointer so it can be used inside node constructors)
class ParamNodeHandle {
public:
  explicit ParamNodeHandle(rclcpp::Node* node) : node_(node) {}

  template<typename T>
  T param(const std::string& name, const T& default_value) {
    if(!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, default_value);
    }
    return node_->get_parameter(name).get_value<T>();
  }

  bool has(const std::string& name) { return node_->has_parameter(name); }

private:
  rclcpp::Node* node_;
};

}  // namespace hdl_graph_slam

#endif
