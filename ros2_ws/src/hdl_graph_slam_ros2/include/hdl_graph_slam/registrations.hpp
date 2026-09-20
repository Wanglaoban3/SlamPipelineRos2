// SPDX-License-Identifier: BSD-2-Clause
// ROS2 port of registrations.hpp
#ifndef HDL_GRAPH_SLAM_REGISTRATIONS_HPP
#define HDL_GRAPH_SLAM_REGISTRATIONS_HPP

#include <pcl/registration/registration.h>

#include <hdl_graph_slam/param_helper.hpp>

namespace hdl_graph_slam {

/**
 * @brief select a scan matching algorithm according to node parameters
 * @param pnh
 * @return selected scan matching
 */
pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr select_registration_method(ParamNodeHandle& pnh);

}  // namespace hdl_graph_slam

#endif  //
